#include "MeshCuttingPanel.h"

#include <algorithm>
#include <sstream>
#include <utility>

#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Destruction/Fracture.h"

namespace Riemann
{
	namespace
	{
		float BoundsMaxDim(const Box3& bounds)
		{
			const Vector3 size = bounds.Max - bounds.Min;
			return std::max(size.x, std::max(size.y, size.z));
		}

		void RefreshStaticMeshPointers(StaticMesh* mesh)
		{
			if (mesh == nullptr)
			{
				return;
			}
			mesh->Vertices = mesh->mVertices.empty() ? nullptr : mesh->mVertices.data();
			mesh->Indices = mesh->mIndices.empty() ? nullptr : mesh->mIndices.data();
		}

		int CountValidTriangles(const DynamicMesh& mesh)
		{
			int count = 0;
			for (int triangleIndex = 0; triangleIndex < mesh.GetTriangleCount(); ++triangleIndex)
			{
				if (mesh.IsTriangleFast(triangleIndex))
				{
					++count;
				}
			}
			return count;
		}

		bool BuildDynamicMeshFromStaticMesh(const StaticMesh& sourceMesh, DynamicMesh* mesh)
		{
			if (mesh == nullptr || sourceMesh.GetVertexCount() == 0 || sourceMesh.GetTriangleCount() == 0 || sourceMesh.Vertices == nullptr)
			{
				return false;
			}

			mesh->Clear();
			for (uint32_t vertexIndex = 0; vertexIndex < sourceMesh.GetVertexCount(); ++vertexIndex)
			{
				mesh->AppendVertex(sourceMesh.Vertices[vertexIndex]);
			}

			for (uint32_t triangleIndex = 0; triangleIndex < sourceMesh.GetTriangleCount(); ++triangleIndex)
			{
				uint32_t a = 0;
				uint32_t b = 0;
				uint32_t c = 0;
				sourceMesh.GetVertIndices(triangleIndex, a, b, c);
				if (a >= sourceMesh.GetVertexCount() || b >= sourceMesh.GetVertexCount() || c >= sourceMesh.GetVertexCount())
				{
					continue;
				}
				mesh->AppendTriangle(Index3((int)a, (int)b, (int)c));
			}

			mesh->BuildBounds();
			mesh->CalculateWeightAverageNormals();
			return CountValidTriangles(*mesh) > 0;
		}

		bool BuildLocalStaticMesh(const DynamicMesh& sourceMesh, const Vector3& center, StaticMesh* mesh)
		{
			if (mesh == nullptr || CountValidTriangles(sourceMesh) == 0)
			{
				return false;
			}

			mesh->Clear();
			mesh->Flags = 0;
			for (int vertexIndex = 0; vertexIndex < sourceMesh.GetVertexCount(); ++vertexIndex)
			{
				mesh->AddVertex(sourceMesh.GetVertex(vertexIndex) - center);
			}

			for (int triangleIndex = 0; triangleIndex < sourceMesh.GetTriangleCount(); ++triangleIndex)
			{
				if (!sourceMesh.IsTriangleFast(triangleIndex))
				{
					continue;
				}

				const Index3 tri = sourceMesh.GetTriangle(triangleIndex);
				if (tri.a < 0 || tri.b < 0 || tri.c < 0 ||
					tri.a >= sourceMesh.GetVertexCount() || tri.b >= sourceMesh.GetVertexCount() || tri.c >= sourceMesh.GetVertexCount())
				{
					continue;
				}
				mesh->AddTriangle((uint32_t)tri.a, (uint32_t)tri.b, (uint32_t)tri.c);
			}

			if (mesh->GetTriangleCount() == 0)
			{
				mesh->Clear();
				return false;
			}

			mesh->CalculateBoundingBox();
			mesh->CalculateWeightAverageNormals();
			return true;
		}

		PlanarCutOptions BuildMeshCuttingOptions(int seed)
		{
			PlanarCutOptions options;
			options.SnapTolerance = 1e-5f;
			options.BoundsPaddingScale = 0.10f;
			options.RandomSeed = seed;
			options.WeldSharedEdges = true;
			options.MinTriangleCount = 4;
			return options;
		}

	}

	bool LoadMeshCuttingSource(const std::string& objPath, MeshCuttingSource* source)
	{
		if (source == nullptr)
		{
			return false;
		}

		source->Mesh.Clear();
		source->Bounds = Box3::Empty();
		source->MaxSeparation = 1.0f;
		source->Status.clear();

		StaticMesh sourceMesh;
		sourceMesh.Flags = 0;
		if (objPath.empty() || !sourceMesh.LoadObj(objPath.c_str()) ||
			sourceMesh.GetVertexCount() == 0 || sourceMesh.GetTriangleCount() == 0)
		{
			source->Status = "Failed to read OBJ";
			return false;
		}

		sourceMesh.CalculateBoundingBox();
		sourceMesh.CalculateWeightAverageNormals();

		source->Bounds = sourceMesh.BoundingVolume;
		source->MaxSeparation = std::max(BoundsMaxDim(source->Bounds), 1e-3f) * 1.15f;
		source->Mesh = std::move(sourceMesh);
		RefreshStaticMeshPointers(&source->Mesh);

		std::ostringstream status;
		status << "Ready: " << source->Mesh.GetTriangleCount() << " triangles";
		source->Status = status.str();
		return true;
	}

	bool BuildMeshCuttingPanel(const MeshCuttingParams& params, MeshCuttingResult* result)
	{
		if (result == nullptr)
		{
			return false;
		}

		result->Pieces.clear();
		result->Status.clear();
		result->MaxSeparation = 1.0f;
		result->SourceBounds = Box3::Empty();

		StaticMesh sourceStaticMesh;
		sourceStaticMesh.Flags = 0;
		if (params.ObjPath.empty() || !sourceStaticMesh.LoadObj(params.ObjPath.c_str()) ||
			sourceStaticMesh.GetVertexCount() == 0 || sourceStaticMesh.GetTriangleCount() == 0)
		{
			result->Status = "Failed to read OBJ";
			return false;
		}

		DynamicMesh sourceMesh;
		if (!BuildDynamicMeshFromStaticMesh(sourceStaticMesh, &sourceMesh))
		{
			result->Status = "Failed to build cut mesh";
			return false;
		}

		const Box3 bounds = sourceMesh.GetBounds();
		const float maxSize = std::max(BoundsMaxDim(bounds), 1e-3f);
		const int mode = std::max((int)MeshCuttingMode_ParallelX, std::min(params.Mode, (int)MeshCuttingMode_Count - 1));
		const int pieceCount = std::max(2, std::min(params.PieceCount, 64));
		const int piecesX = std::max(1, std::min(params.PiecesX, 64));
		const int piecesY = std::max(1, std::min(params.PiecesY, 64));
		const int piecesZ = std::max(1, std::min(params.PiecesZ, 64));
		const int seed = std::max(0, std::min(params.Seed, 999));

		std::vector<FracturePiece> dynamicPieces;
		bool built = false;
		const PlanarCutOptions cutOptions = BuildMeshCuttingOptions(seed);
		switch (mode)
		{
		case MeshCuttingMode_ParallelX:
			built = Fracture::ParallelCutX(sourceMesh, dynamicPieces, pieceCount, cutOptions);
			break;
		case MeshCuttingMode_ParallelY:
			built = Fracture::ParallelCutY(sourceMesh, dynamicPieces, pieceCount, cutOptions);
			break;
		case MeshCuttingMode_ParallelZ:
			built = Fracture::ParallelCutZ(sourceMesh, dynamicPieces, pieceCount, cutOptions);
			break;
		case MeshCuttingMode_VoronoiFracture2D:
			built = Fracture::VoronoiFracture2D(sourceMesh, Vector3::Zero(), dynamicPieces, pieceCount, seed, cutOptions);
			break;
		case MeshCuttingMode_VoronoiFracture3D:
			built = Fracture::VoronoiFracture3D(sourceMesh, dynamicPieces, pieceCount, seed, cutOptions);
			break;
		case MeshCuttingMode_Cluster:
			built = Fracture::ClusterVoronoiFracture(sourceMesh, dynamicPieces, pieceCount, seed, cutOptions);
			break;
		case MeshCuttingMode_Voxel2D:
			built = Fracture::Voxel2D(sourceMesh, Vector3::Zero(), dynamicPieces, piecesX, piecesY, pieceCount, cutOptions);
			break;
		case MeshCuttingMode_Voxel3D:
			built = Fracture::Voxel3D(sourceMesh, dynamicPieces, piecesX, piecesY, piecesZ, pieceCount, cutOptions);
			break;
		default:
			break;
		}

		if (!built || dynamicPieces.empty())
		{
			result->Status = "Cut failed";
			return false;
		}

		result->SourceBounds = bounds;
		result->MaxSeparation = maxSize * 1.15f;
		result->Pieces.reserve(dynamicPieces.size());
		for (const FracturePiece& dynamicPiece : dynamicPieces)
		{
			MeshCuttingPiece piece;
			piece.Center = dynamicPiece.Center;
			piece.Direction = dynamicPiece.Direction;
			if (!BuildLocalStaticMesh(dynamicPiece.Mesh, piece.Center, &piece.Mesh))
			{
				continue;
			}
			result->Pieces.push_back(std::move(piece));
			RefreshStaticMeshPointers(&result->Pieces.back().Mesh);
		}

		if (result->Pieces.empty())
		{
			result->Status = "No renderable pieces";
			return false;
		}

		std::ostringstream status;
		status << "Pieces: " << result->Pieces.size();
		result->Status = status.str();
		return true;
	}
}

#include "MeshCuttingPanel.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/Geometry/ConvexDecomposition.h"
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

		int ClampPanelInt(int value, int minValue, int maxValue)
		{
			return std::max(minValue, std::min(value, maxValue));
		}

		int EstimateVHACDVoxelResolution(const Box3& bounds, uint32_t triangleCount, int pieceCount)
		{
			const Vector3 size = bounds.GetSize();
			const float maxDim = std::max(BoundsMaxDim(bounds), 1e-3f);
			int maxAxisCells = ClampPanelInt(10 + pieceCount / 2, 12, 22);
			if (triangleCount > 10000)
			{
				maxAxisCells = std::max(12, maxAxisCells - 3);
			}
			else if (triangleCount < 1000)
			{
				maxAxisCells = std::min(22, maxAxisCells + 2);
			}
			if (maxDim > 1.0f)
			{
				maxAxisCells = std::min(22, maxAxisCells + ClampPanelInt((int)(std::log(maxDim) / std::log(2.0f)), 0, 2));
			}

			const float voxelSize = std::max(maxDim / (float)maxAxisCells, 1e-4f);
			const int cellsX = std::max(1, (int)std::ceil(std::max(size.x, voxelSize) / voxelSize));
			const int cellsY = std::max(1, (int)std::ceil(std::max(size.y, voxelSize) / voxelSize));
			const int cellsZ = std::max(1, (int)std::ceil(std::max(size.z, voxelSize) / voxelSize));
			return ClampPanelInt(cellsX * cellsY * cellsZ, 2000, 10000);
		}

		ConvexDecomposition::VHACDParameters BuildVHACDParametersForDemo(const StaticMesh& sourceMesh, const Box3& bounds, int pieceCount)
		{
			ConvexDecomposition::VHACDParameters parameters;
			parameters.MaxPieceCount = pieceCount;
			parameters.VoxelResolution = EstimateVHACDVoxelResolution(bounds, sourceMesh.GetTriangleCount(), pieceCount);
			parameters.MinimumVolumePercentErrorAllowed = sourceMesh.GetTriangleCount() > 10000 ? 6.0 : 4.0;
			parameters.MaxRecursionDepth = ClampPanelInt(4 + pieceCount / 8, 4, 7);
			parameters.MaxVerticesPerHull = 48;
			parameters.ShrinkWrap = false;
			return parameters;
		}

		ConvexDecomposition::COACDParameters BuildCOACDParametersForDemo(const StaticMesh& sourceMesh, int pieceCount)
		{
			ConvexDecomposition::COACDParameters parameters;
			parameters.MaxPieceCount = pieceCount;
			parameters.Threshold = sourceMesh.GetTriangleCount() > 10000 ? 0.10 : 0.08;
			parameters.PreprocessResolution = 30;
			parameters.SampleResolution = ClampPanelInt(450 + pieceCount * 35 + (int)sourceMesh.GetTriangleCount() / 25, 500, 1400);
			parameters.MctsNodes = ClampPanelInt(8 + pieceCount / 8, 8, 18);
			parameters.MctsIterations = ClampPanelInt(45 + pieceCount * 3, 50, 120);
			parameters.MctsMaxDepth = 2;
			parameters.Merge = true;
			parameters.Decimate = false;
			parameters.MaxVerticesPerHull = 48;
			parameters.Extrude = false;
			parameters.ExtrudeMargin = 0.01;
			parameters.Seed = 0;
			return parameters;
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

		bool BuildLocalStaticMesh(const ConvexMesh& convexMesh, const Vector3& center, StaticMesh* mesh)
		{
			if (mesh == nullptr || convexMesh.GetNumVertices() == 0 || convexMesh.GetNumFaces() == 0)
			{
				return false;
			}

			std::vector<Vector3> vertices;
			std::vector<uint16_t> indices;
			std::vector<Vector3> normals;
			const_cast<ConvexMesh&>(convexMesh).GetMesh(vertices, indices, normals);
			if (vertices.empty() || indices.size() < 3)
			{
				return false;
			}

			mesh->Clear();
			mesh->Flags = 0;
			for (const Vector3& vertex : vertices)
			{
				mesh->AddVertex(vertex - center);
			}

			for (size_t index = 0; index + 2 < indices.size(); index += 3)
			{
				mesh->AddTriangle(indices[index], indices[index + 1], indices[index + 2]);
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

		Vector3 BuildConvexPieceDirection(const Vector3& pieceCenter, const Box3& sourceBounds, int pieceIndex)
		{
			Vector3 direction = pieceCenter - sourceBounds.GetCenter();
			if (direction.SquareLength() <= 1e-8f)
			{
				const float angle = (float)pieceIndex * 2.39996323f;
				direction = Vector3(cosf(angle), 0.35f, sinf(angle));
			}
			direction.SafeNormalize();
			return direction;
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

		const int mode = std::max((int)MeshCuttingMode_ParallelX, std::min(params.Mode, (int)MeshCuttingMode_Count - 1));
		const int pieceCount = std::max(2, std::min(params.PieceCount, 64));
		const int piecesX = std::max(1, std::min(params.PiecesX, 64));
		const int piecesY = std::max(1, std::min(params.PiecesY, 64));
		const int piecesZ = std::max(1, std::min(params.PiecesZ, 64));
		const int seed = std::max(0, std::min(params.Seed, 999));

		sourceStaticMesh.CalculateBoundingBox();
		const Box3 staticBounds = sourceStaticMesh.BoundingVolume;
		const float staticMaxSize = std::max(BoundsMaxDim(staticBounds), 1e-3f);

		if (mode == MeshCuttingMode_VHACD || mode == MeshCuttingMode_COACD)
		{
			std::vector<ConvexMesh> convexMeshes;
			ConvexDecomposition::VHACDParameters vhacdParameters;
			ConvexDecomposition::COACDParameters coacdParameters;
			bool built = false;
			if (mode == MeshCuttingMode_VHACD)
			{
				vhacdParameters = BuildVHACDParametersForDemo(sourceStaticMesh, staticBounds, pieceCount);
				built = ConvexDecomposition::DecomposeVHACD(sourceStaticMesh, vhacdParameters, convexMeshes);
			}
			else
			{
				coacdParameters = BuildCOACDParametersForDemo(sourceStaticMesh, pieceCount);
				built = ConvexDecomposition::DecomposeCOACD(sourceStaticMesh, coacdParameters, convexMeshes);
			}
			if (!built || convexMeshes.empty())
			{
				result->Status = "Convex decomposition failed";
				return false;
			}

			result->SourceBounds = staticBounds;
			result->MaxSeparation = staticMaxSize * 1.15f;
			result->Pieces.reserve(convexMeshes.size());
			for (size_t pieceIndex = 0; pieceIndex < convexMeshes.size(); ++pieceIndex)
			{
				const ConvexMesh& convexMesh = convexMeshes[pieceIndex];
				MeshCuttingPiece piece;
				piece.Center = convexMesh.Bounds.GetCenter();
				piece.Direction = BuildConvexPieceDirection(piece.Center, staticBounds, (int)pieceIndex);
				if (!BuildLocalStaticMesh(convexMesh, piece.Center, &piece.Mesh))
				{
					continue;
				}
				result->Pieces.push_back(std::move(piece));
				RefreshStaticMeshPointers(&result->Pieces.back().Mesh);
			}

			if (result->Pieces.empty())
			{
				result->Status = "No renderable convex pieces";
				return false;
			}

			std::ostringstream status;
			if (mode == MeshCuttingMode_VHACD)
			{
				status << "VHACD pieces: " << result->Pieces.size()
					<< " voxels: " << vhacdParameters.VoxelResolution
					<< " depth: " << vhacdParameters.MaxRecursionDepth;
			}
			else
			{
				status << "COACD pieces: " << result->Pieces.size()
					<< " samples: " << coacdParameters.SampleResolution
					<< " iter: " << coacdParameters.MctsIterations;
			}
			result->Status = status.str();
			return true;
		}

		DynamicMesh sourceMesh;
		if (!BuildDynamicMeshFromStaticMesh(sourceStaticMesh, &sourceMesh))
		{
			result->Status = "Failed to build cut mesh";
			return false;
		}

		const Box3 bounds = sourceMesh.GetBounds();
		const float maxSize = std::max(BoundsMaxDim(bounds), 1e-3f);

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

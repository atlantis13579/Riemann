#include "MeshCuttingPanel.h"

#include <algorithm>
#include <math.h>
#include <random>
#include <sstream>
#include <utility>

#include "../Src/Destruction/Fracture.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Geometry/MeshCut.h"

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

		float AxisValue(const Vector3& value, int axis)
		{
			return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
		}

		void SetAxisValue(Vector3* value, int axis, float axisValue)
		{
			if (axis == 0)
			{
				value->x = axisValue;
			}
			else if (axis == 1)
			{
				value->y = axisValue;
			}
			else
			{
				value->z = axisValue;
			}
		}

		Vector3 AxisVector(int axis)
		{
			return axis == 0 ? Vector3(1.0f, 0.0f, 0.0f) : (axis == 1 ? Vector3(0.0f, 1.0f, 0.0f) : Vector3(0.0f, 0.0f, 1.0f));
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

		struct DynamicCutPiece
		{
			DynamicMesh Mesh;
			Vector3 Center = Vector3::Zero();
			Vector3 Direction = Vector3::Zero();
		};

		bool BuildParallelCutPieces(const DynamicMesh& sourceMesh, int axis, int pieceCount, float groutScale, std::vector<DynamicCutPiece>* pieces)
		{
			if (pieces == nullptr)
			{
				return false;
			}
			pieces->clear();

			const Box3 bounds = sourceMesh.GetBounds();
			const Vector3 center = bounds.GetCenter();
			const float maxSize = std::max(BoundsMaxDim(bounds), 1e-3f);
			const float minAxis = AxisValue(bounds.Min, axis);
			const float maxAxis = AxisValue(bounds.Max, axis);
			const float axisSize = std::max(maxAxis - minAxis, 1e-3f);
			const int count = std::max(2, std::min(pieceCount, 64));
			const float padding = std::max(maxSize * 0.01f, 1e-4f);

			std::vector<Box3> cells;
			cells.reserve(count);
			for (int i = 0; i < count; ++i)
			{
				Vector3 cellMin = bounds.Min - Vector3(padding, padding, padding);
				Vector3 cellMax = bounds.Max + Vector3(padding, padding, padding);
				const float t0 = static_cast<float>(i) / static_cast<float>(count);
				const float t1 = static_cast<float>(i + 1) / static_cast<float>(count);
				SetAxisValue(&cellMin, axis, minAxis + axisSize * t0 - (i == 0 ? padding : 0.0f));
				SetAxisValue(&cellMax, axis, minAxis + axisSize * t1 + (i == count - 1 ? padding : 0.0f));
				cells.emplace_back(cellMin, cellMax);
			}

			PlanarCells planarCells(cells);
			PlanarCutOptions options;
			options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
			options.BoundsPaddingScale = 0.08f;
			options.Grout = std::max(0.0f, groutScale) * maxSize;
			options.WeldSharedEdges = true;
			options.MinTriangleCount = 4;

			std::vector<PlanarCutPiece> cutPieces;
			if (!MeshCut::CutWithPlanarCells(sourceMesh, planarCells, cutPieces, options))
			{
				return false;
			}

			const Vector3 axisDir = AxisVector(axis);
			for (const PlanarCutPiece& piece : cutPieces)
			{
				if (CountValidTriangles(piece.Mesh) == 0)
				{
					continue;
				}

				const float relative = (AxisValue(piece.Center, axis) - AxisValue(center, axis)) / std::max(axisSize * 0.5f, 1e-3f);
				DynamicCutPiece data;
				data.Mesh = piece.Mesh;
				data.Center = piece.Center;
				data.Direction = axisDir * std::max(-1.0f, std::min(1.0f, relative));
				pieces->push_back(data);
			}

			return !pieces->empty();
		}

		bool BuildVoronoiCutPieces(const DynamicMesh& sourceMesh, int pieceCount, int seed, float groutScale, std::vector<DynamicCutPiece>* pieces)
		{
			if (pieces == nullptr)
			{
				return false;
			}
			pieces->clear();

			const Box3 bounds = sourceMesh.GetBounds();
			const Vector3 center = bounds.GetCenter();
			const Vector3 extent = bounds.GetExtent();
			const float maxSize = std::max(BoundsMaxDim(bounds), 1e-3f);
			const int count = std::max(2, std::min(pieceCount, 64));

			std::mt19937 rng(static_cast<unsigned int>(seed));
			std::uniform_real_distribution<float> random01(0.0f, 1.0f);
			std::vector<Vector3> sites;
			sites.reserve(count);
			for (int i = 0; i < count; ++i)
			{
				const float rx = random01(rng) * 2.0f - 1.0f;
				const float ry = random01(rng) * 2.0f - 1.0f;
				const float rz = random01(rng) * 2.0f - 1.0f;
				sites.push_back(center + Vector3(rx * extent.x, ry * extent.y, rz * extent.z) * 0.78f);
			}

			FractureOptions options;
			options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
			options.BoundsPaddingScale = 0.20f;
			options.Grout = std::max(0.0f, groutScale) * maxSize;
			options.WeldSharedEdges = true;
			options.MinTriangleCount = 4;

			std::vector<FracturePiece> fracturePieces;
			if (!Fracture::VoronoiFracture(sourceMesh, sites, fracturePieces, options))
			{
				return false;
			}

			for (size_t i = 0; i < fracturePieces.size(); ++i)
			{
				const FracturePiece& piece = fracturePieces[i];
				if (CountValidTriangles(piece.Mesh) == 0)
				{
					continue;
				}

				Vector3 direction = piece.Center - center;
				if (direction.SafeNormalize() == 0.0f)
				{
					const float angle = static_cast<float>(i) * 2.39996323f;
					direction = Vector3(cosf(angle), 0.25f * ((i & 1) ? 1.0f : -1.0f), sinf(angle)).SafeUnit();
				}

				DynamicCutPiece data;
				data.Mesh = piece.Mesh;
				data.Center = piece.Center;
				data.Direction = direction;
				pieces->push_back(data);
			}

			return !pieces->empty();
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
		const int mode = std::max((int)MeshCuttingMode_ParallelX, std::min(params.Mode, (int)MeshCuttingMode_VoronoiFracture));
		const int pieceCount = std::max(2, std::min(params.PieceCount, 64));
		const int seed = std::max(0, std::min(params.Seed, 999));
		const float grout = std::max(0.0f, std::min(params.Grout, 0.08f));

		std::vector<DynamicCutPiece> dynamicPieces;
		const bool built = mode == MeshCuttingMode_ParallelX
			? BuildParallelCutPieces(sourceMesh, 0, pieceCount, grout, &dynamicPieces)
			: (mode == MeshCuttingMode_ParallelY
				? BuildParallelCutPieces(sourceMesh, 1, pieceCount, grout, &dynamicPieces)
				: (mode == MeshCuttingMode_ParallelZ
					? BuildParallelCutPieces(sourceMesh, 2, pieceCount, grout, &dynamicPieces)
					: BuildVoronoiCutPieces(sourceMesh, pieceCount, seed, grout, &dynamicPieces)));

		if (!built || dynamicPieces.empty())
		{
			result->Status = "Cut failed";
			return false;
		}

		result->SourceBounds = bounds;
		result->MaxSeparation = maxSize * 1.15f;
		result->Pieces.reserve(dynamicPieces.size());
		for (const DynamicCutPiece& dynamicPiece : dynamicPieces)
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

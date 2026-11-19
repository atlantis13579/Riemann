#include "MeshCuttingPanel.h"
#include "MeshSimplificationPanel.h"

#include <algorithm>
#include <cmath>
#include <numeric>
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

		bool BuildIslandMesh(const DynamicMesh& source, const std::vector<int>& triangles, MeshCuttingPiece& piece)
		{
			auto& mesh = piece.Mesh;
			std::vector<int> remap(source.GetVertexCount(), -1);
			bool closed = true;
			for (int id : triangles)
			{
				const Index3 tri = source.GetTriangle(id);
				for (int corner = 0; corner < 3; ++corner)
				{
					if (source.IsBoundaryEdge(source.GetTriEdges(id, corner))) closed = false;
					const int vertex = tri[corner];
					if (remap[vertex] >= 0) continue;
					remap[vertex] = (int)mesh.GetVertexCount();
					mesh.AddVertex(source.GetVertex(vertex));
				}
				mesh.AddTriangle(remap[tri.a], remap[tri.b], remap[tri.c]);
			}
			mesh.CalculateBoundingBox();
			piece.Center = mesh.BoundingVolume.GetCenter();
			for (uint32_t v = 0; v < mesh.NumVertices; ++v) mesh.Vertices[v] -= piece.Center;
			mesh.CalculateBoundingBox();
			mesh.CalculateWeightAverageNormals();
			return closed;
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

		void BuildSeparatedLayout(MeshCuttingResult& result, const MeshCuttingParams& params)
		{
			auto& pieces = result.Pieces;
			const int count = (int)pieces.size();
			const Vector3 right = params.ViewRight, up = params.ViewUp;
			std::vector<float> widths(count), heights(count);
			float totalWidth = 0, totalHeight = 0;
			for (int i = 0; i < count; ++i)
			{
				const Vector3 size = pieces[i].Mesh.BoundingVolume.GetSize();
				widths[i] = right.Abs().Dot(size);
				heights[i] = up.Abs().Dot(size);
				totalWidth += widths[i];
				totalHeight += heights[i];
			}
			const float gap = std::max(BoundsMaxDim(result.SourceBounds) * 0.04f, 0.28f * std::max(totalWidth, totalHeight) / count);
			const float aspect = std::max(params.ViewAspect, 0.1f);
			const int columns = ClampPanelInt((int)std::round(std::sqrt(count * aspect * (totalHeight + count * gap) / (totalWidth + count * gap))), 1, count);
			const int rows = (count + columns - 1) / columns;
			std::vector<int> order(count);
			std::iota(order.begin(), order.end(), 0);
			// Keep nearby pieces near each other in the inspection view.
			std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return pieces[a].Center.Dot(up) > pieces[b].Center.Dot(up); });
			for (int start = 0; start < count; start += columns)
				std::stable_sort(order.begin() + start, order.begin() + std::min(start + columns, count),
					[&](int a, int b) { return pieces[a].Center.Dot(right) < pieces[b].Center.Dot(right); });
			std::vector<float> columnWidths(columns, 0), rowHeights(rows, 0);
			for (int i = 0; i < count; ++i)
			{
				columnWidths[i % columns] = std::max(columnWidths[i % columns], widths[order[i]]);
				rowHeights[i / columns] = std::max(rowHeights[i / columns], heights[order[i]]);
			}
			const float width = std::accumulate(columnWidths.begin(), columnWidths.end(), gap * (columns - 1));
			const float height = std::accumulate(rowHeights.begin(), rowHeights.end(), gap * (rows - 1));
			const Vector3 center = result.SourceBounds.GetCenter();
			float y = height * 0.5f;
			for (int row = 0; row < rows; ++row)
			{
				float x = -width * 0.5f;
				for (int col = 0; col < columns && row * columns + col < count; ++col)
				{
					MeshCuttingPiece& piece = pieces[order[row * columns + col]];
					const Vector3 target = center + right * (x + columnWidths[col] * 0.5f) + up * (y - rowHeights[row] * 0.5f);
					piece.SeparationOffset = target - piece.Center;
					x += columnWidths[col] + gap;
				}
				y -= rowHeights[row] + gap;
			}
		}

		PlanarCutOptions BuildMeshCuttingOptions(int seed)
		{
			PlanarCutOptions options;
			options.SnapTolerance = 1e-5f;
			options.BoundsPaddingScale = 0.10f;
			options.RandomSeed = seed;
			options.WeldSharedEdges = true;
			options.ClipConvexCells = true;
			options.MinTriangleCount = 1;
			return options;
		}

		bool LoadCuttingStaticMesh(const std::string& meshPath, StaticMesh* mesh,
			std::string* status)
		{
			MeshSimplificationSource source;
			if (!LoadMeshSimplificationSource(meshPath, &source))
			{
				if (status) *status = source.Status;
				return false;
			}
			*mesh = std::move(source.Mesh);
			RefreshStaticMeshPointers(mesh);
			return true;
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
		source->Status.clear();

		StaticMesh sourceMesh;
		std::string loadStatus;
		if (!LoadCuttingStaticMesh(objPath, &sourceMesh, &loadStatus))
		{
			source->Status = loadStatus.empty() ? "Failed to read mesh" : loadStatus;
			return false;
		}

		sourceMesh.CalculateBoundingBox();
		sourceMesh.CalculateWeightAverageNormals();

		source->Bounds = sourceMesh.BoundingVolume;
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
		result->SourceBounds = Box3::Empty();

		StaticMesh sourceStaticMesh;
		std::string loadStatus;
		if (!LoadCuttingStaticMesh(params.ObjPath, &sourceStaticMesh, &loadStatus))
		{
			result->Status = loadStatus.empty() ? "Failed to read mesh" : loadStatus;
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

		if (mode == MeshCuttingMode_VHACD)
		{
			std::vector<ConvexMesh> convexMeshes;
			ConvexDecomposition::VHACDParameters vhacdParameters;
			vhacdParameters = BuildVHACDParametersForDemo(sourceStaticMesh, staticBounds, pieceCount);
			const bool built = ConvexDecomposition::DecomposeVHACD(sourceStaticMesh, vhacdParameters, convexMeshes);
			if (!built || convexMeshes.empty())
			{
				result->Status = "Convex decomposition failed";
				return false;
			}

			result->SourceBounds = staticBounds;
			result->Pieces.reserve(convexMeshes.size());
			for (size_t pieceIndex = 0; pieceIndex < convexMeshes.size(); ++pieceIndex)
			{
				const ConvexMesh& convexMesh = convexMeshes[pieceIndex];
				MeshCuttingPiece piece;
				piece.Center = convexMesh.Bounds.GetCenter();
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

			BuildSeparatedLayout(*result, params);
			std::ostringstream status;
			status << "VHACD pieces: " << result->Pieces.size()
				<< " voxels: " << vhacdParameters.VoxelResolution
				<< " depth: " << vhacdParameters.MaxRecursionDepth;
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

		std::vector<FracturePiece> dynamicPieces;
		bool built = false;
		PlanarCutOptions cutOptions = BuildMeshCuttingOptions(seed);
		cutOptions.CapCutFaces = params.CapCutFaces;
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
		result->Pieces.reserve(dynamicPieces.size());
		const bool parallelCut = mode >= MeshCuttingMode_ParallelX && mode <= MeshCuttingMode_ParallelZ;
		const float sliceGap = 10.0f * (BoundsMaxDim(bounds) / pieceCount);
		int openPieces = 0;
		for (FracturePiece& dynamicPiece : dynamicPieces)
		{
			std::vector<std::vector<int>> islands;
			dynamicPiece.Mesh.BuildIslands(islands);
			for (const auto& triangles : islands)
			{
				MeshCuttingPiece piece;
				if (!BuildIslandMesh(dynamicPiece.Mesh, triangles, piece)) ++openPieces;
				if (parallelCut)
				{
					// Islands in the same slab travel together along its normal.
					const int axis = mode - MeshCuttingMode_ParallelX;
					piece.SeparationOffset[axis] = (dynamicPiece.CellIndex - 0.5f * (pieceCount - 1)) * sliceGap;
				}
				result->Pieces.push_back(std::move(piece));
				RefreshStaticMeshPointers(&result->Pieces.back().Mesh);
			}
		}

		if (result->Pieces.empty())
		{
			result->Status = "No renderable pieces";
			return false;
		}

		if (!parallelCut) BuildSeparatedLayout(*result, params);
		std::ostringstream status;
		status << "Pieces: " << result->Pieces.size();
		if (params.CapCutFaces && openPieces) status << " (" << openPieces << " open)";
		result->Status = status.str();
		return true;
	}
}

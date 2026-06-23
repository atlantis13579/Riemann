#pragma once

#include <vector>

#include "DynamicMesh.h"
#include "../Maths/Box3.h"
#include "../Maths/Index2.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct PlanarCutOptions;

	struct PlanarCutPlane
	{
		Vector3 Normal = Vector3::UnitY();
		float W = 0.0f;

		PlanarCutPlane() = default;

		PlanarCutPlane(const Vector3& InNormal, float InW)
		{
			Normal = InNormal.SafeUnit();
			W = InW;
		}

		PlanarCutPlane(const Vector3& InNormal, const Vector3& Origin)
		{
			Normal = InNormal.SafeUnit();
			W = Normal.Dot(Origin);
		}

		Vector3 GetOrigin() const { return Normal * W; }
		float SignedDistance(const Vector3& Point) const { return Normal.Dot(Point) - W; }
	};

	struct PlanarCellNoiseSettings
	{
		float Amplitude = 0.0f;
		float Frequency = 0.1f;
		int Octaves = 4;
		float PointSpacing = 1.0f;
		float Lacunarity = 2.0f;
		float Persistence = 0.5f;
	};

	struct PlanarCellSurfaceSettings
	{
		int GlobalMaterialID = 0;
		bool GlobalVisibility = true;
		float GlobalUVScale = 1.0f;
		bool EnableNoise = false;
		PlanarCellNoiseSettings Noise;
	};

	struct PlanarCells
	{
		int NumCells = 0;
		bool AssumeConvexCells = false;

		std::vector<PlanarCutPlane> Planes;
		std::vector<Index2> PlaneCells;
		std::vector<std::vector<int>> PlaneBoundaries;
		std::vector<Vector3> PlaneBoundaryVertices;
		PlanarCellSurfaceSettings SurfaceSettings;

		PlanarCells() = default;
		explicit PlanarCells(const PlanarCutPlane& Plane);
		PlanarCells(const std::vector<Vector3>& Sites, const Box3& Bounds, float Eps = 1e-4f);
		explicit PlanarCells(const std::vector<Box3>& Boxes, bool ResolveAdjacencies = false);

		bool IsInfinitePlane() const;
		bool HasValidPlaneBoundaryOrientations(float Tolerance = 1e-4f) const;
		void AddPlane(const PlanarCutPlane& Plane, int NegativeCell, int PositiveCell);
		int AddPlane(const PlanarCutPlane& Plane, int NegativeCell, int PositiveCell, const std::vector<int>& PlaneBoundary);
		void DiscardCells(const std::vector<bool>& KeepCell, bool KeepNeighbors);
	};

	struct PlanarCellMeshes
	{
		std::vector<DynamicMesh> Meshes;
		int OutsideCellIndex = -1;

		void Clear();
		bool Build(const PlanarCells& Cells, const Box3& DomainBounds, const PlanarCutOptions& Options);
	};

	struct PlanarCutPiece
	{
		DynamicMesh Mesh;
		int CellIndex = -1;
		Vector3 Center = Vector3::Zero();
		std::vector<int> NeighborPieceIndices;
	};

	struct PlanarCutOptions
	{
		float SnapTolerance = 1e-5f;
		float BoundsPaddingScale = 0.25f;
		float Grout = 0.0f;
		int RandomSeed = 0;
		bool WeldSharedEdges = true;
		bool SimplifyAlongCut = false;
		bool IncludeOutsideCell = false;
		int MinTriangleCount = 1;
	};

	class MeshCut
	{
	public:
		static bool Cut(
			const DynamicMesh& SourceMesh,
			const Vector3& PlaneOrigin,
			const Vector3& PlaneNormal,
			DynamicMesh* NegativeSide,
			DynamicMesh* PositiveSide,
			const PlanarCutOptions& Options = PlanarCutOptions());

		static bool Cut(
			const DynamicMesh& SourceMesh,
			const Vector3& PlaneOrigin,
			const Vector3& PlaneNormal,
			std::vector<DynamicMesh>& Pieces,
			const PlanarCutOptions& Options = PlanarCutOptions());

		static bool BuildHalfSpaceMesh(
			const DynamicMesh& SourceMesh,
			const Vector3& PlaneOrigin,
			const Vector3& PlaneNormal,
			bool bPositiveSide,
			const PlanarCutOptions& Options,
			DynamicMesh& HalfSpaceMesh);

		static bool BuildCellMeshes(
			const PlanarCells& Cells,
			const Box3& DomainBounds,
			std::vector<DynamicMesh>& CellMeshes,
			const PlanarCutOptions& Options = PlanarCutOptions());

		static bool CutWithPlanarCells(
			const DynamicMesh& SourceMesh,
			const PlanarCells& Cells,
			std::vector<PlanarCutPiece>& Pieces,
			const PlanarCutOptions& Options = PlanarCutOptions());

		static bool CutWithPlanarCells(
			const DynamicMesh& SourceMesh,
			const PlanarCells& Cells,
			std::vector<DynamicMesh>& Pieces,
			const PlanarCutOptions& Options = PlanarCutOptions());

		static bool CreateCuttingSurfacePreview(
			const PlanarCells& Cells,
			const Box3& DomainBounds,
			DynamicMesh& PreviewMesh,
			const PlanarCutOptions& Options = PlanarCutOptions());
	};

}	// namespace Riemann

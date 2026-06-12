#pragma once

#include <cstdint>
#include <vector>

#include "MeshValidator.h"
#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class DynamicMesh;
	class StaticMesh;

	struct MeshTopologyRepairOptions
	{
		float WeldTolerance = 1e-5f;
		float AreaEpsilon = 1e-8f;
		bool RemoveDuplicateTriangles = true;
		bool FixOrientation = true;
		bool RemoveExtraNonManifoldEdgeTriangles = true;
		bool SplitNonManifoldVertices = true;
		bool FillHoles = true;
		int MaxHoleEdges = 64;
		bool KeepLargestComponent = false;
		int MinComponentTriangleCount = 0;
		double MinComponentAbsVolume = 0.0;
		bool FlipToPositiveVolume = true;
		bool ValidateSelfIntersections = false;
	};

	struct MeshTopologyRepairReport
	{
		MeshValidationReport Before;
		MeshValidationReport After;

		int RemovedInvalidTriangles = 0;
		int RemovedDegenerateTriangles = 0;
		int RemovedDuplicateTriangles = 0;
		int RemovedNonManifoldTriangles = 0;
		int SplitVertices = 0;
		int FilledHoleCount = 0;
		int AddedFillTriangles = 0;
		int RemovedComponentTriangles = 0;
		int FlippedTriangles = 0;

		bool IsRepairedSolid() const
		{
			return After.IsSolidCandidate();
		}
	};

	class MeshRepair
	{
	public:
		static bool RepairTopology(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions& Options = MeshTopologyRepairOptions(),
			MeshTopologyRepairReport* Report = nullptr);

		static bool RepairTopology(const std::vector<Vector3>& Vertices,
			const std::vector<Index3>& Triangles,
			DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions& Options = MeshTopologyRepairOptions(),
			MeshTopologyRepairReport* Report = nullptr);

		static bool RepairTopology(const StaticMesh& Mesh,
			DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions& Options = MeshTopologyRepairOptions(),
			MeshTopologyRepairReport* Report = nullptr);

		static bool RepairTopology(const DynamicMesh& Mesh,
			DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions& Options = MeshTopologyRepairOptions(),
			MeshTopologyRepairReport* Report = nullptr);
	};
}

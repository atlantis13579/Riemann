#pragma once

#include <cstdint>
#include <vector>

#include "MeshRepair.h"
#include "../Maths/Box3.h"
#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class DynamicMesh;
	class StaticMesh;

	struct SignedDistanceFieldBuildOptions
	{
		float VoxelSize = 0.0f;
		int Resolution = 128;
		int MaxResolution = 256;
		int PaddingCells = 3;
		float WindingThreshold = 0.5f;
		float FastWindingBeta = 2.0f;
		int LeafTriangleCount = 16;
		bool UseFastWinding = true;
		bool UseAbsoluteWinding = true;
		bool RepairExtractedMesh = true;
		MeshTopologyRepairOptions RepairOptions;
	};

	struct SignedDistanceFieldBuildReport
	{
		Box3 InputBounds = Box3::Empty();
		Box3 GridBounds = Box3::Empty();
		float CellSize = 0.0f;
		int CellsX = 0;
		int CellsY = 0;
		int CellsZ = 0;
		uint64_t SampleCount = 0;
		float MinValue = 0.0f;
		float MaxValue = 0.0f;
		uint32_t ExtractedVertexCount = 0;
		uint32_t ExtractedTriangleCount = 0;
		MeshTopologyRepairReport OutputRepair;
		MeshValidationReport OutputValidation;
	};

	class SignedDistanceField3
	{
	public:
		void Clear();
		bool Reset(const Box3& Bounds, int InCellsX, int InCellsY, int InCellsZ, float InCellSize);

		bool IsValid() const;
		int GetCellsX() const { return CellsX; }
		int GetCellsY() const { return CellsY; }
		int GetCellsZ() const { return CellsZ; }
		float GetCellSize() const { return CellSize; }
		const Box3& GetBounds() const { return Bounds; }
		uint64_t GetSampleCount() const { return (uint64_t)Values.size(); }

		float& At(int X, int Y, int Z);
		float At(int X, int Y, int Z) const;
		Vector3 GridPointToWorld(int X, int Y, int Z) const;

		bool ExtractIsoSurface(float IsoValue, std::vector<Vector3>& OutVertices, std::vector<Index3>& OutTriangles) const;
		bool ExtractIsoSurface(float IsoValue, DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions* RepairOptions = nullptr,
			MeshTopologyRepairReport* RepairReport = nullptr) const;

		static bool BuildFromTriangleSoup(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			SignedDistanceField3& OutField,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);

		static bool BuildFromTriangleSoup(const std::vector<Vector3>& Vertices,
			const std::vector<Index3>& Triangles,
			SignedDistanceField3& OutField,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);

	private:
		int ValueIndex(int X, int Y, int Z) const;

	private:
		Box3 Bounds = Box3::Empty();
		int CellsX = 0;
		int CellsY = 0;
		int CellsZ = 0;
		float CellSize = 0.0f;
		std::vector<float> Values;
	};

	class MeshSDFReconstructor
	{
	public:
		static bool ReconstructSolid(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			DynamicMesh& OutMesh,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);

		static bool ReconstructSolid(const std::vector<Vector3>& Vertices,
			const std::vector<Index3>& Triangles,
			DynamicMesh& OutMesh,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);

		static bool ReconstructSolid(const StaticMesh& Mesh,
			DynamicMesh& OutMesh,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);

		static bool ReconstructSolid(const DynamicMesh& Mesh,
			DynamicMesh& OutMesh,
			const SignedDistanceFieldBuildOptions& Options = SignedDistanceFieldBuildOptions(),
			SignedDistanceFieldBuildReport* Report = nullptr);
	};
}

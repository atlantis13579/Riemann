#pragma once

#include <cstdint>
#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class DynamicMesh;
	class StaticMesh;

	struct MeshSurfaceDistanceOptions
	{
		// 1 samples triangle vertices, 2 adds edge midpoints, larger values add a barycentric grid.
		int SamplesPerTriangleEdge = 2;
		int MaxSamplesPerTriangleEdge = 16;

		// If positive, per-triangle resolution is increased to approach this total sample count.
		int TargetSampleCount = 0;

		int BVHLeafTriangleCount = 16;
		float DegenerateAreaEpsilon = 1e-8f;
		bool AreaWeighted = true;
		bool IncludeTriangleCentroids = true;
	};

	struct MeshDistanceStats
	{
		bool bValid = false;
		uint64_t SampleCount = 0;
		uint32_t SourceTriangleCount = 0;
		uint32_t TargetTriangleCount = 0;

		double TotalWeight = 0.0;
		double MaxDistance = 0.0;
		double MeanDistance = 0.0;
		double RmsDistance = 0.0;
		double MeanSquaredDistance = 0.0;

		Vector3 SourcePointAtMax = Vector3::Zero();
		Vector3 TargetPointAtMax = Vector3::Zero();
		int SourceTriangleAtMax = -1;
		int TargetTriangleAtMax = -1;
	};

	struct MeshSurfaceDistanceReport
	{
		bool bValid = false;

		MeshDistanceStats AToB;
		MeshDistanceStats BToA;

		double OneSidedHausdorffAB = 0.0;
		double OneSidedHausdorffBA = 0.0;
		double HausdorffDistance = 0.0;

		// Area-weighted mean/RMS over both directed surface samples.
		double MeanBoundaryDistance = 0.0;
		double RmsBoundaryDistance = 0.0;

		// Bidirectional Chamfer-style aggregates.
		double ChamferDistance = 0.0;
		double MeanSquaredChamferDistance = 0.0;
	};

	struct MeshVolumeOptions
	{
		float AreaEpsilon = 1e-8f;
		float WeldTolerance = 0.0f;
		bool ValidateSolid = true;
		bool CheckSelfIntersections = false;
	};

	struct MeshVolumeInfo
	{
		bool bHasValidSurface = false;
		bool bIsClosed = false;
		bool bIsOrientable = false;
		bool bIsTwoManifold = false;
		bool bIsSolidCandidate = false;

		uint32_t VertexCount = 0;
		uint32_t TriangleCount = 0;
		int ValidTriangleCount = 0;
		int InvalidIndexCount = 0;
		int DegenerateTriangleCount = 0;
		int BoundaryEdgeCount = 0;
		int NonManifoldEdgeCount = 0;
		int SameDirectionEdgeCount = 0;
		int NonManifoldVertexCount = 0;
		int SelfIntersectionCount = 0;

		double SignedVolume = 0.0;
		double AbsoluteVolume = 0.0;
		double SurfaceArea = 0.0;
		Box3 Bounds = Box3::Empty();
	};

	struct MeshVolumeDistanceReport
	{
		MeshVolumeInfo A;
		MeshVolumeInfo B;

		double SignedVolumeDifference = 0.0;
		double AbsoluteVolumeDifference = 0.0;
		double RelativeAbsoluteVolumeDifference = 0.0;
	};

	struct MeshDistanceReport
	{
		MeshSurfaceDistanceReport Surface;
		MeshVolumeDistanceReport Volume;
	};

	class MeshDistance
	{
	public:
		static MeshDistanceStats ComputeOneSidedSurfaceDistance(
			const Vector3* SourceVertices, uint32_t NumSourceVertices,
			const uint32_t* SourceIndices, uint32_t NumSourceTriangles,
			const Vector3* TargetVertices, uint32_t NumTargetVertices,
			const uint32_t* TargetIndices, uint32_t NumTargetTriangles,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeSurfaceDistance(
			const Vector3* VerticesA, uint32_t NumVerticesA,
			const uint32_t* IndicesA, uint32_t NumTrianglesA,
			const Vector3* VerticesB, uint32_t NumVerticesB,
			const uint32_t* IndicesB, uint32_t NumTrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeBoundaryDistance(
			const Vector3* VerticesA, uint32_t NumVerticesA,
			const uint32_t* IndicesA, uint32_t NumTrianglesA,
			const Vector3* VerticesB, uint32_t NumVerticesB,
			const uint32_t* IndicesB, uint32_t NumTrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static double ComputeHausdorffDistance(
			const Vector3* VerticesA, uint32_t NumVerticesA,
			const uint32_t* IndicesA, uint32_t NumTrianglesA,
			const Vector3* VerticesB, uint32_t NumVerticesB,
			const uint32_t* IndicesB, uint32_t NumTrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshVolumeInfo ComputeVolume(
			const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshVolumeDistanceReport ComputeVolumeDistance(
			const Vector3* VerticesA, uint32_t NumVerticesA,
			const uint32_t* IndicesA, uint32_t NumTrianglesA,
			const Vector3* VerticesB, uint32_t NumVerticesB,
			const uint32_t* IndicesB, uint32_t NumTrianglesB,
			const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshDistanceReport ComputeAll(
			const Vector3* VerticesA, uint32_t NumVerticesA,
			const uint32_t* IndicesA, uint32_t NumTrianglesA,
			const Vector3* VerticesB, uint32_t NumVerticesB,
			const uint32_t* IndicesB, uint32_t NumTrianglesB,
			const MeshSurfaceDistanceOptions& SurfaceOptions = MeshSurfaceDistanceOptions(),
			const MeshVolumeOptions& VolumeOptions = MeshVolumeOptions());

		static MeshDistanceStats ComputeOneSidedSurfaceDistance(
			const std::vector<Vector3>& SourceVertices, const std::vector<Index3>& SourceTriangles,
			const std::vector<Vector3>& TargetVertices, const std::vector<Index3>& TargetTriangles,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeSurfaceDistance(
			const std::vector<Vector3>& VerticesA, const std::vector<Index3>& TrianglesA,
			const std::vector<Vector3>& VerticesB, const std::vector<Index3>& TrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeBoundaryDistance(
			const std::vector<Vector3>& VerticesA, const std::vector<Index3>& TrianglesA,
			const std::vector<Vector3>& VerticesB, const std::vector<Index3>& TrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static double ComputeHausdorffDistance(
			const std::vector<Vector3>& VerticesA, const std::vector<Index3>& TrianglesA,
			const std::vector<Vector3>& VerticesB, const std::vector<Index3>& TrianglesB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshVolumeInfo ComputeVolume(
			const std::vector<Vector3>& Vertices, const std::vector<Index3>& Triangles,
			const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshVolumeDistanceReport ComputeVolumeDistance(
			const std::vector<Vector3>& VerticesA, const std::vector<Index3>& TrianglesA,
			const std::vector<Vector3>& VerticesB, const std::vector<Index3>& TrianglesB,
			const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshDistanceStats ComputeOneSidedSurfaceDistance(
			const StaticMesh& SourceMesh, const StaticMesh& TargetMesh,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeSurfaceDistance(
			const StaticMesh& MeshA, const StaticMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeBoundaryDistance(
			const StaticMesh& MeshA, const StaticMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static double ComputeHausdorffDistance(
			const StaticMesh& MeshA, const StaticMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshVolumeInfo ComputeVolume(
			const StaticMesh& Mesh, const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshVolumeDistanceReport ComputeVolumeDistance(
			const StaticMesh& MeshA, const StaticMesh& MeshB,
			const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshDistanceStats ComputeOneSidedSurfaceDistance(
			const DynamicMesh& SourceMesh, const DynamicMesh& TargetMesh,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeSurfaceDistance(
			const DynamicMesh& MeshA, const DynamicMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshSurfaceDistanceReport ComputeBoundaryDistance(
			const DynamicMesh& MeshA, const DynamicMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static double ComputeHausdorffDistance(
			const DynamicMesh& MeshA, const DynamicMesh& MeshB,
			const MeshSurfaceDistanceOptions& Options = MeshSurfaceDistanceOptions());

		static MeshVolumeInfo ComputeVolume(
			const DynamicMesh& Mesh, const MeshVolumeOptions& Options = MeshVolumeOptions());

		static MeshVolumeDistanceReport ComputeVolumeDistance(
			const DynamicMesh& MeshA, const DynamicMesh& MeshB,
			const MeshVolumeOptions& Options = MeshVolumeOptions());
	};

	class HausdorffDistance
	{
	public:
		void setSamplesPerTriangleEdge(int SamplesPerEdge);
		void setTargetSampleCount(int SampleCount);

		float computeOneSided(const StaticMesh* MeshA, const StaticMesh* MeshB);
		float computeSymmetric(const StaticMesh* MeshA, const StaticMesh* MeshB);
		float computeWithKDTree(const StaticMesh* MeshA, const StaticMesh* MeshB);

	private:
		MeshSurfaceDistanceOptions Options;
	};
}

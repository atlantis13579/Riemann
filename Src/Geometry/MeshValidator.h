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

	struct MeshValidationOptions
	{
		float AreaEpsilon = 1e-8f;
		float WeldTolerance = 0.0f;
		bool CheckSelfIntersections = false;
		bool IgnoreAdjacentTriangleIntersections = true;
		uint64_t MaxSelfIntersectionTests = 2000000;
		int MaxIssueSamples = 32;
	};

	struct MeshValidationEdgeIssue
	{
		uint32_t V0 = 0;
		uint32_t V1 = 0;
		int IncidentCount = 0;
		int Triangle0 = -1;
		int Triangle1 = -1;
	};

	struct MeshValidationVertexIssue
	{
		uint32_t Vertex = 0;
		int IncidentFaceComponents = 0;
	};

	struct MeshValidationTriangleIssue
	{
		int Triangle0 = -1;
		int Triangle1 = -1;
	};

	struct MeshValidationReport
	{
		uint32_t VertexCount = 0;
		uint32_t TriangleCount = 0;
		uint32_t WeldedVertexCount = 0;

		int InvalidVertexCount = 0;
		int DuplicateVertexCount = 0;
		int InvalidIndexCount = 0;
		int TriangleWithInvalidVertexCount = 0;
		int DegenerateTriangleCount = 0;
		int DuplicateTriangleCount = 0;
		int ValidTriangleCount = 0;

		int BoundaryEdgeCount = 0;
		int NonManifoldEdgeCount = 0;
		int SameDirectionEdgeCount = 0;
		int NonManifoldVertexCount = 0;
		int ConnectedComponentCount = 0;

		int PositiveVolumeComponentCount = 0;
		int NegativeVolumeComponentCount = 0;
		int NearZeroVolumeComponentCount = 0;

		int SelfIntersectionCount = 0;
		bool SelfIntersectionTestLimitReached = false;

		double SignedVolume = 0.0;
		double AbsoluteVolume = 0.0;
		double SurfaceArea = 0.0;
		Box3 Bounds = Box3::Empty();

		std::vector<int> InvalidVertexSamples;
		std::vector<int> DuplicateVertexSamples;
		std::vector<int> InvalidIndexTriangleSamples;
		std::vector<int> DegenerateTriangleSamples;
		std::vector<int> DuplicateTriangleSamples;
		std::vector<MeshValidationEdgeIssue> BoundaryEdgeSamples;
		std::vector<MeshValidationEdgeIssue> NonManifoldEdgeSamples;
		std::vector<MeshValidationEdgeIssue> SameDirectionEdgeSamples;
		std::vector<MeshValidationVertexIssue> NonManifoldVertexSamples;
		std::vector<MeshValidationTriangleIssue> SelfIntersectionSamples;
		std::vector<double> ComponentSignedVolumes;

		bool HasValidData() const;
		bool IsClosed() const;
		bool IsOrientable() const;
		bool IsTwoManifold() const;
		bool IsSolidCandidate() const;
	};

	class MeshValidator
	{
	public:
		static MeshValidationReport Validate(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			const MeshValidationOptions& Options = MeshValidationOptions());

		static MeshValidationReport Validate(const std::vector<Vector3>& Vertices,
			const std::vector<Index3>& Triangles,
			const MeshValidationOptions& Options = MeshValidationOptions());

		static MeshValidationReport Validate(const StaticMesh& Mesh,
			const MeshValidationOptions& Options = MeshValidationOptions());

		static MeshValidationReport Validate(const DynamicMesh& Mesh,
			const MeshValidationOptions& Options = MeshValidationOptions());
	};
}


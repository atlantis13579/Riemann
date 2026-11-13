#pragma once

#include <limits>
#include <memory>
#include <vector>
#include "DynamicAABBTree.h"
#include "../Maths/Index3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	struct SurfaceMotion
	{
		int Count = 1;
		Vector3 Start[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
		Vector3 End[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
	};
	struct SurfaceCollisionSettings
	{
		double Thickness = 0;
		double DistanceTolerance = 1e-6;
		double RelativeTolerance = 1e-6;
		double AdvanceFraction = 0.9;
		int MaxIterations = 64;
		bool IsValid() const;
	};
	struct SurfaceContact
	{
		Vector3 PointA = Vector3::Zero(), PointB = Vector3::Zero();
		Vector3 WeightsA = Vector3::Zero(), WeightsB = Vector3::Zero();
		Vector3 Normal = Vector3::Zero();
		double Distance = std::numeric_limits<double>::infinity(), Time = 0;
		bool InitialOverlap = false, Conservative = false;
		int Iterations = 0;
	};
	class SurfaceCollision
	{
	public:
		static SurfaceContact Closest(const Vector3* A, int CountA, const Vector3* B, int CountB);
		static bool Sweep(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceCollisionSettings& Settings,
			SurfaceContact& Contact, const Vector3& NormalHint = Vector3::Zero());
		static bool SweepRefined(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceCollisionSettings& Settings,
			int MaxRefinements, SurfaceContact& Contact, const Vector3& NormalHint = Vector3::Zero());
	};

	struct SurfaceEdge { int A, B; };
	enum class SurfacePairType { VertexTriangle, TriangleVertex, EdgeEdge };
	struct SurfaceCandidate
	{
		SurfacePairType Type;
		int A, B;
	};
	class SurfaceCollisionMesh
	{
	public:
		SurfaceCollisionMesh(int VertexCount, const std::vector<Index3>& Triangles, const std::vector<SurfaceEdge>& Edges = {});
		SurfaceCollisionMesh(const SurfaceCollisionMesh&) = delete;
		SurfaceCollisionMesh& operator=(const SurfaceCollisionMesh&) = delete;
		void Update(const std::vector<Vector3>& Start, const std::vector<Vector3>& End, float Margin);
		void QueryTriangles(const Box3& Bounds, std::vector<int>& Triangles) const;
		void QueryEdges(const Box3& Bounds, std::vector<int>& Edges) const;
		void FindSelfCandidates(std::vector<SurfaceCandidate>& Candidates, bool ExcludeNeighbors = true) const;
		void FindCandidates(const SurfaceCollisionMesh& Other, std::vector<SurfaceCandidate>& Candidates) const;
		const std::vector<Index3>& GetTriangles() const { return m_Triangles; }
		const std::vector<SurfaceEdge>& GetEdges() const { return m_Edges; }
		const Box3& GetVertexBounds(int Vertex) const { RequireReady(); return m_VertexBounds.at(Vertex); }
		bool IsReady() const { return m_Ready; }
	private:
		struct Entry { int Index = 0, Node = -1; Box3 Bounds; };
		bool Related(int A, int B, bool IncludeNeighbors) const;
		static void Query(const DynamicAABBTree& Tree, const Box3& Bounds, std::vector<int>& Indices);
		void RequireReady() const;
		int m_VertexCount;
		bool m_Ready = false;
		std::vector<Index3> m_Triangles;
		std::vector<SurfaceEdge> m_Edges;
		std::vector<std::vector<int>> m_Neighbors;
		std::vector<Box3> m_VertexBounds;
		std::vector<Entry> m_TriangleEntries, m_EdgeEntries;
		DynamicAABBTree m_TriangleTree, m_EdgeTree;
	};

	class Geometry;
	class RigidBody;
	struct SurfaceColliderPlane { Vector3 Point, Normal; };
	struct SurfaceCollider
	{
		RigidBody* Body=nullptr;
		const Geometry* Source=nullptr;
		float Radius=0, ArcBound=0;
		double RotationRadius=0;
		std::vector<Vector3> Local, Start, End;
		std::vector<Index3> Triangles;
		std::vector<SurfaceColliderPlane> Planes;
		std::unique_ptr<SurfaceCollisionMesh> Index;
		Transform Previous, FrameEnd, StepStart, StepEnd;
		bool Initialize(Geometry* Shape);
		bool InitializeCapsule(RigidBody* Owner, float Radius, const Vector3& A, const Vector3& B);
		bool Prepare();
		void CaptureFrame();
		void Update(float StartFraction, float EndFraction, bool Continuous);
		void CommitFrame() { Previous=FrameEnd; }
		double FrameAngle() const;
	};
}

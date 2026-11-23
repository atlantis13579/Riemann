#include "SurfaceCollision.h"
#include "GeometryObject.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace Riemann
{
	/* Surface features are convex sets with one, two or three vertices: points,
	   edges and filled triangles. The same barycentric witness supports cloth
	   vertex/face and edge/edge contacts, rigid mesh faces, soft-body boundaries,
	   and rounded primitives. A sphere is a point with added thickness; a capsule
	   is a segment with added thickness. Triangle/capsule distance must include
	   segment/face intersection, not just distances from triangle vertices.

	   Closest-feature queries enumerate boundary candidates plus admissible face
	   or interior-edge candidates. Exact degeneracy reduces to edges or points.
	   Double-precision relative coordinates avoid the cancellation in the usual
	   parallel-segment determinant (a*c-b*b). We use |edgeA cross edgeB|^2 and do
	   not classify almost-parallel edges as parallel: that approximation can
	   overestimate distance and turn a conservative sweep into a missed impact.

	   Sweep assumes LINEAR vertex trajectories over a normalized interval [0,1].
	   For any barycentric pair, relative displacement is a convex combination of
	   vertex-pair displacements. Their maximum length bounds the rate at which
	   feature distance can decrease. Advancing less than (distance-thickness)/rate
	   therefore cannot step across first contact. This also handles coplanar and
	   grazing paths without relying on a sign change of the oriented volume.
	   DistanceTolerance + RelativeTolerance*featureSize is a numerical contact
	   skin. It scales with feature dimensions, not world translation or speed.

	   A normal points from B to A; barycentric weights reconstruct the witnesses
	   at Time and later distribute equal/opposite contact impulses. Coincident
	   witnesses have no unique normal. A caller's previous normal takes priority;
	   otherwise face/edge orientation and relative motion choose a deterministic
	   fallback. Initial intersecting surfaces still need an application policy for
	   which side should separate; CCD cannot recover an unknown previous ordering.

	   A finite work budget cannot prove every grazing query. On exhaustion or
	   arithmetic stagnation we return Conservative=true at the last tested safe
	   time, rather than silently report no collision. Callers must either refine
	   that interval or restrict its motion to Time; a distant witness is not an
	   ordinary penetration contact. Iterations makes this fallback observable.
	   Rigid rotation must be sampled with a conservative angular envelope or split
	   into bounded arcs by its adapter; linear endpoint chords alone are not CCD
	   for a rotating rigid body. This layer deliberately owns no bodies or solver.

	   References: Ericson, "Real-Time Collision Detection" (2005), closest-point
	   feature regions; Bridson, Fedkiw and Anderson, "Robust Treatment of Collisions,
	   Contact and Friction for Cloth Animation" (2002), vertex/face and edge/edge
	   events; conservative advancement via a Lipschitz bound on distance. */
	namespace
	{
		using DVector = Maths::TVector3<double>;
		DVector ToDouble(const Vector3& V) { return DVector(V.x, V.y, V.z); }
		Vector3 ToFloat(const DVector& V) { return Vector3((float)V.x, (float)V.y, (float)V.z); }
		double Length(const DVector& V) { return std::sqrt(V.SquareLength()); }
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		void Validate(const Vector3* Points, int Count)
		{
			if (!Points || Count < 1 || Count > 3) throw std::invalid_argument("Surface feature must contain one to three vertices");
			for (int I = 0; I < Count; ++I) if (!Finite(Points[I])) throw std::invalid_argument("Non-finite surface vertex");
		}
		double ClampUnit(double X) { return std::max(0.0, std::min(1.0, X)); }
		DVector UnitWeight(int I) { DVector W(0); W[I] = 1; return W; }
		DVector Weighted(const DVector* P, const DVector& W) { return P[0] * W.x + P[1] * W.y + P[2] * W.z; }
		struct Witness
		{
			DVector A = DVector(0), B = DVector(0), WeightsA = DVector(0), WeightsB = DVector(0);
			double SquareDistance = std::numeric_limits<double>::infinity();
		};
		void Consider(Witness& Best, const DVector& A, const DVector& B, const DVector& WA, const DVector& WB)
		{
			const double SquareDistance = (A - B).SquareLength();
			if (SquareDistance < Best.SquareDistance) { Best.A = A; Best.B = B; Best.WeightsA = WA; Best.WeightsB = WB; Best.SquareDistance = SquareDistance; }
		}
		Witness Swapped(const Witness& W) { Witness R = W; std::swap(R.A, R.B); std::swap(R.WeightsA, R.WeightsB); return R; }
		double SegmentParameter(const DVector& P, const DVector& A, const DVector& B)
		{
			const DVector Edge = B - A; const double SquareLength = Edge.SquareLength();
			return SquareLength > 0 ? ClampUnit((P - A).Dot(Edge) / SquareLength) : 0;
		}
		bool FaceWeights(const DVector& P, const DVector* Triangle, DVector& Weights)
		{
			const DVector U = Triangle[1] - Triangle[0], V = Triangle[2] - Triangle[0], N = U.Cross(V), Offset = P - Triangle[0];
			const double Denominator = N.SquareLength(); if (!(Denominator > 0)) return false;
			const double B = Offset.Cross(V).Dot(N) / Denominator, C = U.Cross(Offset).Dot(N) / Denominator;
			Weights = DVector(1 - B - C, B, C); return Weights.x >= 0 && B >= 0 && C >= 0;
		}
		Witness PointTriangle(const DVector& P, const DVector* Triangle)
		{
			Witness Best;
			for (int I = 0; I < 3; ++I)
			{
				const int J = (I + 1) % 3; const double T = SegmentParameter(P, Triangle[I], Triangle[J]);
				const DVector Weights = UnitWeight(I) * (1 - T) + UnitWeight(J) * T;
				Consider(Best, P, Weighted(Triangle, Weights), UnitWeight(0), Weights);
			}
			DVector Weights; if (FaceWeights(P, Triangle, Weights)) Consider(Best, P, Weighted(Triangle, Weights), UnitWeight(0), Weights);
			return Best;
		}
		Witness SegmentSegment(const DVector* A, const DVector* B)
		{
			Witness Best; const DVector U = A[1] - A[0], V = B[1] - B[0];
			for (int I = 0; I < 2; ++I)
			{
				const double T = SegmentParameter(A[I], B[0], B[1]);
				Consider(Best, A[I], B[0] + V * T, UnitWeight(I), DVector(1 - T, T, 0));
				const double S = SegmentParameter(B[I], A[0], A[1]);
				Consider(Best, A[0] + U * S, B[I], DVector(1 - S, S, 0), UnitWeight(I));
			}
			const DVector Cross = U.Cross(V), Offset = B[0] - A[0]; const double Denominator = Cross.SquareLength();
			if (Denominator > 0)
			{
				const double S = Offset.Cross(V).Dot(Cross) / Denominator, T = Offset.Cross(U).Dot(Cross) / Denominator;
				if (S >= 0 && S <= 1 && T >= 0 && T <= 1)
					Consider(Best, A[0] + U * S, B[0] + V * T, DVector(1 - S, S, 0), DVector(1 - T, T, 0));
			}
			return Best;
		}
		Witness SegmentTriangle(const DVector* Segment, const DVector* Triangle)
		{
			Witness Best;
			for (int I = 0; I < 2; ++I)
			{
				auto W = PointTriangle(Segment[I], Triangle); Consider(Best, W.A, W.B, UnitWeight(I), W.WeightsB);
			}
			for (int I = 0; I < 3; ++I)
			{
				const int J = (I + 1) % 3; const DVector Edge[2] = { Triangle[I], Triangle[J] }; const auto W = SegmentSegment(Segment, Edge);
				Consider(Best, W.A, W.B, W.WeightsA, UnitWeight(I) * W.WeightsB.x + UnitWeight(J) * W.WeightsB.y);
			}
			const DVector Normal = (Triangle[1] - Triangle[0]).Cross(Triangle[2] - Triangle[0]), Direction = Segment[1] - Segment[0];
			const double Denominator = Direction.Dot(Normal);
			if (Denominator != 0)
			{
				const double T = (Triangle[0] - Segment[0]).Dot(Normal) / Denominator; DVector Weights;
				const DVector Point = Segment[0] + Direction * T;
				if (T >= 0 && T <= 1 && FaceWeights(Point, Triangle, Weights)) Consider(Best, Point, Point, DVector(1 - T, T, 0), Weights);
			}
			return Best;
		}
		Witness Closest(const DVector* A, int CountA, const DVector* B, int CountB)
		{
			if (CountA > CountB) return Swapped(Closest(B, CountB, A, CountA));
			if (CountA == 1)
			{
				if (CountB == 3) return PointTriangle(A[0], B);
				Witness Best; const double T = CountB == 2 ? SegmentParameter(A[0], B[0], B[1]) : 0;
				Consider(Best, A[0], CountB == 2 ? B[0] + (B[1] - B[0]) * T : B[0], UnitWeight(0), DVector(1 - T, T, 0)); return Best;
			}
			if (CountA == 2) return CountB == 2 ? SegmentSegment(A, B) : SegmentTriangle(A, B);
			Witness Best;
			for (int I = 0; I < 3; ++I)
			{
				const int J = (I + 1) % 3; const DVector EdgeA[2] = { A[I], A[J] }, EdgeB[2] = { B[I], B[J] };
				auto W = SegmentTriangle(EdgeA, B); Consider(Best, W.A, W.B, UnitWeight(I) * W.WeightsA.x + UnitWeight(J) * W.WeightsA.y, W.WeightsB);
				W = SegmentTriangle(EdgeB, A); Consider(Best, W.B, W.A, W.WeightsB, UnitWeight(I) * W.WeightsA.x + UnitWeight(J) * W.WeightsA.y);
			}
			return Best;
		}
		SurfaceContact ContactFrom(const Witness& W)
		{
			SurfaceContact Contact; Contact.PointA = ToFloat(W.A); Contact.PointB = ToFloat(W.B);
			Contact.WeightsA = ToFloat(W.WeightsA); Contact.WeightsB = ToFloat(W.WeightsB); Contact.Distance = std::sqrt(W.SquareDistance);
			if (Contact.Distance > 0) Contact.Normal = ToFloat((W.A - W.B) / Contact.Distance); return Contact;
		}
		DVector FallbackNormal(const DVector* A, int CountA, const DVector* B, int CountB, const DVector& Relative, const Vector3& Hint)
		{
			DVector Normal = ToDouble(Hint); if (Normal.SquareLength() > 0) return Normal / Length(Normal);
			if (CountB == 3) Normal = (B[1] - B[0]).Cross(B[2] - B[0]);
			if (Normal.SquareLength() == 0 && CountA == 3) Normal = -(A[1] - A[0]).Cross(A[2] - A[0]);
			if (Normal.SquareLength() == 0 && CountA >= 2 && CountB >= 2) Normal = (A[1] - A[0]).Cross(B[1] - B[0]);
			if (Normal.SquareLength() == 0) Normal = -Relative;
			if (Normal.SquareLength() == 0)
			{
				const DVector Edge = CountA >= 2 ? A[1] - A[0] : CountB >= 2 ? B[1] - B[0] : DVector(0);
				const DVector Axis = std::abs(Edge.x) < std::abs(Edge.y) ? DVector(1, 0, 0) : DVector(0, 1, 0);
				Normal = Edge.Cross(Axis); if (Normal.SquareLength() == 0) Normal = DVector(0, 0, 1);
			}
			if (Normal.Dot(Relative) > 0) Normal = -Normal;
			return Normal / Length(Normal);
		}
	}
	bool SurfaceCollisionSettings::IsValid() const
	{
		return std::isfinite(Thickness) && Thickness >= 0 && std::isfinite(DistanceTolerance) && DistanceTolerance > 0
			&& std::isfinite(RelativeTolerance) && RelativeTolerance >= 0 && std::isfinite(AdvanceFraction)
			&& AdvanceFraction > 0 && AdvanceFraction < 1 && MaxIterations > 0;
	}
	SurfaceContact SurfaceCollision::Closest(const Vector3* A, int CountA, const Vector3* B, int CountB)
	{
		Validate(A, CountA); Validate(B, CountB); DVector DA[3] = { DVector(0), DVector(0), DVector(0) }, DB[3] = { DVector(0), DVector(0), DVector(0) };
		for (int I = 0; I < CountA; ++I) DA[I] = ToDouble(A[I]); for (int I = 0; I < CountB; ++I) DB[I] = ToDouble(B[I]);
		return ContactFrom(Riemann::Closest(DA, CountA, DB, CountB));
	}
	static bool SweepInterval(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceCollisionSettings& Settings,
		SurfaceContact& Contact, const Vector3& NormalHint, double Begin, double End)
	{
		Contact = SurfaceContact();
		DVector StartA[3] = { DVector(0), DVector(0), DVector(0) }, StartB[3] = { DVector(0), DVector(0), DVector(0) };
		DVector VelocityA[3] = { DVector(0), DVector(0), DVector(0) }, VelocityB[3] = { DVector(0), DVector(0), DVector(0) };
		for (int I = 0; I < A.Count; ++I)
		{
			const DVector Delta = ToDouble(A.End[I]) - ToDouble(A.Start[I]);
			StartA[I] = ToDouble(A.Start[I]) + Delta * Begin; VelocityA[I] = Delta * (End - Begin);
		}
		for (int I = 0; I < B.Count; ++I)
		{
			const DVector Delta = ToDouble(B.End[I]) - ToDouble(B.Start[I]);
			StartB[I] = ToDouble(B.Start[I]) + Delta * Begin; VelocityB[I] = Delta * (End - Begin);
		}
		double Speed = 0, Scale = Settings.Thickness;
		for (int I = 0; I < A.Count; ++I) for (int J = 0; J < B.Count; ++J) Speed = std::max(Speed, Length(VelocityA[I] - VelocityB[J]));
		for (int I = 0; I < A.Count; ++I) for (int J = I + 1; J < A.Count; ++J)
			Scale = std::max(Scale, std::max(Length(StartA[I] - StartA[J]), Length(StartA[I] + VelocityA[I] - StartA[J] - VelocityA[J])));
		for (int I = 0; I < B.Count; ++I) for (int J = I + 1; J < B.Count; ++J)
			Scale = std::max(Scale, std::max(Length(StartB[I] - StartB[J]), Length(StartB[I] + VelocityB[I] - StartB[J] - VelocityB[J])));
		const double Skin = Settings.DistanceTolerance + Settings.RelativeTolerance * Scale;
		double Time = 0; Vector3 LastNormal = NormalHint;
		for (int Iteration = 0; Iteration < Settings.MaxIterations; ++Iteration)
		{
			DVector PA[3] = { DVector(0), DVector(0), DVector(0) }, PB[3] = { DVector(0), DVector(0), DVector(0) };
			for (int I = 0; I < A.Count; ++I) PA[I] = StartA[I] + VelocityA[I] * Time;
			for (int I = 0; I < B.Count; ++I) PB[I] = StartB[I] + VelocityB[I] * Time;
			const auto W = Riemann::Closest(PA, A.Count, PB, B.Count); auto Current = ContactFrom(W); Current.Time = Time; Current.Iterations = Iteration + 1;
			if (Current.Distance > 0) LastNormal = Current.Normal;
			else Current.Normal = ToFloat(FallbackNormal(PA, A.Count, PB, B.Count, Weighted(VelocityA, W.WeightsA) - Weighted(VelocityB, W.WeightsB), LastNormal));
			Current.InitialOverlap = Time == 0 && Current.Distance <= Settings.Thickness;
			if (Current.Distance <= Settings.Thickness + Skin) { Contact = Current; return true; }
			const double Remaining = 1 - Time;
			// Keep half the skin inside the advance target. This absorbs roundoff in
			// the distance lower bound while leaving a finite stopping band.
			const double Gap = Current.Distance - Settings.Thickness - Skin / 2;
			if (Speed == 0 || Gap > Speed * Remaining) return false;
			const double SafeStep = Gap / Speed;
			const double Next = SafeStep >= Remaining ? 1 : Time + Settings.AdvanceFraction * SafeStep;
			if (Iteration + 1 == Settings.MaxIterations || !(Next > Time)) { Current.Conservative = true; Contact = Current; return true; }
			Time = Next;
		}
		return false;
	}
	bool SurfaceCollision::Sweep(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceCollisionSettings& Settings, SurfaceContact& Contact, const Vector3& NormalHint)
	{
		Validate(A.Start, A.Count); Validate(A.End, A.Count); Validate(B.Start, B.Count); Validate(B.End, B.Count);
		if (!Settings.IsValid() || !Finite(NormalHint)) throw std::invalid_argument("Invalid surface sweep settings");
		return SweepInterval(A, B, Settings, Contact, NormalHint, 0, 1);
	}
	bool SurfaceCollision::SweepRefined(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceCollisionSettings& Settings,
		int MaxRefinements, SurfaceContact& Contact, const Vector3& NormalHint)
	{
		/* Conservative advancement may exhaust its iterations before reaching an
		   impact, especially with fast tangential motion. A distant last-safe point
		   is not a contact. Bisect the unresolved time interval and search its left
		   child first, retaining the original motion rather than chaining rounded
		   endpoints. A false result certifies every visited interval; a budget exit
		   still reports Conservative and the earliest unresolved time. Callers must
		   not send that result to a physical contact constraint. */
		if (MaxRefinements < 0) throw std::invalid_argument("Invalid surface sweep refinement budget");
		Validate(A.Start, A.Count); Validate(A.End, A.Count); Validate(B.Start, B.Count); Validate(B.End, B.Count);
		if (!Settings.IsValid() || !Finite(NormalHint)) throw std::invalid_argument("Invalid surface sweep settings");
		struct Interval { double Start, End; };
		std::vector<Interval> Pending = { { 0, 1 } }; int Refinements = 0, Iterations = 0;
		while (!Pending.empty())
		{
			const Interval Range = Pending.back(); Pending.pop_back(); SurfaceContact Hit;
			if (!SweepInterval(A, B, Settings, Hit, NormalHint, Range.Start, Range.End)) continue;
			Iterations += Hit.Iterations;
			Hit.Time = Range.Start + (Range.End - Range.Start) * Hit.Time;
			Hit.InitialOverlap = Hit.InitialOverlap && Hit.Time == 0;
			const double Middle = (Hit.Time + Range.End) / 2;
			if (!Hit.Conservative || Refinements == MaxRefinements || !(Middle > Hit.Time && Middle < Range.End))
			{
				Contact = Hit; Contact.Iterations = Iterations; return true;
			}
			++Refinements; Pending.push_back({ Middle, Range.End }); Pending.push_back({ Hit.Time, Middle });
		}
		Contact = SurfaceContact(); Contact.Iterations = Iterations; return false;
	}

	/* Broad phase stores swept triangle and edge bounds, including the supplied
	   surface margin. Vertex queries use their complete start/end segment. Linear
	   interpolation stays inside these bounds even when a triangle collapses.
	   The existing balanced dynamic AABB tree is reused with zero EXTRA padding:
	   its default rigid-body margin would overwhelm very small cloth triangles.
	   Update refits/reinserts leaves as needed, retaining stable feature indices.

	   Self candidates include vertex/triangle and unique edge/edge pairs. Shared
	   vertices are always excluded; optional one-ring exclusion also prevents
	   elastic neighbors from treating their own material as a collision obstacle.
	   Cross-mesh queries emit vertex/triangle in BOTH directions and all edge/edge
	   pairs. A small rigid triangle or vertex can otherwise pass through a large
	   cloth face without any cloth vertex approaching the obstacle. The caller
	   selects mesh pairs, handles layer/group policy and runs the narrow queries.

	   Topology is immutable and owned. Position arrays are borrowed only for the
	   duration of Update. Sorted query results make contact order independent of
	   tree balancing history. No pointer into the caller's mesh survives a call.
	   A topology change creates a new index; candidate indices are local to that
	   index and must never be reused across topology replacement. */
	namespace
	{
		bool ValidBounds(const Box3& B) { return Finite(B.Min) && Finite(B.Max) && B.Min.x <= B.Max.x && B.Min.y <= B.Max.y && B.Min.z <= B.Max.z; }
	}
	SurfaceCollisionMesh::SurfaceCollisionMesh(int VertexCount, const std::vector<Index3>& Triangles, const std::vector<SurfaceEdge>& AdditionalEdges,
		SurfaceAcceleration Acceleration)
		: m_VertexCount(VertexCount), m_Triangles(Triangles), m_TriangleTree(0), m_EdgeTree(0), m_Acceleration(Acceleration)
	{
		if (VertexCount < 1) throw std::invalid_argument("Surface index requires vertices");
		std::set<std::array<int, 3>> Faces; std::set<std::pair<int, int>> Edges;
		m_Neighbors.resize(VertexCount); m_VertexBounds.resize(VertexCount);
		for (const auto& T : Triangles)
		{
			std::array<int, 3> Key = { T.a, T.b, T.c }; std::sort(Key.begin(), Key.end());
			if (Key[0] < 0 || Key[2] >= VertexCount || Key[0] == Key[1] || Key[1] == Key[2] || !Faces.insert(Key).second)
				throw std::invalid_argument("Invalid surface index triangle");
			for (int I = 0; I < 3; ++I) { const int A = T[I], B = T[(I + 1) % 3]; Edges.emplace(std::min(A, B), std::max(A, B)); }
		}
		for (const auto& E : AdditionalEdges)
		{
			if (E.A<0 || E.B<0 || E.A>=VertexCount || E.B>=VertexCount || E.A==E.B) throw std::invalid_argument("Invalid surface index edge");
			Edges.emplace(std::min(E.A,E.B),std::max(E.A,E.B));
		}
		for (const auto& E : Edges)
		{
			m_Edges.push_back({ E.first, E.second }); m_Neighbors[E.first].push_back(E.second); m_Neighbors[E.second].push_back(E.first);
		}
		for (auto& Neighbors : m_Neighbors) std::sort(Neighbors.begin(), Neighbors.end());
		m_TriangleEntries.resize(Triangles.size()); m_EdgeEntries.resize(m_Edges.size());
		for (size_t I = 0; I < m_TriangleEntries.size(); ++I) m_TriangleEntries[I].Index = (int)I;
		for (size_t I = 0; I < m_EdgeEntries.size(); ++I) m_EdgeEntries[I].Index = (int)I;
	}
	void SurfaceCollisionMesh::Update(const std::vector<Vector3>& Start, const std::vector<Vector3>& End, float Margin)
	{
		if (Start.size() != (size_t)m_VertexCount || End.size() != Start.size() || !std::isfinite(Margin) || Margin < 0)
			throw std::invalid_argument("Invalid surface index update");
		// Validate into temporary bounds before touching the live index, so a bad
		// position does not leave half of the leaves on a different time interval.
		std::vector<Box3> Bounds(Start.size()); const Vector3 Expansion(Margin);
		for (size_t I = 0; I < Start.size(); ++I)
		{
			if (!Finite(Start[I]) || !Finite(End[I])) throw std::invalid_argument("Non-finite surface index position");
			Bounds[I] = Box3(Start[I].Min(End[I]) - Expansion, Start[I].Max(End[I]) + Expansion);
			if (!ValidBounds(Bounds[I])) throw std::invalid_argument("Unrepresentable surface index bounds");
		}
		m_VertexBounds.swap(Bounds);
		const bool Hashed = m_Acceleration == SurfaceAcceleration::HierarchicalHash;
		if (Hashed) m_FeatureBounds.resize(m_Triangles.size());
		for (size_t I = 0; I < m_Triangles.size(); ++I)
		{
			const auto& T = m_Triangles[I]; auto& E = m_TriangleEntries[I]; E.Bounds = Box3(Box3(m_VertexBounds[T.a], m_VertexBounds[T.b]), m_VertexBounds[T.c]);
			if (Hashed) m_FeatureBounds[I] = E.Bounds;
			else if (E.Node < 0) E.Node = m_TriangleTree.Add(E.Bounds, &E); else m_TriangleTree.Update(E.Node, E.Bounds, Vector3::Zero());
		}
		if (Hashed) { m_TriangleHash.Build(m_FeatureBounds, Margin); m_FeatureBounds.resize(m_Edges.size()); }
		for (size_t I = 0; I < m_Edges.size(); ++I)
		{
			const auto& Edge = m_Edges[I]; auto& E = m_EdgeEntries[I]; E.Bounds = Box3(m_VertexBounds[Edge.A], m_VertexBounds[Edge.B]);
			if (Hashed) m_FeatureBounds[I] = E.Bounds;
			else if (E.Node < 0) E.Node = m_EdgeTree.Add(E.Bounds, &E); else m_EdgeTree.Update(E.Node, E.Bounds, Vector3::Zero());
		}
		if (Hashed) m_EdgeHash.Build(m_FeatureBounds, Margin);
		m_Ready = true;
	}
	void SurfaceCollisionMesh::RequireReady() const { if (!m_Ready) throw std::logic_error("Surface index needs a position update before querying"); }
	void SurfaceCollisionMesh::Query(const DynamicAABBTree& Tree, const Box3& Bounds, std::vector<int>& Indices)
	{
		if (!ValidBounds(Bounds)) throw std::invalid_argument("Invalid surface query bounds");
		std::vector<void*> Entries; Tree.Query(Bounds, &Entries); Indices.clear(); Indices.reserve(Entries.size());
		for (const auto* Item : Entries) Indices.push_back(static_cast<const Entry*>(Item)->Index);
		std::sort(Indices.begin(), Indices.end());
	}
	void SurfaceCollisionMesh::QueryTriangles(const Box3& Bounds, std::vector<int>& Triangles) const
	{
		RequireReady();
		if (m_Acceleration == SurfaceAcceleration::HierarchicalHash) m_TriangleHash.Query(Bounds, Triangles); else Query(m_TriangleTree, Bounds, Triangles);
	}
	void SurfaceCollisionMesh::QueryEdges(const Box3& Bounds, std::vector<int>& Edges) const
	{
		RequireReady();
		if (m_Acceleration == SurfaceAcceleration::HierarchicalHash) m_EdgeHash.Query(Bounds, Edges); else Query(m_EdgeTree, Bounds, Edges);
	}
	bool SurfaceCollisionMesh::Related(int A, int B, bool IncludeNeighbors) const
	{
		return A == B || (IncludeNeighbors && std::binary_search(m_Neighbors[A].begin(), m_Neighbors[A].end(), B));
	}
	void SurfaceCollisionMesh::FindSelfCandidates(std::vector<SurfaceCandidate>& Candidates, bool ExcludeNeighbors) const
	{
		RequireReady(); Candidates.clear(); std::vector<int> Hits;
		for (int I = 0; I < m_VertexCount; ++I)
		{
			QueryTriangles(m_VertexBounds[I], Hits);
			for (int J : Hits)
			{
				const auto& T = m_Triangles[J]; if (Related(I, T.a, ExcludeNeighbors) || Related(I, T.b, ExcludeNeighbors) || Related(I, T.c, ExcludeNeighbors)) continue;
				Candidates.push_back({ SurfacePairType::VertexTriangle, I, J });
			}
		}
		for (size_t I = 0; I < m_Edges.size(); ++I)
		{
			QueryEdges(m_EdgeEntries[I].Bounds, Hits); const auto& A = m_Edges[I];
			for (int J : Hits)
			{
				const auto& B = m_Edges[J]; if ((size_t)J <= I || Related(A.A, B.A, ExcludeNeighbors) || Related(A.A, B.B, ExcludeNeighbors)
					|| Related(A.B, B.A, ExcludeNeighbors) || Related(A.B, B.B, ExcludeNeighbors)) continue;
				Candidates.push_back({ SurfacePairType::EdgeEdge, (int)I, J });
			}
		}
	}
	void SurfaceCollisionMesh::FindCandidates(const SurfaceCollisionMesh& Other, std::vector<SurfaceCandidate>& Candidates) const
	{
		RequireReady(); Other.RequireReady(); if (this == &Other) { FindSelfCandidates(Candidates); return; }
		Candidates.clear(); std::vector<int> Hits;
		for (int I = 0; I < m_VertexCount; ++I)
		{
			Other.QueryTriangles(m_VertexBounds[I], Hits); for (int J : Hits) Candidates.push_back({ SurfacePairType::VertexTriangle, I, J });
		}
		for (int J = 0; J < Other.m_VertexCount; ++J)
		{
			QueryTriangles(Other.m_VertexBounds[J], Hits); for (int I : Hits) Candidates.push_back({ SurfacePairType::TriangleVertex, I, J });
		}
		for (size_t I = 0; I < m_Edges.size(); ++I)
		{
			Other.QueryEdges(m_EdgeEntries[I].Bounds, Hits); for (int J : Hits) Candidates.push_back({ SurfacePairType::EdgeEdge, (int)I, J });
		}
	}

	/* Shared prescribed collision geometry
	   ------------------------------------
	   Cloth and strand contacts own a geometry snapshot and borrow only its body.
	   Explicit registration selects colliders; shape edits require re-registration.
	   A source pointer is an identity for removal and is never dereferenced later.
	   Body removal must unregister snapshots before freeing that body.
	   Convex planes support signed interior recovery; triangle/edge BVHs select
	   witnesses for the common SurfaceCollision kernels. Rounded colliders retain
	   their true centerline/radius instead of a tessellated approximation.
	   Swept vertices follow linear chords between shortest-arc pose samples. The
	   sagitta r*(1-cos(theta/2)) bounds rotational deviation within each interval.
	   Callers budget angular subdivisions and commit history only after success. */
	namespace
	{
		Transform BodyPose(const RigidBody* Body)
		{
			if (!Body) return Transform();
			const auto& Q=Body->Q; const double Norm=(double)Q.x*Q.x+(double)Q.y*Q.y+(double)Q.z*Q.z+(double)Q.w*Q.w;
			if (!Finite(Body->X) || !std::isfinite(Norm) || !(Norm>0)) throw std::invalid_argument("Invalid surface collider pose");
			const double Length=std::sqrt(Norm); const auto Position=Body->BodyLocalToWorld(Vector3::Zero());
			if (!Finite(Position)) throw std::invalid_argument("Invalid surface collider center");
			return Transform(Position,Quaternion((float)(Q.x/Length),(float)(Q.y/Length),(float)(Q.z/Length),(float)(Q.w/Length)));
		}
		double Angle(const Transform& A,const Transform& B)
		{
			const auto& X=A.quat; const auto& Y=B.quat;
			const double NX=(double)X.x*X.x+(double)X.y*X.y+(double)X.z*X.z+(double)X.w*X.w, NY=(double)Y.x*Y.x+(double)Y.y*Y.y+(double)Y.z*Y.z+(double)Y.w*Y.w;
			return 2*std::acos(std::min(1.0,std::abs((double)X.x*Y.x+(double)X.y*Y.y+(double)X.z*Y.z+(double)X.w*Y.w)/std::sqrt(NX*NY)));
		}
	}
	bool SurfaceCollider::Initialize(Geometry* Shape)
	{
		if (!Shape) return false; auto* C=this;
		C->Body = Shape->GetParent<RigidBody>(); C->Source = Shape;
		const Transform ShapePose = C->Body ? C->Body->GetGeometryTransform(Shape) : *Shape->GetTransform();
		const Transform LocalPose = BodyPose(C->Body).TransformInv(ShapePose);
		if (const auto* S = Shape->GetShapeObj<Sphere3>()) { C->Local = { S->Center }; C->Radius = S->Radius; }
		else if (const auto* S = Shape->GetShapeObj<Capsule3>()) { C->Local = { S->X0, S->X1 }; C->Radius = S->Radius; }
		else if (const auto* S = Shape->GetShapeObj<AxisAlignedBox3>())
		{
			if (!Finite(S->Min) || !Finite(S->Max) || S->Min.x >= S->Max.x || S->Min.y >= S->Max.y || S->Min.z >= S->Max.z) return false;
			for (int I = 0; I < 8; ++I) C->Local.emplace_back(I & 1 ? S->Max.x : S->Min.x, I & 2 ? S->Max.y : S->Min.y, I & 4 ? S->Max.z : S->Min.z);
			for (int Axis = 0; Axis < 3; ++Axis) for (int Side = 0; Side < 2; ++Side)
			{
				const int Base = Side << Axis, U = 1 << ((Axis + 1) % 3), V = 1 << ((Axis + 2) % 3);
				C->Triangles.emplace_back(Base, Base | U, Base | U | V); C->Triangles.emplace_back(Base, Base | U | V, Base | V);
				Vector3 Normal = Vector3::Zero(); Normal[Axis] = Side ? 1.0f : -1.0f;
				C->Planes.push_back({ C->Local[Base], Normal });
			}
		}
		else if (const auto* S = Shape->GetShapeObj<ConvexMesh>())
		{
			// BuildFromPoints intentionally omits the optional rigid edge table.
			// Validate the face soup we consume; SurfaceCollisionMesh derives edges.
			if (S->NumVertices < 4 || S->NumFaces < 4 || !S->Vertices || !S->Faces || !S->Indices) return false;
			C->Local.assign(S->Vertices, S->Vertices + S->NumVertices);
			for (int I = 0; I < S->NumFaces; ++I)
			{
				const auto& F = S->Faces[I];
				if (F.numVerties < 3 || (size_t)F.first + F.numVerties > S->NumIndices) return false;
				for (int J = 0; J < F.numVerties; ++J) if (S->Indices[F.first + J] >= S->NumVertices) return false;
				const int A = S->Indices[F.first];
				if (!Finite(F.plane.Normal) || F.plane.Normal.IsZero()) return false;
				C->Planes.push_back({ S->Vertices[A], F.plane.Normal.Unit() });
				for (int J = 1; J + 1 < F.numVerties; ++J) C->Triangles.emplace_back(A, S->Indices[F.first + J], S->Indices[F.first + J + 1]);
			}
		}
		else if (const auto* S = Shape->GetShapeObj<TriangleMesh>())
		{
			if (!S->Vertices || !S->Indices || !S->NumTriangles || S->NumVertices > (uint32_t)std::numeric_limits<int>::max()) return false;
			C->Local.assign(S->Vertices, S->Vertices + S->NumVertices);
			for (uint32_t I = 0; I < S->NumTriangles; ++I)
			{
				uint32_t A, B, D; S->GetVertIndices(I, A, B, D);
				if (A >= S->NumVertices || B >= S->NumVertices || D >= S->NumVertices) return false;
				C->Triangles.emplace_back((int)A, (int)B, (int)D);
			}
		}
		else return false;
		if (!std::isfinite(C->Radius) || C->Radius < 0 || (C->Triangles.empty() && C->Radius <= 0)) return false;
		for (auto& P : C->Local) P = LocalPose.LocalToWorld(P);
		for (auto& P : C->Planes) { P.Point = LocalPose.LocalToWorld(P.Point); P.Normal = LocalPose.quat * P.Normal; }
		return Prepare();
	}
	bool SurfaceCollider::InitializeCapsule(RigidBody* Owner, float InputRadius, const Vector3& A, const Vector3& B)
	{
		if (!std::isfinite(InputRadius) || InputRadius<=0 || !Finite(A) || !Finite(B)) return false;
		Body=Owner; Radius=InputRadius; Local.push_back(Body ? Body->WorldToBodyLocal(A) : A);
		if (A!=B) Local.push_back(Body ? Body->WorldToBodyLocal(B) : B); return Prepare();
	}
	bool SurfaceCollider::Prepare()
	{
		if (Local.empty() || Local.size()>(size_t)std::numeric_limits<int>::max()) return false;
		for (const auto& P : Local)
		{ if (!Finite(P)) return false; RotationRadius=std::max(RotationRadius,std::hypot(std::hypot((double)P.x,(double)P.y),(double)P.z)); }
		try
		{
			Previous=BodyPose(Body); FrameEnd=StepStart=StepEnd=Previous; Start.resize(Local.size()); End.resize(Local.size());
			if (!Triangles.empty()) Index.reset(new SurfaceCollisionMesh((int)Local.size(),Triangles));
		}
		catch (const std::invalid_argument&) { return false; } return true;
	}
	void SurfaceCollider::CaptureFrame() { FrameEnd=BodyPose(Body); }
	double SurfaceCollider::FrameAngle() const { return Angle(Previous,FrameEnd); }
	void SurfaceCollider::Update(float StartFraction, float EndFraction, bool Continuous)
	{
		auto Sample=[&](float T) { return Transform(Previous.pos+(FrameEnd.pos-Previous.pos)*T,Quaternion::Slerp(Previous.quat,FrameEnd.quat,T).Unit()); };
		StepStart=Sample(StartFraction); StepEnd=Sample(EndFraction);
		for (size_t I=0; I<Local.size(); ++I) { Start[I]=StepStart.LocalToWorld(Local[I]); End[I]=StepEnd.LocalToWorld(Local[I]); }
		const double Sine=std::sin(Angle(StepStart,StepEnd)/4), Bound=Continuous ? RotationRadius*2*Sine*Sine : 0;
		if (!std::isfinite(Bound) || Bound>std::numeric_limits<float>::max()) throw std::invalid_argument("Surface collider rotation bound overflow");
		ArcBound=(float)Bound; if (Index) Index->Update(Continuous ? Start : End,End,ArcBound);
	}
}

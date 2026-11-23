#include "DeformableCollision.h"
#include "DeformableBody.h"
#include "../Collision/SurfaceCollision.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>

namespace Riemann
{
	/* Surface contact is independent of the volume material model. Only vertices
	   referenced by the boundary triangles enter the index: interior tetrahedral
	   nodes must not collide with their own boundary. Vertex/triangle queries run
	   in both directions and edge/edge queries catch crossing surface edges.

	   This adapter is DISCRETE. ParticleRadius is the surface half-thickness, and
	   pair thickness is its sum. It recovers nearby overlaps but cannot recover a
	   fully contained volume or guarantee separation after a large crossing step.
	   Swept barriers and adaptive retries require a separate trajectory policy.
	   See Bridson, Fedkiw and Anderson (SIGGRAPH 2002) for surface feature contacts.

	   The shared step alternates material, attachment and contact projection before
	   reconstructing velocities. Contacting bodies use one substep clock even if
	   they have different requested budgets. Both endpoints receive mass-weighted
	   corrections, including self contact; pinned endpoints have zero inverse mass.
	   Every iteration refits bounds and refreshes witnesses. Multipliers survive
	   only within this physical substep, while obsolete feature rows are discarded.
	   Friction measures motion from the substep start at the CURRENT material
	   witnesses. Refresh preserves XPBD compliance and transports tangent impulses.

	   Statistics accumulate candidate visits and newly created rows across the
	   frame. MaximumPenetration is the largest pre-projection violation encountered,
	   not a convergence certificate. Surface topology is copied per substep; rows
	   borrow particle positions while DeformableStep holds the topology edit guard.
	   Self exclusion uses the surface one-ring, independently of elastic edge data.
	   Attached overlapping surfaces still need an appropriate collision policy;
	   a zero-gap attachment and positive contact thickness can be incompatible. */
	bool DeformableCollisionSettings::IsValid() const
	{
		return std::isfinite(ContactTolerance) && ContactTolerance > 0 && Material.IsValid();
	}
	bool DeformableCollision::CanInteract(const DeformableBody& A, const DeformableBody& B)
	{
		return &A != &B && A.GetSettings().Collision.InterCollision && B.GetSettings().Collision.InterCollision;
	}
	struct DeformableCollision::Implementation
	{
		struct Surface
		{
			DeformableBody* Body;
			std::vector<int> Vertices;
			std::vector<Vector3> Start, End;
			std::unique_ptr<SurfaceCollisionMesh> Index;
			explicit Surface(DeformableBody* Owner) : Body(Owner)
			{
				std::vector<int> Mapping(Owner->GetParticleCount(), -1);
				std::vector<Index3> Triangles;
				for (const auto& Face : Owner->GetSurfaceTriangles())
				{
					Index3 Local;
					for (int I = 0; I < 3; ++I)
					{
						int& Slot = Mapping.at(Face[I]);
						if (Slot < 0) { Slot = (int)Vertices.size(); Vertices.push_back(Face[I]); }
						Local[I] = Slot;
					}
					Triangles.push_back(Local);
				}
				if (Vertices.empty()) throw std::invalid_argument("Deformable collision requires boundary triangles");
				Index.reset(new SurfaceCollisionMesh((int)Vertices.size(), Triangles));
				Start.resize(Vertices.size()); End.resize(Vertices.size());
			}
			void Update()
			{
				for (size_t I = 0; I < Vertices.size(); ++I)
				{
					const auto& P = Body->GetParticle(Vertices[I]); Start[I] = P.PreviousPosition; End[I] = P.Position;
				}
				const auto& S = Body->GetSettings(); const float Margin = S.ParticleRadius + S.Collision.ContactTolerance;
				Index->Update(End, End, Margin);
			}
		};
		struct Feature
		{
			int Count = 1, Indices[3] = { 0, 0, 0 };
			explicit Feature(int Vertex) { Indices[0] = Vertex; }
			explicit Feature(const SurfaceEdge& Edge) : Count(2) { Indices[0] = Edge.A; Indices[1] = Edge.B; }
			explicit Feature(const Index3& Face) : Count(3) { for (int I = 0; I < Count; ++I) Indices[I] = Face[I]; }
		};
		using Key = std::tuple<size_t, size_t, SurfacePairType, int, int>;
		struct Row { bool Seen = false; std::unique_ptr<SurfaceContactConstraint> Constraint; };
		std::vector<std::unique_ptr<Surface>> Surfaces;
		std::map<Geometry*, size_t> SurfaceIndices;
		BroadPhaseObjectMap<int> ObjectIndices;
		std::map<Key, Row> Rows;
		explicit Implementation(const std::vector<DeformableBody*>& Bodies)
		{
			for (auto* Body : Bodies)
			{
				const auto& S = Body->GetSettings().Collision;
				if (S.SelfCollision || S.InterCollision)
				{
					SurfaceIndices.emplace(Body->GetRenderGeometry(), Surfaces.size());
					Surfaces.emplace_back(new Surface(Body));
				}
			}
		}
		void Add(size_t IA, size_t IB, const SurfaceCandidate& Pair)
		{
			auto& A = *Surfaces[IA]; auto& B = *Surfaces[IB]; const bool Self = IA == IB;
			auto& StatsA = A.Body->m_CollisionStats; auto& StatsB = B.Body->m_CollisionStats;
			++StatsA.Candidates; if (!Self) ++StatsB.Candidates;
			const auto& SA = A.Body->GetSettings(); const auto& SB = B.Body->GetSettings();
			const double Thickness = (double)SA.ParticleRadius + SB.ParticleRadius;
			const double Tolerance = std::max(SA.Collision.ContactTolerance, SB.Collision.ContactTolerance);
			const Feature FA = Pair.Type == SurfacePairType::VertexTriangle ? Feature(Pair.A)
				: Pair.Type == SurfacePairType::TriangleVertex ? Feature(A.Index->GetTriangles()[Pair.A]) : Feature(A.Index->GetEdges()[Pair.A]);
			const Feature FB = Pair.Type == SurfacePairType::TriangleVertex ? Feature(Pair.B)
				: Pair.Type == SurfacePairType::VertexTriangle ? Feature(B.Index->GetTriangles()[Pair.B]) : Feature(B.Index->GetEdges()[Pair.B]);
			SurfaceMotion MA, MB; MA.Count = FA.Count; MB.Count = FB.Count;
			Vector3 OldA[3], OldB[3];
			for (int I = 0; I < FA.Count; ++I) { MA.Start[I] = MA.End[I] = A.End[FA.Indices[I]]; OldA[I] = A.Start[FA.Indices[I]]; }
			for (int I = 0; I < FB.Count; ++I) { MB.Start[I] = MB.End[I] = B.End[FB.Indices[I]]; OldB[I] = B.Start[FB.Indices[I]]; }
			auto Contact = SurfaceCollision::Closest(MA.End, MA.Count, MB.End, MB.Count);
			if (Contact.Distance > Thickness + Tolerance) return;
			if (Contact.Normal.IsZero())
			{
				SurfaceCollisionSettings Query; Query.Thickness = Thickness; Query.DistanceTolerance = Tolerance; Query.RelativeTolerance = 0;
				const auto Old = SurfaceCollision::Closest(OldA, FA.Count, OldB, FB.Count);
				if (!SurfaceCollision::Sweep(MA, MB, Query, Contact, Old.Normal) || Contact.Conservative) return;
			}
			SurfaceContactParticle PA[3], PB[3]; Contact.PointA = Contact.PointB = Vector3::Zero();
			for (int I = 0; I < FA.Count; ++I)
			{
				const int Vertex = A.Vertices[FA.Indices[I]];
				PA[I] = { &A.Body->m_Particles[Vertex].Position, A.Body->GetInverseMass(Vertex) };
				Contact.PointA += OldA[I] * Contact.WeightsA[I];
			}
			for (int I = 0; I < FB.Count; ++I)
			{
				const int Vertex = B.Vertices[FB.Indices[I]];
				PB[I] = { &B.Body->m_Particles[Vertex].Position, B.Body->GetInverseMass(Vertex) };
				Contact.PointB += OldB[I] * Contact.WeightsB[I];
			}
			const double Penetration = std::max(0.0, Thickness - Contact.Distance);
			StatsA.MaximumPenetration = std::max(StatsA.MaximumPenetration, Penetration);
			StatsB.MaximumPenetration = std::max(StatsB.MaximumPenetration, Penetration);
			auto& R = Rows[Key(IA, IB, Pair.Type, Pair.A, Pair.B)]; R.Seen = true;
			if (R.Constraint) R.Constraint->Refresh(PA, FA.Count, PB, FB.Count, Contact, Thickness);
			else
			{
				const auto& CA = SA.Collision.Material; const auto& CB = SB.Collision.Material;
				SurfaceContactMaterial Material; Material.Compliance = std::max(CA.Compliance, CB.Compliance);
				Material.StaticFriction = std::sqrt(CA.StaticFriction) * std::sqrt(CB.StaticFriction);
				Material.DynamicFriction = std::sqrt(CA.DynamicFriction) * std::sqrt(CB.DynamicFriction);
				R.Constraint.reset(new SurfaceContactConstraint(PA, FA.Count, PB, FB.Count, Contact, Thickness, Material));
				++StatsA.Contacts;
				if (Self) ++StatsA.SelfContacts;
				else { ++StatsB.Contacts; ++StatsA.InterContacts; ++StatsB.InterContacts; }
			}
		}
		void Solve(float Dt, const BroadPhaseResult& Pairs)
		{
			if (Surfaces.empty()) return;
			for (auto& R : Rows) R.second.Seen = false;
			for (auto& S : Surfaces) S->Update();
			std::vector<SurfaceCandidate> Candidates;
			for (size_t I = 0; I < Surfaces.size(); ++I)
			{
				auto& A = *Surfaces[I]; const auto& S = A.Body->GetSettings().Collision;
				if (S.SelfCollision)
				{
					A.Index->FindSelfCandidates(Candidates, S.ExcludeSelfNeighbors);
					for (const auto& Pair : Candidates) Add(I, I, Pair);
				}
			}
			const auto InterPairs = Pairs.GetPairs(BroadPhasePairType::Deformable);
			if (Surfaces.size() > 1 && !InterPairs.empty()) ObjectIndices.Update(Pairs, [&](int Index)
			{
				if (Pairs.GetObjectType(Index) != BroadPhaseObjectType::Deformable) return -1;
				const auto It = SurfaceIndices.find(Pairs.GetGeometry(Index));
				return It != SurfaceIndices.end() ? (int)It->second : -1;
			});
			if (Surfaces.size() > 1) for (const auto& P : InterPairs)
			{
				auto* GA = Pairs.GetGeometry(P.index1); auto* GB = Pairs.GetGeometry(P.index2);
				if (!GA || !GB) continue;
				const int IA = ObjectIndices[P.index1], IB = ObjectIndices[P.index2];
				if (IA < 0 || IB < 0 || IA == IB) continue;
				const size_t I = (size_t)std::min(IA, IB), J = (size_t)std::max(IA, IB);
				auto& A = *Surfaces[I]; auto& B = *Surfaces[J];
				if (!CanInteract(*A.Body, *B.Body)) continue;
				A.Index->FindCandidates(*B.Index, Candidates);
				for (const auto& Pair : Candidates) Add(I, J, Pair);
			}
			for (auto It = Rows.begin(); It != Rows.end();)
			{
				if (!It->second.Seen) It = Rows.erase(It);
				else { It->second.Constraint->Solve(Dt); ++It; }
			}
		}
	};
	DeformableCollision::DeformableCollision(const std::vector<DeformableBody*>& Bodies) : m_Data(new Implementation(Bodies)) {}
	DeformableCollision::~DeformableCollision() = default;
	void DeformableCollision::Solve(float Dt, const BroadPhaseResult& Pairs) { m_Data->Solve(Dt, Pairs); }
}

#include "DeformableRigidCollision.h"
#include "DeformableBody.h"
#include "RigidSurfaceContact.h"
#include "../Collision/SurfaceCollision.h"
#include "../Collision/GeometryQuery.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>

namespace Riemann
{
	/* This discrete adapter discovers contacts independently of rigid manifolds,
	   but joins the same PBD/XPBD iterations. Filled soft triangles against rounded
	   centerlines, bidirectional vertex/triangle and edge/edge queries cover face
	   interiors that vertex-only collision misses. Closed convex planes recover
	   embedded boundary vertices. Interior volume nodes never generate contacts.
	   The shared world promotes soft substeps to physical world substeps; rigid
	   prediction and reconstruction each happen once. Geometry is copied while
	   bodies and rows are borrowed under the world's coupled-step lifetime guard.

	   Each refresh binds the CURRENT rigid material point and transports its local
	   anchor back to the substep start for friction. This includes sphere/capsule
	   surface rotation. Shape-local transforms are converted to body-local origin
	   coordinates, then the rigid Jacobian subtracts the center-of-mass offset.
	   RigidSurfaceContact applies both reaction endpoints, including torque; no
	   delayed force is replayed. Contacted rigids remain awake while soft bodies
	   continue simulating. Simulation flags, triggers and the world's collision
	   filter apply before discovery, independently of query visibility.

	   Supported geometry is sphere, capsule, box, convex mesh, triangle mesh and
	   plane. An overlapping unsupported shape fails explicitly. TODO: height-field,
	   cylinder and SDF feature adapters, and swept constraint-generated trajectories.
	   SI/LCP require a coordinated impulse adapter and are rejected by the world. */
	namespace
	{
		Transform Pose(const RigidBody* B) { return Transform(B->X - B->Q * B->CM, B->Q); }
		struct Feature
		{
			int Count = 1, Indices[3] = {};
			explicit Feature(int I) { Indices[0] = I; }
			explicit Feature(const Index3& T) : Count(3) { for (int I = 0; I < Count; ++I) Indices[I] = T[I]; }
			explicit Feature(const SurfaceEdge& E) : Count(2) { Indices[0] = E.A; Indices[1] = E.B; }
		};
		struct Plane { Vector3 Point, Normal; };
	}
	struct DeformableRigidCollision::Implementation
	{
		struct Surface
		{
			DeformableBody* Body;
			std::vector<Vector3> Positions;
			std::vector<bool> Boundary;
			SurfaceCollisionMesh Index;
			explicit Surface(DeformableBody* B) : Body(B), Boundary(B->GetParticleCount(), false), Index(B->GetParticleCount(), B->GetSurfaceTriangles())
			{
				if (B->GetSurfaceTriangles().empty()) throw std::invalid_argument("Rigid deformable collision requires boundary triangles");
				for (const auto& T : B->GetSurfaceTriangles()) for (int I = 0; I < 3; ++I) Boundary[T[I]] = true;
			}
			void Update()
			{
				Positions.clear(); for (const auto& P : Body->GetParticles()) Positions.push_back(P.Position);
				const auto& S = Body->GetSettings(); Index.Update(Positions, Positions, S.ParticleRadius + S.Collision.ContactTolerance);
			}
		};
		struct Collider
		{
			DeformableRigidCollider Source;
			std::vector<Vector3> Local, End;
			std::vector<Index3> Triangles;
			std::vector<Plane> Planes;
			std::unique_ptr<SurfaceCollisionMesh> Index;
			float Radius = 0;
			bool Infinite = false, Supported = true;
			explicit Collider(const DeformableRigidCollider& C) : Source(C)
			{
				auto* Shape = C.Shape; const auto LocalPose = Pose(C.Body).TransformInv(C.Body->GetGeometryTransform(Shape));
				if (const auto* S = Shape->GetShapeObj<Sphere3>()) { Local = { S->Center }; Radius = S->Radius; }
				else if (const auto* S = Shape->GetShapeObj<Capsule3>()) { Local = { S->X0, S->X1 }; Radius = S->Radius; }
				else if (const auto* S = Shape->GetShapeObj<AxisAlignedBox3>())
				{
					for (int I = 0; I < 8; ++I) Local.emplace_back(I & 1 ? S->Max.x : S->Min.x, I & 2 ? S->Max.y : S->Min.y, I & 4 ? S->Max.z : S->Min.z);
					for (int Axis = 0; Axis < 3; ++Axis) for (int Side = 0; Side < 2; ++Side)
					{
						const int Base = Side << Axis, U = 1 << ((Axis + 1) % 3), V = 1 << ((Axis + 2) % 3);
						Triangles.emplace_back(Base, Base | U, Base | U | V); Triangles.emplace_back(Base, Base | U | V, Base | V);
						Vector3 Normal(0); Normal[Axis] = Side ? 1.0f : -1.0f; Planes.push_back({ Local[Base], Normal });
					}
				}
				else if (const auto* S = Shape->GetShapeObj<ConvexMesh>())
				{
					if (!S->Vertices || !S->Faces || !S->Indices) throw std::invalid_argument("Invalid deformable convex collider");
					Local.assign(S->Vertices, S->Vertices + S->NumVertices);
					for (int I = 0; I < S->NumFaces; ++I)
					{
						const auto& F = S->Faces[I];
						if (F.numVerties < 3 || (size_t)F.first + F.numVerties > S->NumIndices) throw std::invalid_argument("Invalid deformable convex face");
						for (int J = 0; J < F.numVerties; ++J) if (S->Indices[F.first + J] >= S->NumVertices) throw std::invalid_argument("Invalid deformable convex index");
						const int A = S->Indices[F.first]; Planes.push_back({ Local[A], F.plane.Normal.Unit() });
						for (int J = 1; J + 1 < F.numVerties; ++J) Triangles.emplace_back(A, S->Indices[F.first + J], S->Indices[F.first + J + 1]);
					}
				}
				else if (const auto* S = Shape->GetShapeObj<TriangleMesh>())
				{
					if (!S->Vertices || !S->Indices) throw std::invalid_argument("Invalid deformable mesh collider");
					Local.assign(S->Vertices, S->Vertices + S->NumVertices);
					for (uint32_t I = 0; I < S->NumTriangles; ++I)
					{
						uint32_t A, B, D; S->GetVertIndices(I, A, B, D);
						if (A >= S->NumVertices || B >= S->NumVertices || D >= S->NumVertices) throw std::invalid_argument("Invalid deformable mesh index");
						Triangles.emplace_back((int)A, (int)B, (int)D);
					}
				}
				else if (const auto* S = Shape->GetShapeObj<Plane3>()) { Planes.push_back({ -S->Normal * S->D, S->Normal }); Infinite = true; }
				else Supported = false;
				for (auto& P : Local) P = LocalPose.LocalToWorld(P);
				for (auto& P : Planes) { P.Point = LocalPose.LocalToWorld(P.Point); P.Normal = LocalPose.quat * P.Normal; }
				if (!Triangles.empty()) Index.reset(new SurfaceCollisionMesh((int)Local.size(), Triangles));
			}
			void Update()
			{
				End.clear(); const auto Current = Pose(Source.Body);
				for (const auto& P : Local) End.push_back(Current.LocalToWorld(P));
				if (Index) Index->Update(End, End, 0);
			}
		};
		using Key = std::tuple<size_t, size_t, int, int, int>;
		enum ContactKind { VertexTriangle, TriangleVertex, EdgeEdge, Rounded, Interior };
		struct Row { bool Seen = false; RigidBody* Body = nullptr; std::unique_ptr<RigidSurfaceContact> Constraint; };
		std::vector<std::unique_ptr<Surface>> Surfaces;
		std::vector<std::unique_ptr<Collider>> Colliders;
		std::map<Key, Row> Rows;
		std::vector<RigidBody*> Touched;
		CollisionFilter* Filter;
		Implementation(const std::vector<DeformableBody*>& Bodies, const std::vector<DeformableRigidCollider>& Shapes, CollisionFilter* Policy) : Filter(Policy)
		{
			for (auto* B : Bodies) if (B->GetSettings().Collision.RigidCollision) Surfaces.emplace_back(new Surface(B));
			if (Surfaces.empty()) return;
			for (const auto& C : Shapes) if (C.Shape && C.Body && C.Shape->IsSimulationEnabled() && !C.Shape->IsTriggerEnabled())
			{
				bool Allowed = false;
				for (const auto& S : Surfaces) if (CollisionFilter::Allows(Filter, S->Body->GetSettings().Collision.FilterData, C.Shape->GetFilterData())) Allowed = true;
				if (!Allowed) continue;
				if (C.Body->ReducedRagdoll || (C.Body->CastDynamic() && C.Body->CastDynamic()->mMotionType == MorionType::Continuous))
					throw std::invalid_argument("Deformable rigid contacts require independent discrete colliders");
				Colliders.emplace_back(new Collider(C));
			}
		}
		void Bind(const Key& K, Surface& S, Collider& C, const Feature& A, SurfaceContact Contact, const Vector3& WorldPoint)
		{
			SurfaceContactParticle Particles[3]; Contact.PointA = Vector3::Zero();
			for (int I = 0; I < A.Count; ++I)
			{
				const int J = A.Indices[I]; Particles[I] = { &S.Body->m_Particles[J].Position, S.Body->GetInverseMass(J) };
				Contact.PointA += S.Body->GetParticle(J).PreviousPosition * Contact.WeightsA[I];
			}
			auto* Body = C.Source.Body; auto* Dynamic = Body->CastDynamic(); const bool Movable = Dynamic && !Dynamic->Freezing;
			const Vector3 Anchor = Body->WorldToBodyLocal(WorldPoint); Contact.PointB = C.Source.StartPose.LocalToWorld(Anchor);
			RigidConstraintBody Rigid{ &Body->X, &Body->Q, Movable ? Body->InvMass : 0, Movable ? Body->InvInertia : Matrix3::Zero() };
			auto& R = Rows[K]; R.Seen = true; R.Body = Body;
			const auto& Settings = S.Body->GetSettings(); auto& Stats = S.Body->m_CollisionStats;
			Stats.MaximumPenetration = std::max(Stats.MaximumPenetration, std::max(0.0, Settings.ParticleRadius - Contact.Distance));
			if (R.Constraint) R.Constraint->Refresh(Particles, A.Count, Rigid, Anchor - Body->CM, Contact, Settings.ParticleRadius);
			else
			{
				auto Material = Settings.Collision.Material;
				Material.StaticFriction = std::sqrt(Material.StaticFriction) * std::sqrt((double)Body->GetFrictionStatic());
				Material.DynamicFriction = std::sqrt(Material.DynamicFriction) * std::sqrt((double)Body->GetFrictionDynamic());
				R.Constraint.reset(new RigidSurfaceContact(Particles, A.Count, Rigid, Anchor - Body->CM, Contact, Settings.ParticleRadius, Material));
				++Stats.Contacts; ++Stats.RigidContacts;
			}
			if (std::find(Touched.begin(), Touched.end(), Body) == Touched.end()) Touched.push_back(Body);
			if (Dynamic) Dynamic->Wakeup();
		}
		void Contact(size_t IS, size_t IC, int Kind, int IA, int IB, const Feature& A, const Feature& B)
		{
			auto& S = *Surfaces[IS]; auto& C = *Colliders[IC]; ++S.Body->m_CollisionStats.Candidates;
			Vector3 PA[3], PB[3]; for (int I = 0; I < A.Count; ++I) PA[I] = S.Positions[A.Indices[I]];
			for (int I = 0; I < B.Count; ++I) PB[I] = C.End[B.Indices[I]];
			auto Hit = SurfaceCollision::Closest(PA, A.Count, PB, B.Count); const auto& Settings = S.Body->GetSettings();
			if (Hit.Distance > Settings.ParticleRadius + C.Radius + Settings.Collision.ContactTolerance) return;
			if (Hit.Normal.IsZero())
			{
				SurfaceMotion MA, MB; MA.Count = A.Count; MB.Count = B.Count;
				for (int I = 0; I < A.Count; ++I) MA.Start[I] = MA.End[I] = PA[I];
				for (int I = 0; I < B.Count; ++I) MB.Start[I] = MB.End[I] = PB[I];
				SurfaceCollisionSettings Query; Query.Thickness = Settings.ParticleRadius + C.Radius;
				if (!SurfaceCollision::Sweep(MA, MB, Query, Hit) || Hit.Conservative) return;
			}
			const Vector3 Point = Hit.PointB + Hit.Normal * C.Radius; Hit.Distance -= C.Radius;
			Bind(Key(IS, IC, Kind, IA, IB), S, C, A, Hit, Point);
		}
		void Detect(size_t IS, size_t IC)
		{
			auto& S = *Surfaces[IS]; auto& C = *Colliders[IC]; const auto& Settings = S.Body->GetSettings();
			if (!CollisionFilter::Allows(Filter, Settings.Collision.FilterData, C.Source.Shape->GetFilterData())) return;
			const Box3 Bounds = C.Source.Body->GetGeometryBounds(C.Source.Shape);
			if (!C.Infinite)
			{
				bool Near = false; for (size_t I = 0; I < S.Boundary.size(); ++I)
					if (S.Boundary[I] && S.Index.GetVertexBounds((int)I).Intersect(Bounds)) { Near = true; break; }
				// A small collider can meet the middle of a large face without a nearby
				// surface vertex. Triangle bounds are therefore the pair rejection test.
				std::vector<int> Hits; S.Index.QueryTriangles(Bounds, Hits); Near = Near || !Hits.empty();
				if (!Near) return;
			}
			if (!C.Supported) throw std::invalid_argument("Unsupported overlapping deformable rigid collider");
			std::vector<bool> Inside(S.Boundary.size(), false);
			const auto Current = Pose(C.Source.Body);
			if (!C.Planes.empty()) for (size_t I = 0; I < S.Boundary.size(); ++I) if (S.Boundary[I])
			{
				const Vector3 Local = Current.WorldToLocal(S.Positions[I]); double Gap = -std::numeric_limits<double>::infinity(); int PlaneIndex = -1;
				for (size_t J = 0; J < C.Planes.size(); ++J)
				{
					const double D = C.Planes[J].Normal.Dot(Local - C.Planes[J].Point); if (D > Gap) { Gap = D; PlaneIndex = (int)J; }
				}
				if (PlaneIndex < 0 || Gap > (C.Infinite ? Settings.ParticleRadius + Settings.Collision.ContactTolerance : 0)) continue;
				Inside[I] = true; SurfaceContact Hit; Hit.Normal = Current.quat * C.Planes[PlaneIndex].Normal; Hit.WeightsA = Vector3::UnitX(); Hit.Distance = Gap;
				Bind(Key(IS, IC, Interior, (int)I, 0), S, C, Feature((int)I), Hit, Current.LocalToWorld(Local - C.Planes[PlaneIndex].Normal * (float)Gap));
			}
			std::vector<int> Hits;
			if (C.Radius > 0)
			{
				S.Index.QueryTriangles(Bounds, Hits); const Feature B = C.End.size() == 1 ? Feature(0) : Feature(SurfaceEdge{ 0, 1 });
				for (int I : Hits) Contact(IS, IC, Rounded, I, 0, Feature(S.Index.GetTriangles()[I]), B);
			}
			else if (C.Index)
			{
				std::vector<SurfaceCandidate> Candidates; S.Index.FindCandidates(*C.Index, Candidates);
				for (const auto& P : Candidates)
				{
					const Feature A = P.Type == SurfacePairType::VertexTriangle ? Feature(P.A)
						: P.Type == SurfacePairType::TriangleVertex ? Feature(S.Index.GetTriangles()[P.A]) : Feature(S.Index.GetEdges()[P.A]);
					bool Skip = false; for (int I = 0; I < A.Count; ++I) if (!S.Boundary[A.Indices[I]] || Inside[A.Indices[I]]) Skip = true;
					if (Skip) continue;
					const Feature B = P.Type == SurfacePairType::TriangleVertex ? Feature(P.B)
						: P.Type == SurfacePairType::VertexTriangle ? Feature(C.Index->GetTriangles()[P.B]) : Feature(C.Index->GetEdges()[P.B]);
					const int Kind = P.Type == SurfacePairType::VertexTriangle ? VertexTriangle : P.Type == SurfacePairType::TriangleVertex ? TriangleVertex : EdgeEdge;
					Contact(IS, IC, Kind, P.A, P.B, A, B);
				}
			}
		}
		void Solve(float Dt)
		{
			for (auto& R : Rows) R.second.Seen = false;
			for (auto& S : Surfaces) S->Update(); for (auto& C : Colliders) C->Update();
			for (size_t I = 0; I < Surfaces.size(); ++I) for (size_t J = 0; J < Colliders.size(); ++J) Detect(I, J);
			for (auto It = Rows.begin(); It != Rows.end();)
			{
				if (!It->second.Seen) It = Rows.erase(It);
				else { It->second.Constraint->Solve(Dt); It->second.Body->UpdateGeometries(); ++It; }
			}
		}
	};
	DeformableRigidCollision::DeformableRigidCollision(const std::vector<DeformableBody*>& Bodies, const std::vector<DeformableRigidCollider>& Colliders, CollisionFilter* Filter)
		: m_Data(new Implementation(Bodies, Colliders, Filter)) {}
	DeformableRigidCollision::~DeformableRigidCollision() = default;
	void DeformableRigidCollision::Solve(float Dt) { m_Data->Solve(Dt); }
	const std::vector<RigidBody*>& DeformableRigidCollision::GetTouchedBodies() const { return m_Data->Touched; }
}

#include "Test.h"
#include "../Src/Deformation/RigidSurfaceContact.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Viewer/SceneWorld.h"
#include <cfloat>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	using DVector = RigidPointResponse::DVector;
	DVector Double(const Vector3& V) { return DVector(V.x, V.y, V.z); }
	double Length(const DVector& V) { return std::hypot(std::hypot(V.x, V.y), V.z); }
	template<class F> void Reject(F Function)
	{
		bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected);
	}
	SurfaceContact Hit(Vector3 Normal = Vector3::UnitY())
	{
		SurfaceContact C; C.Normal = Normal; C.WeightsA = Vector3::UnitX(); return C;
	}
	void KernelResponse()
	{
		Vector3 Center(0), Point(1, -0.01f, 0); Quaternion Q = Quaternion::One();
		RigidConstraintBody Rigid{ &Center, &Q, 0.5, Matrix3(1, 1, 2) }; SurfaceContactParticle Soft{ &Point, 1 };
		auto C = Hit(); C.PointA = Point; C.PointB = Vector3(1, 0, 0);
		SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 0;
		RigidSurfaceContact Row(&Soft, 1, Rigid, Vector3::UnitX(), C, 0, M);
		const auto Initial = Double(Point) + Double(Center) * 2; Row.Solve(0.01f);
		EXPECT_NEAR(Row.GetNormalMultiplier(), 0.01 / 3.5, 1e-9);
		EXPECT_NEAR(Row.GetRigidAngularMultiplier().z, -Row.GetNormalMultiplier(), 1e-12);
		EXPECT(Length(Double(Point) + Double(Center) * 2 - Initial) < 1e-7);
		EXPECT(Q.z < 0); for (int I = 0; I < 12; ++I) Row.Solve(0.01f); EXPECT(std::abs(Row.GetNormalGap()) < 1e-7);
	}
	void CoupledTangentMass()
	{
		Vector3 Center(0), Arm(1, 2, 3), Point = Arm + Vector3(0.001f, -0.01f, 0.003f); Quaternion Q = Quaternion::One();
		RigidConstraintBody Rigid{ &Center, &Q, 0.5, Matrix3(1, 2, 3) }; SurfaceContactParticle Soft{ &Point, 1 };
		auto C = Hit(); C.PointA = Arm; C.PointB = Arm; SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 100;
		RigidPointResponse Response(Rigid, Arm); auto K = Response.Evaluate().InversePointMass; for (int I = 0; I < 3; ++I) K[I][I] += 1;
		const auto Error = Double(Point) - Double(Arm); const double N = -Error.y / K[1][1];
		const double X = Error.x + K[0][1] * N, Z = Error.z + K[2][1] * N, Determinant = K[0][0] * K[2][2] - K[0][2] * K[2][0];
		const double TX = (-X * K[2][2] + Z * K[0][2]) / Determinant, TZ = (-Z * K[0][0] + X * K[2][0]) / Determinant;
		RigidSurfaceContact Row(&Soft, 1, Rigid, Arm, C, 0, M); Row.Solve(0.01f);
		EXPECT_NEAR(Row.GetNormalMultiplier(), N, 1e-12); EXPECT_NEAR(Row.GetTangentMultiplier().x, TX, 1e-12);
		EXPECT_NEAR(Row.GetTangentMultiplier().z, TZ, 1e-12); EXPECT(!Row.IsSliding());
	}
	void SlidingDirection()
	{
		Vector3 Center(0), Arm(1, 2, 3), Point = Arm + Vector3(0.1f, -0.01f, 0.02f); Quaternion Q = Quaternion::One();
		RigidConstraintBody R{ &Center, &Q, 0.5, Matrix3(1, 2, 3) }; SurfaceContactParticle P{ &Point, 1 };
		auto C = Hit(); C.PointA = C.PointB = Arm; SurfaceContactMaterial M;
		RigidPointResponse Response(R, Arm); auto K = Response.Evaluate().InversePointMass; for (int I = 0; I < 3; ++I) K[I][I] += 1;
		const auto InitialSlip = Double(Point) - Double(Arm); RigidSurfaceContact Row(&P, 1, R, Arm, C, 0, M); Row.Solve(0.01f);
		const auto T = Row.GetTangentMultiplier(); const auto FinalSlip = InitialSlip + K * (DVector(0, Row.GetNormalMultiplier(), 0) + T);
		EXPECT(Row.IsSliding()); EXPECT_NEAR(Length(T), M.DynamicFriction * Row.GetNormalMultiplier(), 1e-12);
		// The sliding impulse opposes the post-response slip, including anisotropic
		// point mass. Clipping the free impulse radially fails this cone condition.
		EXPECT_NEAR(T.x * FinalSlip.z - T.z * FinalSlip.x, 0, 1e-12); EXPECT(T.x * FinalSlip.x + T.z * FinalSlip.z <= 0);
	}
	void KernelPolicies()
	{
		for (int Iterations : { 1, 32 })
		{
			Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One(); auto C = Hit();
			RigidConstraintBody R{ &Center, &Q, 0, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 };
			SurfaceContactMaterial M; M.Compliance = 0.0001; RigidSurfaceContact Row(&P, 1, R, Vector3::Zero(), C, 0, M);
			for (int I = 0; I < Iterations; ++I) { Row.Refresh(&P, 1, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); }
			EXPECT_NEAR(Point.y, -0.005, 1e-8);
		}
		for (double InverseMass : { 1e-200, 1.0, 1e200 })
		{
			Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One();
			RigidConstraintBody R{ &Center, &Q, InverseMass, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 }; auto C = Hit();
			RigidSurfaceContact Row(&P, 1, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); EXPECT(std::abs(Row.GetNormalGap()) < 1e-7);
		}
		Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One();
		RigidConstraintBody R{ &Center, &Q, 0, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 }; auto C = Hit();
		C.WeightsA = Vector3(0.5f, 0.5f, 0); SurfaceContactParticle Duplicate[] = { P, P };
		RigidSurfaceContact Row(Duplicate, 2, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); EXPECT_NEAR(Point.y, 0, 1e-7);
		C = Hit(); C.Normal = Vector3::Zero(); Reject([&] { RigidSurfaceContact Bad(&P, 1, R, Vector3::Zero(), C, 0); });
		C = Hit(); SurfaceContactParticle Alias{ &Center, 1 }; Reject([&] { RigidSurfaceContact Bad(&Alias, 1, R, Vector3::Zero(), C, 0); });
		Vector3 Positions[] = { Vector3(FLT_MAX, 0, 0), Vector3(FLT_MAX, 0, 0) }; Center = Vector3(-FLT_MAX, 0, 0);
		SurfaceContactParticle Large[] = { { &Positions[0], 1 }, { &Positions[1], 0 } };
		C = Hit(-Vector3::UnitX()); C.WeightsA = Vector3(0.001f, 0.999f, 0);
		RigidSurfaceContact Overflow(Large, 2, R, Vector3::Zero(), C, 0); Reject([&] { Overflow.Solve(0.01f); });
		EXPECT(Positions[0].x == FLT_MAX); EXPECT(Center.x == -FLT_MAX); EXPECT(Overflow.GetNormalMultiplier() == 0);
		Center = Vector3::Zero(); Point = Vector3(0, -0.01f, 0); R.InverseMass = 1e308; C = Hit();
		RigidSurfaceContact MassOverflow(&P, 1, R, Vector3::Zero(), C, 0);
		Reject([&] { MassOverflow.Solve(FLT_MAX); }); EXPECT(Center == Vector3::Zero()); EXPECT(Point.y == -0.01f);
		EXPECT(MassOverflow.GetNormalMultiplier() == 0);
	}
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Substeps = 4; S.SolverIterations = 32; S.Damping = 0;
		S.Gravity = Vector3::Zero(); S.GroundEnabled = false; S.ParticleRadius = 0.02f; S.Collision.RigidCollision = true;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0; return S;
	}
	DeformableBody* Triangle(PhysicsWorld& W, const DeformableBodySettings& S, float Y, float Size = 1)
	{
		auto* B = W.CreateDeformableBody(S);
		B->AddParticle(Vector3(-Size, Y, -Size)); B->AddParticle(Vector3(Size, Y, -Size)); B->AddParticle(Vector3(0, Y, Size));
		B->SetSurfaceTriangles({ Index3(0, 1, 2) }); return B;
	}
	RigidBodyDynamic* Dynamic(PhysicsWorld& W, Geometry* Shape)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* R = W.CreateRigidBody(Shape, P)->CastDynamic(); R->InvMass = 0.5f; R->InvInertia = Matrix3(1, 1, 1); R->SetFriction(1, 1); return R;
	}
	DVector Momentum(const DeformableBody* B)
	{
		DVector Result(0); for (const auto& P : B->GetParticles()) if (!P.Pinned && P.InverseMass) Result += Double(P.Velocity) / P.InverseMass; return Result;
	}
	void WorldMomentum()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD }) for (bool Friction : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld W(P); auto S = Settings();
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = Friction ? 1 : 0;
			auto* Soft = Triangle(W, S, 0.51f); auto* R = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
			for (int I = 0; I < 3; ++I) Soft->SetParticleVelocity(I, Vector3(0.2f, -0.1f, 0));
			const auto Before = Momentum(Soft); W.Simulate(0.01f); const auto Total = Momentum(Soft) + Double(R->V) * 2;
			printf("Soft-rigid contact solver=%d friction=%d momentumError=%g angularZ=%g contacts=%zu\n", (int)Solver, Friction, Length(Total - Before), R->W.z, Soft->GetCollisionStats().RigidContacts);
			EXPECT(Length(Total - Before) < 5e-4); EXPECT(Soft->GetCollisionStats().RigidContacts > 0); EXPECT(R->V.y < 0);
			if (Friction) EXPECT(std::abs(R->W.z) > 1e-4); else EXPECT(std::abs(R->W.z) < 1e-5);
			EXPECT(!R->Sleeping);
		}
	}
	void ShapesAndFiltering()
	{
		for (int Shape = 0; Shape < 6; ++Shape)
		{
			printf("Soft-rigid shape=%d\n", Shape); fflush(stdout);
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
			Geometry* G = nullptr; float Y = 0.51f;
			if (Shape == 0) G = GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f);
			if (Shape == 1) G = GeometryFactory::CreateCapsule(Vector3(-0.4f, 0, 0), Vector3(0.4f, 0, 0), 0.5f);
			if (Shape == 2) G = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f));
			if (Shape == 3)
			{
				G = GeometryFactory::CreateConvexMesh(); std::vector<Vector3> V;
				for (int I = 0; I < 8; ++I) V.emplace_back(I & 1 ? 0.5f : -0.5f, I & 2 ? 0.5f : -0.5f, I & 4 ? 0.5f : -0.5f);
				auto* M = G->GetShapeObj<ConvexMesh>(); EXPECT(M->BuildFromPoints(V.data(), (int)V.size())); G->SetShapeBounds(M->Bounds);
			}
			if (Shape == 4) { G = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()); Y = 0.01f; }
			if (Shape == 5)
			{
				G = GeometryFactory::CreateTriangleMesh(); auto* M = G->GetShapeObj<TriangleMesh>();
				for (const auto& V : { Vector3(-3, 0, -3), Vector3(3, 0, -3), Vector3(0, 0, 3) }) M->AddVertex(V);
				M->AddTriangle(0, 1, 2); M->Compact(); G->SetShapeBounds(M->BoundingVolume); Y = 0.01f;
			}
			if (Shape == 2 || Shape == 3) Y = 0.4f;
			RigidBodyParam R; W.CreateRigidBody(G, R); auto* Soft = Triangle(W, S, Y, Shape >= 2 ? 0.2f : 1);
			W.Simulate(0.01f); EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
			for (const auto& V : Soft->GetParticles()) EXPECT(std::isfinite(V.Position.SquareLength()));
			if (Shape >= 2) for (const auto& V : Soft->GetParticles()) EXPECT(V.Position.y >= Y + 0.009f);
			if (Shape == 2 || Shape == 3) for (const auto& V : Soft->GetParticles()) EXPECT(V.Position.y >= 0.52f - 1e-5f);
		}
		for (int Policy = 0; Policy < 4; ++Policy)
		{
			printf("Soft-rigid filter policy=%d\n", Policy); fflush(stdout);
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::PBD; PhysicsWorld W(P); auto S = Settings();
			S.Collision.FilterData = CollisionData::ForLayer(1, Policy == 1 ? 0 : 1u << 2);
			W.SetCollisionFilter(std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask()));
			auto* Soft = Triangle(W, S, 0.51f); auto* G = GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f);
			G->SetFilterData(CollisionData::ForLayer(2, 1u << 1)); G->SetQueryEnabled(false);
			if (Policy == 2) G->SetSimulationEnabled(false); if (Policy == 3) G->SetTriggerEnabled(true);
			RigidBodyParam R; W.CreateRigidBody(G, R); W.Simulate(0.01f);
			EXPECT((Soft->GetCollisionStats().RigidContacts > 0) == (Policy == 0));
		}
	}
	void MeshFeatureContacts()
	{
		for (bool EdgeOnly : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
			auto* Soft = W.CreateDeformableBody(S);
			const std::vector<Vector3> A = EdgeOnly
				? std::vector<Vector3>{ Vector3(-1, 0.01f, 0), Vector3(1, 0.01f, 0), Vector3(-1, 0.01f, -0.1f) }
				: std::vector<Vector3>{ Vector3(-1, 0.01f, -1), Vector3(1, 0.01f, -1), Vector3(0, 0.01f, 1) };
			const std::vector<Vector3> B = EdgeOnly
				? std::vector<Vector3>{ Vector3(0, 0, -1), Vector3(0, 0, 1), Vector3(0.1f, 0, -1) }
				: std::vector<Vector3>{ Vector3(-0.1f, 0, -0.1f), Vector3(0.1f, 0, -0.1f), Vector3(0, 0, 0.1f) };
			for (const auto& V : A) Soft->AddParticle(V); Soft->SetSurfaceTriangles({ Index3(0, 1, 2) });
			auto* G = GeometryFactory::CreateTriangleMesh(); auto* M = G->GetShapeObj<TriangleMesh>();
			for (const auto& V : B) M->AddVertex(V); M->AddTriangle(0, 1, 2); M->Compact(); G->SetShapeBounds(M->BoundingVolume);
			RigidBodyParam R; W.CreateRigidBody(G, R); W.Simulate(0.01f);
			Vector3 Current[3]; for (int I = 0; I < 3; ++I) Current[I] = Soft->GetParticle(I).Position;
			const double Distance = SurfaceCollision::Closest(Current, 3, B.data(), 3).Distance;
			printf("Soft-rigid mesh edgeOnly=%d distance=%g contacts=%zu\n", EdgeOnly, Distance, Soft->GetCollisionStats().RigidContacts);
			EXPECT(Soft->GetCollisionStats().RigidContacts > 0); EXPECT(Distance >= S.ParticleRadius - 1e-5);
		}
	}
	class SpinField : public ForceField
	{
	public:
		RigidBody* Body;
		explicit SpinField(RigidBody* B) : Body(B) {}
		void Update(float) override { Body->Q = Quaternion(0.01f, Vector3::UnitZ()); Body->UpdateGeometries(); }
		bool ApplyForce(RigidBodyDynamic*) override { return false; }
	};
	void MovingSurfaceAndPolicies()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 10;
		auto* Soft = Triangle(W, S, 0.51f); RigidBodyParam R;
		auto* Body = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f), R); Body->SetFriction(10, 10);
		W.AddForceField(new SpinField(Body)); W.Simulate(0.01f); EXPECT(Momentum(Soft).x < -0.01);
		EXPECT(!W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
		auto Support = W.GetSupportStabilizationSettings(); Support.Enabled = true; Reject([&] { W.SetSupportStabilizationSettings(Support); });
		Reject([&] { Soft->Simulate(0.01f); }); auto C = Soft->GetSettings().Collision; C.RigidCollision = false; Soft->SetCollisionSettings(C);
		EXPECT(W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
		Reject([&] { W.CreateDeformableBody(S); });
	}
	void OffsetCenterAndWake()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; P.islandSleeping.Enabled = true;
		PhysicsWorld W(P); auto* Soft = Triangle(W, Settings(), 0.51f);
		auto* R = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
		R->CM = R->X = Vector3::UnitX(); R->UpdateGeometries(); R->Sleeping = true;
		W.Simulate(0.01f); EXPECT(!R->Sleeping); EXPECT(R->W.z > 0);
		EXPECT(Length(Momentum(Soft) + Double(R->V) * 2) < 0.002);
		EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		const std::string Path = TestContentsPath() + "Scenes/08_cloth_sphere.json";
		EXPECT(Scene.LoadFromFile(Path, &Error)); auto* W = Scene.GetSimulation();
		const auto Fields = W->GetForceFieldCount(), Bodies = W->GetDeformableBodies().size();
		// The cloth demo now starts in XPBD; select SI explicitly for the rejection check.
		EXPECT(Scene.SetClothRigidInteraction(false));
		EXPECT(W->SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
		EXPECT(!Scene.ApplyDeformableContactDemo(true, true));
		EXPECT(W->SetSolverConfiguration(ResolutionPhaseSolver::XPBD, SolverBackend::CPU, false));
		EXPECT(Scene.SetClothRigidInteraction(true));
		EXPECT(Scene.ApplyDeformableContactDemo(true, true)); auto* Ball = Scene.GetDemoContactRigid(); EXPECT(Ball); if (!Ball) return;
		size_t Contacts = 0; const float InitialY = Ball->X.y;
		for (int I = 0; I < 120; ++I)
		{
			Scene.Step(1.0f / 60);
			Contacts += Scene.GetDemoContactBody(0)->GetCollisionStats().RigidContacts + Scene.GetDemoContactBody(1)->GetCollisionStats().RigidContacts;
			EXPECT(std::isfinite(Ball->X.SquareLength()));
		}
		printf("Soft-rigid existing demo contacts=%zu ballY=%g initialY=%g\n", Contacts, Ball->X.y, InitialY);
		EXPECT(Contacts > 0); EXPECT(Ball->X.y > 1.8f); EXPECT(Ball->X.y < InitialY);
		EXPECT(Scene.ApplyDeformableContactDemo(true, false)); EXPECT(!Scene.GetDemoContactRigid()); EXPECT(W->GetForceFieldCount() == Fields);
		EXPECT(Scene.ApplyDeformableContactDemo(true, true)); Ball = Scene.GetDemoContactRigid();
		W->RemoveRigidBody(Ball); Ball->ReleaseGeometries(); delete Ball;
		Scene.Step(1.0f / 60); EXPECT(!Scene.GetDemoContactBody(0)); EXPECT(W->GetForceFieldCount() == Fields);
		EXPECT(Scene.ApplyDeformableContactDemo(true, true)); W->RemoveDeformableBody(Scene.GetDemoContactBody(1));
		Scene.Step(1.0f / 60); EXPECT(!Scene.GetDemoContactRigid()); EXPECT(W->GetDeformableBodies().size() == Bodies);
		EXPECT(Scene.ApplyDeformableContactDemo(true, true)); EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(!Scene.GetDemoContactBody(0)); EXPECT(!Scene.GetDemoContactRigid());
	}
	void CallbackLifetime()
	{
		bool Called = false; PhysicsWorld* Active = nullptr; DeformableBody* Soft = nullptr; RigidBodyDynamic* Rigid = nullptr;
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::PBD;
		P.onTrigger = [&](const PhysicsTriggerInfo&)
		{
			if (Called) return; Called = true;
			Reject([&] { Active->RemoveRigidBody(Rigid); }); Reject([&] { Active->Reset(); });
			Reject([&] { Active->CreateDeformableBody(DeformableBodySettings()); }); Reject([&] { Active->RemoveDeformableBody(Soft); });
			RigidBodyParam R; Reject([&] { Active->CreateRigidBody(R, Transform::Identity()); });
			Reject([&] { Soft->SetCollisionSettings(Soft->GetSettings().Collision); });
			Reject([&] { Active->SetSupportStabilizationSettings(Active->GetSupportStabilizationSettings()); });
			EXPECT(!Active->SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false));
			throw std::runtime_error("coupled contact callback fixture");
		};
		PhysicsWorld W(P); Active = &W; Soft = Triangle(W, Settings(), 0.51f);
		Rigid = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
		auto* Shape = GeometryFactory::CreateSphere(Vector3::Zero(), 1); Shape->SetTriggerEnabled(true); RigidBodyParam R; W.CreateRigidBody(Shape, R);
		Reject([&] { W.Simulate(0.01f); }); EXPECT(Called);
		EXPECT(Soft->AddParticle(Vector3(4)) == 3); EXPECT(W.RemoveRigidBody(Rigid)); Rigid->ReleaseGeometries(); delete Rigid;
		W.Simulate(0.01f); EXPECT(Soft->GetCollisionStats().RigidContacts == 0);
	}
}
void TestDeformableRigidContact()
{
	KernelResponse(); CoupledTangentMass(); SlidingDirection(); KernelPolicies(); WorldMomentum(); ShapesAndFiltering(); MeshFeatureContacts(); MovingSurfaceAndPolicies(); OffsetCenterAndWake(); ExistingDemo(); CallbackLifetime();
}

#include "Test.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	using DVector = Maths::TVector3<double>;
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Substeps = 1; S.SolverIterations = 24;
		S.Gravity = Vector3::Zero(); S.Damping = 0; S.GroundEnabled = false; S.ParticleRadius = 0.02f;
		S.Collision.InterCollision = true; S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0;
		return S;
	}
	void Triangle(DeformableBody& Body, float Height, float Size, float InverseMass = 1)
	{
		Body.AddParticle(Vector3(-Size, Height, -Size), InverseMass);
		Body.AddParticle(Vector3(Size, Height, -Size), InverseMass);
		Body.AddParticle(Vector3(0, Height, Size), InverseMass);
		EXPECT(Body.SetSurfaceTriangles({ Index3(0, 1, 2) }));
	}
	DVector Momentum(const DeformableBody& Body)
	{
		DVector P(0); for (const auto& V : Body.GetParticles()) if (V.InverseMass && !V.Pinned)
			P += DVector(V.Velocity.x, V.Velocity.y, V.Velocity.z) / V.InverseMass;
		return P;
	}
	double Distance(const DeformableBody& A, const DeformableBody& B)
	{
		double Result = std::numeric_limits<double>::infinity();
		for (const auto& TA : A.GetSurfaceTriangles()) for (const auto& TB : B.GetSurfaceTriangles())
		{
			Vector3 PA[3], PB[3]; for (int I = 0; I < 3; ++I) { PA[I] = A.GetParticle(TA[I]).Position; PB[I] = B.GetParticle(TB[I]).Position; }
			Result = std::min(Result, SurfaceCollision::Closest(PA, 3, PB, 3).Distance);
		}
		return Result;
	}
	void FreePair()
	{
		for (bool Reverse : { false, true }) for (int Substeps : { 1, 4 })
		{
			auto S = Settings(); DeformableBody A(S); S.Substeps = Substeps; DeformableBody B(S);
			Triangle(A, 0.03f, 1); Triangle(B, 0, 1, 0.5f);
			for (int I = 0; I < 3; ++I) { A.SetParticleVelocity(I, Vector3(0, -0.1f, 0)); B.SetParticleVelocity(I, Vector3(0, 0.05f, 0)); }
			const auto Group = Reverse ? std::vector<DeformableBody*>{ &B, &A } : std::vector<DeformableBody*>{ &A, &B };
			DeformableBody::SimulateGroup(Group, 0.01f);
			EXPECT(Distance(A, B) >= 0.04 - 2e-5);
			EXPECT_NEAR((Momentum(A) + Momentum(B)).Length(), 0, 5e-5);
			EXPECT(A.GetCollisionStats().InterContacts > 0); EXPECT(B.GetCollisionStats().InterContacts > 0);
			EXPECT(A.GetCollisionStats().SelfContacts == 0);
			const float Center = (A.GetParticle(0).Position.y + B.GetParticle(0).Position.y * 2) / 3;
			EXPECT_NEAR(Center, 0.01, 2e-5);
		}
	}
	void SelfAndInterior()
	{
		auto S = Settings(); S.Collision.InterCollision = false; S.Collision.SelfCollision = true;
		DeformableBody Body(S); Triangle(Body, 0, 1, 0);
		for (int I = 0; I < 3; ++I) Body.AddParticle(Body.GetParticle(I).Position + Vector3(0, 0.03f, 0));
		Body.SetSurfaceTriangles({ Index3(0, 1, 2), Index3(3, 4, 5) });
		Body.Simulate(0.01f);
		for (int I = 3; I < 6; ++I) EXPECT(Body.GetParticle(I).Position.y >= 0.04f - 2e-5f);
		EXPECT(Body.GetCollisionStats().SelfContacts > 0); EXPECT(Body.GetCollisionStats().InterContacts == 0);
		DeformableBody Interior(S); Triangle(Interior, 0, 1);
		Interior.AddParticle(Vector3(0, 0.005f, 0)); Interior.Simulate(0.01f);
		EXPECT_NEAR(Interior.GetParticle(3).Position.y, 0.005, 1e-7);
		EXPECT(Interior.GetCollisionStats().Contacts == 0);
	}
	void EdgeContacts()
	{
		auto S = Settings(); DeformableBody A(S), B(S);
		for (const auto& P : { Vector3(-1, 0.03f, 0), Vector3(1, 0.03f, 0), Vector3(-1, 0.03f, -0.1f) }) A.AddParticle(P);
		for (const auto& P : { Vector3(0, 0, -1), Vector3(0, 0, 1), Vector3(0.1f, 0, -1) }) B.AddParticle(P, 0);
		A.SetSurfaceTriangles({ Index3(0, 1, 2) }); B.SetSurfaceTriangles({ Index3(0, 1, 2) });
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
		EXPECT(A.GetCollisionStats().InterContacts > 0); EXPECT(Distance(A, B) >= 0.04 - 2e-5);
	}
	void FrictionAndCompliance()
	{
		for (double Friction : { 0.0, 1.0 }) for (int Iterations : { 1, 24 })
		{
			auto S = Settings(); S.SolverIterations = Iterations;
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = Friction;
			DeformableBody A(S), B(S); Triangle(A, 0.03f, 0.2f); Triangle(B, 0, 3, 0);
			for (int I = 0; I < 3; ++I) A.SetParticleVelocity(I, Vector3(0.5f, 0, 0));
			DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(A.GetParticle(I).Velocity.x, Friction ? 0 : 0.5, 2e-5);
		}
		for (int Iterations : { 1, 24 })
		{
			auto S = Settings(); S.SolverIterations = Iterations; S.Collision.Material.Compliance = 0.0001;
			DeformableBody A(S), B(S); Triangle(A, 0.03f, 0.2f); Triangle(B, 0, 3, 0);
			DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(A.GetParticle(I).Position.y, 0.035, 2e-6);
		}
	}
	void PolicyAndValidation()
	{
		auto S = Settings(); DeformableBody A(S); S.Collision.InterCollision = false; DeformableBody B(S);
		Triangle(A, 0.03f, 1); Triangle(B, 0, 1);
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f); EXPECT(A.GetCollisionStats().Contacts == 0);
		B.SetCollisionSettings(A.GetSettings().Collision); DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
		EXPECT(A.GetCollisionStats().Contacts > 0); A.Reset(); EXPECT(A.GetCollisionStats().Contacts == 0);
		auto Bad = S.Collision; Bad.ContactTolerance = 0;
		bool Rejected = false; try { A.SetCollisionSettings(Bad); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		S.Collision = Bad; Rejected = false; try { DeformableBody Invalid(S); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		DeformableBody Empty(Settings()); Empty.AddParticle(Vector3(0)); Rejected = false;
		try { Empty.Simulate(0.01f); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		EXPECT(Empty.AddParticle(Vector3(1)) == 1);
		// Replacing topology and growing particle storage must invalidate every
		// old feature index; collision rows cannot outlive their physical substep.
		for (const auto& P : { Vector3(10, 0, 0), Vector3(11, 0, 0), Vector3(10, 0, 1) }) A.AddParticle(P);
		A.SetSurfaceTriangles({ Index3(3, 4, 5) });
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f); EXPECT(A.GetCollisionStats().Contacts == 0);
	}
	void RigidAttachmentContactChain()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld World(P);
			auto S = Settings(); S.SolverIterations = 48; S.Substeps = 4;
			auto* A = World.CreateDeformableBody(S); S.Substeps = 1; auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0, 1); Triangle(*B, 0.03f, 1);
			for (int I = 0; I < 3; ++I) B->SetParticleVelocity(I, Vector3(0, -0.1f, 0));
			const auto Point = BarycentricPoint::Triangle(Index3(0, 1, 2), Vector3(1.0f / 3));
			const Vector3 Anchor = A->GetPointPosition(Point);
			RigidBodyParam R; R.rigidType = RigidType::Dynamic; R.linearDamping = R.angularDamping = 1; R.sleepThreshold = 0;
			auto* Rigid = World.CreateRigidBody(R, Transform(Anchor))->CastDynamic();
			Rigid->InvMass = 0.5f; Rigid->InvInertia = Matrix3(1, 1, 1);
			World.AttachDeformableToRigid(A, Point, Rigid, Anchor);
			World.Simulate(0.01f);
			const DVector Total = Momentum(*A) + Momentum(*B) + DVector(Rigid->V.x, Rigid->V.y, Rigid->V.z) * 2;
			printf("Soft contact attachment chain solver=%d momentumY=%g rigidV=%g gap=%g\n", (int)Solver, Total.y, Rigid->V.y, Distance(*A, *B));
			EXPECT_NEAR((Total - DVector(0, -0.3, 0)).Length(), 0, 2e-4);
			EXPECT(Rigid->V.y < 0); EXPECT(Distance(*A, *B) >= 0.04 - 2e-5);
			EXPECT_NEAR((A->GetPointPosition(Point) - Rigid->X).Length(), 0, 2e-5);
			EXPECT(B->GetCollisionStats().Contacts > 0);
		}
	}
	void WorldRoutes()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam P; PhysicsWorld World(P);
			EXPECT(World.SetSolverConfiguration(Solver, SolverBackend::CPU, false));
			auto S = Settings(); auto* A = World.CreateDeformableBody(S); auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0.03f, 1); Triangle(*B, 0, 1); World.Simulate(0.01f);
			EXPECT(Distance(*A, *B) >= 0.04 - 2e-5); EXPECT_NEAR((Momentum(*A) + Momentum(*B)).Length(), 0, 5e-5);
		}
	}
	void TemporalFrameLifetime()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
			for (int WorldSteps : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = WorldSteps;
			PhysicsWorld World(P); auto S = Settings(); S.Substeps = 2;
			auto* A = World.CreateDeformableBody(S); auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0.03f, 1); Triangle(*B, 0, 1);
			for (int I = 0; I < 3; ++I) A->AddForce(I, Vector3::UnitX());
			World.Simulate(0.04f);
			EXPECT_NEAR((Momentum(*A) + Momentum(*B)).x, 0.12, 2e-4);
			// These surfaces separate during the first physical substep. Statistics
			// must retain that collision even when later world substeps have no rows.
			EXPECT(A->GetCollisionStats().Contacts > 0);
			World.Simulate(0.04f);
			EXPECT_NEAR((Momentum(*A) + Momentum(*B)).x, 0.12, 4e-4);
			EXPECT(A->GetCollisionStats().Contacts == 0);
			S.Collision.InterCollision = false;
			auto* Held = World.CreateDeformableBody(S); Held->AddParticle(Vector3(10, 0, 0));
			const auto Id = Held->AttachToWorld(BarycentricPoint::Vertex(0), Vector3(10, 0, 0));
			Held->AddForce(0, Vector3(0, -3, 0)); World.Simulate(0.04f);
			EXPECT_NEAR(Held->GetAttachmentForce(Id).y, 3, 1e-5);
		}
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		const std::string Path = TestContentsPath() + "Scenes/08_cloth_sphere.json";
		EXPECT(Scene.LoadFromFile(Path, &Error));
		const auto Baseline = Scene.GetSimulation()->GetDeformableBodies().size();
		EXPECT(Scene.ApplyClothDemo(false, false, false, false, false, true, false, false, false, false, false, false, true));
		auto* Lower = Scene.GetDemoContactBody(0); auto* Upper = Scene.GetDemoContactBody(1);
		EXPECT(Lower && Upper); if (!Lower || !Upper) return;
		const auto Center = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
		const auto Anchor = Lower->GetPointPosition(Center); size_t Contacts = 0; double MinimumDistance = 1;
		for (int Frame = 0; Frame < 120; ++Frame)
		{
			Scene.Step(1.0f / 60); Contacts += Upper->GetCollisionStats().InterContacts;
			MinimumDistance = std::min(MinimumDistance, Distance(*Lower, *Upper));
			for (auto* Body : { Lower, Upper }) for (const auto& Particle : Body->GetParticles()) EXPECT(std::isfinite(Particle.Position.SquareLength()));
		}
		printf("Soft contact demo: contacts=%zu minimumDistance=%g anchorError=%g upperY=%g\n", Contacts, MinimumDistance,
			(Lower->GetPointPosition(Center) - Anchor).Length(), Upper->GetPointPosition(Center).y);
		EXPECT(Contacts > 0); EXPECT(MinimumDistance >= 0.07 - 0.002);
		EXPECT((Lower->GetPointPosition(Center) - Anchor).Length() < 0.002);
		EXPECT(Upper->GetPointPosition(Center).y > Lower->GetPointPosition(Center).y);
		EXPECT(Scene.ApplyDeformableAttachmentDemo(true, false));
		Scene.Step(1.0f / 60); EXPECT(Scene.GetDemoContactBody(0) == Lower);
		EXPECT(Scene.ApplyDeformableContactDemo(false)); EXPECT(!Scene.GetDemoContactBody(0));
		EXPECT(Scene.ApplyDeformableAttachmentDemo(false, false));
		EXPECT(Scene.GetSimulation()->GetDeformableBodies().size() == Baseline);
		EXPECT(Scene.ApplyDeformableContactDemo(true));
		Scene.GetSimulation()->RemoveDeformableBody(Scene.GetDemoContactBody(1));
		Scene.Step(1.0f / 60); EXPECT(!Scene.GetDemoContactBody(0));
		EXPECT(Scene.ApplyDeformableContactDemo(true)); EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(!Scene.GetDemoContactBody(0));
	}
}
void TestDeformableCollision()
{
	FreePair(); SelfAndInterior(); EdgeContacts(); FrictionAndCompliance(); PolicyAndValidation(); WorldRoutes(); TemporalFrameLifetime(); RigidAttachmentContactChain(); ExistingDemo();
}

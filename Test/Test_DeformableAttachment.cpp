#include "Test.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <cfloat>
#include <memory>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	using DVector = Maths::TVector3<double>;
	template<class F> void Reject(F Function)
	{
		bool Threw = false; try { Function(); } catch (const std::exception&) { Threw = true; } EXPECT(Threw);
	}
	DeformableBodySettings FreeSettings()
	{
		DeformableBodySettings S; S.Substeps = 1; S.SolverIterations = 12; S.Gravity = Vector3::Zero(); S.Damping = 0; return S;
	}
	void WeightedKernel()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			Vector3 A(2, 3, 4), B[] = { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() };
			SurfaceContactParticle PA{ &A, Scale }, PB[4]; const double WA = 1, WB[] = { 0.1, 0.2, 0.3, 0.4 };
			double Weight = Scale; Vector3 Before = A / Scale;
			for (int I = 0; I < 4; ++I) { PB[I] = { &B[I], Scale * (I + 2) }; Weight += PB[I].InverseMass * WB[I] * WB[I]; Before += B[I] / PB[I].InverseMass; }
			BarycentricAttachmentConstraint C(&PA, &WA, 1, PB, WB, 4); const auto Error = C.GetError();
			EXPECT_NEAR(C.GetEffectiveInverseMass(), Weight, Weight * 1e-7); EXPECT(C.Solve(0.1f));
			EXPECT_NEAR(C.GetError().SquareLength(), 0, 1e-11); const auto Lambda = C.GetMultiplier();
			EXPECT_NEAR((Lambda + Error / Weight).SquareLength(), 0, 1e-12);
			Vector3 After = A / Scale; for (int I = 0; I < 4; ++I) After += B[I] / PB[I].InverseMass;
			EXPECT_NEAR((After - Before).Length(), 0, 1e-5 / Scale);
			for (int I = 0; I < 20; ++I) C.Solve(0.1f);
			EXPECT_NEAR(C.GetError().SquareLength(), 0, 1e-11);
		}
		Vector3 P[] = { Vector3(0), Vector3(4, 0, 0) }; SurfaceContactParticle Particles[] = { { &P[0], 2 }, { &P[1], 4 } };
		const double WA[] = { 0.75, 0.25 }, WB[] = { 0.25, 0.75 };
		BarycentricAttachmentConstraint Shared(Particles, WA, 2, Particles, WB, 2);
		EXPECT_NEAR(Shared.GetEffectiveInverseMass(), 1.5, 1e-12); Shared.Solve(0.1f);
		EXPECT_NEAR(P[0].x, 4.0 / 3, 1e-6); EXPECT_NEAR(P[1].x, 4.0 / 3, 1e-6);
		BarycentricAttachmentConstraint Identical(Particles, WA, 2, Particles, WA, 2);
		EXPECT(!Identical.Solve(0.1f)); EXPECT(Identical.GetEffectiveInverseMass() == 0);
	}
	void ComplianceAndValidation()
	{
		const double Weight = 1, Compliance = 0.001;
		for (float Dt : { 0.000001f, 0.01f, 1.0f })
		{
			Vector3 A(1, 2, 3), Target(0); SurfaceContactParticle PA{ &A, 2 }, PB{ &Target, 0 };
			BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1, Compliance);
			const double Alpha = Compliance / ((double)Dt * Dt); const DVector Expected = DVector(-1, -2, -3) / (2 + Alpha);
			C.Solve(Dt); const Vector3 Once = A;
			EXPECT_NEAR((C.GetMultiplier() - Expected).SquareLength(), 0, 1e-12);
			for (int I = 0; I < 32; ++I) C.Solve(Dt);
			EXPECT_NEAR((A - Once).Length(), 0, 1e-6);
			EXPECT_NEAR((C.GetError() + C.GetMultiplier() * Alpha).SquareLength(), 0, 1e-12);
			EXPECT_NEAR((C.GetForce(Dt) - C.GetMultiplier() / ((double)Dt * Dt)).SquareLength(), 0, 1e-12);
		}
		Vector3 A(1), B(0); SurfaceContactParticle PA{ &A, 0 }, PB{ &B, 0 };
		BarycentricAttachmentConstraint Fixed(&PA, &Weight, 1, &PB, &Weight, 1);
		EXPECT(!Fixed.Solve(0.1f)); EXPECT(A == Vector3(1)); EXPECT(Fixed.GetForce(0.1f).SquareLength() == 0);
		BarycentricAttachmentConstraint PrescribedSpring(&PA, &Weight, 1, &PB, &Weight, 1, Compliance);
		PrescribedSpring.Solve(0.1f); EXPECT(A == Vector3(1)); EXPECT(B == Vector3(0));
		EXPECT_NEAR((PrescribedSpring.GetForce(0.1f) + DVector(1 / Compliance)).SquareLength(), 0, 1e-12);
		Reject([&] { Fixed.Solve(0); }); Reject([&] { Fixed.GetForce(-1); });
		for (double Bad : { -1.0, 0.0, 0.9, std::numeric_limits<double>::quiet_NaN() })
			Reject([&] { BarycentricAttachmentConstraint C(&PA, &Bad, 1, &PB, &Weight, 1); });
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 5, &PB, &Weight, 1); });
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1, -1); });
		SurfaceContactParticle Conflict{ &A, 1 };
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 1, &Conflict, &Weight, 1); });
		PA.InverseMass = 1; BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1);
		B.x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { C.Solve(0.1f); }); EXPECT(A == Vector3(1));
		auto Point = BarycentricPoint::Vertex(0); EXPECT(Point.IsValid(1)); Point.Weights[3] = 1; EXPECT(!Point.IsValid(1));
		Vector3 Large[] = { Vector3(FLT_MAX, 0, 0), Vector3(FLT_MAX, 0, 0) }, Target(-FLT_MAX, 0, 0);
		SurfaceContactParticle Weighted[] = { { &Large[0], 1 }, { &Large[1], 0 } }, Anchor{ &Target, 0 };
		const double SmallWeight[] = { 0.001, 0.999 };
		BarycentricAttachmentConstraint Overflow(Weighted, SmallWeight, 2, &Anchor, &Weight, 1);
		Reject([&] { Overflow.Solve(0.1f); }); EXPECT(Large[0].x == FLT_MAX); EXPECT(Overflow.GetMultiplier().SquareLength() == 0);
	}
	void WorldAnchorAndStorage()
	{
		auto S = FreeSettings(); S.Substeps = 4; S.Gravity = Vector3(0, -9.8f, 0);
		DeformableBody Body(S);
		for (const auto& P : { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }) Body.AddParticle(P);
		Body.AddTetrahedron(0, 1, 2, 3);
		const auto Point = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
		const Vector3 Target = Body.GetPointPosition(Point); const auto Id = Body.AttachToWorld(Point, Target);
		// Reallocation must not invalidate an attachment's stored material point.
		for (int I = 0; I < 128; ++I) Body.AddParticle(Vector3((float)I), 0);
		for (int I = 0; I < 30; ++I) Body.Simulate(1.0f / 60);
		EXPECT_NEAR((Body.GetPointPosition(Point) - Target).Length(), 0, 2e-6);
		EXPECT_NEAR(Body.GetAttachmentForce(Id).y, 4 * 9.8, 0.02);
		EXPECT(Body.SetAttachmentTarget(Id, Target + Vector3::UnitZ())); Body.Simulate(0.1f);
		EXPECT_NEAR((Body.GetPointPosition(Point) - Target - Vector3::UnitZ()).Length(), 0, 2e-6);
		Body.Reset(); EXPECT(Body.GetAttachmentForce(Id).SquareLength() == 0);
		EXPECT(Body.RemoveAttachment(Id)); EXPECT(!Body.RemoveAttachment(Id));
		const auto NewId = Body.AttachToWorld(Point, Target, 0.001); EXPECT(NewId != Id);
		Reject([&] { Body.AttachToWorld(BarycentricPoint::Vertex(999), Target); });
		Reject([&] { Body.GetAttachmentForce(Id); });
	}
	void CoupledBodies()
	{
		for (bool Reverse : { false, true })
		{
			auto S = FreeSettings(); DeformableBody A(S), Independent(S); S.Substeps = 4; DeformableBody B(S);
			A.AddParticle(Vector3(0), 1); B.AddParticle(Vector3(0), 0.5f); Independent.AddParticle(Vector3(0));
			A.SetParticleVelocity(0, Vector3(2, 0, 0)); B.SetParticleVelocity(0, Vector3(-1, 0, 0));
			Independent.AddForce(0, Vector3::UnitY());
			const auto P = BarycentricPoint::Vertex(0); const auto Id = A.AttachToBody(P, &B, P);
			Reject([&] { A.Simulate(0.1f); }); Reject([&] { B.Simulate(0.1f); });
			EXPECT(A.GetParticle(0).Position == Vector3(0)); EXPECT(B.GetParticle(0).Position == Vector3(0));
			DeformableBody::SimulateGroup(Reverse ? std::vector<DeformableBody*>{ &B, &Independent, &A } : std::vector<DeformableBody*>{ &A, &Independent, &B }, 0.1f);
			EXPECT_NEAR((A.GetParticle(0).Position - B.GetParticle(0).Position).Length(), 0, 1e-6);
			EXPECT_NEAR((A.GetParticle(0).Velocity + B.GetParticle(0).Velocity * 2).Length(), 0, 1e-6);
			EXPECT_NEAR(A.GetAttachmentForce(Id).x, -20, 1e-4);
			EXPECT_NEAR(Independent.GetParticle(0).Position.y, 0.01, 1e-7); // One independent substep.
			EXPECT(!A.SetAttachmentTarget(Id, Vector3(1)));
			Reject([&] { DeformableBody::SimulateGroup({ &A, &A, &B }, 0.1f); });
			EXPECT(A.RemoveAttachments(&B) == 1); B.Simulate(0.1f); A.Simulate(0.1f);
		}
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Solver; PhysicsWorld World(Param);
			auto* A = World.CreateDeformableBody(FreeSettings()); auto* B = World.CreateDeformableBody(FreeSettings());
			A->AddParticle(Vector3(0)); B->AddParticle(Vector3(1, 0, 0)); const auto Point = BarycentricPoint::Vertex(0);
			A->AttachToBody(Point, B, Point); B->AttachToBody(Point, A, Point);
			World.Simulate(0.1f); EXPECT_NEAR(A->GetParticle(0).Position.x, 0.5, 1e-6);
			EXPECT_NEAR(B->GetParticle(0).Position.x, 0.5, 1e-6);
			EXPECT(World.RemoveDeformableBody(B)); EXPECT(A->GetAttachmentCount() == 0); World.Simulate(0.1f);
			A->AttachToBody(Point, A, Point); World.Reset(); EXPECT(World.GetDeformableBodyCount() == 0);
		}
	}
	void PinsAndLifetime()
	{
		const auto Point = BarycentricPoint::Vertex(0);
		DeformableBody A(FreeSettings()); A.AddParticle(Vector3(0));
		std::unique_ptr<DeformableBody> B(new DeformableBody(FreeSettings())); B->AddParticle(Vector3(2, 0, 0), 0.5f);
		const auto Id = A.AttachToBody(Point, B.get(), Point);
		A.PinParticle(0, Vector3(0)); DeformableBody::SimulateGroup({ &A, B.get() }, 0.1f);
		EXPECT(A.GetParticle(0).Position == Vector3(0)); EXPECT_NEAR(B->GetParticle(0).Position.Length(), 0, 1e-6);
		EXPECT_NEAR(A.GetAttachmentForce(Id).x, 400, 1e-3);
		A.UnpinParticle(0); A.SetParticlePosition(0, Vector3(0)); B->SetParticlePosition(0, Vector3(2, 0, 0));
		DeformableBody::SimulateGroup({ &A, B.get() }, 0.1f);
		EXPECT_NEAR(A.GetParticle(0).Position.x, 4.0 / 3, 1e-6); EXPECT_NEAR(B->GetParticle(0).Position.x, 4.0 / 3, 1e-6);
		A.AttachToBody(Point, B.get(), Point); EXPECT(A.RemoveAttachment(Id));
		Reject([&] { B->Simulate(0.1f); }); B.reset(); EXPECT(A.GetAttachmentCount() == 0); A.Simulate(0.1f);
		{
			DeformableBody Owner(FreeSettings()); Owner.AddParticle(Vector3(0)); Owner.AttachToBody(Point, &A, Point);
			Reject([&] { A.Simulate(0.1f); });
		}
		A.Simulate(0.1f);
		A.AddParticle(Vector3(4, 0, 0)); A.SetParticlePosition(0, Vector3(0));
		A.AttachToBody(Point, &A, BarycentricPoint::Vertex(1)); A.Simulate(0.1f);
		EXPECT_NEAR(A.GetParticle(0).Position.x, 2, 1e-6); EXPECT_NEAR(A.GetParticle(1).Position.x, 2, 1e-6);
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
		const auto Center = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
		const auto Face = BarycentricPoint::Triangle(Index3(0, 1, 2), Vector3(1.0f / 3));
		float Gaps[2] = {};
		for (int Mode = 0; Mode < 2; ++Mode)
		{
			EXPECT(Scene.ApplyDeformableAttachmentDemo(true, Mode != 0));
			auto* A = Scene.GetDemoDeformableBody(0); auto* B = Scene.GetDemoDeformableBody(1); EXPECT(A && B); if (!A || !B) return;
			const Vector3 Anchor = A->GetPointPosition(Center);
			for (int I = 0; I < 120; ++I) Scene.Step(1.0f / 60);
			Gaps[Mode] = (B->GetPointPosition(Face) - A->GetParticle(3).Position).Length();
			printf("Soft attachment demo compliant=%d gap=%g anchor=%g\n", Mode, Gaps[Mode], (A->GetPointPosition(Center) - Anchor).Length());
			for (auto* Body : { A, B }) for (const auto& P : Body->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()));
			if (!Mode) { EXPECT(Gaps[Mode] < 2e-3f); EXPECT((A->GetPointPosition(Center) - Anchor).Length() < 2e-3f); }
		}
		EXPECT(Gaps[1] > Gaps[0] + 0.01f);
		EXPECT(Scene.ApplyDeformableAttachmentDemo(false, false)); EXPECT(!Scene.GetDemoDeformableBody(0));
		EXPECT(Scene.ApplyDeformableAttachmentDemo(true, false));
		Scene.GetSimulation()->RemoveDeformableBody(Scene.GetDemoDeformableBody(1));
		Scene.Step(1.0f / 60); EXPECT(!Scene.GetDemoDeformableBody(0));
		EXPECT(Scene.ApplyDeformableAttachmentDemo(true, false));
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error)); EXPECT(!Scene.GetDemoDeformableBody(0));
	}
}
void TestDeformableAttachment()
{
	WeightedKernel(); ComplianceAndValidation(); WorldAnchorAndStorage(); CoupledBodies(); PinsAndLifetime(); ExistingDemo();
}

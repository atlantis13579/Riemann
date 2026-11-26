#include "Test.h"
#include "../Src/Deformation/BarycentricAttachment.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include <cfloat>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	using DVector = RigidPointResponse::DVector;
	DVector Double(const Vector3& P) { return DVector(P.x, P.y, P.z); }
	template<class F> void Reject(F Function)
	{
		bool Threw = false; try { Function(); } catch (const std::exception&) { Threw = true; } EXPECT(Threw);
	}
	void Near(const DVector& A, const DVector& B, double Tolerance)
	{
		if (std::abs(A.x - B.x) > Tolerance || std::abs(A.y - B.y) > Tolerance || std::abs(A.z - B.z) > Tolerance)
			printf("Vector mismatch: actual=(%.12g, %.12g, %.12g), expected=(%.12g, %.12g, %.12g), tolerance=%g\n", A.x, A.y, A.z, B.x, B.y, B.z, Tolerance);
		EXPECT_NEAR(A.x, B.x, Tolerance); EXPECT_NEAR(A.y, B.y, Tolerance); EXPECT_NEAR(A.z, B.z, Tolerance);
	}
	void PointResponse()
	{
		Vector3 Center(0); Quaternion Rotation = Quaternion::One();
		RigidConstraintBody Body{ &Center, &Rotation, 0.25, Matrix3(2, 3, 5) };
		RigidPointResponse Response(Body, Vector3(1, 2, 3)); const auto State = Response.Evaluate();
		const double Expected[3][3] = { { 47.25, -10, -9 }, { -10, 23.25, -12 }, { -9, -12, 11.25 } };
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(State.InversePointMass[I][J], Expected[I][J], 1e-12);
		const DVector Impulse(0.3, -0.2, 0.4); const double Epsilon = 0.0001;
		Response.Commit(Response.Prepare(State, Impulse * Epsilon)); const auto Positive = Response.Evaluate().Point;
		Center = Vector3(0); Rotation = Quaternion::One();
		Response.Commit(Response.Prepare(State, Impulse * -Epsilon)); const auto Negative = Response.Evaluate().Point;
		Near((Positive - Negative) / (2 * Epsilon), State.InversePointMass * Impulse, 0.003);
		Near(Response.SolveMass(State.InversePointMass, State.InversePointMass * Impulse), Impulse, 1e-10);
		for (double Scale : { 1e-200, 1e200 })
		{
			auto Scaled = State.InversePointMass;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Scaled[I][J] *= Scale;
			Near(Response.SolveMass(Scaled, Scaled * Impulse), Impulse, 1e-10);
		}
		auto Asymmetric = State.InversePointMass; Asymmetric[0][1] += 1;
		Reject([&] { Response.SolveMass(Asymmetric, Impulse); });
		Center = Vector3(4, -3, 2); Rotation.FromRotationAxis(Vector3(1, 2, -1), 0.7f);
		const auto Transformed = Response.Evaluate(); const auto R = Rotation.ToRotationMatrix3();
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
		{
			double Value = 0;
			for (int K = 0; K < 3; ++K) for (int L = 0; L < 3; ++L) Value += R[I][K] * Expected[K][L] * R[J][L];
			EXPECT_NEAR(Transformed.InversePointMass[I][J], Value, 1e-5);
		}
		Near(Transformed.Point, Double(Center + Rotation * Vector3(1, 2, 3)), 1e-6);
	}
	void CenteredAndCompliant()
	{
		const double Weight = 1;
		for (double RigidMass : { 1e-6, 0.5, 1e6 }) for (double Compliance : { 0.0, 0.001 })
		{
			Vector3 Point(1, 2, 3), Center(0); Quaternion Rotation = Quaternion::One();
			SurfaceContactParticle Particle{ &Point, 1 };
			RigidConstraintBody Body{ &Center, &Rotation, RigidMass, Matrix3::Zero() };
			BarycentricAttachmentConstraint Row(&Particle, &Weight, 1, Body, Vector3(0), Compliance);
			const float Dt = 0.1f; const double Alpha = Compliance / ((double)Dt * Dt);
			const DVector Expected = DVector(-1, -2, -3) / (1 + RigidMass + Alpha);
			Row.Solve(Dt); Near(Row.GetMultiplier(), Expected, 1e-12);
			Near(Double(Point) + Double(Center) / RigidMass, DVector(1, 2, 3), 5e-7);
			Near(Row.GetError() + Row.GetMultiplier() * Alpha, DVector(0), 5e-7);
			const auto Once = Point; for (int I = 0; I < 20; ++I) Row.Solve(Dt);
			Near(Double(Point), Double(Once), 5e-7); Near(Row.GetRigidTorque(Dt), DVector(0), 1e-12);
			EXPECT(Rotation == Quaternion::One()); Reject([&] { Row.GetEffectiveInverseMass(); });
		}
	}
	void EccentricAndReaction()
	{
		Vector3 Center(0), Point(1.3f, 2.2f, 3.1f); Quaternion Rotation = Quaternion::One();
		RigidConstraintBody Body{ &Center, &Rotation, 0.5, Matrix3(2, 3, 5) };
		const Vector3 LocalArm(1, 2, 3); const double Weight = 1; SurfaceContactParticle Particle{ &Point, 1 };
		BarycentricAttachmentConstraint Row(&Particle, &Weight, 1, Body, LocalArm);
		const auto Before = Double(Point); const auto InitialError = Row.GetError(); const auto InitialMass = Row.GetEffectiveInverseMassMatrix();
		const float Dt = 0.1f; Row.Solve(Dt);
		Near(InitialMass * Row.GetMultiplier(), -InitialError, 1e-10);
		DVector Angular = Double(LocalArm).Cross(-Row.GetMultiplier());
		for (int I = 0; I < 30; ++I)
		{
			const auto Arm = RigidPointResponse(Body, LocalArm).Evaluate().Arm; const auto Old = Row.GetMultiplier(); Row.Solve(Dt);
			Angular += Arm.Cross(-(Row.GetMultiplier() - Old));
		}
		Near(Row.GetError(), DVector(0), 2e-6);
		Near(Double(Point) + Double(Center) / Body.InverseMass, Before, 2e-6);
		Near(Row.GetRigidAngularMultiplier(), Angular, 1e-12);
		Near(Row.GetRigidTorque(Dt), Angular / ((double)Dt * Dt), 1e-10);
		const auto FinalArm = RigidPointResponse(Body, LocalArm).Evaluate().Arm;
		EXPECT((Angular - FinalArm.Cross(-Row.GetMultiplier())).SquareLength() > 1e-9);
		EXPECT(std::abs(Rotation.SquareLength() - 1) < 1e-6f);
	}
	void LockedAndLimited()
	{
		Vector3 Center(0); Quaternion Rotation = Quaternion::One();
		RigidConstraintBody Body{ &Center, &Rotation, 0, Matrix3(1, 1, 1) };
		RigidPointResponse Response(Body, Vector3::UnitX()); const auto State = Response.Evaluate();
		Near(Response.SolveMass(State.InversePointMass, DVector(4, 5, 6)), DVector(0, 5, 6), 1e-12);
		Vector3 Point(2, 0, 0); SurfaceContactParticle Particle{ &Point, 0 }; const double Weight = 1;
		BarycentricAttachmentConstraint Unreachable(&Particle, &Weight, 1, Body, Vector3::UnitX());
		EXPECT(!Unreachable.Solve(0.1f)); Near(Unreachable.GetError(), DVector(1, 0, 0), 1e-12);
		BarycentricAttachmentConstraint Spring(&Particle, &Weight, 1, Body, Vector3::UnitX(), 0.001);
		Spring.Solve(0.1f); Near(Spring.GetForce(0.1f), DVector(-1000, 0, 0), 1e-8);
		EXPECT(Center == Vector3(0)); EXPECT(Rotation == Quaternion::One());
		Point = Vector3(1, 100, 0); Particle.InverseMass = 1; Body.InverseMass = 0.01;
		RigidResponseSettings Settings; Settings.MaximumRotation = 0.02;
		BarycentricAttachmentConstraint Limited(&Particle, &Weight, 1, Body, Vector3::UnitX(), 0, Settings);
		const auto Before = Double(Point); Limited.Solve(0.1f);
		const double Angle = 2 * std::atan2(std::hypot(Rotation.x, std::hypot(Rotation.y, Rotation.z)), Rotation.w);
		EXPECT_NEAR(Angle, Settings.MaximumRotation, 1e-7);
		Near(Double(Point) + Double(Center) / Body.InverseMass, Before, 1e-5);
		Near(Limited.GetMultiplier(), DVector(0, -Settings.MaximumRotation, 0), 1e-10);
	}
	void ValidationAndAtomicity()
	{
		Vector3 Center(0), Point(1); Quaternion Rotation = Quaternion::One();
		RigidConstraintBody Body{ &Center, &Rotation, 1, Matrix3(1, 1, 1) };
		SurfaceContactParticle Particle{ &Point, 1 }; const double Weight = 1;
		for (int Mode = 0; Mode < 7; ++Mode)
		{
			auto Bad = Body; auto Settings = RigidResponseSettings();
			if (Mode == 0) Bad.InverseMass = -1;
			if (Mode == 1) Bad.LocalInverseInertia[2][2] = -1;
			if (Mode == 2) Bad.LocalInverseInertia[0][1] = 1;
			if (Mode == 3) Bad.Center = nullptr;
			if (Mode == 4) Settings.RelativeTolerance = 0;
			if (Mode == 5) Settings.MaximumRotation = 0;
			if (Mode == 6) Settings.MaximumDiagonalIterations = 0;
			Reject([&] { BarycentricAttachmentConstraint Row(&Particle, &Weight, 1, Bad, Vector3(0), 0, Settings); });
		}
		SurfaceContactParticle Alias{ &Center, 1 };
		Reject([&] { BarycentricAttachmentConstraint Row(&Alias, &Weight, 1, Body, Vector3(0)); });
		Rotation = Quaternion(0, 0, 0, 0); Reject([&] { RigidPointResponse Response(Body, Vector3(0)); }); Rotation = Quaternion::One();
		BarycentricAttachmentConstraint Invalidated(&Particle, &Weight, 1, Body, Vector3(0));
		Center.x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { Invalidated.Solve(0.1f); }); EXPECT(Point == Vector3(1));
		Center = Point = Vector3(FLT_MAX, 0, 0); Body.LocalInverseInertia = Matrix3::Zero();
		BarycentricAttachmentConstraint Overflow(&Particle, &Weight, 1, Body, Vector3(-FLT_MAX, 0, 0));
		Reject([&] { Overflow.Solve(0.1f); }); EXPECT(Point.x == FLT_MAX); EXPECT(Center.x == FLT_MAX);
		EXPECT(Overflow.GetMultiplier().SquareLength() == 0); EXPECT(Overflow.GetRigidAngularMultiplier().SquareLength() == 0);
		EXPECT(Rotation == Quaternion::One());
	}
	DeformableBodySettings SoftSettings(int Substeps = 1)
	{
		DeformableBodySettings S; S.Substeps = Substeps; S.SolverIterations = 32; S.Gravity = Vector3::Zero(); S.Damping = 0; return S;
	}
	RigidBodyDynamic* Dynamic(PhysicsWorld& World, Vector3 Position = Vector3::Zero(), bool Shape = false)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* Body = (Shape ? World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.5f), P)
			: World.CreateRigidBody(P, Transform(Position)))->CastDynamic();
		Body->InvMass = 0.5f; Body->InvInertia = Matrix3(1, 1, 1); Body->SetRestitution(0); return Body;
	}
	void WorldMomentumAndForces()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD }) for (int Substeps : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld World(P); auto* Rigid = Dynamic(World);
			auto* Soft = World.CreateDeformableBody(SoftSettings(Substeps)); Soft->AddParticle(Vector3(0));
			Soft->SetParticleVelocity(0, Vector3(3, 0, 0));
			const auto Id = World.AttachDeformableToRigid(Soft, BarycentricPoint::Vertex(0), Rigid, Vector3(0));
			Reject([&] { Soft->Simulate(0.1f); });
			World.Simulate(0.1f);
			Near(Double(Soft->GetParticle(0).Position), DVector(0.1, 0, 0), 2e-6);
			Near(Double(Rigid->X), DVector(0.1, 0, 0), 2e-6);
			Near(Double(Soft->GetParticle(0).Velocity) + Double(Rigid->V) * 2, DVector(3, 0, 0), 1e-5);
			Near(World.GetDeformableRigidAttachmentForce(Id), DVector(-20, 0, 0), 1e-4);
			Soft->AddForce(0, Vector3(0, 6, 0)); World.Simulate(0.1f);
			Near(Double(Soft->GetParticle(0).Velocity) + Double(Rigid->V) * 2, DVector(3, 0.6, 0), 1e-5);
			Near(World.GetDeformableRigidAttachmentForce(Id), DVector(0, -4, 0), 1e-4);
			EXPECT(Soft->GetParticle(0).Force == Vector3::Zero()); World.Simulate(0.1f);
			Near(Double(Soft->GetParticle(0).Velocity) + Double(Rigid->V) * 2, DVector(3, 0.6, 0), 1e-5);
			EXPECT(World.RemoveDeformableRigidAttachment(Id)); EXPECT(!World.RemoveDeformableRigidAttachment(Id));
		}
	}
	void WorldSharedRigidAndOffset()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld World(P); auto* Rigid = Dynamic(World);
		Rigid->CM = Vector3(3, -2, 1);
		auto* A = World.CreateDeformableBody(SoftSettings(2)); auto* B = World.CreateDeformableBody(SoftSettings(5));
		A->AddParticle(Vector3::UnitX()); B->AddParticle(-Vector3::UnitX());
		A->SetParticleVelocity(0, Vector3::UnitY()); B->SetParticleVelocity(0, -Vector3::UnitY());
		const auto Point = BarycentricPoint::Vertex(0);
		const auto IA = World.AttachDeformableToRigid(A, Point, Rigid, Vector3::UnitX());
		const auto IB = World.AttachDeformableToRigid(B, Point, Rigid, -Vector3::UnitX());
		constexpr float FrameDt = 0.05f, SubstepDt = FrameDt / 5;
		World.Simulate(FrameDt);
		Near(Double(A->GetParticle(0).Position + B->GetParticle(0).Position + Rigid->X * 2), DVector(0), 1e-6);
		// Reconstructing float positions at unit distance resolves momentum to
		// approximately total mass * position epsilon / substep duration.
		constexpr double TotalMass = 4, PositionScale = 1;
		const double MomentumTolerance = TotalMass * PositionScale * std::numeric_limits<float>::epsilon() / SubstepDt;
		Near(Double(A->GetParticle(0).Velocity) + Double(B->GetParticle(0).Velocity) + Double(Rigid->V) * 2, DVector(0), MomentumTolerance);
		EXPECT(Rigid->W.z > 0.1f); EXPECT(Rigid->Q.z > 0.001f);
		Near(Double(A->GetParticle(0).Position), Double(Rigid->X + Rigid->Q * Vector3::UnitX()), 1e-5);
		EXPECT(World.GetDeformableRigidAttachmentTorque(IA).z > 0); EXPECT(World.GetDeformableRigidAttachmentTorque(IB).z > 0);
		EXPECT(World.RemoveDeformableBody(A)); EXPECT(World.GetDeformableRigidAttachmentCount() == 1);
		EXPECT(World.RemoveRigidBody(Rigid)); EXPECT(World.GetDeformableRigidAttachmentCount() == 0); delete Rigid;
		World.Simulate(0.05f);
	}
	void WorldContactAndGuards()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; P.islandSleeping.Enabled = true;
		P.narrowphase = NarrowPhaseSolver::PCM; PhysicsWorld World(P);
		World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, -0.5f, 0), Vector3(5, 0.5f, 5), Quaternion::One()), RigidBodyParam());
		auto* Rigid = Dynamic(World, Vector3(0, 0.5f, 0), true);
		auto Settings = SoftSettings(4); Settings.Gravity = Vector3(0, -9.8f, 0);
		constexpr double Compliance = 0.001;
		const auto Anchor = Rigid->X + Vector3::UnitY();
		// Start at the spring's loaded equilibrium. A zero-extension undamped
		// spring instead measures transient oscillation, not its static weight.
		auto* Soft = World.CreateDeformableBody(Settings); Soft->AddParticle(Anchor + Settings.Gravity * (float)Compliance);
		const auto Point = BarycentricPoint::Vertex(0); const auto Id = World.AttachDeformableToRigid(Soft, Point, Rigid, Anchor, Compliance);
		for (int I = 0; I < 120; ++I) World.Simulate(1.0f / 60);
		EXPECT(Rigid->X.y > 0.49f); EXPECT(!Rigid->Sleeping);
		EXPECT_NEAR(Soft->GetParticle(0).Position.y, Rigid->X.y + 1 - 0.0098, 0.002);
		printf("Compliant load: force=%g rigidY=%g softY=%g\n", World.GetDeformableRigidAttachmentForce(Id).y, Rigid->X.y, Soft->GetParticle(0).Position.y);
		EXPECT_NEAR(World.GetDeformableRigidAttachmentForce(Id).y, 9.8, 0.05);
		std::string Error; EXPECT(!World.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false, &Error)); EXPECT(!Error.empty());
		SupportStabilizationSettings Support; Support.Enabled = true; Reject([&] { World.SetSupportStabilizationSettings(Support); });
		Rigid->mMotionType = MorionType::Continuous; const auto Before = Rigid->X; Reject([&] { World.Simulate(0.1f); }); EXPECT(Rigid->X == Before);
		Rigid->mMotionType = MorionType::Discrete;
		PhysicsWorld Other(P); auto* Foreign = Dynamic(Other); Reject([&] { World.AttachDeformableToRigid(Soft, Point, Foreign, Vector3(0)); });
		Reject([&] { World.AttachDeformableToRigid(Soft, BarycentricPoint::Vertex(3), Rigid, Vector3(0)); });
		EXPECT(World.RemoveDeformableRigidAttachment(Id)); EXPECT(World.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false, &Error));
		Reject([&] { World.AttachDeformableToRigid(Soft, Point, Rigid, Vector3(0)); });
		World.Reset(); EXPECT(World.GetDeformableRigidAttachmentCount() == 0);
	}
	void PrescribedAndLinkedEndpoints()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld World(P);
		auto* Rigid = World.CreateRigidBody(RigidBodyParam(), Transform(Vector3(0, 2, 0)));
		auto S = SoftSettings(4); S.Gravity = Vector3(0, -9.8f, 0);
		auto* A = World.CreateDeformableBody(S); auto* B = World.CreateDeformableBody(S);
		A->AddParticle(Rigid->X, 1); B->AddParticle(Rigid->X, 0.5f);
		const auto Point = BarycentricPoint::Vertex(0); A->AttachToBody(Point, B, Point);
		const auto Id = World.AttachDeformableToRigid(A, Point, Rigid, Rigid->X);
		// Use a 10 ms physical substep: reaction reconstruction divides float
		// position corrections by h^2, magnifying quantization at shorter steps.
		constexpr float FrameDt = 0.04f;
		for (int I = 0; I < 10; ++I) World.Simulate(FrameDt);
		EXPECT_NEAR(A->GetParticle(0).Position.y, 2, 1e-6); EXPECT_NEAR(B->GetParticle(0).Position.y, 2, 1e-6);
		printf("Prescribed chain load: force=%g softY=%g\n", World.GetDeformableRigidAttachmentForce(Id).y, B->GetParticle(0).Position.y);
		EXPECT_NEAR(World.GetDeformableRigidAttachmentForce(Id).y, 29.4, 0.05);
		Rigid->X.y += 0.1f; World.Simulate(0.01f); EXPECT_NEAR(A->GetParticle(0).Position.y, 2.1, 1e-6);
		DeformableBody External(S); External.AddParticle(Vector3(0)); B->AttachToBody(Point, &External, Point);
		const auto Before = A->GetParticle(0).Position; Reject([&] { World.Simulate(0.01f); }); EXPECT(A->GetParticle(0).Position == Before);
		B->RemoveAttachments(&External); World.Simulate(0.01f);
		World.Reset(); EXPECT(World.GetDeformableRigidAttachmentCount() == 0);
	}
	void CoupledContactBackends()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend))
			{
				printf("Rigid-soft contact solver=%d backend=%d unavailable\n", (int)Solver, (int)Backend); continue;
			}
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.solverBackend = Backend; P.allowBackendFallback = false;
			P.gravityAcc = Vector3(0, -9.8f, 0); P.pbdSettings.PositionIterations = P.xpbdSettings.PositionIterations = 8;
			PhysicsWorld World(P);
			World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, -0.5f, 0), Vector3(5, 0.5f, 5)), RigidBodyParam());
			auto* Rigid = Dynamic(World, Vector3(0, 0.5f, 0), true);
			auto S = SoftSettings(2); S.Gravity = P.gravityAcc; S.SolverIterations = 8;
			auto* Soft = World.CreateDeformableBody(S); Soft->AddParticle(Vector3(0, 1.5f, 0));
			const auto Id = World.AttachDeformableToRigid(Soft, BarycentricPoint::Vertex(0), Rigid, Vector3(0, 1.5f, 0));
			Soft->AddImpulse(0, Vector3(0, -2, 0));
			for (int I = 0; I < 32; ++I) World.Simulate(1.0f / 60);
			const auto& Stats = World.GetPositionSolver()->GetBackendStatistics();
			EXPECT(Stats.Backend == Backend && !Stats.UsedFallback);
			EXPECT(Stats.PositionDispatches > 0 && Stats.ReconstructionDispatches > 0 && Stats.VelocityDispatches > 0);
			EXPECT(Rigid->X.y > 0.49f);
			Near(Double(Soft->GetParticle(0).Position), Double(Rigid->BodyLocalToWorld(Vector3::UnitY())), 1e-4);
			EXPECT(World.GetDeformableRigidAttachmentForce(Id).y > 1);
			printf("Rigid-soft contact solver=%d backend=%d force=%g bodyY=%.9g softY=%.9g bodyV=%.9g softV=%.9g iterations=%d\n", (int)Solver, (int)Backend,
				World.GetDeformableRigidAttachmentForce(Id).y, Rigid->X.y, Soft->GetParticle(0).Position.y, Rigid->V.y, Soft->GetParticle(0).Velocity.y,
				World.GetPositionSolver()->GetSettings().PositionIterations);
			EXPECT_NEAR(World.GetDeformableRigidAttachmentForce(Id).y, 9.8, 0.1);
		}
	}
	void CoupledCallbackLifetime()
	{
		PhysicsWorld* Active = nullptr; DeformableBody* Soft = nullptr; RigidBodyDynamic* Rigid = nullptr;
		DeformableBody External(SoftSettings()); External.AddParticle(Vector3(0));
		const auto Point = BarycentricPoint::Vertex(0); uint64_t Attachment = 0;
		bool Checked = false, FailCallback = false;
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD;
		P.onTrigger = [&](const PhysicsTriggerInfo&)
		{
			if (FailCallback) throw std::runtime_error("Test callback failure");
			if (Checked) return; Checked = true;
			Reject([&] { Soft->AddParticle(Vector3(0)); });
			Reject([&] { Soft->PinParticle(0, Vector3(0)); });
			Reject([&] { Soft->ClearAttachments(); });
			Reject([&] { External.AttachToBody(Point, Soft, Point); });
			Reject([&] { Active->CreateDeformableBody(SoftSettings()); });
			Reject([&] { Active->RemoveDeformableBody(Soft); });
			Reject([&] { Active->RemoveRigidBody(Rigid); });
			Reject([&] { Active->RemoveDeformableRigidAttachment(Attachment); });
			Reject([&] { Active->Reset(); });
			Reject([&] { Active->Simulate(0.01f); });
			std::string Error;
			EXPECT(!Active->SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false, &Error));
		};
		PhysicsWorld World(P); Active = &World; Rigid = Dynamic(World, Vector3(0), true);
		Soft = World.CreateDeformableBody(SoftSettings()); Soft->AddParticle(Vector3(0));
		Attachment = World.AttachDeformableToRigid(Soft, Point, Rigid, Vector3(0));
		auto* Trigger = GeometryFactory::CreateSphere(Vector3(0.2f, 0, 0), 1); Trigger->SetTriggerEnabled(true);
		World.CreateRigidBody(Trigger, RigidBodyParam());
		World.Simulate(0.01f); EXPECT(Checked); EXPECT(Soft->GetParticleCount() == 1); EXPECT(External.GetAttachmentCount() == 0);
		FailCallback = true; Reject([&] { World.Simulate(0.01f); });
		// Stack unwinding must release borrowed particle storage and world guards.
		EXPECT(Soft->AddParticle(Vector3::UnitX()) == 1); EXPECT(World.RemoveDeformableRigidAttachment(Attachment));
		FailCallback = false; World.Simulate(0.01f); World.Reset();
		EXPECT(World.GetDeformableRigidAttachmentCount() == 0);
	}
}
void TestRigidAttachment()
{
	PointResponse(); CenteredAndCompliant(); EccentricAndReaction(); LockedAndLimited(); ValidationAndAtomicity();
	WorldMomentumAndForces(); WorldSharedRigidAndOffset(); WorldContactAndGuards();
	PrescribedAndLinkedEndpoints();  CoupledContactBackends(); CoupledCallbackLifetime();
}

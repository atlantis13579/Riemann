#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Viewer/SceneWorld.h"
#include <stdexcept>

namespace
{
	using namespace Riemann;
	const float Dt = 1.0f / 60.0f;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	class TorqueField : public ForceField
	{
	public:
		bool ApplyForce(RigidBodyDynamic* B) override { B->ApplyTorgue(Vector3(0, 0, 4)); return true; }
	};
	RigidBody* Body(PhysicsWorld& World, const Vector3& X, bool Dynamic, bool Collision = false)
	{
		RigidBodyParam P; P.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* Shape = GeometryFactory::CreateSphere(X, 0.5f); Shape->SetSimulationEnabled(Collision);
		auto* Result = World.CreateRigidBody(Shape, P);
		if (Dynamic) { Result->InvMass = 1; Result->InvInertia = Matrix3(1, 1, 1); }
		Result->SetRestitution(0);
		return Result;
	}
	JointBreakSettings BreakAt(float Force, float Torque = std::numeric_limits<float>::max())
	{
		JointBreakSettings Settings; Settings.Enabled = true; Settings.Force = Force; Settings.Torque = Torque;
		return Settings;
	}
	PositionConstraint* Joint(PhysicsWorld& World, int Type, RigidBody* A, RigidBody* B)
	{
		switch (Type)
		{
		case 0: return World.CreateFixedConstraint(A, B);
		case 1: return World.CreatePointConstraint(A, B, B->X);
		case 2: return World.CreateDistanceConstraint(A, B, A->X, B->X, 1, 1);
		case 3: return World.CreateHingeConstraint(A, B, B->X, Vector3::UnitY(), -1, 1);
		case 4: return World.CreateSliderConstraint(A, B, B->X, Vector3::UnitX(), -1, 1);
		case 5: return World.CreateConeConstraint(A, B, B->X, Vector3::UnitY(), Vector3::UnitY(), 0);
		default: return World.CreateSixDOFConstraint(A, B, B->X, SixDOFConstraintSettings());
		}
	}
	void TestImpulseReports()
	{
		for (int Type = 0; Type < 7; ++Type)
		for (int Iterations : { 1, 32 })
		for (float Step : { Dt / 2, Dt * 2 })
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* A = Body(World, Vector3::Zero(), false), * B = Body(World, Vector3::UnitY(), true);
			PositionConstraint* C = Joint(World, Type, A, B);
			C->BeginStep(Step, false); B->V = Vector3(0, -2, 0);
			for (int I = 0; I < Iterations; ++I) C->SolveVelocity(Step);
			EXPECT(!C->EndStep());
			EXPECT((C->GetReaction().Force - Vector3(0, 2 / Step, 0)).Length() < 0.001f);
			EXPECT(C->GetReaction().Torque.Length() < 0.001f);
			C->BeginStep(Step, false); B->W = Vector3::UnitZ();
			for (int I = 0; I < Iterations; ++I) C->SolveVelocity(Step);
			C->EndStep();
			const float ExpectedTorque = Type == 1 || Type == 2 ? 0 : -1 / Step;
			EXPECT_NEAR(C->GetReaction().Torque.z, ExpectedTorque, 0.001f);
			C->BeginStep(Step, false); C->EndStep();
			EXPECT(C->GetReaction().Force == Vector3::Zero() && C->GetReaction().Torque == Vector3::Zero());
		}
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* A = Body(World, Vector3::Zero(), false), * B = Body(World, Vector3::Zero(), true);
		PointConstraint Offset(A, B, Vector3::UnitX());
		Offset.SetBreakSettings(BreakAt(1000, 0)); Offset.BeginStep(Dt, false); B->V = Vector3(0, -2, 0);
		Offset.SolveVelocity(Dt); EXPECT(!Offset.EndStep());
		EXPECT_NEAR(Offset.GetReaction().Force.y, 1 / Dt, 0.001f);
		EXPECT(Offset.GetReaction().Torque == Vector3::Zero());
		EXPECT_NEAR(B->W.z, 1, 0.001f); // r x F changes angular momentum, without torsional joint load.
		FixedConstraint Fixed(A, B); Fixed.SetBreakSettings(BreakAt(1, 1));
		B->V = Vector3(0, -2, 0); B->W = Vector3::Zero(); Fixed.BeginStep(Dt, false);
		Fixed.SolveVelocity(Dt); B->V.y += 2; Fixed.SolveVelocity(Dt);
		EXPECT(!Fixed.EndStep()); EXPECT(Fixed.GetReaction().Force.Length() < 0.001f);
		Fixed.BeginStep(Dt, false); B->V = Vector3(0, -2, 0); Fixed.SolveVelocity(Dt);
		EXPECT(Fixed.EndStep()); EXPECT(!Fixed.EndStep()); EXPECT(Fixed.IsBroken() && !Fixed.IsEnabled());
		Fixed.SetEnabled(true); EXPECT(!Fixed.IsEnabled());
		B->V = Vector3(0, 3, 0); Fixed.SolveVelocity(Dt); EXPECT(B->V == Vector3(0, 3, 0));
		Fixed.Repair(); EXPECT(Fixed.IsEnabled());
		const auto Good = Fixed.GetBreakSettings(); auto Bad = Good; Bad.Force = -1;
		bool Rejected = false; try { Fixed.SetBreakSettings(Bad); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected && Fixed.GetBreakSettings().Force == Good.Force);
		// Report the load on B even when B is an immovable support.
		PointConstraint Reversed(B, A, Vector3::Zero()); B->V = Vector3(0, -2, 0); B->W = Vector3::Zero();
		Reversed.BeginStep(Dt, false); Reversed.SolveVelocity(Dt); Reversed.EndStep();
		EXPECT_NEAR(Reversed.GetReaction().Force.y, -2 / Dt, 0.001f);
		HingeConstraint Motor(A, B, Vector3::Zero(), Vector3::UnitY(), -1, 1);
		ScalarJointDrive Drive; Drive.Settings.PositionEnabled = true; Drive.Settings.Stiffness = 100;
		Drive.Settings.Damping = 20; Drive.Settings.MaxForce = 6; Drive.TargetPosition = 1;
		Motor.SetDrive(Drive); Motor.SetBreakSettings(BreakAt(1000, 5)); Motor.BeginStep(Dt, false);
		for (int I = 0; I < 32; ++I) Motor.SolveVelocity(Dt);
		EXPECT(Motor.EndStep()); EXPECT_NEAR(Motor.GetReaction().Torque.y, 6, 0.001f);
		EXPECT(Motor.GetReaction().Force.Length() < 0.001f);
	}
	void TestPositionReports()
	{
		for (bool XPBD : { false, true })
		for (bool PhysicalProjection : { false, true })
		for (float Step : { Dt / 2, Dt * 2 })
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* A = Body(World, Vector3::Zero(), false), * B = Body(World, Vector3::Zero(), true);
			PointConstraint C(A, B, Vector3::Zero()); C.SetCompliance(0.01f, 0);
			C.BeginStep(Step, XPBD, true, PhysicalProjection); B->X.y = -0.1f;
			for (int I = 0; I < 32; ++I) C.SolvePosition(Step);
			C.EndStep();
			const float Expected = XPBD ? 0.1f / (Step * Step + 0.01f) : PhysicalProjection ? 0.1f / (Step * Step) : 0;
			EXPECT_NEAR(C.GetReaction().Force.y, Expected, 0.002f);
		}
		for (int Count : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; P.gravityAcc = Vector3(0, -10, 0);
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Count;
			PhysicsWorld World(P);
			// Fields supply SI loads after the legacy queued-impulse conversion in PreIntegrate.
			World.AddForceField(new TorqueField());
			auto* A = Body(World, Vector3::Zero(), false), * B = Body(World, Vector3::Zero(), true);
			auto* C = World.CreateFixedConstraint(A, B); C->SetCompliance(0.005f, 0.005f);
			// The corresponding scalar spring has a closed implicit-Euler recurrence.
			// More substeps reduce numerical damping, so after the same six seconds
			// the refined spring can still oscillate; static equilibrium is not the oracle.
			const double H = (double)Dt / Count, Compliance = 0.005;
			double ExpectedY = 0, ExpectedVelocity = 0, ExpectedForce = 0;
			for (int Frame = 0; Frame < 360; ++Frame)
			{
				World.Simulate(Dt); ExpectedForce = 0;
				for (int Step = 0; Step < Count; ++Step)
				{
					ExpectedVelocity = (ExpectedVelocity - 10 * H - ExpectedY * H / Compliance) / (1 + H * H / Compliance);
					ExpectedY += ExpectedVelocity * H;
					ExpectedForce -= ExpectedY / (Compliance * Count);
				}
			}
			printf("XPBD joint reaction force=%g torque=%g sag=%g\n", C->GetReaction().Force.y, C->GetReaction().Torque.z, B->X.y);
			EXPECT_NEAR(C->GetReaction().Force.y, ExpectedForce, 0.02f);
			EXPECT_NEAR(C->GetReaction().Torque.z, -ExpectedForce * 0.4, 0.02f);
			EXPECT_NEAR(B->X.y, ExpectedY, 0.001f);
		}
	}
	void TestWorldBreaking()
	{
		for (auto Solver : Solvers)
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
			int Events = 0; PhysicsJointBreakInfo Snapshot{};
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.solverBackend = Backend; P.allowBackendFallback = false;
			P.jointBreaking = false;
			P.onJointBreak = [&](const PhysicsJointBreakInfo& Info)
			{
				++Events; Snapshot = Info;
				EXPECT(Info.Constraint->IsBroken() && Info.Constraint->GetId() == Info.ConstraintId);
			};
			PhysicsWorld World(P);
			auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
			auto* B = Body(World, Vector3(0, 0.5f, 0), true, true);
			Floor->SetRestitution(0);
			auto* C = World.CreateSliderConstraint(Floor, B, B->X, Vector3::UnitY(), -2, 2);
			ScalarJointDrive Drive; Drive.Settings.PositionEnabled = true; Drive.Settings.Stiffness = 100;
			Drive.Settings.Damping = 20; Drive.Settings.MaxForce = 6; Drive.TargetPosition = -1;
			C->SetDrive(Drive); C->SetBreakSettings(BreakAt(5));
			for (int Frame = 0; Frame < 3; ++Frame) World.Simulate(Dt);
			EXPECT_NEAR(C->GetReaction().Force.y, -6, 0.01f);
			EXPECT(!C->IsBroken() && Events == 0);
			World.SetJointBreakingEnabled(true); World.Simulate(Dt);
			EXPECT(Events == 1 && C->IsBroken()); EXPECT_NEAR(Snapshot.Reaction.Force.y, -6, 0.01f);
			EXPECT(Snapshot.BodyA == Floor && Snapshot.BodyB == B && Snapshot.Tick == 4 && Snapshot.DeltaTime == Dt);
			for (int Frame = 0; Frame < 3; ++Frame) World.Simulate(Dt);
			EXPECT(Events == 1 && C->GetReaction().Force == Vector3::Zero());
			World.SetJointBreakingEnabled(false); C->Repair(); World.Simulate(Dt);
			EXPECT(!C->IsBroken() && Events == 1);
			World.SetJointBreakingEnabled(true); World.Simulate(Dt);
			EXPECT(C->IsBroken() && Events == 2);
			printf("Joint break solver=%d backend=%d force=%g events=%d\n", (int)Solver, (int)Backend, Snapshot.Reaction.Force.y, Events);
		}
	}
	void TestEventLifetime()
	{
		for (int Count : { 1, 4 })
		{
			PhysicsWorld* Active = nullptr; RigidBody* OtherBody = nullptr; PositionConstraint* Replacement = nullptr;
			int Events = 0; uint64_t FirstId = 0;
			PhysicsWorldParam P; P.gravityAcc = Vector3(0, -10, 0);
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Count;
			P.onJointBreak = [&](const PhysicsJointBreakInfo& Info)
			{
				++Events; FirstId = Info.ConstraintId;
				EXPECT_NEAR(Info.Reaction.Force.y, 10, 0.001f);
				EXPECT(Active->RemoveConstraint(Info.Constraint));
				EXPECT(Active->RemoveRigidBody(OtherBody)); // Removes another queued event's constraint.
				Replacement = Active->CreateFixedConstraint(Info.BodyA, Info.BodyB);
			};
			PhysicsWorld World(P); Active = &World;
			auto* A = Body(World, Vector3::Zero(), false), * B = Body(World, Vector3::Zero(), true);
			OtherBody = Body(World, Vector3(3, 0, 0), true);
			auto* First = World.CreateFixedConstraint(A, B); First->SetBreakSettings(BreakAt(5));
			auto* Second = World.CreateFixedConstraint(A, OtherBody); Second->SetBreakSettings(BreakAt(5));
			const uint64_t SecondId = Second->GetId();
			World.Simulate(Dt);
			EXPECT(Events == 1 && World.GetConstraintCount() == 1);
			EXPECT(Replacement && Replacement->GetId() != FirstId && Replacement->GetId() != SecondId && !Replacement->IsBroken());
			World.Simulate(Dt); EXPECT(Events == 1);
		}
	}
	void TestChainLoads()
	{
		for (auto Solver : Solvers)
		for (bool Unified : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.gravityAcc = Vector3(0, -10, 0);
			P.unifiedConstraints = Unified;
			P.constraintIterations = 32; P.jointBreaking = false;
			int Events = 0; P.onJointBreak = [&](const PhysicsJointBreakInfo&) { ++Events; };
			PhysicsWorld World(P);
			auto* Support = Body(World, Vector3(0, 3, 0), false);
			auto* Upper = Body(World, Vector3(0, 2, 0), true), * Lower = Body(World, Vector3(0, 1, 0), true);
			auto* First = World.CreateFixedConstraint(Support, Upper);
			auto* Second = World.CreateFixedConstraint(Upper, Lower);
			First->SetBreakSettings(BreakAt(15)); Second->SetBreakSettings(BreakAt(12));
			World.Simulate(Dt);
			EXPECT_NEAR(First->GetReaction().Force.y, 20, 0.01f);
			EXPECT_NEAR(Second->GetReaction().Force.y, 10, 0.01f);
			World.SetJointBreakingEnabled(true); World.Simulate(Dt);
			EXPECT(First->IsBroken() && !Second->IsBroken() && Events == 1);
			for (int Frame = 0; Frame < 10; ++Frame) World.Simulate(Dt);
			EXPECT(Second->GetReaction().Force.Length() < 0.01f && Events == 1);
			EXPECT(Upper->V.y < -1 && Lower->V.y < -1);
		}
	}
	void TestConfigurationAndDemo()
	{
		SceneWorld Scene; std::string Error;
		EXPECT(!Scene.LoadFromText(R"({"physics":{"jointBreaking":1}})", "bad break switch", &Error));
		EXPECT(Error.find("jointBreaking") != std::string::npos);
		for (const char* Settings : { "false", R"({"enabled":1})", R"({"force":-1})", R"({"torque":"high"})", R"({"force":1e100})" })
		{
			const std::string Text = std::string(R"({"objects":[{"id":"a","type":"sphere"},{"id":"b","type":"sphere","body":"dynamic"}],"constraints":[{"type":"fixed","bodyA":"a","bodyB":"b","breaking":)") + Settings + "}]}";
			EXPECT(!Scene.LoadFromText(Text, "bad joint break", &Error));
			EXPECT(Error.find("breaking") != std::string::npos);
		}
		for (auto Solver : Solvers)
		{
			EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/06_constraints.json", &Error));
			auto* World = Scene.GetSimulation(); EXPECT(!World->GetJointBreakingEnabled());
			EXPECT(World->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			EXPECT(!World->GetJointBreakingEnabled());
			Scene.Step(Dt); EXPECT(!Scene.GetConstraintControls()[0].Broken);
			World->SetJointBreakingEnabled(true); Scene.Step(Dt);
			auto State = Scene.GetConstraintControls()[0]; EXPECT(State.Broken && !State.Enabled && State.ReactionForce > 5);
			for (int Frame = 0; Frame < 60; ++Frame) Scene.Step(Dt);
			for (const auto& Object : Scene.GetObjects())
				if (Object.Id == "fixed_body") EXPECT(Object.GeometryPtr->GetParent<RigidBody>()->X.y < 5);
			World->SetJointBreakingEnabled(false); State.RepairRequested = true; State.Enabled = true;
			EXPECT(Scene.ApplyConstraintControl(State)); EXPECT(!Scene.GetConstraintControls()[0].Broken);
			Scene.Step(Dt); EXPECT(Scene.GetConstraintControls()[0].Enabled);
		}
	}
}

void TestJointReactions()
{
	TestImpulseReports(); TestPositionReports(); TestWorldBreaking(); TestEventLifetime(); TestChainLoads(); TestConfigurationAndDemo();
}

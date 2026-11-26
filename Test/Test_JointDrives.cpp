#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include <limits>
#include <stdexcept>
#include "../Src/RigidBodyDynamics/ForceField.h"

namespace
{
	using namespace Riemann;
	const float Dt = 1.0f / 60.0f;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	RigidBody* Body(PhysicsWorld& World, const Vector3& Position, bool Dynamic, bool Collision = false)
	{
		RigidBodyParam P; P.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		P.linearDamping = P.angularDamping = 1.0f; P.sleepThreshold = 0;
		Geometry* Shape = GeometryFactory::CreateSphere(Position, 0.5f); Shape->SetSimulationEnabled(Collision);
		RigidBody* Result = World.CreateRigidBody(Shape, P);
		if (Dynamic) { Result->InvMass = 1; Result->InvInertia = Matrix3(1, 1, 1); }
		return Result;
	}
	ScalarJointDrive Spring(float Target, float Limit = 1000.0f)
	{
		ScalarJointDrive Drive;
		Drive.Settings.PositionEnabled = true; Drive.Settings.Stiffness = 60; Drive.Settings.Damping = 16;
		Drive.Settings.MaxForce = Limit; Drive.TargetPosition = Target;
		return Drive;
	}
	void TestScalarDrives()
	{
		for (float InverseMass : { 1.0f, 0.1f })
		for (bool Acceleration : { false, true })
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* A = Body(World, Vector3::Zero(), false);
			auto* B = Body(World, Vector3::Zero(), true); B->InvMass = InverseMass;
			SliderConstraint Joint(A, B, Vector3::Zero(), Vector3::UnitX(), -10, 10);
			auto Drive = Spring(1); Drive.Settings.Acceleration = Acceleration;
			Joint.SetDrive(Drive); Joint.BeginStep(Dt, false);
			Joint.SolveVelocity(Dt);
			const float ResponseMass = Acceleration ? 1.0f : InverseMass;
			const float Expected = Dt * Drive.Settings.Stiffness * ResponseMass
				/ (1.0f + Dt * (Drive.Settings.Damping + Dt * Drive.Settings.Stiffness) * ResponseMass);
			EXPECT_NEAR(B->V.x, Expected, 1e-5f);
			for (int I = 0; I < 32; ++I) Joint.SolveVelocity(Dt);
			EXPECT_NEAR(B->V.x, Expected, 1e-5f);
			// A motor has one force budget per step, regardless of solver sweeps.
			Drive.Settings.MaxForce = 6; Joint.SetDrive(Drive); Joint.BeginStep(Dt, false); B->V = Vector3::Zero();
			for (int I = 0; I < 32; ++I) Joint.SolveVelocity(Dt);
			EXPECT_NEAR(Joint.GetDriveImpulse(), Drive.Settings.MaxForce * Dt, 1e-6f);
			EXPECT_NEAR(B->V.x, Drive.Settings.MaxForce * Dt * InverseMass, 1e-6f);
			Drive.Settings.PositionEnabled = false; Drive.Settings.VelocityEnabled = true;
			Drive.Settings.MaxForce = 1000; Drive.TargetVelocity = 2;
			Joint.SetDrive(Drive); Joint.BeginStep(Dt, false); B->V = Vector3::Zero(); Joint.SolveVelocity(Dt);
			EXPECT_NEAR(B->V.x, 2 * Dt * Drive.Settings.Damping * ResponseMass / (1 + Dt * Drive.Settings.Damping * ResponseMass), 1e-5f);
			Joint.BeginStep(Dt, false, false); B->V = Vector3::Zero(); Joint.SolveVelocity(Dt);
			EXPECT(B->V == Vector3::Zero());
			// Invalid tuning is rejected before replacing a working drive.
			const auto Good = Joint.GetDrive(); Drive.Settings.Damping = -1;
			bool Rejected = false; try { Joint.SetDrive(Drive); } catch (const std::invalid_argument&) { Rejected = true; }
			EXPECT(Rejected && Joint.GetDrive().Settings.Damping == Good.Settings.Damping);
			B->CastDynamic()->Sleep(); Joint.SetDrive(Good); EXPECT(!B->CastDynamic()->Sleeping);
		}
	}
	Vector3 AngularMomentum(RigidBody* B)
	{
		return B->X.Cross(B->V / B->InvMass) + B->GetInverseInertia_WorldSpace().Inverse() * B->W;
	}
	void TestMomentumAndLimits()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* A = Body(World, Vector3::Zero(), true);
		auto* B = Body(World, Vector3(2, 0, 0), true);
		A->InvInertia = Matrix3(1, 2, 3); B->InvInertia = Matrix3(3, 1, 2);
		SliderConstraint Joint(A, B, Vector3(0, 1, 0), Vector3::UnitX(), -5, 5);
		Joint.SetDrive(Spring(1)); Joint.BeginStep(Dt, false);
		for (int I = 0; I < 32; ++I) Joint.SolveVelocity(Dt);
		EXPECT((A->V + B->V).Length() < 1e-5f);
		EXPECT((AngularMomentum(A) + AngularMomentum(B)).Length() < 1e-4f);
		// An outward motor cannot override a hard travel limit.
		B->X.x += 5; A->V = B->V = A->W = B->W = Vector3::Zero();
		Joint.SetDrive(Spring(8)); Joint.BeginStep(Dt, false);
		for (int I = 0; I < 32; ++I) Joint.SolveVelocity(Dt);
		const Vector3 Relative = B->V - A->V - A->W.Cross(B->X - A->X);
		EXPECT(Relative.x < 1e-4f);
	}
	void TestSlerp()
	{
		Vector3 Reference;
		for (int Variant = 0; Variant < 3; ++Variant)
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* A = Body(World, Vector3::Zero(), false);
			auto* B = Body(World, Vector3::Zero(), true);
			Quaternion Common = Quaternion::One();
			if (Variant == 1) Common.FromRotationAxis(Vector3(2, -1, 3).Unit(), 0.7f);
			A->Q = B->Q = Common; B->InvInertia = Matrix3(1, 2, 3);
			SixDOFConstraintSettings Limits; Limits.AngularMin = Vector3(-PI); Limits.AngularMax = Vector3(PI);
			SixDOFConstraint Joint(A, B, Vector3::Zero(), Limits);
			SixDOFDriveSettings Drives; Drives.AngularMode = AngularDriveMode::Slerp; Drives.Slerp = Spring(0, 6).Settings;
			Drives.OrientationTarget.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.7f);
			if (Variant == 2) Drives.OrientationTarget = -Drives.OrientationTarget;
			Joint.SetDrives(Drives); Joint.BeginStep(Dt, false);
			for (int I = 0; I < 32; ++I) Joint.SolveVelocity(Dt);
			if (Variant == 0) Reference = B->W;
			else EXPECT((B->W - Common * Reference).Length() < 1e-4f);
			EXPECT_NEAR(AngularMomentum(B).Length(), Drives.Slerp.MaxForce * Dt, 1e-5f);
			Drives.OrientationTarget = Quaternion(0, 0, 0, 0);
			EXPECT(!Drives.IsValid());
		}
	}
	void TestWorldRoutes()
	{
		for (auto Solver : Solvers)
		for (bool Unified : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.unifiedConstraints = Unified;
			PhysicsWorld World(P);
			auto* A = Body(World, Vector3::Zero(), false);
			auto* B = Body(World, Vector3::Zero(), true);
			auto* Slider = World.CreateSliderConstraint(A, B, Vector3::Zero(), Vector3::UnitX(), -2, 2);
			Slider->SetDrive(Spring(1));
			auto* C = Body(World, Vector3(4, 0, 0), false);
			auto* D = Body(World, Vector3(4, 0, 0), true);
			auto* Hinge = World.CreateHingeConstraint(C, D, D->X, Vector3::UnitZ(), -1, 1);
			Hinge->SetDrive(Spring(0.5f));
			auto* E = Body(World, Vector3(8, 0, 0), false);
			auto* F = Body(World, Vector3(8, 0, 0), true);
			SixDOFConstraintSettings Limits; Limits.AngularMin = Vector3(-1); Limits.AngularMax = Vector3(1);
			auto* Six = World.CreateSixDOFConstraint(E, F, F->X, Limits);
			SixDOFDriveSettings Drives; Drives.AngularMode = AngularDriveMode::Slerp;
			Drives.Slerp = Spring(0).Settings; Drives.OrientationTarget.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.7f);
			Six->SetDrives(Drives);
			auto* G = Body(World, Vector3(12, 0, 0), false);
			auto* H = Body(World, G->X, true);
			Limits.LinearMin = Vector3(-2); Limits.LinearMax = Vector3(2);
			auto* Axes = World.CreateSixDOFConstraint(G, H, H->X, Limits);
			SixDOFDriveSettings AxisDrives;
			const Vector3 Target(0.6f, -0.3f, 0.2f), Angles(0.2f, -0.3f, 0.25f);
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				AxisDrives.Linear[Axis] = Spring(Target[Axis]);
				AxisDrives.Angular[Axis] = Spring(Angles[Axis]);
			}
			Axes->SetDrives(AxisDrives);
			for (int Frame = 0; Frame < 240; ++Frame) World.Simulate(Dt);
			printf("Drive world solver=%d unified=%d slide=%g hinge=%g orientationDot=%g\n", (int)Solver, Unified, Slider->GetCurrentPosition(), Hinge->GetCurrentAngle(), F->Q.Dot(Drives.OrientationTarget));
			EXPECT_NEAR(Slider->GetCurrentPosition(), 1, 0.002f);
			EXPECT_NEAR(Hinge->GetCurrentAngle(), 0.5f, 0.002f);
			EXPECT(std::abs(F->Q.Dot(Drives.OrientationTarget)) > 0.9999f);
			EXPECT((H->X - G->X - Target).Length() < 0.002f);
			Vector3 Actual; H->Q.ToEuler(Actual.y, Actual.z, Actual.x);
			EXPECT((Actual - Angles).Length() < 0.002f);
			World.SetJointDrivesEnabled(false); EXPECT(!Slider->HasActiveDrive());
			const Vector3 AtRest = B->X; B->V = Vector3::Zero(); Slider->SetDrive(Spring(-1));
			for (int Frame = 0; Frame < 30; ++Frame) World.Simulate(Dt);
			EXPECT((B->X - AtRest).Length() < 0.002f);
			World.SetJointDrivesEnabled(true);
			for (int Frame = 0; Frame < 240; ++Frame) World.Simulate(Dt);
			EXPECT_NEAR(Slider->GetCurrentPosition(), -1, 0.002f);
		}
	}

	void TestContactDriveCoupling()
	{
		for (auto Type : Solvers)
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			const bool SI = Type == ResolutionPhaseSolver::SequentialImpulse;
			const bool LCP = Type == ResolutionPhaseSolver::LCPGlobal;
			if (!PhysicsWorld::IsSolverBackendAvailable(Type, Backend)) continue;
			for (float Step : { Dt / 2, Dt * 2 })
			{
				PhysicsWorld World{ PhysicsWorldParam() };
				auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
				auto* B = Body(World, Vector3(0, 0.5f, 0), true, true)->CastDynamic();
				Floor->SetRestitution(0); B->SetRestitution(0);
				SliderConstraint Joint(Floor, B, B->X, Vector3::UnitY(), -10, 10);
				Joint.SetDrive(Spring(-1, 6)); Joint.BeginStep(Step, Type == ResolutionPhaseSolver::XPBD);
				std::vector<Geometry*> Geometries{ Floor->mGeometries[0], B->mGeometries[0] };
				std::vector<PositionConstraint*> Joints{ &Joint };
				Contact C; C.Normal = Vector3::UnitY(); C.Tangent = Vector3::UnitX(); C.Binormal = Vector3::UnitZ();
				C.PositionLocalB = Vector3(0, -0.5f, 0); C.SolverSlop = 0.001f;
				ContactManifold M; M.AddNewContact(0, 1, C);
				std::vector<ContactManifold*> Contacts{ &M };
				std::unique_ptr<ConstraintSolver> Native;
				std::unique_ptr<PBDSolver> Position;
				if (SI) Native.reset(ConstraintSolver::CreateSequentialImpulseSolver());
				else if (LCP)
				{
					LCPSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Native.reset(CreateGlobalLCPSolver(Settings));
				}
				else
				{
					XPBDSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Position.reset(Type == ResolutionPhaseSolver::PBD ? new PBDSolver(Settings) : new XPBDSolver(Settings));
				}
				if (Native) Native->BeginFrame(1);
				else { Position->BeginStep({ B }, Step); Position->AddContacts(Geometries, { &M }); }
				auto Solve = [&]()
				{
					if (Native)
					{
						Native->PreResolve(Geometries, Joints);
						Native->ResolveConstraints(Geometries, Contacts, Joints, Step, 32);
						Native->PostResolve(Geometries);
					}
					else
					{
						for (int Iteration = 0; Iteration < 32; ++Iteration)
						{
							Position->SolveJointVelocities(Joints, Step);
							Position->SolveVelocities(Step);
						}
						Position->EndStep(Step);
					}
				};
				Solve();
				EXPECT_NEAR(Joint.GetDriveImpulse(), -6 * Step, 1e-5f);
				EXPECT(B->V.Length() < 0.001f);
				const ContactManifold* Result = Native ? &M : Position->GetContactManifolds()[0];
				EXPECT_NEAR(Result->ContactPoints[0].totalImpulseNormal, 6 * Step, 1e-4f);
				// Another solver pass shares the motor budget, then the next frame gets a new one.
				Solve(); EXPECT_NEAR(Joint.GetDriveImpulse(), -6 * Step, 1e-5f);
				EXPECT(B->V.Length() < 0.001f);
				B->V.y = 2; Joint.BeginStep(Step, Type == ResolutionPhaseSolver::XPBD);
				if (Native) Native->BeginFrame(2);
				else { Position->BeginStep({ B }, Step); Position->AddContacts(Geometries, { &M }); }
				Solve();
				Result = Native ? &M : Position->GetContactManifolds()[0];
				EXPECT_NEAR(B->V.y, 2 - 6 * Step, 1e-4f);
				EXPECT_NEAR(Joint.GetDriveImpulse(), -6 * Step, 1e-5f);
				EXPECT(std::abs(Result->ContactPoints[0].totalImpulseNormal) < 1e-4f);
			}
			printf("Drive contacts solver=%d backend=%d: force cap and release checked\n", (int)Type, (int)Backend);
		}
	}

	void TestDriveBoundaries()
	{
		PhysicsWorldParam P; P.jointDrives = false;
		PhysicsWorld World(P);
		auto* A = Body(World, Vector3::Zero(), false);
		auto* B = Body(World, Vector3::Zero(), true);
		auto* Joint = World.CreateSliderConstraint(A, B, B->X, Vector3::UnitX(), -10, 10);
		Joint->SetDrive(Spring(1)); EXPECT(!Joint->HasActiveDrive());
		World.SetJointDrivesEnabled(true);
		for (float Step : { Dt / 4, Dt * 4 })
		{
			auto Drive = Spring(1, 6); Drive.Settings.Stiffness = std::numeric_limits<float>::max();
			Joint->SetDrive(Drive); Joint->BeginStep(Step, false); B->V = Vector3::Zero();
			Joint->SolveVelocity(Step); EXPECT_NEAR(B->V.x, 6 * Step, 1e-5f);
			Drive.Settings.MaxForce = 0; Joint->SetDrive(Drive); Joint->BeginStep(Step, false); B->V = Vector3::Zero();
			Joint->SolveVelocity(Step); EXPECT(B->V == Vector3::Zero());
			Drive.Settings.MaxForce = 6; Drive.TargetVelocity = std::numeric_limits<float>::infinity();
			EXPECT(!Drive.IsValid());
		}
		SixDOFConstraintSettings Limits; Limits.AngularMin = Vector3(-PI); Limits.AngularMax = Vector3(PI);
		SixDOFConstraint Angular(A, B, B->X, Limits);
		SixDOFDriveSettings Drives; Drives.AngularMode = AngularDriveMode::Slerp;
		Drives.Slerp = Spring(0, 6).Settings; Drives.Slerp.Stiffness = std::numeric_limits<float>::max();
		Drives.OrientationTarget.FromRotationAxis(Vector3::UnitX(), PI - 0.001f);
		B->InvInertia = Matrix3(0, 1, 2); // A locked angular mode receives no impulse.
		Angular.SetDrives(Drives); Angular.BeginStep(Dt, false); Angular.SolveVelocity(Dt);
		EXPECT(B->W == Vector3::Zero());
		Drives.OrientationTarget.FromRotationAxis(Vector3::UnitY(), PI - 0.001f);
		Angular.SetDrives(Drives); Angular.BeginStep(Dt, false); Angular.SolveVelocity(Dt);
		EXPECT_NEAR(B->W.y, 6 * Dt, 1e-5f);
		// Velocity-only SLERP follows a moving kinematic frame and preserves its motion.
		RigidBodyKinematics Kinematic;
		auto* Moving = &Kinematic;
		Moving->W = Vector3(0, 2, 0); B->W = Vector3::Zero(); B->InvInertia = Matrix3(1, 1, 1);
		SixDOFConstraint Motor(Moving, B, B->X, Limits);
		Drives.Slerp = JointDriveSettings(); Drives.Slerp.VelocityEnabled = true; Drives.Slerp.Damping = 16;
		Drives.AngularVelocityTarget = Vector3(0, 1, 0);
		Motor.SetDrives(Drives); Motor.BeginStep(Dt, false); Motor.SolveVelocity(Dt);
		EXPECT_NEAR(B->W.y, 3 * Dt * 16 / (1 + Dt * 16), 1e-5f);
		EXPECT(Moving->W == Vector3(0, 2, 0));
		// Driving a free coordinate must not rigidify the XPBD support in another direction.
		PhysicsWorldParam Soft; Soft.resolutionSolver = ResolutionPhaseSolver::XPBD; Soft.gravityAcc = Vector3(0, -10, 0);
		PhysicsWorld SoftWorld(Soft);
		auto* Support = Body(SoftWorld, Vector3::Zero(), false);
		auto* Load = Body(SoftWorld, Vector3::Zero(), true);
		auto* SoftJoint = SoftWorld.CreateSliderConstraint(Support, Load, Load->X, Vector3::UnitX(), -2, 2);
		SoftJoint->SetCompliance(0.005f, 0); SoftJoint->SetDrive(Spring(1));
		for (int Frame = 0; Frame < 240; ++Frame) SoftWorld.Simulate(Dt);
		EXPECT_NEAR(Load->X.x, 1, 0.002f); EXPECT_NEAR(Load->X.y, -0.05f, 0.002f);
	}

}

void TestJointDrives()
{
	TestScalarDrives(); TestMomentumAndLimits(); TestSlerp(); TestWorldRoutes();
	TestContactDriveCoupling(); TestDriveBoundaries();
}

namespace Test_JointReactions_Cases
{
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
}

void TestJointReactions()
{
	TestImpulseReports(); TestPositionReports(); TestWorldBreaking(); TestEventLifetime(); TestChainLoads();
}
}

void TestJointReactions() { Test_JointReactions_Cases::TestJointReactions(); }

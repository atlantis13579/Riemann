#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Collision/GeometryObject.h"
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 1.0f / 60;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	PhysicsWorldParam Parameters(ResolutionPhaseSolver Solver, int Substeps = 1, SolverBackend Backend = SolverBackend::CPU)
	{
		PhysicsWorldParam P; P.resolutionSolver = Solver; P.narrowphase = NarrowPhaseSolver::PCM;
		P.solverBackend = Backend; P.allowBackendFallback = false;
		P.pbdSettings.Backend = P.xpbdSettings.Backend = Backend;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
		P.islandSleeping.Enabled = true; P.islandSleeping.QuietTime = 0.2f;
		return P;
	}
	RigidBody* Box(PhysicsWorld& World, const Vector3& X, bool Dynamic = true, bool Collision = false)
	{
		RigidBodyParam P; P.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		P.linearDamping = P.angularDamping = 1; P.maxContactImpulse = std::numeric_limits<float>::max();
		auto* Shape = GeometryFactory::CreateOBB(X, Vector3(0.5f)); Shape->SetSimulationEnabled(Collision);
		auto* Body = World.CreateRigidBody(Shape, P); Body->SetRestitution(0); Body->SetFriction(0.5f, 0.6f);
		if (Dynamic) { Body->InvMass = 1; Body->InvInertia = Matrix3(6, 6, 6); }
		return Body;
	}
	void Step(PhysicsWorld& World, int Frames) { for (int I = 0; I < Frames; ++I) World.Simulate(Dt); }
	void TestTimersAndJoints()
	{
		for (auto Solver : Solvers) for (int Count : { 1, 4 })
		{
			PhysicsWorld World(Parameters(Solver, Count));
			auto* A = Box(World, Vector3::Zero())->CastDynamic();
			auto* B = Box(World, Vector3(2, 0, 0))->CastDynamic();
			World.CreateFixedConstraint(A, B);
			Step(World, 8); EXPECT(!A->Sleeping && !B->Sleeping);
			Step(World, 8); EXPECT(A->Sleeping && B->Sleeping);
			const Vector3 XA = A->X, XB = B->X;
			Step(World, 30); EXPECT(A->Sleeping && B->Sleeping); EXPECT(A->X == XA && B->X == XB);
			EXPECT(World.GetLastIslandSleepingStats().SleepingIslandCount == 1);
			B->ApplyForce(Vector3(2, 0, 0)); World.Simulate(Dt);
			EXPECT(!A->Sleeping && !B->Sleeping); EXPECT(A->V.x > 0 && B->V.x > 0);
			EXPECT_NEAR(A->V.x + B->V.x, 1, 1e-4f);
			printf("Island sleep joint solver=%d substeps=%d wake=%d\n", (int)Solver, Count, World.GetLastIslandSleepingStats().WakeTransitions);
		}
		// Hysteresis preserves accumulated time in the band but does not advance it.
		PhysicsWorld World(Parameters(ResolutionPhaseSolver::SequentialImpulse));
		auto* Body = Box(World, Vector3::Zero())->CastDynamic(); Step(World, 6);
		float Quiet = World.GetIslandQuietTime(Body); EXPECT(Quiet > 0);
		Body->V = Vector3(0.04f, 0, 0); World.Simulate(Dt); // An external velocity edit resets the timer.
		EXPECT_NEAR(World.GetIslandQuietTime(Body), 0, 1e-6f); Step(World, 10); EXPECT(!Body->Sleeping);
		Body->V.SetZero(); Step(World, 6); Quiet = World.GetIslandQuietTime(Body);
		EXPECT(Quiet > 0); Body->Wakeup(); World.Simulate(Dt); EXPECT_NEAR(World.GetIslandQuietTime(Body), 0, 1e-6f);
		Step(World, 16); EXPECT(Body->Sleeping);
		Body->SleepThreshold = 0; Body->Wakeup(); Step(World, 30); EXPECT(!Body->Sleeping);
	}
	void TestContactsAndBackends()
	{
		for (auto Solver : Solvers) for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (Solver == ResolutionPhaseSolver::SequentialImpulse && Backend != SolverBackend::CPU) continue;
			if (Solver == ResolutionPhaseSolver::LCPGlobal ? !LCPSolver::IsBackendAvailable(Backend) : !IsPBDBackendAvailable(Backend)) continue;
			for (int Count : { 1, 4 }) for (bool Support : { false, true })
			{
				auto P = Parameters(Solver, Count, Backend); P.gravityAcc = Vector3(0, -10, 0); P.supportStabilization.Enabled = Support;
				PhysicsWorld World(P); auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
				Floor->SetRestitution(0);
				auto* A = Box(World, Vector3(0, 0.5f, 0), true, true)->CastDynamic();
				auto* B = Box(World, Vector3(0, 1.5f, 0), true, true)->CastDynamic();
				auto* C = Box(World, Vector3(4, 0.5f, 0), true, true)->CastDynamic();
				Step(World, 180);
				printf("Island stack solver=%d backend=%d substeps=%d support=%d sleep=%d%d%d islands=%d quiet=%g,%g\n", (int)Solver, (int)Backend, Count, Support,
					A->Sleeping, B->Sleeping, C->Sleeping, World.GetLastIslandSleepingStats().IslandCount, World.GetIslandQuietTime(A), World.GetIslandQuietTime(B));
				EXPECT(A->Sleeping && B->Sleeping && C->Sleeping);
				EXPECT(World.GetLastIslandSleepingStats().SleepingIslandCount == 2);
				const Vector3 X = B->X; Step(World, 30); EXPECT(B->X == X);
				B->ApplyForce(Vector3(1, 0, 0)); World.Simulate(Dt);
				EXPECT(!A->Sleeping && !B->Sleeping && C->Sleeping);
				// Removing a shared boundary wakes every supported island, without retaining its pointer.
				EXPECT(World.RemoveRigidBody(Floor)); Floor->ReleaseGeometries(); delete Floor;
				World.Simulate(Dt); EXPECT(!A->Sleeping && !B->Sleeping && !C->Sleeping); EXPECT(C->V.y < 0);
			}
		}
	}
	void TestBoundariesAndMutations()
	{
		for (auto Solver : Solvers)
		{
			PhysicsWorld World(Parameters(Solver, 4));
			auto* Root = Box(World, Vector3::Zero(), false); Root->mRigidType = RigidType::Kinematic;
			auto* A = Box(World, Vector3(1, 0, 0))->CastDynamic();
			auto* B = Box(World, Vector3(2, 0, 0))->CastDynamic();
			auto* Unrelated = Box(World, Vector3(8, 0, 0))->CastDynamic();
			auto* J = World.CreateFixedConstraint(Root, A); World.CreateFixedConstraint(A, B);
			Step(World, 20); EXPECT(A->Sleeping && B->Sleeping && Unrelated->Sleeping);
			Root->X.y += 0.1f; Root->V.y = 1; World.Simulate(Dt);
			EXPECT(!A->Sleeping && !B->Sleeping && Unrelated->Sleeping); EXPECT(B->X.y > 0);
			Root->V.SetZero(); Step(World, 30); EXPECT(A->Sleeping && B->Sleeping);
			J->SetEnabled(false); World.Simulate(Dt); EXPECT(!A->Sleeping && !B->Sleeping && Unrelated->Sleeping);
			Step(World, 30); EXPECT(A->Sleeping && B->Sleeping);
			EXPECT(World.RemoveConstraint(J)); World.Simulate(Dt); EXPECT(!A->Sleeping && !B->Sleeping);
			Step(World, 30); A->X.x += 1; World.Simulate(Dt); EXPECT(!A->Sleeping && !B->Sleeping);
			Step(World, 30); EXPECT(World.RemoveRigidBody(A)); A->ReleaseGeometries(); delete A;
			World.Simulate(Dt); EXPECT(!B->Sleeping); World.Reset(); World.Simulate(Dt);
			EXPECT(World.GetLastIslandSleepingStats().IslandCount == 0);
		}
		// A static body moved into a sleeping body must become a broadphase candidate.
		PhysicsWorld World(Parameters(ResolutionPhaseSolver::SequentialImpulse));
		auto* A = Box(World, Vector3::Zero(), true, true)->CastDynamic();
		auto* Wall = Box(World, Vector3(4, 0, 0), false, true); Step(World, 20); EXPECT(A->Sleeping);
		Wall->X.x = 0.8f; World.Simulate(Dt); EXPECT(!A->Sleeping);
	}
	void TestDrivesLoadsAndPolicies()
	{
		for (auto Solver : Solvers)
		{
			PhysicsWorld World(Parameters(Solver)); auto* Root = Box(World, Vector3::Zero(), false);
			auto* A = Box(World, Vector3::Zero())->CastDynamic(); auto* B = Box(World, Vector3(3, 0, 0))->CastDynamic();
			auto* Joint = World.CreateSliderConstraint(Root, A, Vector3::Zero(), Vector3::UnitX(), -10, 10);
			ScalarJointDrive Drive; Drive.Settings.PositionEnabled = true; Drive.Settings.Stiffness = 10; Drive.Settings.Damping = 2; Drive.Settings.MaxForce = 10;
			Joint->SetDrive(Drive); Step(World, 30); EXPECT(!A->Sleeping && B->Sleeping);
			Joint->SetDrive(ScalarJointDrive()); Step(World, 30); EXPECT(A->Sleeping && B->Sleeping);
			Joint->SetDistanceLimits(1, 2); World.Simulate(Dt); EXPECT(!A->Sleeping);
			if (Solver == ResolutionPhaseSolver::XPBD) { Joint->SetCompliance(0.01f, 0); Step(World, 60); EXPECT(!A->Sleeping); }
		}
		class Load : public ForceField
		{
		public:
			RigidBodyDynamic* Target = nullptr;
			bool ApplyForce(RigidBodyDynamic* Body) override { if (Body == Target) Body->ApplyForce(Vector3(1, 0, 0)); return true; }
		};
		auto P = Parameters(ResolutionPhaseSolver::SequentialImpulse, 4); PhysicsWorld World(P);
		auto* A = Box(World, Vector3::Zero())->CastDynamic(); auto* B = Box(World, Vector3(2, 0, 0))->CastDynamic();
		World.CreateFixedConstraint(A, B); Step(World, 20); EXPECT(A->Sleeping && B->Sleeping);
		auto* Field = new Load; Field->Target = A; World.AddForceField(Field); World.Simulate(Dt);
		EXPECT(!A->Sleeping && !B->Sleeping); EXPECT_NEAR(A->V.x + B->V.x, Dt, 1e-5f);
		World.RemoveForceField(Field);
		auto Settings = World.GetIslandSleepingSettings(); Settings.Enabled = false; World.SetIslandSleepingSettings(Settings);
		EXPECT(!A->IslandSleepManaged && !B->IslandSleepManaged);
		Settings.QuietTime = 0; bool Rejected = false;
		try { World.SetIslandSleepingSettings(Settings); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected && World.GetIslandSleepingSettings().QuietTime > 0);
	}
	void TestHysteresisAndLateWake()
	{
		PhysicsWorld World(Parameters(ResolutionPhaseSolver::SequentialImpulse));
		auto* A = Box(World, Vector3::Zero())->CastDynamic(); auto* B = Box(World, Vector3(1, 0, 0))->CastDynamic();
		IslandSleeping Sleeping; auto Settings = World.GetIslandSleepingSettings(); Settings.QuietTime = 1; Sleeping.Configure(Settings);
		std::vector<RigidBodyDynamic*> Bodies = { A, B }; std::vector<PositionConstraint*> Joints;
		uint64_t Tick = 0;
		auto Advance = [&](float Speed, float Correction)
		{
			Sleeping.Prepare(Bodies, Joints, ++Tick, true, false);
			A->V.x = Speed; A->X.x += Correction; // Model motion generated within the solver.
			Sleeping.Finish(Bodies, Joints, 0.1f, true, false);
		};
		Advance(0, 0); Advance(0, 0); EXPECT_NEAR(Sleeping.GetQuietTime(A), 0.2f, 1e-6f);
		Advance(0.04f, 0); EXPECT_NEAR(Sleeping.GetQuietTime(A), 0.2f, 1e-6f);
		Advance(0.07f, 0); EXPECT_NEAR(Sleeping.GetQuietTime(A), 0, 1e-6f);
		Advance(0, 0); EXPECT_NEAR(Sleeping.GetQuietTime(A), 0.1f, 1e-6f);
		Advance(0, 0.1f); EXPECT_NEAR(Sleeping.GetQuietTime(A), 0, 1e-6f);
		// Seed a touching edge, then omit collision output as the broadphase does for sleepers.
		ContactManifold Contact; Contact.indexA = 0; Contact.indexB = 1; Contact.NumContactPointCount = 1;
		Contact.ContactPoints[0].PenetrationDepth = 0;
		std::vector<Geometry*> Geometries = { A->Geometries()[0], B->Geometries()[0] };
		for (auto* Shape : Geometries) Shape->SetSimulationEnabled(true);
		Sleeping.AddContacts(Geometries, { &Contact }, Bodies, Joints);
		A->Sleep(); B->Sleep(); Sleeping.Finish(Bodies, Joints, 0.1f, true, false);
		Sleeping.Prepare(Bodies, Joints, ++Tick, true, false);
		A->Wakeup(); // Model a late CCD hit, after discrete contact generation.
		Sleeping.Finish(Bodies, Joints, 0.1f, true, false); EXPECT(!A->Sleeping && !B->Sleeping);
	}
	void TestNewContactsAndTriggers()
	{
		for (auto Solver : Solvers)
		{
			PhysicsWorld World(Parameters(Solver));
			auto* A = Box(World, Vector3::Zero(), true, true)->CastDynamic();
			auto* B = Box(World, Vector3(1, 0, 0), true, true)->CastDynamic();
			Step(World, 20); EXPECT(A->Sleeping && B->Sleeping);
			auto* Trigger = Box(World, Vector3(-0.8f, 0, 0), true, true)->CastDynamic();
			Trigger->Geometries()[0]->SetTriggerEnabled(true); Trigger->V.x = 1;
			// Initial filter revision refresh conservatively wakes the scene; let it settle again.
			Step(World, 30); EXPECT(A->Sleeping && B->Sleeping);
			Trigger->X = Vector3(-0.8f, 0, 0); Trigger->V.x = 1; World.Simulate(Dt);
			EXPECT(A->Sleeping && B->Sleeping);
			auto* Projectile = Box(World, Vector3(-0.8f, 0, 0), true, true)->CastDynamic(); Projectile->V.x = 2;
			World.Simulate(Dt); EXPECT(!A->Sleeping && !B->Sleeping);
		}
		PhysicsWorld World(Parameters(ResolutionPhaseSolver::SequentialImpulse));
		auto* A = Box(World, Vector3::Zero(), true, true)->CastDynamic(); Step(World, 20); EXPECT(A->Sleeping);
		Box(World, Vector3(0.8f, 0, 0), false, true); World.Simulate(Dt); EXPECT(!A->Sleeping);
	}
	void TestFilteringAndCallbackLoads()
	{
		auto P = Parameters(ResolutionPhaseSolver::SequentialImpulse); P.gravityAcc = Vector3(0, -10, 0);
		PhysicsWorld World(P);
		auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		auto* A = Box(World, Vector3(0, 0.5f, 0), true, true)->CastDynamic();
		auto* B = Box(World, Vector3(0, 1.5f, 0), true, true)->CastDynamic();
		auto* C = Box(World, Vector3(4, 0.5f, 0), true, true)->CastDynamic();
		Step(World, 120); EXPECT(A->Sleeping && B->Sleeping && C->Sleeping);
		A->IgnoreCollisionWith(Floor); World.Simulate(Dt);
		EXPECT(!A->Sleeping && !B->Sleeping && C->Sleeping); EXPECT(A->V.y < 0);
		for (auto Solver : Solvers)
		{
			RigidBodyDynamic* Target = nullptr; bool Enabled = false, Sent = false;
			auto Settings = Parameters(Solver, 4);
			Settings.onContact = [&](const PhysicsContactInfo&)
			{
				if (Enabled && !Sent) { Target->ApplyForce(Vector3(4, 0, 0)); Sent = true; }
			};
			PhysicsWorld W(Settings);
			W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
			auto* Source = Box(W, Vector3(0, 0.49f, 0), true, true)->CastDynamic();
			Target = Box(W, Vector3(5, 0, 0))->CastDynamic(); auto* Partner = Box(W, Vector3(7, 0, 0))->CastDynamic();
			W.CreateFixedConstraint(Target, Partner); Step(W, 30); EXPECT(Target->Sleeping && Partner->Sleeping);
			Source->Wakeup(); Source->X.y = 0.49f; Enabled = true; W.Simulate(Dt);
			EXPECT(Sent && !Target->Sleeping && !Partner->Sleeping); EXPECT_NEAR(Target->ExtForce.x, 4, 1e-6f);
			EXPECT_NEAR(Target->V.x + Partner->V.x, 0, 1e-6f);
			W.Simulate(Dt); EXPECT_NEAR(Target->V.x + Partner->V.x, 2, 1e-4f);
		}
	}
}
void TestIslandSleeping()
{
	TestTimersAndJoints(); TestContactsAndBackends(); TestBoundariesAndMutations(); TestDrivesLoadsAndPolicies();
	 TestHysteresisAndLateWake(); TestNewContactsAndTriggers(); TestFilteringAndCallbackLoads();
}

#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/MotionIntegration.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/DistanceFieldShape.h"
#include <memory>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	RigidBodyDynamic* Ball(PhysicsWorld& World, Vector3 Position, Vector3 Velocity, bool Continuous = true, float Bounce = 1)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = Continuous ? MorionType::Continuous : MorionType::Discrete;
		P.linearVelocity = Velocity; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* B = World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.5f), P)->CastDynamic();
		B->InvMass = 1; B->InvInertia = Matrix3(10, 10, 10); B->SetRestitution(Bounce); B->SetFriction(0, 0); return B;
	}
	RigidBody* Wall(PhysicsWorld& World, Vector3 Position, Vector3 Normal)
	{
		RigidBodyParam P; P.rigidType = RigidType::Static;
		auto* B = World.CreateRigidBody(GeometryFactory::CreatePlane(Position, Normal), P); B->SetRestitution(0); B->SetFriction(0, 0); return B;
	}
	PhysicsWorldParam Parameters(ResolutionPhaseSolver Solver = ResolutionPhaseSolver::SequentialImpulse, SolverBackend Backend = SolverBackend::CPU)
	{
		PhysicsWorldParam P; P.ccdContinuation.Enabled = true; P.resolutionSolver = Solver; P.solverBackend = Backend;
		P.allowBackendFallback = false; P.pbdSettings.Backend = Backend; P.xpbdSettings.Backend = Backend;
		return P;
	}
	void TestMultipleImpacts()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
			PhysicsWorld World(Parameters(Solver, Backend));
			Wall(World, Vector3::Zero(), Vector3::UnitX()); Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
			auto* B = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0)); World.Simulate(0.2f);
			printf("CCD bounce solver=%d backend=%d x=%f vx=%f impacts=%d\n", (int)Solver, (int)Backend, B->X.x, B->V.x, World.GetLastCCDHitCount());
			EXPECT_NEAR(B->X.x, 7, 0.01f); EXPECT_NEAR(B->V.x, 100, 0.01f);
			EXPECT(World.GetLastCCDHitCount() == 2); EXPECT(World.GetLastCCDContinuationStats().ClampedBodyCount == 0);
			EXPECT(!B->CCDPoseResolved); EXPECT(B->CCDMotionFraction < 0.25f);
			World.Simulate(0.02f); EXPECT_NEAR(B->X.x, 9, 0.01f);
			auto Legacy = World.GetCCDContinuationSettings(); Legacy.Enabled = false; World.SetCCDContinuationSettings(Legacy);
			B->X = Vector3(5, 0, 0); B->V = Vector3(100, 0, 0); B->UpdateGeometries(); World.Simulate(0.2f);
			EXPECT_NEAR(B->X.x, 9.5f, 0.01f); EXPECT_NEAR(B->V.x, 0, 0.01f);
			EXPECT(World.GetLastCCDHitCount() == 1 && World.GetLastCCDContinuationStats().ImpactCount == 0);

		}
		PhysicsWorld World(Parameters());
		Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
		auto* B = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 20), true, 0); World.Simulate(0.1f);
		EXPECT_NEAR(B->X.x, 9.5f, 0.01f); EXPECT_NEAR(B->X.z, 2, 0.01f);
		EXPECT_NEAR(B->V.z, 20, 0.001f); EXPECT(World.GetLastCCDContinuationStats().ClampedBodyCount == 0);
		// Zero-time contact must block approach without consuming the tangential interval.
		B->X = Vector3(9.5f, 0, 0); B->V = Vector3(100, 0, 20); B->UpdateGeometries(); World.Simulate(0.1f);
		EXPECT_NEAR(B->X.x, 9.5f, 0.01f); EXPECT_NEAR(B->X.z, 2, 0.01f);
	}
	void TestDynamicChain()
	{
		for (bool Reverse : { false, true })
		{
			PhysicsWorld World(Parameters()); RigidBodyDynamic* B[3];
			for (int I = 0; I < 3; ++I)
			{
				const int Index = Reverse ? 2 - I : I;
				B[Index] = Ball(World, Vector3(float(Index * 3), 0, 0), Vector3(Index == 0 ? 100.0f : 0, 0, 0), Index == 0);
			}
			B[2]->Sleeping = true;
			World.Simulate(0.1f);
			printf("CCD chain reverse=%d positions=(%f,%f,%f) velocities=(%f,%f,%f) hits=%d\n", Reverse,
				B[0]->X.x, B[1]->X.x, B[2]->X.x, B[0]->V.x, B[1]->V.x, B[2]->V.x, World.GetLastCCDHitCount());
			EXPECT_NEAR(B[0]->X.x, 2, 0.02f); EXPECT_NEAR(B[1]->X.x, 5, 0.02f); EXPECT_NEAR(B[2]->X.x, 12, 0.02f);
			EXPECT_NEAR(B[0]->V.x + B[1]->V.x + B[2]->V.x, 100, 0.01f);
			EXPECT_NEAR(B[0]->V.SquareLength() + B[1]->V.SquareLength() + B[2]->V.SquareLength(), 10000, 2);
			EXPECT(!B[2]->Sleeping && B[1]->HadCCDContact && B[2]->HadCCDContact);
			EXPECT(B[1]->mMotionType == MorionType::Discrete && B[2]->mMotionType == MorionType::Discrete);
		}
		// A stationary continuous target must still sweep a moving discrete source.
		PhysicsWorld World(Parameters());
		auto* Target = Ball(World, Vector3::Zero(), Vector3::Zero());
		auto* Source = Ball(World, Vector3(-3, 0, 0), Vector3(100, 0, 0), false);
		World.Simulate(0.1f); EXPECT_NEAR(Target->X.x, 8, 0.02f); EXPECT_NEAR(Source->X.x, -1, 0.02f);
	}
	void TestCornerAndBudget()
	{
		PhysicsWorld World(Parameters());
		Wall(World, Vector3(10, 0, 0), -Vector3::UnitX()); Wall(World, Vector3(0, 0, 10), -Vector3::UnitZ());
		auto* B = Ball(World, Vector3(5, 0, 5), Vector3(100, 0, 100)); World.Simulate(0.1f);
		EXPECT_NEAR(B->X.x, 4, 0.01f); EXPECT_NEAR(B->X.z, 4, 0.01f); EXPECT(World.GetLastCCDHitCount() == 2);
		auto P = Parameters(); P.ccdContinuation.MaxPasses = 1; PhysicsWorld Limited(P);
		Wall(Limited, Vector3::Zero(), Vector3::UnitX()); Wall(Limited, Vector3(10, 0, 0), -Vector3::UnitX());
		auto* Fast = Ball(Limited, Vector3(5, 0, 0), Vector3(100, 0, 0));
		auto* Free = Ball(Limited, Vector3(-20, 10, 0), Vector3(0, 5, 0), false);
		Limited.Simulate(0.2f); EXPECT_NEAR(Fast->X.x, 9.5f, 0.01f); EXPECT_NEAR(Free->X.y, 11, 0.001f);
		EXPECT(Limited.GetLastCCDContinuationStats().ClampedBodyCount == 1);
		EXPECT_NEAR(Limited.GetLastCCDContinuationStats().MaximumDroppedTime, 0.155f, 0.001f);
		EXPECT_NEAR(Fast->V.x, -100, 0.001f);
		Limited.Simulate(0.01f); EXPECT_NEAR(Fast->X.x, 8.5f, 0.01f);
		EXPECT(Limited.GetLastCCDContinuationStats().ClampedBodyCount == 0);
	}
	void TestSegmentTriggers()
	{
		auto P = Parameters(); std::vector<PhysicsTriggerInfo> Events;
		P.onTrigger = [&](const PhysicsTriggerInfo& Event) { if (Event.IsCCD) Events.push_back(Event); };
		PhysicsWorld World(P); Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
		RigidBodyParam Static; Static.rigidType = RigidType::Static;
		Geometry* Behind = GeometryFactory::CreateOBB(Vector3(15, 0, 0), Vector3(0.1f, 5, 5));
		Geometry* Return = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.1f, 5, 5));
		for (auto* Shape : { Behind, Return })
		{
			Shape->SetTriggerEnabled(true); Shape->SetTriggerCCDEnabled(true); World.CreateRigidBody(Shape, Static);
		}
		Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0)); World.Simulate(0.2f);
		EXPECT(Events.size() == 1);
		if (Events.size() == 1)
		{
			EXPECT(Events[0].GeomA == Return || Events[0].GeomB == Return);
			EXPECT(Events[0].Fraction > 0.6f && Events[0].Fraction < 0.7f);
		}
	}
	void TestGeometryRoutes()
	{
		for (int Kind = 0; Kind < 3; ++Kind)
		{
			PhysicsWorld World(Parameters()); Geometry* Ground = nullptr;
			if (Kind == 0) Ground = GeometryFactory::CreateOBB(Vector3(0, -0.1f, 0), Vector3(4, 0.1f, 4));
			if (Kind == 1)
			{
				Ground = GeometryFactory::CreateTriangleMesh(); auto* Mesh = Ground->GetShapeObj<TriangleMesh>();
				Mesh->AddAABB(Vector3(-4, -0.2f, -4), Vector3(4, 0, 4)); Mesh->Compact(); Mesh->BuildBVH(); Ground->UpdateVolumeProperties();
			}
			if (Kind == 2)
			{
				Ground = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4), Vector3(4, 1, 4)), 2, 2);
				auto* Field = Ground->GetShapeObj<HeightField3>(); Field->HeightScale = 0.001f; Field->AllocMemory();
				for (int X = 0; X < 2; ++X) for (int Z = 0; Z < 2; ++Z) EXPECT(Field->SetHeight(X, Z, 0));
				Ground->UpdateVolumeProperties();
			}
			World.CreateRigidBody(Ground, RigidBodyParam())->SetRestitution(0);
			auto* B = Ball(World, Vector3(0, 5, 0), Vector3(0, -100, 0)); World.Simulate(0.1f);
			printf("CCD ground kind=%d y=%f vy=%f hits=%d\n", Kind, B->X.y, B->V.y, World.GetLastCCDHitCount());
			EXPECT_NEAR(B->X.y, 6, 0.02f); EXPECT_NEAR(B->V.y, 100, 0.05f);
			EXPECT(World.GetLastCCDContinuationStats().ClampedBodyCount == 0);
		}
		for (bool Moving : { false, true })
		{
			PhysicsWorld World(Parameters()); Wall(World, Vector3::Zero(), -Vector3::UnitX());
			RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = MorionType::Continuous;
			P.angularVelocity = Vector3(0, 0, 120); P.linearVelocity = Moving ? Vector3(20, 0, 0) : Vector3::Zero();
			P.sleepThreshold = 0; P.linearDamping = P.angularDamping = 1;
			auto* Shape = GeometryFactory::CreateCapsule(Vector3(-0.8f, -1, 0), Vector3(-0.8f, 1, 0), 0.1f);
			auto* B = World.CreateRigidBody(Shape, P)->CastDynamic(); B->SetRestitution(0);
			World.Simulate(1.0f / 60);
			const auto* Capsule = Shape->GetShapeObj<Capsule3>(); const auto Pose = B->GetGeometryTransform(Shape);
			const float MaximumX = std::max(Pose.LocalToWorld(Capsule->X0).x, Pose.LocalToWorld(Capsule->X1).x) + Capsule->Radius;
			printf("CCD angular moving=%d maximumX=%f hits=%d clamped=%d\n", Moving, MaximumX,
				World.GetLastCCDHitCount(), World.GetLastCCDContinuationStats().ClampedBodyCount);
			EXPECT(B->HadCCDContact && MaximumX < 0.01f); EXPECT(std::isfinite(B->W.Length()));
		}
		// Separating at t=0 does not exclude an angular re-impact later in the frame.
		{
			PhysicsWorld World(Parameters()); auto* Plane = Wall(World, Vector3::Zero(), -Vector3::UnitX());
			RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = MorionType::Continuous;
			P.angularVelocity = Vector3(0, 0, 80); P.linearVelocity = Vector3(0, 5, 0);
			P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
			const Vector3 Center(-1, 0, 0), Axis(0.9f, std::sqrt(1 - 0.9f * 0.9f), 0);
			auto* Shape = GeometryFactory::CreateCapsule(Center - Axis, Center + Axis, 0.1f);
			auto* B = World.CreateRigidBody(Shape, P)->CastDynamic(); B->SetRestitution(0);
			std::vector<RigidBodyDynamic*> Dynamic = { B }; const auto Before = B->X;
			const auto Stats = ContinuousCollisionDetection::ResolveRemainingTime(Dynamic, { Plane, B }, 0.07f, CCDContinuationSettings(), {});
			EXPECT(B->X == Before && B->CCDPoseResolved && Stats.ImpactCount > 0);
			MotionIntegration::IntegratePoses(Dynamic, 0.07f, 0);
			const auto* Capsule = Shape->GetShapeObj<Capsule3>(); const auto Pose = B->GetGeometryTransform(Shape);
			const float MaximumX = std::max(Pose.LocalToWorld(Capsule->X0).x, Pose.LocalToWorld(Capsule->X1).x) + Capsule->Radius;
			EXPECT(MaximumX < 0.01f);
		}
		// The front shape of a compound sets the first event; the body then rebounds.
		PhysicsWorld World(Parameters()); Wall(World, Vector3::Zero(), -Vector3::UnitX());
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = MorionType::Continuous;
		P.linearVelocity = Vector3(100, 0, 0); P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* Front = GeometryFactory::CreateSphere(Vector3(-4, 0, 0), 0.5f);
		auto* Back = GeometryFactory::CreateSphere(Vector3(-6, 0, 0), 0.5f);
		auto* Compound = World.CreateRigidBody({ Front, Back }, P, Transform(Vector3(-5, 0, 0)))->CastDynamic();
		Compound->SetRestitution(1); World.Simulate(0.1f);
		EXPECT_NEAR(Compound->GetGeometryTransform(Front).pos.x, -7, 0.02f); EXPECT_NEAR(Compound->V.x, -100, 0.02f);
	}
	DistanceFieldShape FieldSphere()
	{
		SignedDistanceField3 Grid; const int Cells = 24; const float Radius = 0.75f;
		EXPECT(Grid.Reset(Box3(Vector3(-Radius), Vector3(Radius)), Cells, Cells, Cells, 2 * Radius / Cells));
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Grid.At(X, Y, Z) = Grid.GridPointToWorld(X, Y, Z).Length() - 0.5f;
		RuntimeDistanceField Field; EXPECT(Field.Build(Grid)); DistanceFieldShape Shape; EXPECT(Shape.Build(Field)); return Shape;
	}
	void TestFieldAndFiltering()
	{
		const auto Sphere = FieldSphere();
		for (bool Limited : { false, true })
		{
			PhysicsWorld World(Parameters());
			if (Limited)
			{
				RuntimeDistanceFieldSettings Settings; Settings.MaximumSearchNodes = 1;
				RuntimeDistanceField Field; EXPECT(Field.Build(*Sphere.GetField().GetField(), Settings));
				DistanceFieldShape Shape; EXPECT(Shape.Build(Field));
				World.CreateRigidBody(GeometryFactory::CreateDistanceField(Shape, Vector3(3, 0, 0)), RigidBodyParam());
			}
			else { Wall(World, Vector3::Zero(), Vector3::UnitX()); Wall(World, Vector3(10, 0, 0), -Vector3::UnitX()); }
			RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = MorionType::Continuous;
			P.linearVelocity = Vector3(100, 0, 0); P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
			auto* B = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere, Limited ? Vector3::Zero() : Vector3(5, 0, 0)), P)->CastDynamic();
			B->SetRestitution(1); World.Simulate(0.2f);
			printf("CCD field limited=%d x=%f vx=%f hits=%d clamped=%d\n", Limited, B->X.x, B->V.x,
				World.GetLastCCDHitCount(), World.GetLastCCDContinuationStats().ClampedBodyCount);
			if (Limited) { EXPECT(B->X.x <= 2); EXPECT(World.GetLastCCDContinuationStats().ClampedBodyCount == 1); }
			else { EXPECT_NEAR(B->X.x, 7, 0.03f); EXPECT_NEAR(B->V.x, 100, 0.05f); }
		}
		// A separating gradient at an initial field contact does not certify a
		// rotating/nonconvex remainder. Keep the pose, and retain velocity to retry.
		{
			PhysicsWorld World(Parameters());
			auto* Target = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(1, 0, 0), 0.5f), RigidBodyParam());
			RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.motionType = MorionType::Continuous;
			P.linearVelocity = Vector3(-10, 0, 0); P.angularVelocity = Vector3(0, 0, 100);
			auto* B = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere), P)->CastDynamic();
			std::vector<RigidBodyDynamic*> Dynamic = { B };
			const auto Stats = ContinuousCollisionDetection::ResolveRemainingTime(Dynamic, { B, Target }, 0.1f, CCDContinuationSettings(), {});
			EXPECT(Stats.ClampedBodyCount == 1 && Stats.MaximumDroppedTime > 0.099f);
			MotionIntegration::IntegratePoses(Dynamic, 0.1f, 0); EXPECT(B->X.Length() < 0.001f);
			EXPECT_NEAR(B->V.x, -10, 0.001f); EXPECT_NEAR(B->W.z, 100, 0.001f);
		}
		for (bool Filtered : { false, true })
		{
			PhysicsWorld World(Parameters());
			auto* Plane = Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
			auto* B = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0));
			if (Filtered) World.SetCollisionFilter(std::shared_ptr<CollisionFilter>(CollisionFilter::CreateCallback([](const CollisionData&, const CollisionData&) { return false; })));
			else B->IgnoreCollisionWith(Plane);
			World.Simulate(0.2f); EXPECT_NEAR(B->X.x, 25, 0.001f); EXPECT(World.GetLastCCDHitCount() == 0);
		}
	}
	void TestJointCoordination()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (bool Unified : { false, true })
		{
			auto P = Parameters(Solver); P.unifiedConstraints = Unified; PhysicsWorld World(P);
			Wall(World, Vector3::Zero(), Vector3::UnitX()); Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
			auto* B = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0));
			auto* SupportShape = GeometryFactory::CreateSphere(B->X, 0.1f); SupportShape->SetSimulationEnabled(false);
			auto* Support = World.CreateRigidBody(SupportShape, RigidBodyParam());
			auto* Slider = World.CreateSliderConstraint(Support, B, B->X, Vector3::UnitX(), -100, 100);
			ScalarJointDrive Drive; Drive.Settings.VelocityEnabled = true; Drive.Settings.Damping = 100;
			Drive.Settings.MaxForce = 2; Drive.TargetVelocity = 200; Slider->SetDrive(Drive);
			World.Simulate(0.2f);
			EXPECT(World.GetLastCCDHitCount() == 2); EXPECT(std::abs(Slider->GetDriveImpulse()) <= 0.40001f);
			EXPECT(std::abs(Slider->GetReaction().Force.x) <= 2.001f);
			EXPECT(B->X.x > 6.9f && B->X.x < 7.2f);
		}
		// Joint-connected discrete bodies receive tested motion after an impact.
		PhysicsWorld World(Parameters()); Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
		auto* A = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0), true, 0);
		auto* B = Ball(World, Vector3(4, 2, 0), Vector3(100, 0, 0), false, 0);
		auto* Joint = World.CreateFixedConstraint(A, B); A->IgnoreCollisionWith(B); World.Simulate(0.1f);
		printf("CCD fixed positions=%f,%f reaction=%f hits=%d\n", A->X.x, B->X.x, Joint->GetReaction().Force.Length(), World.GetLastCCDHitCount());
		EXPECT(A->X.x <= 9.51f && B->X.x <= 9.51f);
		EXPECT(std::isfinite(Joint->GetReaction().Force.Length()));
		// A per-body work limit propagates through joints without stopping a free body.
		auto P = Parameters(); P.ccdContinuation.MaxPasses = 1; PhysicsWorld Limited(P);
		Wall(Limited, Vector3(10, 0, 0), -Vector3::UnitX());
		auto* First = Ball(Limited, Vector3(5, 0, 0), Vector3(100, 0, 0), true, 0);
		auto* Linked = Ball(Limited, Vector3(4, 2, 0), Vector3(100, 0, 0), false, 0);
		First->IgnoreCollisionWith(Linked); Limited.CreateFixedConstraint(First, Linked);
		auto* Free = Ball(Limited, Vector3(5, 20, 10), Vector3(0, 1, 0), false);
		Limited.Simulate(0.1f);
		EXPECT(Limited.GetLastCCDContinuationStats().ClampedBodyCount == 2);
		EXPECT_NEAR(First->X.x, 9.5f, 0.01f); EXPECT_NEAR(Linked->X.x, 8.5f, 0.01f);
		EXPECT_NEAR(Free->X.y, 20.1f, 0.001f);

	}
	void TestExistingDemo()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (bool Enabled : { false, true })
		{
			SceneWorld Scene; std::string Error;
			EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/04_see_saw.json", &Error));
			auto* World = Scene.GetSimulation(); EXPECT(World->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			auto Settings = World->GetCCDContinuationSettings(); Settings.Enabled = Enabled; World->SetCCDContinuationSettings(Settings);
			for (int Frame = 0; Frame < 120; ++Frame) Scene.Step(1.0f / 60);
			for (const auto& Object : Scene.GetObjects())
			{
				auto* B = Object.GeometryPtr->GetParent<RigidBody>();
				EXPECT(std::isfinite(B->X.Length()) && std::isfinite(B->V.Length()) && std::isfinite(B->W.Length()));
				if (Object.Id == "ball") EXPECT(B->CastDynamic()->mMotionType == MorionType::Continuous);
			}
		}
	}
	void TestExceptionalQueryCleanup()
	{
		for (bool AfterImpact : { false, true })
		{
			PhysicsWorld World(Parameters());
			auto* Left = Wall(World, Vector3::Zero(), Vector3::UnitX());
			auto* Right = Wall(World, Vector3(10, 0, 0), -Vector3::UnitX());
			auto* Fast = Ball(World, Vector3(5, 0, 0), Vector3(100, 0, 0));
			auto* Discrete = Ball(World, Vector3(5, 10, 0), Vector3(0, 3, 0), false);
			Discrete->W = Vector3(0, 0, 2);
			auto* Frozen = Ball(World, Vector3(5, 20, 0), Vector3(0, 4, 0), false);
			Frozen->W = Vector3(0, 0, 3); Frozen->Freezing = true;
			auto* Sensor = GeometryFactory::CreateSphere(Vector3(7, 0, 0), 0.25f);
			Sensor->SetTriggerEnabled(true); Sensor->SetTriggerCCDEnabled(true);
			RigidBodyParam Static; Static.rigidType = RigidType::Static;
			auto* Trigger = World.CreateRigidBody(Sensor, Static);
			const Vector3 FastStart = Fast->X, DiscreteStart = Discrete->X;
			const Quaternion Rotation = Discrete->Q;
			std::vector<RigidBodyDynamic*> Moving{ Fast, Discrete, Frozen };
			std::vector<RigidBody*> Bodies{ Left, Right, Fast, Discrete, Frozen, Trigger };
			std::vector<ContinuousCollisionHit> Triggers;
			bool PartialTriggerSeen = false;
			std::unique_ptr<CollisionFilter> Filter(CollisionFilter::CreateCallback(
				[&](const CollisionData&, const CollisionData&)
				{
					if (!AfterImpact || Fast->X.x > FastStart.x)
					{
						PartialTriggerSeen = !Triggers.empty();
						throw std::runtime_error("Interrupted CCD query");
					}
					return true;
				}));
			bool Interrupted = false;
			try { ContinuousCollisionDetection::ResolveRemainingTime(Moving, Bodies, 0.2f,
				World.GetCCDContinuationSettings(), {}, &Triggers, Filter.get()); }
			catch (const std::runtime_error&) { Interrupted = true; }
			EXPECT(Interrupted && PartialTriggerSeen == AfterImpact);
			EXPECT(Fast->mMotionType == MorionType::Continuous && Discrete->mMotionType == MorionType::Discrete);
			EXPECT(Fast->X == FastStart && Discrete->X == DiscreteStart);
			EXPECT(Discrete->Q.x == Rotation.x && Discrete->Q.y == Rotation.y && Discrete->Q.z == Rotation.z && Discrete->Q.w == Rotation.w);
			EXPECT(!Fast->CCDPoseResolved && !Discrete->CCDPoseResolved && Triggers.empty());
			EXPECT(!Fast->HadCCDContact && Fast->CCDMotionFraction == 1);
			EXPECT(Frozen->mMotionType == MorionType::Discrete && Frozen->Freezing);
			EXPECT(Frozen->V == Vector3(0, 4, 0) && Frozen->W == Vector3(0, 0, 3));
			EXPECT_NEAR(Fast->GetGeometryTransform(Fast->mGeometries[0]).pos.x, FastStart.x, 1e-6f);
			EXPECT_NEAR(Discrete->GetGeometryTransform(Discrete->mGeometries[0]).pos.y, DiscreteStart.y, 1e-6f);
			// Failed world steps retain physical impulses; only temporary CCD state unwinds.
			EXPECT_NEAR(Fast->V.x, AfterImpact ? -100.0f : 100.0f, 0.01f);
			Fast->V = Vector3(100, 0, 0);
			World.Simulate(0.2f);
			EXPECT_NEAR(Fast->X.x, 7, 0.01f);
			EXPECT(Discrete->mMotionType == MorionType::Discrete);
		}
	}

	void TestConfiguration()
	{
		SceneWorld Scene; std::string Error;
		for (const char* Json : { R"({"physics":{"ccdContinuation":true}})",
			R"({"physics":{"ccdContinuation":{"enabled":1}}})", R"({"physics":{"ccdContinuation":{"maxPasses":0}}})",
			R"({"physics":{"ccdContinuation":{"maxPasses":1.5}}})", R"({"physics":{"ccdContinuation":{"velocityTolerance":-1}}})" })
		{
			EXPECT(!Scene.LoadFromText(Json, "invalid CCD", &Error)); EXPECT(Error.find("ccdContinuation") != std::string::npos);
		}
		EXPECT(Scene.LoadFromText(R"({"physics":{"ccdContinuation":{"enabled":true,"maxPasses":5,"impactIterations":20}},"objects":[]})", "CCD", &Error));
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			EXPECT(Scene.GetSimulation()->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			EXPECT(Scene.GetSimulation()->GetCCDContinuationSettings().Enabled);
			EXPECT(Scene.GetSimulation()->GetCCDContinuationSettings().MaxPasses == 5);
		}
		auto Bad = Scene.GetSimulation()->GetCCDContinuationSettings(); Bad.ImpactIterations = 0;
		bool Rejected = false; try { Scene.GetSimulation()->SetCCDContinuationSettings(Bad); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected && Scene.GetSimulation()->GetCCDContinuationSettings().ImpactIterations == 20);
	}
}
void TestCCDContinuation()
{
	printf("Running TestCCDContinuation\n");
	TestMultipleImpacts(); TestDynamicChain(); TestCornerAndBudget(); TestSegmentTriggers(); TestGeometryRoutes(); TestFieldAndFiltering(); TestJointCoordination(); TestExistingDemo(); TestExceptionalQueryCleanup(); TestConfiguration();
}

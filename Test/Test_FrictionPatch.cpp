#include "Test.h"
#include "../Src/RigidBodyDynamics/FrictionPatch.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Viewer/SceneWorld.h"
#include <memory>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	const float Dt = 1.0f / 60.0f;
	const float DynamicFriction = 0.3f, StaticFriction = 0.8f;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	const SolverBackend Backends[] = { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA };

	RigidBodyDynamic* MakeBody(PhysicsWorld& World, Geometry* Shape)
	{
		RigidBodyParam Param; Param.rigidType = RigidType::Dynamic;
		Param.sleepThreshold = 0; Param.maxContactImpulse = 100;
		Param.linearDamping = Param.angularDamping = 1;
		auto* Body = World.CreateRigidBody(Shape, Param)->CastDynamic();
		Body->InvMass = 1; Body->InvInertia = Matrix3(0, 0, 0);
		Body->SetFriction(DynamicFriction, StaticFriction);
		return Body;
	}
	RigidBody* MakeFloor(PhysicsWorld& World, const Vector3& Normal = Vector3::UnitY())
	{
		auto* Body = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Normal), RigidBodyParam());
		Body->SetFriction(DynamicFriction, StaticFriction);
		return Body;
	}
	Contact Point(const Vector3& Position, float NormalImpulse)
	{
		Contact C;
		C.PositionWorldA = C.PositionWorldB = Position;
		C.Normal = Vector3::UnitY(); C.Tangent = Vector3::UnitX(); C.Binormal = -Vector3::UnitZ();
		C.totalImpulseNormal = NormalImpulse; C.SolverSlop = 0.001f;
		return C;
	}
	void TestAnchors()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Contact C = Point(Vector3::Zero(), 2);
		std::vector<FrictionContact> Inputs = { { Floor->mGeometries[0], Body->mGeometries[0], 0, &C } };
		Solver.BeginFrame(1); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		Body->V.x = 1.0f;
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 0.0f, 1e-5f);
		EXPECT_NEAR(std::abs(C.totalImpulseTangent), 1.0f, 1e-5f);
		EXPECT(Solver.GetStatistics().SlidingAnchors == 0);
		Body->V.x = 3.0f;
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 3.4f, 1e-5f); // Release the old 1 Ns, then apply the 0.6 Ns kinetic bound.
		EXPECT(Solver.GetStatistics().SlidingAnchors == 1);
		Solver.BeginFrame(2); Solver.BeginSolve(); Body->V = Vector3::Zero();
		Solver.Prepare(Inputs, Dt); Solver.Solve(Dt); Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		EXPECT_NEAR(Body->V.x, 0.0f, 1e-5f);
		EXPECT(Solver.GetStatistics().SlidingAnchors == 0);
		// Fresh narrow-phase points move with the body; the sticking anchor must not.
		Body->X.x = 0.001f; C.PositionWorldA.x = C.PositionWorldB.x = Body->X.x;
		Solver.BeginFrame(3); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt); Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		EXPECT_NEAR(Body->V.x, -Settings.PositionBias * Body->X.x / Dt, 1e-5f);
		// A contact gap cannot carry friction even when the normal solver anticipates impact.
		C.PenetrationDepth = -0.01f; Body->V.x = 10;
		Solver.BeginFrame(4); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt); Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 10, 1e-5f);
		EXPECT(C.totalImpulseTangent == 0 && C.totalImpulseBinormal == 0);
		C.PenetrationDepth = 0;
		Solver.BeginFrame(7); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 0);
		Body->mGeometries[0]->SetSimulationEnabled(false);
		Solver.BeginFrame(8); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().Patches == 0);
		Body->mGeometries[0]->SetSimulationEnabled(true);
		Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 0);
		Settings.PositionBias = std::numeric_limits<float>::quiet_NaN();
		EXPECT(!Settings.IsValid());
		bool Rejected = false;
		try { Body->SetFriction(1, 0); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
		Body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		EXPECT(Body->GetFrictionDynamic() == 0 && Body->GetFrictionStatic() == 0);
	}

	void TestPatchMoment()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1, 0.5f, 0.5f)));
		Body->InvInertia = Matrix3(1, 1, 1); Body->W = Vector3(0, 3, 0);
		Contact A = Point(Vector3(-1, 0, 0), 1), B = Point(Vector3(1, 0, 0), 1);
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Solver.BeginFrame(1); Solver.BeginSolve();
		Solver.Prepare({ { Floor->mGeometries[0], Body->mGeometries[0], 0, &A },
			{ Floor->mGeometries[0], Body->mGeometries[0], 0, &B } }, Dt);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().Patches == 1 && Solver.GetStatistics().Anchors == 2);
		EXPECT_NEAR(Body->W.y, 2.4f, 1e-4f);
		EXPECT(Body->V.Length() < 1e-4f);
		// A pinned center of mass can still rotate and must receive friction torque.
		Body->InvMass = 0; Body->W = Vector3(0, 3, 0); Body->V = Vector3::Zero();
		Solver.Reset(); Solver.BeginFrame(2); Solver.BeginSolve();
		Solver.Prepare({ { Floor->mGeometries[0], Body->mGeometries[0], 0, &A },
			{ Floor->mGeometries[0], Body->mGeometries[0], 0, &B } }, Dt);
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->W.y, 2.4f, 1e-4f);
		EXPECT(Body->V.Length() < 1e-4f);
	}

	void TestSpatialPersistence()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
		Body->InvInertia = Matrix3(1, 2, 3);
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Contact C = Point(Vector3(0.2f, 0, 0.1f), 2);
		std::vector<FrictionContact> Inputs = { { Floor->mGeometries[0], Body->mGeometries[0], 0, &C } };
		Body->V = Vector3(8, 0, 6);
		Solver.BeginFrame(1); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		const Vector3 Velocity = Body->V, Angular = Body->W;
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.7f);
		const Vector3 Translation(4, -2, 7);
		Body->X = Rotation * Body->X + Translation; Floor->X = Translation;
		Body->Q = Floor->Q = Rotation; Body->V = Rotation * Vector3(8, 0, 6); Body->W = Vector3::Zero();
		C.PositionWorldA = C.PositionWorldB = Rotation * C.PositionWorldA + Translation;
		C.Normal = Rotation * C.Normal; C.Tangent = Rotation * C.Tangent; C.Binormal = Rotation * C.Binormal;
		Solver.BeginFrame(2); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		EXPECT((Body->V - Rotation * Velocity).Length() < 1e-4f);
		EXPECT((Body->W - Rotation * Angular).Length() < 1e-4f);
		std::swap(Inputs[0].ShapeA, Inputs[0].ShapeB);
		C.Normal = -C.Normal; C.Tangent = -C.Tangent;
		Solver.BeginFrame(3); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
	}

	void TestSolverRoutes()
	{
		for (auto Type : Solvers)
		for (auto Backend : Backends)
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Type, Backend)) continue;
			for (bool Joined : { false, true })
			for (float Angle : { 0.0f, 0.4f, 0.785398163f, 1.3f })
			{
				PhysicsWorld World{ PhysicsWorldParam() };
				auto* Floor = MakeFloor(World);
				auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
				auto* Partner = Joined ? MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 1.5f, 0), 0.5f)) : nullptr;
				std::vector<PositionConstraint*> Joints;
				if (Partner) Joints.push_back(World.CreateFixedConstraint(Body, Partner));
				std::vector<Geometry*> Geometries = { Floor->mGeometries[0], Body->mGeometries[0] };
				Contact C = Point(Vector3::Zero(), 0); C.PositionLocalB = Vector3(0, -0.5f, 0);
				ContactManifold M; M.AddNewContact(0, 1, C); std::vector<ContactManifold*> Contacts = { &M };
				const Vector3 Direction(std::cos(Angle), 0, std::sin(Angle));
				FrictionPatchSettings Friction; Friction.Enabled = true;
				std::unique_ptr<ConstraintSolver> Native;
				std::unique_ptr<PBDSolver> Position;
				if (Type == ResolutionPhaseSolver::SequentialImpulse) Native.reset(ConstraintSolver::CreateSequentialImpulseSolver());
				else if (Type == ResolutionPhaseSolver::LCPGlobal)
				{
					LCPSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Native.reset(CreateGlobalLCPSolver(Settings));
				}
				else
				{
					XPBDSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Position.reset(Type == ResolutionPhaseSolver::PBD ? new PBDSolver(Settings) : new XPBDSolver(Settings));
				}
				if (Native) Native->SetFrictionPatchSettings(Friction);
				if (Position) Position->SetFrictionPatchSettings(Friction);
				auto Step = [&](uint64_t Frame)
				{
					if (Partner) Partner->V = Body->V;
					for (auto* Joint : Joints) Joint->BeginStep(Dt, Type == ResolutionPhaseSolver::XPBD);
					if (Native)
					{
						Native->BeginFrame(Frame); Native->PreResolve(Geometries, Joints);
						Native->ResolveConstraints(Geometries, Contacts, Joints, Dt, 32); Native->PostResolve(Geometries);
					}
					else
					{
						Position->BeginStep(Partner ? std::vector<RigidBodyDynamic*>{ Body, Partner } : std::vector<RigidBodyDynamic*>{ Body }, Dt);
						Position->AddContacts(Geometries, Contacts);
						for (int Iteration = 0; Iteration < 32; ++Iteration)
						{
							Position->SolveJointVelocities(Joints, Dt);
							Position->SolveVelocities(Dt);
						}
						Position->EndStep(Dt);
					}
					if (Partner) EXPECT((Body->V - Partner->V).Length() < 0.001f);
				};
				Body->V = Direction * 10.0f + Vector3(0, -2, 0);
				Step(1);
				EXPECT((Body->V - Direction * 9.4f).Length() < 0.001f);
				Body->V = Vector3(0, -2, 0); Step(2);
				Body->V = Direction * 0.8f + Vector3(0, -2, 0); Step(3);
				EXPECT(Body->V.Length() < 0.001f); // Above the kinetic bound, below the static bound.
				Body->V = Direction * 3.0f + Vector3(0, -2, 0); Step(4);
				EXPECT((Body->V - Direction * 2.4f).Length() < 0.001f);
			}
			printf("Friction route solver=%d backend=%d: rotation, static load, slip and restick checked\n", (int)Type, (int)Backend);
		}
	}

	void TestSlopeAndDemos()
	{
		{
			PhysicsWorldParam Param; Param.resolutionSolver = ResolutionPhaseSolver::XPBD;
			Param.gravityAcc = Vector3(0, -10, 0); Param.frictionPatches.Enabled = true;
			Param.xpbdSettings.ContactCompliance = 0.001f;
			PhysicsWorld World(Param); MakeFloor(World);
			auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
			for (int Frame = 0; Frame < 120; ++Frame) World.Simulate(Dt);
			EXPECT_NEAR(Body->X.y, 0.5f - Param.xpbdSettings.PenetrationSlop - 10.0f * Param.xpbdSettings.ContactCompliance, 0.001f);
		}
		for (auto Type : Solvers)
		for (float Angle : { 0.45f, 0.8f })
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Type; Param.gravityAcc = Vector3(0, -10, 0);
			Param.frictionPatches.Enabled = true;
			float ReportedNormal = 0.0f;
			Param.onContact = [&](const PhysicsContactInfo& Info)
			{
				for (int Index = 0; Index < Info.NumContactPoints; ++Index) ReportedNormal += Info.ContactPoints[Index].NormalImpulse;
			};
			PhysicsWorld World(Param);
			const Vector3 Normal(-std::sin(Angle), std::cos(Angle), 0), Tangent(std::cos(Angle), std::sin(Angle), 0);
			MakeFloor(World, Normal);
			auto* Body = MakeBody(World, GeometryFactory::CreateOBB(Normal * 0.5f, Vector3(0.5f, 0.5f, 0.5f)));
			Body->Q.FromRotationAxis(Vector3::UnitZ(), Angle);
			for (int Frame = 0; Frame < 120; ++Frame) { ReportedNormal = 0; World.Simulate(Dt); }
			printf("Friction slope solver=%d angle=%g drift=%g speed=%g\n", (int)Type, Angle, Body->X.Dot(Tangent), Body->V.Dot(Tangent));
			if (Angle < 0.5f)
			{
				EXPECT(std::abs(Body->X.Dot(Tangent)) < 0.02f);
				EXPECT(std::abs(Body->V.Dot(Tangent)) < 0.02f);
				EXPECT_NEAR(ReportedNormal, 10.0f * std::cos(Angle) * Dt, 0.002f);
				// Static anchors must stop continuing creep, not merely slow it down.
				const Vector3 Settled = Body->X;
				for (int Frame = 0; Frame < 120; ++Frame) World.Simulate(Dt);
				EXPECT(std::abs((Body->X - Settled).Dot(Tangent)) < 0.001f);
			}
			else EXPECT(Body->V.Dot(Tangent) < -1.0f);
		}
		SceneWorld Scene;
		std::string Error;
		EXPECT(Scene.LoadFromText(R"({"physics":{"frictionPatches":true},"objects":[]})", "friction", &Error));
		EXPECT(Scene.GetSimulation()->GetFrictionPatchSettings().Enabled);
		for (auto Type : Solvers)
		{
			EXPECT(Scene.GetSimulation()->SetSolverConfiguration(Type, SolverBackend::CPU, false, &Error));
			EXPECT(Scene.GetSimulation()->GetFrictionPatchSettings().Enabled);
		}
		EXPECT(!Scene.LoadFromText(R"({"physics":{"frictionPatches":1},"objects":[]})", "invalid friction", &Error));
		EXPECT(Error.find("frictionPatches") != std::string::npos);
		EXPECT(Scene.LoadFromText(R"({"objects":[{"type":"box","frictionDynamic":0.3,"frictionStatic":0.8}]})", "friction material", &Error));
		RigidBody* MaterialBody = Scene.GetObjects()[0].GeometryPtr->GetParent<RigidBody>();
		EXPECT_NEAR(MaterialBody->GetFrictionDynamic(), DynamicFriction, 1e-6f);
		EXPECT_NEAR(MaterialBody->GetFrictionStatic(), StaticFriction, 1e-6f);
		for (const char* Fields : { R"("frictionDynamic":-1)", R"("frictionStatic":"high")", R"("frictionDynamic":0.8,"frictionStatic":0.3)" })
		{
			EXPECT(!Scene.LoadFromText(std::string(R"({"objects":[{"type":"box",)") + Fields + "}]}", "invalid friction material", &Error));
			EXPECT(Error.find("friction") != std::string::npos);
		}
		for (const char* File : { "Scenes/02_stack_boxes.json", "Scenes/06_constraints.json" })
		for (auto Type : Solvers)
		{
			EXPECT(Scene.LoadFromFile(TestContentsPath() + File, &Error));
			PhysicsWorld* World = Scene.GetSimulation();
			EXPECT(World->SetSolverConfiguration(Type, SolverBackend::CPU, false, &Error));
			FrictionPatchSettings Settings = World->GetFrictionPatchSettings(); Settings.Enabled = true;
			World->SetFrictionPatchSettings(Settings);
			for (int Frame = 0; Frame < 60; ++Frame) Scene.Step(Dt);
			for (const auto& Object : Scene.GetObjects())
			{
				RigidBody* Body = Object.GeometryPtr->GetParent<RigidBody>();
				EXPECT(std::isfinite(Body->V.Length()) && std::isfinite(Body->W.Length()));
				EXPECT(Body->X.y > -5.1f); // This existing demo's ground plane is at y = -5.
			}
		}
	}
}

void TestFrictionPatches()
{
	TestAnchors(); TestPatchMoment(); TestSpatialPersistence(); TestSolverRoutes(); TestSlopeAndDemos();
}

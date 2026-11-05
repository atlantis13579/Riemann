#include "Test.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Cloth/Rope.h"
#include "../Src/Deformation/VehicleDeformation.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/RigidBodyDynamics/LCPSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse,
		ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	const SolverBackend Backends[] = { SolverBackend::CPU, SolverBackend::SIMD,
		SolverBackend::ComputeShader, SolverBackend::CUDA };
	const char* SolverNames[] = { "SI", "LCP", "PBD", "XPBD" };
	const char* BackendNames[] = { "CPU", "SIMD", "ComputeShader", "CUDA" };

	bool Finite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	struct BodyState
	{
		RigidBodyDynamic* Body;
		Vector3 Position, Velocity, AngularVelocity, Force, Torque;
		Quaternion Rotation;
		Geometry* Shape;
		GeometryHandle Handle;
		bool Sleeping, Freezing;

		explicit BodyState(RigidBodyDynamic* body)
			: Body(body), Position(body->X), Velocity(body->V), AngularVelocity(body->W),
			Force(body->ExtForce), Torque(body->ExtTorque), Rotation(body->Q),
			Shape(body->Geometries()[0]), Handle(body->GetGeometryHandle(0)),
			Sleeping(body->Sleeping), Freezing(body->Freezing) {}

		void Check(bool unchangedSleep) const
		{
			EXPECT((Body->X - Position).SquareLength() == 0.0f);
			EXPECT((Body->Q - Rotation).SquareLength() == 0.0f);
			EXPECT((Body->V - Velocity).SquareLength() == 0.0f);
			EXPECT((Body->W - AngularVelocity).SquareLength() == 0.0f);
			EXPECT((Body->ExtForce - Force).SquareLength() == 0.0f);
			EXPECT((Body->ExtTorque - Torque).SquareLength() == 0.0f);
			EXPECT(Body->Geometries()[0] == Shape);
			EXPECT(Shape->GetParent<RigidBody>() == Body);
			EXPECT(Body->GetGeometryHandle(0) == Handle);
			EXPECT(Body->Freezing == Freezing);
			if (unchangedSleep) EXPECT(Body->Sleeping == Sleeping);
		}
	};

	class CountingField : public ForceField
	{
	public:
		int Updates = 0;
		void Update(float) override { ++Updates; }
		bool ApplyForce(RigidBodyDynamic*) override { return false; }
	};

	PhysicsWorldParam SwitchSettings(int* contacts)
	{
		PhysicsWorldParam settings;
		settings.gravityAcc = Vector3(0, -9.8f, 0);
		settings.allowBackendFallback = false;
		settings.contactSolverIterations = 41;
		settings.contactSolverTolerance = 2.0e-6f;
		settings.contactSolverRelaxation = 0.9f;
		settings.pbdSettings.PositionIterations = 7;
		settings.pbdSettings.VelocityIterations = 3;
		settings.pbdSettings.MaxPositionCorrection = 0.13f;
		settings.xpbdSettings.PositionIterations = 9;
		settings.xpbdSettings.VelocityIterations = 5;
		settings.xpbdSettings.ContactCompliance = 0.00002f;
		settings.onContact = [contacts](const PhysicsContactInfo&) { ++*contacts; };
		return settings;
	}

	struct SwitchFixture
	{
		int Contacts = 0;
		PhysicsWorldParam Settings = SwitchSettings(&Contacts);
		PhysicsWorld World{ Settings };
		RigidBodyDynamic* ContactBody;
		RigidBodyDynamic* JointBody;
		RigidBodyDynamic* FrozenBody;
		RigidBody* Anchor;
		DistanceConstraint* Joint;
		RopeSystem* Rope;
		CountingField* Field;

		RigidBodyDynamic* Dynamic(const Vector3& position, bool collision = true)
		{
			Geometry* shape = GeometryFactory::CreateSphere(position, 0.5f);
			shape->SetSimulationEnabled(collision);
			RigidBodyParam settings;
			settings.rigidType = RigidType::Dynamic;
			settings.linearDamping = settings.angularDamping = 1.0f;
			settings.sleepThreshold = 0.0f;
			settings.maxContactImpulse = 1000.0f;
			RigidBodyDynamic* body = World.CreateRigidBody(shape, settings)->CastDynamic();
			body->InvMass = 1.0f;
			body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
			body->SetRestitution(0.0f);
			return body;
		}

		SwitchFixture()
		{
			RigidBodyParam fixed;
			RigidBody* ground = World.CreateRigidBody(
				GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), fixed);
			ground->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
			ground->SetRestitution(0.0f);
			ContactBody = Dynamic(Vector3(0, 0.48f, 0));
			JointBody = Dynamic(Vector3(3, 2, 0), false);
			FrozenBody = Dynamic(Vector3(6, 2, 0), false);
			FrozenBody->Freeze();
			Anchor = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(3, 3, 0), 0.1f), fixed);
			Anchor->Geometries()[0]->SetSimulationEnabled(false);
			Joint = World.CreateDistanceConstraint(Anchor, JointBody, Anchor->X, JointBody->X, 1.0f, 1.0f);
			Joint->SetCompliance(0.001f, 0.002f);
			RopeSettings ropeSettings;
			ropeSettings.SegmentCount = 4;
			ropeSettings.Gravity = Vector3::Zero();
			Rope = World.CreateRope(ropeSettings, Vector3(10, 3, 0), Vector3(12, 3, 0));
			EXPECT(Rope->PinParticle(0, Vector3(10, 3, 0)));
			EXPECT(Rope->PinParticle(4, Vector3(12, 3, 0)));
			EXPECT(Rope->SetParticleVelocity(1, Vector3(0, 0.1f, 0)));
			Field = new CountingField;
			EXPECT(World.AddForceField(Field) == Field);
		}
	};

	struct WorldState
	{
		BodyState Contact, JointBody, Frozen;
		std::vector<Geometry*> Geometries;
		std::vector<RopeParticle> Particles;
		size_t Fields;
		int Updates;

		explicit WorldState(SwitchFixture& fixture)
			: Contact(fixture.ContactBody), JointBody(fixture.JointBody), Frozen(fixture.FrozenBody),
			Fields(fixture.World.GetForceFieldCount()), Updates(fixture.Field->Updates)
		{
			fixture.World.GetAllGeometries(&Geometries);
			for (int i = 0; i < fixture.Rope->GetParticleCount(); ++i) Particles.push_back(fixture.Rope->GetParticle(i));
		}

		void Check(SwitchFixture& fixture, bool unchangedSleep = false) const
		{
			Contact.Check(unchangedSleep);
			JointBody.Check(unchangedSleep);
			Frozen.Check(true);
			std::vector<Geometry*> geometries;
			fixture.World.GetAllGeometries(&geometries);
			EXPECT(geometries == Geometries);
			EXPECT(fixture.World.GetConstraintCount() == 1);
			EXPECT(fixture.Joint->GetBodyA() == fixture.Anchor && fixture.Joint->GetBodyB() == fixture.JointBody);
			EXPECT(fixture.Joint->IsEnabled());
			EXPECT_NEAR(fixture.Joint->GetMinDistance(), 1.0f, 0.0f);
			EXPECT_NEAR(fixture.Joint->GetMaxDistance(), 1.0f, 0.0f);
			EXPECT_NEAR(fixture.Joint->GetLinearCompliance(), 0.001f, 0.0f);
			EXPECT_NEAR(fixture.Joint->GetAngularCompliance(), 0.002f, 0.0f);
			EXPECT(fixture.World.GetRopeCount() == 1);
			EXPECT(fixture.Rope->GetParticleCount() == (int)Particles.size());
			for (size_t i = 0; i < Particles.size(); ++i)
			{
				const RopeParticle& particle = fixture.Rope->GetParticle((int)i);
				EXPECT((particle.Position - Particles[i].Position).SquareLength() == 0.0f);
				EXPECT((particle.PreviousPosition - Particles[i].PreviousPosition).SquareLength() == 0.0f);
				EXPECT(particle.RenderGeometry == Particles[i].RenderGeometry);
				EXPECT(particle.Pinned == Particles[i].Pinned);
			}
			EXPECT(fixture.World.GetForceFieldCount() == Fields);
			EXPECT(fixture.Field->Updates == Updates);
		}
	};

	void CheckPositionSettings(const SwitchFixture& fixture, ResolutionPhaseSolver solver)
	{
		EXPECT(fixture.World.GetResolutionSolver() == solver);
		if (fixture.World.GetResolutionSolver() != solver) return;
		const PBDSolver* position = fixture.World.GetPositionSolver();
		if (solver != ResolutionPhaseSolver::PBD && solver != ResolutionPhaseSolver::XPBD)
		{
			EXPECT(position == nullptr);
			return;
		}
		EXPECT(position != nullptr);
		if (!position) return;
		const PBDSolverSettings& expected = solver == ResolutionPhaseSolver::PBD
			? fixture.Settings.pbdSettings : fixture.Settings.xpbdSettings;
		const PBDSolverSettings& actual = position->GetSettings();
		EXPECT(actual.PositionIterations == expected.PositionIterations);
		EXPECT(actual.VelocityIterations == expected.VelocityIterations);
		EXPECT_NEAR(actual.MaxPositionCorrection, expected.MaxPositionCorrection, 0.0f);
		if (solver == ResolutionPhaseSolver::XPBD)
			EXPECT_NEAR(static_cast<const XPBDSolver*>(position)->GetXPBDSettings().ContactCompliance,
				fixture.Settings.xpbdSettings.ContactCompliance, 0.0f);
	}

	void TestSwitchMatrix()
	{
		SwitchFixture fixture;
		for (int solverIndex = 0; solverIndex < 4; ++solverIndex)
		for (int backendIndex = 0; backendIndex < 4; ++backendIndex)
		{
			const ResolutionPhaseSolver solver = Solvers[solverIndex];
			const SolverBackend backend = Backends[backendIndex];
			const bool expectedAvailable = solver == ResolutionPhaseSolver::SequentialImpulse
				? backend == SolverBackend::CPU : (solver == ResolutionPhaseSolver::LCPGlobal
					? LCPSolver::IsBackendAvailable(backend) : IsPBDBackendAvailable(backend));
			EXPECT(PhysicsWorld::IsSolverBackendAvailable(solver, backend) == expectedAvailable);
			if (!expectedAvailable) continue;
			fixture.ContactBody->X.y = 0.48f;
			fixture.ContactBody->V = Vector3(0.1f, -0.2f, 0.05f);
			fixture.JointBody->ExtForce = Vector3(0.1f, 0.2f, 0.3f);
			fixture.JointBody->ExtTorque = Vector3(0.02f, 0.03f, 0.01f);
			const WorldState before(fixture);
			std::string error = "stale error";
			EXPECT(fixture.World.SetSolverConfiguration(solver, backend, false, &error));
			EXPECT(error.empty());
			EXPECT(fixture.World.GetResolutionSolver() == solver);
			EXPECT(fixture.World.GetSolverBackend() == backend);
			EXPECT(fixture.World.GetRequestedSolverBackend() == backend);
			EXPECT_FALSE(fixture.World.GetAllowBackendFallback());
			before.Check(fixture);
			CheckPositionSettings(fixture, solver);
			const int previousContacts = fixture.Contacts;
			for (int frame = 0; frame < 4; ++frame) fixture.World.Simulate(1.0f / 60.0f);
			EXPECT(fixture.Contacts > previousContacts);
			EXPECT(fixture.Field->Updates > before.Updates);
			EXPECT(Finite(fixture.ContactBody->X) && Finite(fixture.ContactBody->V) && Finite(fixture.JointBody->X));
			EXPECT(fixture.ContactBody->X.y > 0.4f);
			EXPECT_NEAR(fixture.JointBody->Q.SquareLength(), 1.0f, 1.0e-4f);
			const PBDSolver* position = fixture.World.GetPositionSolver();
			if (position)
			{
				const PBDBackendStatistics stats = position->GetBackendStatistics();
				EXPECT(stats.RequestedBackend == backend && stats.Backend == backend && !stats.UsedFallback);
				EXPECT(stats.PositionDispatches > 0 && stats.ReconstructionDispatches > 0 && stats.VelocityDispatches > 0);
				const WorldState noOp(fixture);
				EXPECT(fixture.World.SetSolverConfiguration(solver, backend, false, &error));
				EXPECT(fixture.World.GetPositionSolver() == position);
				EXPECT(position->GetBackendStatistics().PositionDispatches == stats.PositionDispatches);
				noOp.Check(fixture, true);
			}
			printf("  runtime switch %s/%s: bodies, joint, rope, field and contacts preserved\n",
				SolverNames[solverIndex], BackendNames[backendIndex]);
		}
		// Restore earlier algorithms after every backend has been exercised.
		for (ResolutionPhaseSolver solver : Solvers)
		{
			EXPECT(fixture.World.SetSolverConfiguration(solver, SolverBackend::CPU, false));
			CheckPositionSettings(fixture, solver);
			fixture.World.Simulate(1.0f / 60.0f);
		}
	}

	void CheckRejectedSwitch(SwitchFixture& fixture, ResolutionPhaseSolver solver, SolverBackend backend, bool fallback)
	{
		const WorldState before(fixture);
		const PBDSolver* previousSolver = fixture.World.GetPositionSolver();
		const ResolutionPhaseSolver previousAlgorithm = fixture.World.GetResolutionSolver();
		const SolverBackend previousBackend = fixture.World.GetSolverBackend();
		const SolverBackend previousRequested = fixture.World.GetRequestedSolverBackend();
		const bool previousFallback = fixture.World.GetAllowBackendFallback();
		std::string error;
		EXPECT_FALSE(fixture.World.SetSolverConfiguration(solver, backend, fallback, &error));
		EXPECT(!error.empty());
		EXPECT(fixture.World.GetPositionSolver() == previousSolver);
		EXPECT(fixture.World.GetResolutionSolver() == previousAlgorithm);
		EXPECT(fixture.World.GetSolverBackend() == previousBackend);
		EXPECT(fixture.World.GetRequestedSolverBackend() == previousRequested);
		EXPECT(fixture.World.GetAllowBackendFallback() == previousFallback);
		before.Check(fixture, true);
	}

	void TestRejectedAndFallbackSwitches()
	{
		SwitchFixture fixture;
		EXPECT(fixture.World.SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false));
		fixture.World.Simulate(1.0f / 60.0f);
		fixture.ContactBody->Sleeping = true;
		for (bool fallback : { false, true })
		{
			CheckRejectedSwitch(fixture, static_cast<ResolutionPhaseSolver>(255), SolverBackend::CPU, fallback);
			CheckRejectedSwitch(fixture, ResolutionPhaseSolver::PBD, static_cast<SolverBackend>(255), fallback);
			for (int i = 1; i < 4; ++i)
				CheckRejectedSwitch(fixture, ResolutionPhaseSolver::SequentialImpulse, Backends[i], fallback);
		}
		EXPECT_FALSE(PhysicsWorld::IsSolverBackendAvailable(static_cast<ResolutionPhaseSolver>(255), SolverBackend::CPU));
		EXPECT_FALSE(PhysicsWorld::IsSolverBackendAvailable(ResolutionPhaseSolver::PBD, static_cast<SolverBackend>(255)));
		for (int solverIndex = 1; solverIndex < 4; ++solverIndex)
		for (SolverBackend backend : Backends)
		{
			const ResolutionPhaseSolver solver = Solvers[solverIndex];
			if (PhysicsWorld::IsSolverBackendAvailable(solver, backend)) continue;
			CheckRejectedSwitch(fixture, solver, backend, false);
			const WorldState before(fixture);
			std::string error;
			EXPECT(fixture.World.SetSolverConfiguration(solver, backend, true, &error));
			EXPECT(error.empty());
			EXPECT(fixture.World.GetResolutionSolver() == solver);
			EXPECT(fixture.World.GetRequestedSolverBackend() == backend);
			EXPECT(fixture.World.GetSolverBackend() == SolverBackend::CPU);
			EXPECT(fixture.World.GetAllowBackendFallback());
			before.Check(fixture);
			if (const PBDSolver* position = fixture.World.GetPositionSolver())
			{
				EXPECT(position->GetBackendStatistics().UsedFallback);
				EXPECT(!position->GetBackendStatistics().Message.empty());
			}
			fixture.World.Simulate(1.0f / 60.0f);
			EXPECT(Finite(fixture.ContactBody->X));
			EXPECT(fixture.World.SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false));
		}
	}

	void TestSwitchPreservesVehicleScene()
	{
		SceneWorld scene;
		std::string error;
		EXPECT(scene.LoadFromFile(TestContentsPath() + "Scenes/19_vehicle_deformation.json", &error));
		EXPECT(scene.GetVehicleCount() == 1);
		if (scene.GetVehicleCount() != 1) return;
		VehicleDeformation* vehicle = scene.GetVehicle(0);
		// Wait for the longer approach to produce damage before exercising solver changes.
		for (int frame = 0; frame < 120 && vehicle->GetMaxPlasticStrain() == 0.0f; ++frame)
			scene.Step(1.0f / 60.0f);
		PhysicsWorld* world = scene.GetSimulation();
		RigidBody* chassis = vehicle->GetBody();
		EXPECT(vehicle->GetMaxPlasticStrain() > 0.0f);
		for (ResolutionPhaseSolver solver : Solvers)
		{
			const std::vector<SceneObjectInstance> objects = scene.GetObjects();
			const uint32_t revision = scene.GetRenderRevision();
			const float strain = vehicle->GetMaxPlasticStrain();
			const float displacement = vehicle->GetMaxDisplacement();
			const Vector3 position = chassis->X;
			EXPECT(world->SetSolverConfiguration(solver, SolverBackend::CPU, false, &error));
			EXPECT(scene.GetSimulation() == world && scene.GetVehicle(0) == vehicle && vehicle->GetBody() == chassis);
			EXPECT(scene.GetRenderRevision() == revision);
			EXPECT(scene.GetObjects().size() == objects.size());
			for (size_t i = 0; i < objects.size() && i < scene.GetObjects().size(); ++i)
			{
				EXPECT(scene.GetObjects()[i].Id == objects[i].Id);
				EXPECT(scene.GetObjects()[i].GeometryPtr == objects[i].GeometryPtr);
			}
			EXPECT((chassis->X - position).SquareLength() == 0.0f);
			EXPECT_NEAR(vehicle->GetMaxPlasticStrain(), strain, 0.0f);
			EXPECT_NEAR(vehicle->GetMaxDisplacement(), displacement, 0.0f);
			scene.Step(1.0f / 60.0f);
			EXPECT(Finite(chassis->X) && Finite(chassis->V));
			EXPECT(vehicle->GetMaxPlasticStrain() > 0.0f);
		}
	}

	void TestInactiveSceneSolverSettings()
	{
		SceneWorld scene;
		std::string error;
		const char* text = R"JSON({
			"name": "Solver settings retention",
			"physics": {
				"resolutionSolver": "sequentialimpulse",
				"pbd": { "positionIterations": 7 },
				"xpbd": { "positionIterations": 11, "contactCompliance": 0.003 }
			},
			"objects": [{ "id": "body", "type": "sphere", "radius": 0.5,
				"position": [0, 2, 0], "body": "dynamic" }]
		})JSON";
		EXPECT(scene.LoadFromText(text, "runtime_solver_settings", &error));
		PhysicsWorld* world = scene.GetSimulation();
		EXPECT(world->GetResolutionSolver() == ResolutionPhaseSolver::SequentialImpulse);
		EXPECT(scene.GetObjects().size() == 1);
		if (scene.GetObjects().size() != 1) return;
		Geometry* shape = scene.GetObjects()[0].GeometryPtr;
		EXPECT(world->SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false, &error));
		EXPECT(world->GetPositionSolver() != nullptr);
		if (const PBDSolver* position = world->GetPositionSolver())
			EXPECT(position->GetSettings().PositionIterations == 7);
		EXPECT(world->SetSolverConfiguration(ResolutionPhaseSolver::XPBD, SolverBackend::CPU, false, &error));
		EXPECT(world->GetPositionSolver() != nullptr);
		if (const PBDSolver* position = world->GetPositionSolver())
		{
			EXPECT(position->GetSettings().PositionIterations == 11);
			EXPECT_NEAR(static_cast<const XPBDSolver*>(position)->GetXPBDSettings().ContactCompliance, 0.003f, 0.0f);
		}
		EXPECT(scene.GetSimulation() == world);
		EXPECT(scene.GetObjects()[0].GeometryPtr == shape);
	}
}

void TestSolverSwitch()
{
	printf("Running TestSolverSwitch\n");
	const char* requireGPU = std::getenv("RIEMANN_REQUIRE_PBD_GPU");
	if (requireGPU && std::strcmp(requireGPU, "1") == 0)
	{
		EXPECT(PhysicsWorld::IsSolverBackendAvailable(ResolutionPhaseSolver::PBD, SolverBackend::ComputeShader));
		EXPECT(PhysicsWorld::IsSolverBackendAvailable(ResolutionPhaseSolver::XPBD, SolverBackend::CUDA));
	}
	TestSwitchMatrix();
	TestRejectedAndFallbackSwitches();
	TestSwitchPreservesVehicleScene();
	TestInactiveSceneSolverSettings();
}

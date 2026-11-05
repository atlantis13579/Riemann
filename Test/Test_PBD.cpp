#include "Test.h"

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Cloth/Rope.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	PhysicsWorldParam PBDWorldSettings(const Vector3& gravity = Vector3::Zero())
	{
		PhysicsWorldParam settings;
		settings.resolutionSolver = ResolutionPhaseSolver::PBD;
		settings.gravityAcc = gravity;
		settings.pbdSettings.PositionIterations = 20;
		settings.pbdSettings.VelocityIterations = 8;
		return settings;
	}

	RigidBodyDynamic* PBDDynamic(PhysicsWorld& world, Geometry* geometry, const Vector3& velocity = Vector3::Zero())
	{
		RigidBodyParam settings;
		settings.rigidType = RigidType::Dynamic;
		settings.linearVelocity = velocity;
		settings.linearDamping = 1.0f;
		settings.angularDamping = 1.0f;
		settings.sleepThreshold = 0.0f;
		settings.maxContactImpulse = 100.0f;
		RigidBodyDynamic* body = world.CreateRigidBody(geometry, settings)->CastDynamic();
		body->InvMass = 1.0f;
		return body;
	}

	RigidBody* PBDGround(PhysicsWorld& world, unsigned short material = DefaultPhysicsMaterial::Ice)
	{
		RigidBodyParam settings;
		settings.rigidType = RigidType::Static;
		RigidBody* ground = world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), settings);
		ground->SetDefaultPhysicsMaterial(material);
		return ground;
	}

	void TestPBDFreeMotion()
	{
		PhysicsWorld world(PBDWorldSettings());
		EXPECT(world.GetResolutionSolver() == ResolutionPhaseSolver::PBD);
		EXPECT(world.GetPositionSolver() != nullptr);
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateSphere(Vector3(1, 2, 3), 0.2f), Vector3(2, -1, 0.5f));
		body->W = Vector3(0, 0, 1);
		for (int frame = 0; frame < 60; ++frame) world.Simulate();
		EXPECT_SAME(body->X, Vector3(3, 1, 3.5f));
		EXPECT_SAME(body->V, Vector3(2, -1, 0.5f));
		EXPECT_NEAR(body->W.z, 1.0f, 0.002f);
		EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1e-5f);
		const Vector3 rotatedX = body->Q * Vector3::UnitX();
		EXPECT_NEAR(rotatedX.x, std::cos(1.0f), 0.003f);
		EXPECT_NEAR(rotatedX.y, std::sin(1.0f), 0.003f);

		PhysicsWorld gravityWorld(PBDWorldSettings(Vector3(0, -10, 0)));
		RigidBodyDynamic* falling = PBDDynamic(gravityWorld, GeometryFactory::CreateSphere(Vector3(0, 5, 0), 0.2f));
		for (int frame = 0; frame < 30; ++frame) gravityWorld.Simulate();
		EXPECT_NEAR(falling->V.y, -5.0f, 0.002f);
		EXPECT_NEAR(falling->X.y, 3.75f, 0.06f);

		// A small unsupported body must not sleep just because its first-frame energy is low.
		PhysicsWorld defaultSleepWorld(PBDWorldSettings(Vector3(0, -10, 0)));
		RigidBodyParam defaultBody;
		defaultBody.rigidType = RigidType::Dynamic;
		RigidBodyDynamic* small = defaultSleepWorld.CreateRigidBody(
			GeometryFactory::CreateSphere(Vector3(0, 5, 0), 0.2f), defaultBody)->CastDynamic();
		defaultSleepWorld.Simulate();
		EXPECT_FALSE(small->Sleeping);
		EXPECT(small->V.y < -0.1f);
		for (int frame = 1; frame < 30; ++frame) defaultSleepWorld.Simulate();
		EXPECT(small->X.y < 4.0f);
		EXPECT_FALSE(small->Sleeping);
	}

	float PBDBounceVelocity(float restitution)
	{
		int reportedPoints = 0;
		float reportedImpulse = 0.0f;
		PhysicsWorldParam settings = PBDWorldSettings();
		settings.onContact = [&](const PhysicsContactInfo& info)
		{
			EXPECT(info.BodyA != nullptr && info.BodyB != nullptr && info.GeomA != nullptr && info.GeomB != nullptr);
			for (int i = 0; i < info.NumContactPoints; ++i)
			{
				const PhysicsContactPoint& point = info.ContactPoints[i];
				EXPECT_NEAR(point.Normal.Length(), 1.0f, 1e-4f);
				EXPECT(std::isfinite(point.NormalImpulse) && point.NormalImpulse >= 0.0f);
				EXPECT(std::isfinite(point.Position.x) && std::isfinite(point.Position.y) && std::isfinite(point.Position.z));
				EXPECT((point.Impulse - point.Normal * point.NormalImpulse).Length() < 1e-4f);
				reportedImpulse += point.NormalImpulse;
				++reportedPoints;
			}
		};
		PhysicsWorld world(settings);
		RigidBody* ground = PBDGround(world);
		ground->SetRestitution(restitution);
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateSphere(Vector3(0, 0.55f, 0), 0.5f), Vector3(0, -6, 0));
		body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		body->SetRestitution(restitution);
		world.Simulate();
		printf("  PBD bounce e=%.1f y=%.5f vy=%.5f contacts=%zu reportedImpulse=%.5f\n",
			restitution, body->X.y, body->V.y, world.GetPositionSolver()->GetContactCount(), reportedImpulse);
		EXPECT(body->X.y > 0.49f && body->X.y < 0.65f);
		EXPECT(world.GetPositionSolver()->GetContactCount() > 0);
		EXPECT(reportedPoints > 0);
		EXPECT(reportedImpulse > 0.1f && reportedImpulse < 20.0f);
		EXPECT_SAME(ground->X, Vector3::Zero());
		return body->V.y;
	}

	void TestPBDRestitutionAndAngularResponse()
	{
		const float stopped = PBDBounceVelocity(0.0f);
		const float bounced = PBDBounceVelocity(1.0f);
		printf("  PBD restitution velocities: zero=%.4f elastic=%.4f\n", stopped, bounced);
		EXPECT(std::abs(stopped) < 0.1f);
		EXPECT(bounced > 5.0f && bounced < 6.2f);

		PhysicsWorld world(PBDWorldSettings());
		PBDGround(world);
		Quaternion tilted;
		tilted.FromRotationAxis(Vector3::UnitZ(), 0.5f);
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateOBB(
			Vector3(0, 0.6f, 0), Vector3(1, 0.25f, 0.5f), tilted), Vector3(0, -2, 0));
		body->InvInertia = Matrix3(1.0f, 2.0f, 3.0f);
		body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		world.Simulate();
		EXPECT(body->W.z < -0.05f);
		EXPECT(std::isfinite(body->W.x) && std::isfinite(body->W.y) && std::isfinite(body->W.z));
		EXPECT(body->W.Length() < 20.0f);
		EXPECT((body->Q * Vector3::UnitX()).y < std::sin(0.5f));
	}

	void TestPBDInitialPenetrationEnergy()
	{
		PhysicsWorld world(PBDWorldSettings());
		RigidBody* ground = PBDGround(world);
		Geometry* shape = GeometryFactory::CreateOBB(Vector3(0, 0.3f, 0), Vector3(1, 0.5f, 0.5f));
		RigidBodyDynamic* body = PBDDynamic(world, shape);
		body->InvInertia = Matrix3::Identity();
		body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		ContactManifold manifold;
		manifold.indexA = 0;
		manifold.indexB = 1;
		manifold.NumContactPointCount = 1;
		Contact& contact = manifold.ContactPoints[0];
		contact.PositionWorldA = Vector3(-1, 0, 0);
		contact.PositionWorldB = Vector3(-1, -0.2f, 0);
		contact.Normal = Vector3::UnitY();
		contact.PenetrationDepth = 0.2f;
		PBDSolverSettings settings;
		settings.PositionIterations = 20;
		settings.VelocityIterations = 8;
		PBDSolver solver(settings);
		const float dt = 1.0f / 60.0f;
		solver.BeginStep({ body }, dt);
		solver.PredictPoses(dt);
		solver.AddContacts({ ground->Geometries()[0], shape }, { &manifold });
		for (int i = 0; i < settings.PositionIterations; ++i) solver.SolvePositions(dt);
		solver.ReconstructVelocities(dt);
		for (int i = 0; i < settings.VelocityIterations; ++i) solver.SolveVelocities(dt);
		solver.EndStep(dt);
		printf("  PBD depenetration: y=%.5f speed=%.6f angularSpeed=%.6f\n", body->X.y, body->V.Length(), body->W.Length());
		EXPECT(body->X.y > 0.3f);
		EXPECT((body->Q * Vector3::UnitX()).y < -0.01f);
		EXPECT(body->V.Length() < 0.001f);
		EXPECT(body->W.Length() < 0.001f);
		EXPECT(solver.GetContactManifolds().size() == 1);
		if (!solver.GetContactManifolds().empty())
			EXPECT(solver.GetContactManifolds()[0]->ContactPoints[0].totalImpulseNormal < 0.001f);
	}

	float PBDSlide(unsigned short material, float& distance)
	{
		PhysicsWorld world(PBDWorldSettings(Vector3(0, -10, 0)));
		PBDGround(world, material);
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateOBB(Vector3(0, 0.5f, 0), Vector3(0.5f)), Vector3(3, 0, 0));
		body->InvInertia = Matrix3::Zero();
		body->SetDefaultPhysicsMaterial(material);
		for (int frame = 0; frame < 60; ++frame) world.Simulate();
		distance = body->X.x;
		EXPECT_NEAR(body->X.y, 0.5f, 0.03f);
		EXPECT(std::abs(body->V.y) < 0.1f);
		return body->V.x;
	}

	void TestPBDFrictionAndStack()
	{
		float iceDistance, roughDistance;
		const float iceSpeed = PBDSlide(DefaultPhysicsMaterial::Ice, iceDistance);
		const float roughSpeed = PBDSlide(DefaultPhysicsMaterial::Concrete, roughDistance);
		printf("  PBD sliding: ice=(%.4f, %.4f) rough=(%.4f, %.4f)\n", iceSpeed, iceDistance, roughSpeed, roughDistance);
		EXPECT_NEAR(iceSpeed, 3.0f, 0.05f);
		EXPECT(std::abs(roughSpeed) < 0.5f);
		EXPECT(roughDistance < iceDistance * 0.5f);

		PhysicsWorld world(PBDWorldSettings(Vector3(0, -10, 0)));
		PBDGround(world, DefaultPhysicsMaterial::Concrete);
		std::vector<RigidBodyDynamic*> boxes;
		for (int i = 0; i < 3; ++i)
		{
			RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateOBB(Vector3(0, 0.5f + i, 0), Vector3(0.5f)));
			body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Concrete);
			boxes.push_back(body);
		}
		for (int frame = 0; frame < 240; ++frame) world.Simulate();
		for (size_t i = 0; i < boxes.size(); ++i)
		{
			const RigidBodyDynamic* body = boxes[i];
			printf("  PBD stack[%zu] y=%.4f speed=%.4f\n", i, body->X.y, body->V.Length());
			EXPECT_NEAR(body->X.y, 0.5f + i, 0.12f);
			EXPECT(std::abs(body->X.x) < 0.15f && std::abs(body->X.z) < 0.15f);
			EXPECT((body->Q * Vector3::UnitY()).y > 0.95f);
			EXPECT(body->V.Length() < 0.3f);
		}
	}

	void TestPBDFrozenAndJoints()
	{
		PhysicsWorld world(PBDWorldSettings(Vector3(0, -10, 0)));
		RigidBodyDynamic* frozen = PBDDynamic(world, GeometryFactory::CreateOBB(Vector3::Zero(), Vector3::One()));
		frozen->Freeze();
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateSphere(Vector3(0, 1.4f, 0), 0.5f));
		for (int frame = 0; frame < 60; ++frame) world.Simulate();
		EXPECT_SAME(frozen->X, Vector3::Zero());
		EXPECT(frozen->Q == Quaternion::One());
		EXPECT(frozen->Sleeping && frozen->Freezing);
		EXPECT_NEAR(body->X.y, 1.5f, 0.04f);

		RigidBodyKinematics kinematic;
		PhysicsWorld jointWorld(PBDWorldSettings());
		Geometry* followerShape = GeometryFactory::CreateSphere(Vector3::Zero(), 0.2f);
		followerShape->SetSimulationEnabled(false);
		RigidBodyDynamic* follower = PBDDynamic(jointWorld, followerShape);
		EXPECT(jointWorld.CreatePointConstraint(&kinematic, follower, Vector3::Zero()) != nullptr);
		kinematic.V = Vector3(1.2f, 0, 0);
		for (int frame = 0; frame < 12; ++frame)
		{
			kinematic.SetPosition(Vector3(0.02f * (frame + 1), 0, 0));
			jointWorld.Simulate();
		}
		EXPECT_SAME(kinematic.X, Vector3(0.24f, 0, 0));
		EXPECT_SAME(kinematic.V, Vector3(1.2f, 0, 0));
		EXPECT((follower->X - kinematic.X).Length() < 0.025f);
		EXPECT_NEAR(follower->V.x, 1.2f, 0.05f);

		PhysicsWorld pendulum(PBDWorldSettings(Vector3(0, -10, 0)));
		RigidBodyParam fixed;
		fixed.rigidType = RigidType::Static;
		Geometry* anchorShape = GeometryFactory::CreateSphere(Vector3::Zero(), 0.1f);
		anchorShape->SetSimulationEnabled(false);
		RigidBody* anchor = pendulum.CreateRigidBody(anchorShape, fixed);
		Geometry* bobShape = GeometryFactory::CreateSphere(Vector3::UnitX(), 0.1f);
		bobShape->SetSimulationEnabled(false);
		RigidBodyDynamic* bob = PBDDynamic(pendulum, bobShape);
		DistanceConstraint* joint = pendulum.CreateDistanceConstraint(anchor, bob, anchor->X, bob->X, 1.0f, 1.0f);
		EXPECT(joint != nullptr);
		for (int frame = 0; frame < 120; ++frame) pendulum.Simulate();
		EXPECT_NEAR(joint->GetCurrentDistance(), 1.0f, 0.01f);
		EXPECT(std::isfinite(bob->V.Length()) && bob->V.Length() < 10.0f);
		EXPECT_SAME(anchor->X, Vector3::Zero());
	}

	void TestPBDTriggerEvents()
	{
		int triggerCount = 0, contactCount = 0;
		Geometry* trigger = nullptr;
		PhysicsWorldParam settings = PBDWorldSettings();
		settings.onTrigger = [&](const PhysicsTriggerInfo& info)
		{
			EXPECT(info.GeomA == trigger || info.GeomB == trigger);
			EXPECT(info.BodyA != nullptr && info.BodyB != nullptr);
			++triggerCount;
		};
		settings.onContact = [&](const PhysicsContactInfo&) { ++contactCount; };
		PhysicsWorld world(settings);
		RigidBodyParam fixed;
		fixed.rigidType = RigidType::Static;
		trigger = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		trigger->SetTriggerEnabled(true);
		EXPECT(world.CreateRigidBody(trigger, fixed) != nullptr);
		RigidBodyDynamic* body = PBDDynamic(world, GeometryFactory::CreateSphere(Vector3(-2, 0, 0), 0.2f), Vector3(4, 0, 0));
		for (int frame = 0; frame < 60; ++frame) world.Simulate();
		EXPECT(triggerCount > 0);
		EXPECT(contactCount == 0);
		EXPECT_SAME(body->X, Vector3(2, 0, 0));
		EXPECT_SAME(body->V, Vector3(4, 0, 0));
	}

	void TestPBDCCDAndRope()
	{
		PhysicsWorld world(PBDWorldSettings());
		RigidBodyParam fixed;
		fixed.rigidType = RigidType::Static;
		world.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 5, 5)), fixed);
		RigidBodyParam moving;
		moving.rigidType = RigidType::Dynamic;
		moving.motionType = MorionType::Continuous;
		moving.linearVelocity = Vector3(600, 0, 0);
		moving.linearDamping = 1.0f;
		moving.sleepThreshold = 0.0f;
		RigidBodyDynamic* sphere = world.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5, 0, 0), 0.5f), moving)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() > 0 && sphere->HadCCDContact);
		EXPECT(sphere->X.x < -0.49f && sphere->X.x > -0.8f);
		EXPECT(std::abs(sphere->V.x) < 0.1f);

		PhysicsWorld ropeWorld(PBDWorldSettings());
		Geometry* box = GeometryFactory::CreateOBB(Vector3(2, 0, 0), Vector3(0.2f));
		box->SetSimulationEnabled(false);
		RigidBodyDynamic* pulled = PBDDynamic(ropeWorld, box);
		pulled->InvInertia = Matrix3::Identity();
		RopeSettings ropeSettings;
		ropeSettings.SegmentCount = 4;
		ropeSettings.RestLength = 0.25f;
		ropeSettings.SolverIterations = 20;
		ropeSettings.Gravity = Vector3::Zero();
		ropeSettings.Damping = 0.95f;
		RopeSystem* rope = ropeWorld.CreateRope(ropeSettings, Vector3::Zero(), pulled->X);
		EXPECT(rope->PinParticle(0, Vector3::Zero()));
		EXPECT(rope->AttachRigidBody(4, pulled, pulled->X));
		for (int frame = 0; frame < 30; ++frame) ropeWorld.Simulate();
		EXPECT(pulled->X.x < 1.9f);
		EXPECT(std::isfinite(pulled->X.x) && std::isfinite(pulled->V.Length()));
		EXPECT((rope->GetParticle(4).Position - pulled->X).Length() < 0.15f);
		EXPECT_SAME(rope->GetParticle(0).Position, Vector3::Zero());
	}
}

void TestPBDRigidSolver()
{
	printf("Running TestPBDRigidSolver\n");
	TestPBDFreeMotion();
	TestPBDRestitutionAndAngularResponse();
	TestPBDInitialPenetrationEnergy();
	TestPBDFrictionAndStack();
	TestPBDFrozenAndJoints();
	TestPBDTriggerEvents();
	TestPBDCCDAndRope();
}

#include "Test.h"

#include <limits>

#include "../Src/Cloth/Rope.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/Segment3.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	RopeSettings FreeRopeSettings()
	{
		RopeSettings settings;
		settings.SegmentCount = 1;
		settings.SolverIterations = 16;
		settings.Damping = 1.0f;
		settings.Gravity = Vector3::Zero();
		settings.ParticleRadius = 0.1f;
		settings.CollisionFriction = 0.0f;
		return settings;
	}

	void TestRopeTimeStep()
	{
		RopeSettings settings = FreeRopeSettings();
		RopeSystem rope(settings, Vector3::Zero(), Vector3::UnitY());
		EXPECT(rope.SetParticleVelocity(0, Vector3(4.0f, 0.0f, 0.0f)));
		EXPECT(rope.SetParticleVelocity(1, Vector3(4.0f, 0.0f, 0.0f)));
		rope.Simulate(0.01f);
		rope.Simulate(0.03f);
		EXPECT_NEAR(rope.GetParticle(0).Position.x, 0.16f, 1.0e-5f);
		EXPECT_NEAR(rope.GetParticleVelocity(0).x, 4.0f, 1.0e-4f);
		const Vector3 saved = rope.GetParticle(0).Position;
		rope.Simulate(0.0f);
		rope.Simulate(std::numeric_limits<float>::quiet_NaN());
		EXPECT_SAME(rope.GetParticle(0).Position, saved);
		EXPECT_FALSE(rope.SetParticleVelocity(-1, Vector3::Zero()));
		EXPECT_FALSE(rope.PinParticle(0, Vector3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f)));

		settings.Damping = 0.99f;
		RopeSystem first(settings, Vector3::Zero(), Vector3::UnitY());
		RopeSystem second(settings, Vector3::Zero(), Vector3::UnitY());
		for (int i = 0; i < 2; ++i)
		{
			first.SetParticleVelocity(i, Vector3::UnitX());
			second.SetParticleVelocity(i, Vector3::UnitX());
		}
		for (int i = 0; i < 60; ++i) first.Simulate(1.0f / 60.0f);
		for (int i = 0; i < 30; ++i) second.Simulate(1.0f / 30.0f);
		EXPECT_NEAR(first.GetParticleVelocity(0).x, second.GetParticleVelocity(0).x, 1.0e-4f);
	}

	float RopeExtension(float dt, int iterations)
	{
		RopeSettings settings = FreeRopeSettings();
		settings.StretchCompliance = 0.01f;
		settings.Gravity = Vector3(10.0f, 0.0f, 0.0f);
		settings.Damping = 0.95f;
		settings.SolverIterations = iterations;
		RopeSystem rope(settings, Vector3::Zero(), Vector3::UnitX());
		rope.PinParticle(0, Vector3::Zero());
		for (int frame = 0; frame < (int)(5.0f / dt); ++frame) rope.Simulate(dt);
		return rope.GetParticle(1).Position.x - 1.0f;
	}

	void TestRopeCompliance()
	{
		EXPECT_NEAR(RopeExtension(1.0f / 60.0f, 1), 0.1f, 0.003f);
		EXPECT_NEAR(RopeExtension(1.0f / 60.0f, 16), 0.1f, 0.003f);
		EXPECT_NEAR(RopeExtension(1.0f / 120.0f, 8), 0.1f, 0.003f);
		RopeSettings settings = FreeRopeSettings();
		settings.RestLength = 2.0f;
		settings.TensionOnly = true;
		RopeSystem slack(settings, Vector3::Zero(), Vector3::UnitX());
		slack.PinParticle(0, Vector3::Zero());
		slack.Simulate(1.0f / 60.0f);
		EXPECT_SAME(slack.GetParticle(1).Position, Vector3::UnitX());
		settings.TensionOnly = false;
		RopeSystem rod(settings, Vector3::Zero(), Vector3::UnitX());
		rod.PinParticle(0, Vector3::Zero());
		rod.Simulate(1.0f / 60.0f);
		EXPECT_NEAR(rod.GetParticle(1).Position.x, 2.0f, 1.0e-5f);
	}

	void TestRopeAngularAttachment()
	{
		PhysicsWorldParam worldSettings;
		PhysicsWorld world(worldSettings);
		RigidBodyParam bodySettings;
		bodySettings.rigidType = RigidType::Dynamic;
		bodySettings.disableGravity = true;
		Geometry* shape = GeometryFactory::CreateOBB(Vector3(2.0f, 0.0f, 0.0f), Vector3::One());
		RigidBodyDynamic* body = world.CreateRigidBody(shape, bodySettings)->CastDynamic();
		body->InvMass = 1.0f;
		body->InvInertia = Matrix3(1.0f, 1.0f, 1.0f);
		RopeSettings settings = FreeRopeSettings();
		settings.RestLength = 1.0f;
		RopeSystem rope(settings, Vector3::Zero(), Vector3(2.0f, 1.0f, 0.0f));
		rope.PinParticle(0, Vector3::Zero());
		EXPECT(rope.AttachRigidBody(1, body, Vector3(2.0f, 1.0f, 0.0f)));
		const Vector3 position = body->X;
		const Quaternion rotation = body->Q;
		rope.Simulate(1.0f / 60.0f);
		EXPECT_SAME(body->X, position);
		EXPECT(body->Q == rotation);
		EXPECT(body->V.x < -0.1f);
		EXPECT(body->W.z > 0.1f);
		EXPECT(std::isfinite(body->W.z));
		body->Freezing = true;
		body->Sleeping = true;
		const Vector3 velocity = body->V;
		const Vector3 angularVelocity = body->W;
		rope.Simulate(1.0f / 60.0f);
		EXPECT_SAME(body->V, velocity);
		EXPECT_SAME(body->W, angularVelocity);
		EXPECT_SAME(body->X, position);
		EXPECT(body->Q == rotation);
		EXPECT(body->Sleeping);
	}

	void TestRopeContinuousCollisions()
	{
		RopeSettings settings = FreeRopeSettings();
		RopeSystem sphere(settings, Vector3(-3.0f, -2.0f, 0.0f), Vector3(-3.0f, 2.0f, 0.0f));
		EXPECT(sphere.AddSphereCollider(nullptr, 0.5f, Vector3::Zero()));
		sphere.SetParticleVelocity(0, Vector3(360.0f, 0.0f, 0.0f));
		sphere.SetParticleVelocity(1, Vector3(360.0f, 0.0f, 0.0f));
		sphere.Simulate(1.0f / 60.0f);
		// Both endpoint trajectories miss the sphere; only the swept segment hits.
		EXPECT(sphere.GetParticle(0).Position.x < -0.59f);
		EXPECT(sphere.GetParticle(1).Position.x < -0.59f);
		EXPECT(Segment3::SqrDistancePointToSegment(Vector3::Zero(), sphere.GetParticle(0).Position,
			sphere.GetParticle(1).Position) >= 0.359f);

		RopeSystem box(settings, Vector3(-3.0f, -2.0f, 0.0f), Vector3(-3.0f, 2.0f, 0.0f));
		Quaternion rotation;
		rotation.FromRotateY(0.4f);
		const Transform boxPose(Vector3::Zero(), rotation);
		EXPECT(box.AddBoxCollider(nullptr, Vector3(0.05f, 1.0f, 2.0f), boxPose));
		box.SetParticleVelocity(0, Vector3(360.0f, 0.0f, 0.0f));
		box.SetParticleVelocity(1, Vector3(360.0f, 0.0f, 0.0f));
		box.Simulate(1.0f / 60.0f);
		const Vector3 middle = (box.GetParticle(0).Position + box.GetParticle(1).Position) * 0.5f;
		EXPECT(boxPose.WorldToLocal(middle).x < -0.149f);

		RopeSystem particle(settings, Vector3(-3.0f, 0.0f, 0.0f), Vector3(-5.0f, 0.0f, 0.0f));
		particle.AddSphereCollider(nullptr, 0.5f, Vector3::Zero());
		particle.SetParticleVelocity(0, Vector3(360.0f, 0.0f, 0.0f));
		particle.SetParticleVelocity(1, Vector3(360.0f, 0.0f, 0.0f));
		particle.Simulate(1.0f / 60.0f);
		EXPECT(particle.GetParticle(0).Position.x < -0.59f);
		EXPECT(particle.GetParticle(1).Position.x < -0.59f);

		settings.TensionOnly = true;
		settings.RestLength = 10.0f;
		RopeSystem resting(settings, Vector3::Zero(), Vector3::Zero());
		resting.AddSphereCollider(nullptr, 1.0f, Vector3::Zero());
		for (int frame = 0; frame < 120; ++frame) resting.Simulate(1.0f / 60.0f);
		EXPECT_NEAR(resting.GetParticle(0).Position.Length(), 1.1f, 1.0e-4f);
		EXPECT(resting.GetParticleVelocity(0).Length() < 1.0e-4f);
	}

	void TestRopeColliderLifetime()
	{
		PhysicsWorldParam worldSettings;
		PhysicsWorld world(worldSettings);
		RigidBodyParam bodySettings;
		bodySettings.rigidType = RigidType::Static;
		Quaternion rotation;
		rotation.FromRotateZ(0.5f);
		Geometry* shape = GeometryFactory::CreateOBB(Vector3(4.0f, 2.0f, 0.0f), Vector3::One(), rotation);
		RigidBody* body = world.CreateRigidBody(shape, bodySettings);
		RopeSettings settings = FreeRopeSettings();
		RopeSystem* rope = world.CreateRope(settings, body->X, body->X + Vector3(0.0f, 0.2f, 0.0f));
		EXPECT(rope->AddCollider(shape));
		EXPECT(rope->AddCollider(shape));
		EXPECT(rope->GetColliderCount() == 1);
		rope->Simulate(1.0f / 60.0f);
		const Vector3 localPoint = body->GetGeometryTransform(shape).WorldToLocal(rope->GetParticle(0).Position);
		EXPECT(std::max(std::abs(localPoint.x), std::max(std::abs(localPoint.y), std::abs(localPoint.z))) >= 1.09f);
		EXPECT(rope->AttachRigidBody(1, body, body->X));
		EXPECT(world.RemoveRigidBody(body));
		EXPECT(rope->GetColliderCount() == 0);
		EXPECT_FALSE(rope->DetachRigidBody(body));
		delete body;
		rope->Simulate(1.0f / 60.0f);
		EXPECT_FALSE(rope->AddSphereCollider(nullptr, -1.0f, Vector3::Zero()));
		EXPECT_FALSE(rope->AddBoxCollider(nullptr, Vector3::Zero(), Transform::Identity()));
		EXPECT_FALSE(rope->AddCollider(nullptr));
		EXPECT(rope->AddSphereCollider(nullptr, 1.0f, Vector3::Zero()));
		rope->ClearColliders();
		EXPECT(rope->GetColliderCount() == 0);
	}
}

void TestRopeRegression()
{
	printf("Running TestRopeRegression\n");
	TestRopeTimeStep();
	TestRopeCompliance();
	TestRopeAngularAttachment();
	TestRopeContinuousCollisions();
	TestRopeColliderLifetime();
}

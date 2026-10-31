#include "Test.h"

#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	RigidBody* CreateConstraintBody(PhysicsWorld& world, const Vector3& position, RigidType type)
	{
		Geometry* shape = GeometryFactory::CreateSphere(position, 0.2f);
		shape->SetSimulationEnabled(false);
		RigidBodyParam settings;
		settings.rigidType = type;
		settings.disableGravity = true;
		settings.sleepThreshold = 0.0f;
		RigidBody* body = world.CreateRigidBody(shape, settings);
		if (type == RigidType::Dynamic)
		{
			body->InvMass = 1.0f;
			body->InvInertia = Matrix3(1.0f, 2.0f, 3.0f);
		}
		return body;
	}

	Vector3 AngularMomentum(RigidBody* body)
	{
		return body->X.Cross(body->V / body->InvMass) + body->GetInverseInertia_WorldSpace().Inverse() * body->W;
	}

	Vector3 EulerAngles(const Quaternion& rotation)
	{
		Vector3 angles;
		rotation.ToEuler(angles.y, angles.z, angles.x);
		return angles;
	}

	Vector3 MeasureEulerRate(RigidBody* body, const Quaternion& frame)
	{
		const float step = 0.0001f;
		Quaternion change = Quaternion::One();
		if (body->W.Length() > 1e-7f) change.FromRotationAxis(body->W.SafeUnit(), body->W.Length() * step);
		return (EulerAngles(frame.Conjugate() * change * body->Q)
			- EulerAngles(frame.Conjugate() * body->Q)) / step;
	}
}

void TestConstraintSolverRegression()
{
	printf("Running TestConstraintSolverRegression\n");
	const float dt = 1.0f / 60.0f;

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3(2.0f, 0.0f, 0.0f), RigidType::Dynamic);
		SliderConstraint slider(bodyA, bodyB, Vector3::Zero(), Vector3::UnitX(), -5.0f, 5.0f);
		bodyB->X.x = 4.0f;
		bodyA->W = bodyB->W = Vector3::UnitZ();
		bodyB->V = Vector3(0.0f, 4.0f, 0.0f);
		const Vector3 momentum = bodyA->V + bodyB->V;
		const Vector3 angularMomentum = AngularMomentum(bodyA) + AngularMomentum(bodyB);
		// A rigid rotation of the complete joint is allowed, including its rotating slide axis.
		for (int iteration = 0; iteration < 20; ++iteration) slider.SolveVelocity(dt);
		EXPECT_SAME(bodyA->V, Vector3::Zero());
		EXPECT_SAME(bodyB->V, Vector3(0.0f, 4.0f, 0.0f));
		EXPECT_SAME(bodyA->W, Vector3::UnitZ());
		EXPECT_SAME(bodyB->W, Vector3::UnitZ());
		EXPECT_SAME(bodyA->V + bodyB->V, momentum);
		EXPECT_SAME(AngularMomentum(bodyA) + AngularMomentum(bodyB), angularMomentum);

		bodyA->V = Vector3(1.0f, -2.0f, 0.5f);
		bodyB->V = Vector3(-1.0f, 2.0f, -0.5f);
		bodyA->W = Vector3(0.2f, -0.1f, 0.4f);
		bodyB->W = Vector3(-0.7f, 0.8f, -0.5f);
		const Vector3 initialAngularMomentum = AngularMomentum(bodyA) + AngularMomentum(bodyB);
		for (int iteration = 0; iteration < 80; ++iteration) slider.SolveVelocity(dt);
		EXPECT((bodyB->W - bodyA->W).Length() < 1e-5f);
		const Vector3 relativeVelocity = bodyB->V - bodyA->V - bodyA->W.Cross(bodyB->X - bodyA->X);
		EXPECT(std::abs(relativeVelocity.y) < 1e-5f && std::abs(relativeVelocity.z) < 1e-5f);
		EXPECT((AngularMomentum(bodyA) + AngularMomentum(bodyB) - initialAngularMomentum).Length() < 1e-4f);
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3(3.0f, 0.0f, 0.0f), RigidType::Dynamic);
		const Vector3 anchorA(0.0f, 1.0f, 0.0f);
		const Vector3 anchorB(3.0f, 2.0f, 1.0f);
		const Vector3 axis = (anchorB - anchorA).SafeUnit();
		DistanceConstraint distance(bodyA, bodyB, anchorA, anchorB, 0.0f, 2.0f);
		const Vector3 originalA = bodyA->X;
		const Vector3 originalB = bodyB->X;
		distance.SolvePosition(dt);
		// A distance limit applies a scalar impulse along the separation, never tangentially.
		EXPECT((bodyA->X - originalA).Cross(axis).Length() < 1e-6f);
		EXPECT((bodyB->X - originalB).Cross(axis).Length() < 1e-6f);
		EXPECT((bodyA->X + bodyB->X - originalA - originalB).Length() < 1e-6f);
		EXPECT(distance.GetCurrentDistance() < (anchorB - anchorA).Length());
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		DistanceConstraint distance(bodyA, bodyB, Vector3::Zero(), Vector3::Zero(), 2.0f, 2.0f);
		for (int iteration = 0; iteration < 20; ++iteration) distance.SolvePosition(dt);
		EXPECT_NEAR(distance.GetCurrentDistance(), 2.0f, 1e-5f);
		bodyB->V = -Vector3::UnitX();
		distance.SolveVelocity(dt);
		EXPECT(bodyB->V.Length() < 1e-5f);
		DistanceConstraint point(bodyA, bodyB, Vector3::Zero(), bodyB->X, 0.0f, 0.0f);
		for (int iteration = 0; iteration < 20; ++iteration) point.SolvePosition(dt);
		bodyB->V = Vector3(1.0f, 2.0f, 3.0f);
		point.SolveVelocity(dt);
		EXPECT(bodyB->V.Length() < 1e-5f);
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		bodyA->Q.FromRotationAxis(Vector3::UnitZ(), 0.6f);
		Quaternion reference;
		reference.FromRotationAxis(Vector3::UnitX(), 0.4f);
		bodyB->Q = bodyA->Q * reference;
		SixDOFConstraintSettings settings;
		settings.LinearMin = Vector3(-10.0f, -10.0f, -10.0f);
		settings.LinearMax = Vector3(10.0f, 10.0f, 10.0f);
		settings.AngularMin = Vector3(-0.7f, -0.5f, -0.5f);
		settings.AngularMax = Vector3(0.7f, 0.5f, 0.5f);
		SixDOFConstraint joint(bodyA, bodyB, Vector3::Zero(), settings);
		const Quaternion frame = bodyA->Q * reference;
		Quaternion relative;
		relative.FromEuler(0.5f, 0.1f, 0.3f);
		bodyB->Q = frame * relative;
		bodyB->W = frame * Vector3::UnitY();
		joint.SolveVelocity(dt);
		EXPECT(std::abs(MeasureEulerRate(bodyB, frame).y) < 0.003f);
		bodyB->W = frame * Vector3(std::cos(0.5f), 0.0f, -std::sin(0.5f));
		const Vector3 freeVelocity = bodyB->W;
		joint.SolveVelocity(dt);
		EXPECT((bodyB->W - freeVelocity).Length() < 1e-5f);
		relative.FromEuler(0.9f, 0.8f, 0.4f);
		bodyB->Q = frame * relative;
		for (int iteration = 0; iteration < 80; ++iteration) joint.SolvePosition(dt);
		const Vector3 angles = EulerAngles(frame.Conjugate() * bodyB->Q);
		EXPECT(std::abs(angles.x) <= 0.701f);
		EXPECT(std::abs(angles.y) <= 0.501f);
		EXPECT(std::abs(angles.z) <= 0.501f);
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		SixDOFConstraint joint(bodyA, bodyB, Vector3(0.0f, 1.0f, 0.0f), SixDOFConstraintSettings());
		bodyB->V = Vector3(1.0f, 2.0f, 3.0f);
		bodyB->W = Vector3(2.0f, -3.0f, 4.0f);
		for (int iteration = 0; iteration < 80; ++iteration) joint.SolveVelocity(dt);
		EXPECT(bodyB->V.Length() < 1e-5f);
		EXPECT(bodyB->W.Length() < 1e-5f);
		SixDOFConstraintSettings invalid;
		invalid.LinearMin.x = 2.0f;
		invalid.LinearMax.x = -2.0f;
		invalid.AngularMin.y = -10.0f;
		invalid.AngularMax.y = 10.0f;
		SixDOFConstraint sanitized(bodyA, bodyB, Vector3::Zero(), invalid);
		EXPECT_NEAR(sanitized.GetSettings().LinearMax.x, 2.0f, 1e-6f);
		EXPECT_NEAR(sanitized.GetSettings().AngularMin.y, -PI, 1e-6f);
		EXPECT_NEAR(sanitized.GetSettings().AngularMax.y, PI, 1e-6f);
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* bodyA = CreateConstraintBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* bodyB = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		HingeConstraint hinge(bodyA, bodyB, Vector3::Zero(), Vector3::UnitY(), -PI, PI);
		bodyB->Q.FromRotationAxis(Vector3::UnitX(), PI);
		for (int iteration = 0; iteration < 80; ++iteration) hinge.SolvePosition(dt);
		EXPECT((bodyB->Q * Vector3::UnitY()).Dot(Vector3::UnitY()) > 0.9999f);
		bodyB->Q = Quaternion::One();
		ConeConstraint cone(bodyA, bodyB, Vector3::Zero(), Vector3::UnitY(), Vector3::UnitY(), 0.25f);
		bodyB->Q.FromRotationAxis(Vector3::UnitX(), PI);
		for (int iteration = 0; iteration < 80; ++iteration) cone.SolvePosition(dt);
		EXPECT(cone.GetCurrentAngle() < 0.251f);
		bodyB->Q = Quaternion::One();
		cone.SetMaxAngle(0.0f);
		bodyB->W = Vector3(1.0f, 2.0f, 3.0f);
		for (int iteration = 0; iteration < 20; ++iteration) cone.SolveVelocity(dt);
		EXPECT_SAME(bodyB->W, Vector3(0.0f, 2.0f, 0.0f));
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBodyKinematics kinematic;
		RigidBody* body = CreateConstraintBody(world, Vector3::Zero(), RigidType::Dynamic);
		PointConstraint point(&kinematic, body, Vector3::Zero());
		kinematic.V = Vector3(2.0f, -1.0f, 0.5f);
		point.SolveVelocity(dt);
		EXPECT_SAME(body->V, kinematic.V);
	}
}

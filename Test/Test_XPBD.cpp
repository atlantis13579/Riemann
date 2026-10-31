#include "Test.h"

#include <memory>
#include <limits>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Deformation/XPBDSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	RigidBody* XPBDBody(PhysicsWorld& world, const Vector3& position, RigidType type, bool collision = false)
	{
		Geometry* geometry = GeometryFactory::CreateSphere(position, 0.5f);
		geometry->SetSimulationEnabled(collision);
		RigidBodyParam settings;
		settings.rigidType = type;
		settings.linearDamping = settings.angularDamping = 1.0f;
		settings.maxContactImpulse = 100000.0f;
		RigidBody* body = world.CreateRigidBody(geometry, settings);
		body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		body->SetRestitution(0.0f);
		if (type == RigidType::Dynamic)
		{
			body->InvMass = 1.0f;
			body->InvInertia = Matrix3::Identity();
		}
		return body;
	}

	float ZAngle(const Quaternion& rotation)
	{
		const Vector3 axis = rotation * Vector3::UnitX();
		return std::atan2(axis.y, axis.x);
	}

	struct XPBDContactFixture
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		RigidBody* Ground;
		RigidBodyDynamic* Body;
		std::vector<Geometry*> Geometries;
		ContactManifold Manifold;

		XPBDContactFixture(float depth)
		{
			RigidBodyParam fixed;
			Ground = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), fixed);
			Ground->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
			Ground->SetRestitution(0.0f);
			Body = XPBDBody(World, Vector3(0, 0.5f - depth, 0), RigidType::Dynamic, true)->CastDynamic();
			Body->InvInertia = Matrix3::Zero();
			Geometries = { Ground->Geometries()[0], Body->Geometries()[0] };
			Manifold.indexA = 0;
			Manifold.indexB = 1;
			Manifold.NumContactPointCount = 1;
			Contact& point = Manifold.ContactPoints[0];
			point.PositionWorldA = Vector3::Zero();
			point.PositionWorldB = Vector3(0, -depth, 0);
			point.Normal = Vector3::UnitY();
			point.PenetrationDepth = depth;
		}

		void Begin(XPBDSolver& solver, float dt)
		{
			solver.BeginStep({ Body }, dt);
			solver.PredictPoses(dt);
			solver.AddContacts(Geometries, { &Manifold });
		}
	};

	void TestXPBDContactCompliance()
	{
		for (float dt : { 0.1f, 0.05f })
		for (float compliance : { 0.0f, 0.01f })
		{
			XPBDContactFixture fixture(0.1f);
			XPBDSolverSettings settings;
			settings.PenetrationSlop = 0.0f;
			settings.ContactCompliance = compliance;
			XPBDSolver solver(settings);
			fixture.Begin(solver, dt);
			EXPECT(solver.GetContactCount() == 1);
			const float correction = 0.1f / (1.0f + compliance / (dt * dt));
			solver.SolvePositions(dt);
			EXPECT_NEAR(fixture.Body->X.y, 0.4f + correction, 1e-6f);
			EXPECT_NEAR(solver.GetContactPositionLambda(0), correction, 1e-6f);
			for (int iteration = 1; iteration < 30; ++iteration) solver.SolvePositions(dt);
			// Iteration count changes convergence, not the material stiffness.
			EXPECT_NEAR(fixture.Body->X.y, 0.4f + correction, 1e-6f);
			EXPECT_NEAR(solver.GetContactPositionLambda(0), correction, 1e-6f);
			solver.ReconstructVelocities(dt);
			for (int iteration = 0; iteration < 8; ++iteration) solver.SolveVelocities(dt);
			solver.EndStep(dt);
			EXPECT(fixture.Body->V.Length() < 1e-5f);
			EXPECT(fixture.Body->W.Length() < 1e-5f);
			solver.ResetState();
			EXPECT(solver.GetContactCount() == 0);
			EXPECT(solver.GetContactManifolds().empty());
		}

		XPBDContactFixture fixture(0.1f);
		XPBDSolverSettings settings;
		settings.PenetrationSlop = 0.0f;
		settings.ContactCompliance = 0.01f;
		XPBDSolver solver(settings);
		fixture.Begin(solver, 0.1f);
		solver.SolvePositions(0.1f);
		EXPECT(solver.GetContactPositionLambda(0) > 0.0f);
		fixture.Body->X.y += 0.5f;
		solver.SolvePositions(0.1f);
		EXPECT_NEAR(solver.GetContactPositionLambda(0), 0.0f, 1e-6f);
		EXPECT(fixture.Body->X.y > 0.5f);
		const Vector3 separated = fixture.Body->X;
		for (int iteration = 0; iteration < 10; ++iteration) solver.SolvePositions(0.1f);
		EXPECT_SAME(fixture.Body->X, separated);
		solver.BeginStep({ fixture.Body }, 0.1f);
		EXPECT(solver.GetContactCount() == 0);
		EXPECT_NEAR(solver.GetContactPositionLambda(0), 0.0f, 1e-6f);
	}

	void TestXPBDPointCompliance()
	{
		for (float dt : { 0.1f, 0.05f })
		{
			PhysicsWorld world{ PhysicsWorldParam() };
			RigidBody* a = XPBDBody(world, Vector3::Zero(), RigidType::Static);
			RigidBody* b = XPBDBody(world, Vector3::Zero(), RigidType::Dynamic);
			PointConstraint point(a, b, Vector3::Zero());
			point.SetCompliance(0.01f, 0.0f);
			b->X.x = 0.2f;
			point.BeginStep(dt, true);
			point.SolvePosition(dt);
			const float alpha = 0.01f / (dt * dt);
			const float residual = 0.2f * alpha / (1.0f + alpha);
			EXPECT_NEAR(b->X.x, residual, 1e-6f);
			for (int iteration = 1; iteration < 30; ++iteration) point.SolvePosition(dt);
			EXPECT_NEAR(b->X.x, residual, 1e-6f);
			b->V = Vector3(1, 2, 3);
			point.SolveVelocity(dt);
			EXPECT_SAME(b->V, Vector3(1, 2, 3));
			point.BeginStep(dt, true);
			point.SolvePosition(dt);
			EXPECT_NEAR(b->X.x, residual * alpha / (1.0f + alpha), 1e-6f);
		}

		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* a = XPBDBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* b = XPBDBody(world, Vector3::Zero(), RigidType::Dynamic);
		PointConstraint point(a, b, Vector3::Zero());
		point.SetCompliance(0.01f, 0.0f);
		point.BeginStep(0.1f, true);
		b->X.x = 0.2f;
		point.SolvePosition(0.1f);
		b->X.x = 0.0f;
		point.SolvePosition(0.1f);
		// Zero geometric error still has to release the existing elastic multiplier.
		EXPECT_NEAR(b->X.x, 0.05f, 1e-6f);
		point.BeginStep(0.1f, false);
		b->X.x = 0.2f;
		point.SolvePosition(0.1f);
		EXPECT_NEAR(b->X.x, 0.0f, 1e-6f);
		point.SetCompliance(0.0f, 0.0f);
		point.BeginStep(0.1f, true);
		b->X.x = 0.2f;
		point.SolvePosition(0.1f);
		EXPECT_NEAR(b->X.x, 0.0f, 1e-6f);
	}

	void TestXPBDDistanceLimits()
	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBody* a = XPBDBody(world, Vector3::Zero(), RigidType::Static);
		RigidBody* b = XPBDBody(world, Vector3(2, 0, 0), RigidType::Dynamic);
		DistanceConstraint joint(a, b, a->X, b->X, 1.0f, 2.0f);
		joint.SetCompliance(0.01f, 0.0f);
		joint.BeginStep(0.1f, true);
		b->X.x = 2.4f;
		for (int iteration = 0; iteration < 20; ++iteration) joint.SolvePosition(0.1f);
		EXPECT_NEAR(joint.GetCurrentDistance(), 2.2f, 1e-5f);
		b->X.x = 1.5f;
		joint.SolvePosition(0.1f);
		EXPECT(joint.GetCurrentDistance() > 1.0f && joint.GetCurrentDistance() < 2.0f);
		b->X.x = 0.6f;
		for (int iteration = 0; iteration < 20; ++iteration) joint.SolvePosition(0.1f);
		EXPECT_NEAR(joint.GetCurrentDistance(), 0.8f, 1e-5f);
		joint.BeginStep(0.1f, true);
		joint.SolvePosition(0.1f);
		EXPECT_NEAR(joint.GetCurrentDistance(), 0.9f, 1e-5f);
	}

	void TestXPBDAngularAndMixedJoints()
	{
		const float dt = 0.1f;
		for (int type = 0; type < 5; ++type)
		{
			PhysicsWorld world{ PhysicsWorldParam() };
			RigidBody* a = XPBDBody(world, Vector3::Zero(), RigidType::Static);
			RigidBody* b = XPBDBody(world, Vector3::Zero(), RigidType::Dynamic);
			std::unique_ptr<PositionConstraint> joint;
			if (type == 0) joint.reset(new FixedConstraint(a, b));
			if (type == 1) joint.reset(new HingeConstraint(a, b, Vector3::Zero(), Vector3::UnitZ(), 0.0f, 0.0f));
			if (type == 2) joint.reset(new SliderConstraint(a, b, Vector3::Zero(), Vector3::UnitX(), -1.0f, 1.0f));
			if (type == 3) joint.reset(new SixDOFConstraint(a, b, Vector3::Zero(), SixDOFConstraintSettings()));
			if (type == 4) joint.reset(new ConeConstraint(a, b, Vector3::Zero(), Vector3::UnitY(), Vector3::UnitY(), 0.0f));
			joint->SetCompliance(0.0f, 0.01f);
			joint->BeginStep(dt, true);
			b->X.y = 0.2f;
			b->Q.FromRotationAxis(Vector3::UnitZ(), 0.2f);
			for (int iteration = 0; iteration < 30; ++iteration) joint->SolvePosition(dt);
			EXPECT_NEAR(b->X.y, 0.0f, 1e-5f);
			EXPECT_NEAR(ZAngle(b->Q), 0.1f, 2e-4f);
			joint->SetCompliance(0.0f, 0.0f);
			joint->BeginStep(dt, true);
			for (int iteration = 0; iteration < 30; ++iteration) joint->SolvePosition(dt);
			EXPECT(std::abs(ZAngle(b->Q)) < 2e-4f);
		}

		for (int type = 0; type < 2; ++type)
		{
			PhysicsWorld world{ PhysicsWorldParam() };
			RigidBody* a = XPBDBody(world, Vector3::Zero(), RigidType::Static);
			RigidBody* b = XPBDBody(world, Vector3::Zero(), RigidType::Dynamic);
			std::unique_ptr<PositionConstraint> joint(type == 0
				? static_cast<PositionConstraint*>(new SliderConstraint(a, b, Vector3::Zero(), Vector3::UnitX(), -1.0f, 1.0f))
				: static_cast<PositionConstraint*>(new SixDOFConstraint(a, b, Vector3::Zero(), SixDOFConstraintSettings())));
			joint->SetCompliance(0.01f, 0.0f);
			joint->BeginStep(dt, true);
			b->X.y = 0.2f;
			b->Q.FromRotationAxis(Vector3::UnitZ(), 0.2f);
			for (int iteration = 0; iteration < 30; ++iteration) joint->SolvePosition(dt);
			EXPECT_NEAR(b->X.y, 0.1f, 1e-5f);
			EXPECT(std::abs(ZAngle(b->Q)) < 2e-4f);
			b->V = Vector3::UnitY();
			b->W = Vector3::UnitZ();
			joint->SolveVelocity(dt);
			EXPECT_NEAR(b->V.y, 1.0f, 1e-5f);
			EXPECT(b->W.Length() < 1e-5f);
		}
	}

	PhysicsWorldParam XPBDWorldSettings(float contactCompliance, int iterations = 20)
	{
		PhysicsWorldParam settings;
		settings.resolutionSolver = ResolutionPhaseSolver::XPBD;
		settings.gravityAcc = Vector3(0, -10, 0);
		settings.xpbdSettings.ContactCompliance = contactCompliance;
		settings.xpbdSettings.PositionIterations = iterations;
		settings.xpbdSettings.VelocityIterations = 8;
		settings.xpbdSettings.PenetrationSlop = 0.001f;
		return settings;
	}

	void TestXPBDWorldContactEquilibrium()
	{
		for (float inverseMass : { 1.0f, 0.5f })
		for (int iterations : { 1, 20 })
		for (int frequency : { 60, 120 })
		{
			PhysicsWorld world(XPBDWorldSettings(0.001f, iterations));
			EXPECT(world.GetResolutionSolver() == ResolutionPhaseSolver::XPBD);
			EXPECT(world.GetPositionSolver() != nullptr);
			RigidBodyParam fixed;
			RigidBody* ground = world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), fixed);
			ground->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
			ground->SetRestitution(0.0f);
			RigidBodyDynamic* body = XPBDBody(world, Vector3(0, 0.5f, 0), RigidType::Dynamic, true)->CastDynamic();
			body->InvMass = inverseMass;
			body->InvInertia = Matrix3::Zero();
			for (int frame = 0; frame < 4 * frequency; ++frame) world.Simulate(1.0f / frequency);
			const float expectedHeight = 0.5f - 0.001f - 0.001f * 10.0f / inverseMass;
			printf("  XPBD contact equilibrium: mass=%.1f iterations=%d hz=%d y=%.6f expected=%.6f\n",
				1.0f / inverseMass, iterations, frequency, body->X.y, expectedHeight);
			EXPECT_NEAR(body->X.y, expectedHeight, 0.001f);
			EXPECT(std::abs(body->V.y) < 1e-4f);
			EXPECT_FALSE(body->Sleeping);
			EXPECT_SAME(ground->X, Vector3::Zero());
		}
	}

	void TestXPBDWorldJointEquilibrium()
	{
		for (int type = 0; type < 2; ++type)
		for (float inverseMass : { 1.0f, 0.5f })
		for (int frequency : { 60, 120 })
		{
			PhysicsWorld world(XPBDWorldSettings(0.0f));
			RigidBody* anchor = XPBDBody(world, Vector3(0, 3, 0), RigidType::Static);
			const float restDistance = type == 0 ? 0.0f : 1.0f;
			RigidBodyDynamic* body = XPBDBody(world, Vector3(0, 3.0f - restDistance, 0), RigidType::Dynamic)->CastDynamic();
			body->InvMass = inverseMass;
			body->InvInertia = Matrix3::Zero();
			PositionConstraint* joint = type == 0
				? static_cast<PositionConstraint*>(world.CreatePointConstraint(anchor, body, anchor->X))
				: static_cast<PositionConstraint*>(world.CreateDistanceConstraint(anchor, body, anchor->X, body->X, 1.0f, 1.0f));
			joint->SetCompliance(0.002f, 0.0f);
			for (int frame = 0; frame < 8 * frequency; ++frame) world.Simulate(1.0f / frequency);
			const float extension = 0.002f * 10.0f / inverseMass;
			printf("  XPBD joint equilibrium: type=%d mass=%.1f hz=%d extension=%.6f expected=%.6f\n",
				type, 1.0f / inverseMass, frequency, 3.0f - restDistance - body->X.y, extension);
			EXPECT_NEAR(body->X.y, 3.0f - restDistance - extension, 0.002f);
			EXPECT(body->V.Length() < 0.01f);
			EXPECT_FALSE(body->Sleeping);
			EXPECT_SAME(anchor->X, Vector3(0, 3, 0));
		}
	}

	void TestXPBDWorldStepInterval()
	{
		PhysicsWorldParam settings = XPBDWorldSettings(0.0f);
		settings.gravityAcc = Vector3::Zero();
		PhysicsWorld world(settings);
		RigidBody* body = XPBDBody(world, Vector3::Zero(), RigidType::Dynamic);
		body->V = Vector3::UnitX();
		world.Simulate(0.0f);
		world.Simulate(-0.1f);
		world.Simulate(std::numeric_limits<float>::quiet_NaN());
		EXPECT_SAME(body->X, Vector3::Zero());
		world.Simulate(0.02f);
		world.Simulate();
		EXPECT_NEAR(body->X.x, 0.02f + 1.0f / 60.0f, 1e-6f);
		EXPECT_SAME(body->V, Vector3::UnitX());
	}
}

void TestXPBDRigidSolver()
{
	printf("Running TestXPBDRigidSolver\n");
	TestXPBDContactCompliance();
	TestXPBDPointCompliance();
	TestXPBDDistanceLimits();
	TestXPBDAngularAndMixedJoints();
	TestXPBDWorldContactEquilibrium();
	TestXPBDWorldJointEquilibrium();
	TestXPBDWorldStepInterval();
}

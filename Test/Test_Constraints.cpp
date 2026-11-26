#include "Test.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/Island.h"
#include "../Src/Solver/XPBDSolver.h"
#include "../Src/Solver/LCPSolver.h"
#include <memory>

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

namespace Test_ConstraintGraph_Cases
{
namespace
{
	using namespace Riemann;
	RigidBodyDynamic* Body(PhysicsWorld& World, const Vector3& Position)
	{
		RigidBodyParam Param; Param.rigidType = RigidType::Dynamic; Param.sleepThreshold = 0;
		Param.maxContactImpulse = 100.0f;
		auto* Result = World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.5f), Param)->CastDynamic();
		Result->SetRestitution(0.0f);
		Result->InvMass = 1.0f; Result->InvInertia = Matrix3(1.0f, 1.0f, 1.0f); return Result;
	}
	void TestTopology()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* A = Body(World, Vector3::Zero()); auto* B = Body(World, Vector3(2, 0, 0));
		auto* C = Body(World, Vector3(4, 0, 0)); auto* Isolated = Body(World, Vector3(6, 0, 0));
		auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		PointConstraint Joint(A, B, Vector3(1, 0, 0));
		for (bool Enabled : { false, true })
		{
			Joint.SetEnabled(Enabled); B->Sleeping = true;
			ConstraintGraph Graph; Graph.Begin({ A, B, C, Isolated });
			Graph.AddContact(Floor, A, nullptr, 0); Graph.AddContact(Floor, B, nullptr, 1);
			Graph.AddContact(B, C, nullptr, 2); Graph.AddJoint(&Joint); Graph.Finish();
			EXPECT(Graph.GetIslands().size() == (Enabled ? 2 : 3));
			EXPECT(Graph.GetIslands().front().Bodies.size() == (Enabled ? 3 : 1));
			EXPECT(Graph.GetIslands().back().Bodies == std::vector<RigidBodyDynamic*>{ Isolated });
			size_t Edges = 0, Joints = 0;
			for (const auto& Island : Graph.GetIslands()) { Edges += Island.ContactIndices.size(); Joints += Island.Joints.size(); }
			EXPECT(Edges == 3 && Joints == (Enabled ? 1 : 0));
		}
		RigidBodyKinematics Platform;
		ConstraintGraph Graph; Graph.Begin({ A, B }); Graph.AddContact(&Platform, A); Graph.AddContact(&Platform, B); Graph.Finish();
		EXPECT(Graph.GetIslands().size() == 2);
		B->Freezing = true; Graph.Begin({ A, B, C }); Graph.AddContact(A, B); Graph.AddContact(B, C); Graph.Finish();
		EXPECT(Graph.GetIslands().size() == 2); B->Freezing = false;
		// Multiple geometry indices for one rigid body cannot split that body's island.
		ContactManifold AB, AC; AB.indexA = 0; AB.indexB = 2; AC.indexA = 1; AC.indexB = 3;
		AB.NumContactPointCount = AC.NumContactPointCount = 1;
		std::vector<Geometry*> Geometries = { A->mGeometries[0], A->mGeometries[0], B->mGeometries[0], C->mGeometries[0] };
		Graph.Build(Geometries, { &AB, &AC }, {}, { Isolated });
		EXPECT(Graph.GetIslands().size() == 2); EXPECT(Graph.GetIslands()[1].Bodies.size() == 3 && Graph.GetIslands()[1].Contacts.size() == 2);
	}
	void TestVelocityCoupling()
	{
		const float Dt = 1.0f / 60.0f;
		for (bool Sequential : { true, false })
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if ((Sequential && Backend != SolverBackend::CPU) || (!Sequential && !LCPSolver::IsBackendAvailable(Backend))) continue;
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
			auto* A = Body(World, Vector3(0, 0.5f, 0)); auto* B = Body(World, Vector3(0, 1.5f, 0));
			DistanceConstraint Joint(A, B, A->X, B->X, 1.0f, 1.0f); Joint.BeginStep(Dt, false);
			// B has no geometry in the contact batch; the joint still registers it.
			std::vector<Geometry*> Geometries = { Floor->mGeometries[0], A->mGeometries[0] };
			Contact C; C.Normal = Vector3::UnitY(); C.Tangent = Vector3::UnitX(); C.Binormal = Vector3::UnitZ();
			C.PositionLocalB = Vector3(0, -0.5f, 0); C.SolverSlop = 0.001f;
			ContactManifold M; M.AddNewContact(0, 1, C); std::vector<ContactManifold*> Contacts{ &M };
			LCPSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
			std::unique_ptr<ConstraintSolver> Solver(Sequential ? ConstraintSolver::CreateSequentialImpulseSolver() : CreateGlobalLCPSolver(Settings));
			A->V = B->V = Vector3(0, -10, 0);
			Solver->BeginFrame(1); Solver->PreResolve(Geometries, { &Joint }); Solver->ResolveConstraints(Geometries, Contacts, { &Joint }, Dt, 32); Solver->PostResolve(Geometries);
			printf("Unified velocity SI=%d backend=%d A=%g B=%g impulse=%g\n", Sequential, (int)Backend, A->V.y, B->V.y, M.ContactPoints[0].totalImpulseNormal);
			EXPECT(std::abs(A->V.y) < 0.001f && std::abs(B->V.y) < 0.001f);
			EXPECT_NEAR(M.ContactPoints[0].totalImpulseNormal, 20.0f, 0.002f);
			A->V = B->V = Vector3(0, 5, 0);
			Solver->BeginFrame(2); Solver->PreResolve(Geometries, { &Joint }); Solver->ResolveConstraints(Geometries, Contacts, { &Joint }, Dt, 32); Solver->PostResolve(Geometries);
			EXPECT_NEAR(A->V.y, 5.0f, 0.001f); EXPECT_NEAR(B->V.y, 5.0f, 0.001f);
			EXPECT(std::abs(M.ContactPoints[0].totalImpulseNormal) < 0.001f);
			// Joint-only islands use the same synchronization path and conserve momentum.
			A->V = Vector3(0, -10, 0); B->V = Vector3::Zero(); Contacts.clear();
			Solver->BeginFrame(3); Solver->PreResolve({}, { &Joint }); Solver->ResolveConstraints({}, Contacts, { &Joint }, Dt, 32); Solver->PostResolve({});
			EXPECT_NEAR(A->V.y, -5.0f, 0.001f); EXPECT_NEAR(B->V.y, -5.0f, 0.001f);
		}
	}

	void TestPositionSolverCoupling()
	{
		const float Dt = 1.0f / 60.0f;
		const int Iterations = 32;
		for (bool Extended : { false, true })
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (!IsPBDBackendAvailable(Backend)) continue;
			PhysicsWorld World{ PhysicsWorldParam() };
			auto* Floor = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
			auto* A = Body(World, Vector3(0, 0.5f, 0));
			auto* B = Body(World, Vector3(0, 1.5f, 0));
			Floor->SetRestitution(0.0f);
			auto* UnrelatedA = Body(World, Vector3(4, 0.5f, 0));
			auto* UnrelatedB = Body(World, Vector3(4, 1.5f, 0));
			DistanceConstraint Joint(A, B, A->X, B->X, 1.0f, 1.0f);
			Joint.BeginStep(Dt, Extended);
			XPBDSolverSettings Settings;
			Settings.Backend = Backend; Settings.AllowBackendFallback = false;
			std::unique_ptr<PBDSolver> Solver(Extended ? new XPBDSolver(Settings) : new PBDSolver(Settings));
			std::vector<Geometry*> Geometries = { Floor->mGeometries[0], A->mGeometries[0],
				UnrelatedA->mGeometries[0], UnrelatedB->mGeometries[0] };
			Contact C; C.Normal = Vector3::UnitY(); C.SolverSlop = 0.001f;
			ContactManifold M; M.AddNewContact(0, 1, C);
			Contact Unrelated = C;
			Unrelated.PositionWorldA = Unrelated.PositionWorldB = Vector3(4, 0, 0);
			ContactManifold FloorContact; FloorContact.AddNewContact(0, 2, Unrelated);
			Unrelated.PositionWorldA.y = Unrelated.PositionWorldB.y = 1.0f;
			ContactManifold StackContact; StackContact.AddNewContact(2, 3, Unrelated);
			A->V = B->V = Vector3(0, -10, 0);
			UnrelatedA->V = UnrelatedB->V = Vector3(0, -10, 0);
			Solver->BeginStep({ A, B, UnrelatedA, UnrelatedB }, Dt);
			Solver->AddContacts(Geometries, { &M, &FloorContact, &StackContact });
			auto Solve = [&]()
			{
				Vector3 UnrelatedVelocityA, UnrelatedVelocityB;
				for (int Iteration = 0; Iteration < Iterations; ++Iteration)
				{
					Solver->SolveJointVelocities({ &Joint }, Dt);
					Solver->SolveVelocities(Dt, Iteration >= Settings.VelocityIterations);
					if (Iteration == Settings.VelocityIterations - 1)
					{
						UnrelatedVelocityA = UnrelatedA->V; UnrelatedVelocityB = UnrelatedB->V;
					}
				}
				// Sharing the static floor must not grant this separate stack extra sweeps.
				EXPECT_SAME(UnrelatedA->V, UnrelatedVelocityA);
				EXPECT_SAME(UnrelatedB->V, UnrelatedVelocityB);
				Solver->EndStep(Dt);
			};
			Solve();
			EXPECT(std::abs(A->V.y) < 0.001f && std::abs(B->V.y) < 0.001f);
			EXPECT_NEAR(Solver->GetContactManifolds()[0]->ContactPoints[0].totalImpulseNormal, 20.0f, 0.002f);
			// Apply an upward impulse within the same step. The next coupled sweeps
			// must release the stored contact impulse, preserving total momentum.
			A->V.y += 20.0f; B->V.y += 20.0f;
			Solve();
			EXPECT_NEAR(A->V.y, 10.0f, 0.001f); EXPECT_NEAR(B->V.y, 10.0f, 0.001f);
			EXPECT(std::abs(Solver->GetContactManifolds()[0]->ContactPoints[0].totalImpulseNormal) < 0.001f);
			// A speculative contact at a 0.1 m gap allows exactly 6 m/s approach.
			A->X.y += 0.1f; B->X.y += 0.1f; A->V = B->V = Vector3(0, -10, 0);
			C.PositionWorldB.y = 0.1f; C.PenetrationDepth = -0.1f;
			M = ContactManifold(); M.AddNewContact(0, 1, C);
			Solver->BeginStep({ A, B }, Dt);
			Solver->SolveSpeculativeContacts(Geometries, { &M }, Dt, { &Joint }, Iterations);
			EXPECT_NEAR(A->V.y, -6.0f, 0.002f); EXPECT_NEAR(B->V.y, -6.0f, 0.002f);
			EXPECT(Solver->GetContactCount() == 0);
			printf("Unified PBD XPBD=%d backend=%d speculative A=%g B=%g\n", Extended, (int)Backend, A->V.y, B->V.y);
		}
	}

}

void TestConstraintGraph()
{
	TestTopology(); TestVelocityCoupling(); TestPositionSolverCoupling();
}
}

void TestConstraintGraph() { Test_ConstraintGraph_Cases::TestConstraintGraph(); }

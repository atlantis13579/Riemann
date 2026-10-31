#include "Test.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"

using namespace Riemann;

static void TestSpherePlaneContactTransforms()
{
	PhysicsWorld world{PhysicsWorldParam()};
	const Vector3 normal(0.6f, 0.8f, 0.0f), origin(3, 2, -1);
	Geometry* plane = GeometryFactory::CreatePlane(origin, normal);
	Geometry* sphere = GeometryFactory::CreateSphere(origin + normal * 0.4f, 0.5f);
	RigidBodyParam staticBody;
	staticBody.rigidType = RigidType::Static;
	RigidBodyParam dynamicBody;
	dynamicBody.rigidType = RigidType::Dynamic;
	world.CreateRigidBody(plane, staticBody);
	world.CreateRigidBody(sphere, dynamicBody);
	std::unique_ptr<NarrowPhase> narrow(NarrowPhase::Create_GJKEPA());
	for (int reverse = 0; reverse < 2; ++reverse)
	{
		Geometry* geometries[2] = { reverse ? sphere : plane, reverse ? plane : sphere };
		GeometryWorldState states[2];
		const GeometryWorldState* pointers[2] = { &states[0], &states[1] };
		for (int i = 0; i < 2; ++i)
		{
			RigidBody* body = geometries[i]->GetParent<RigidBody>();
			states[i].Geom = geometries[i];
			states[i].Body = body;
			states[i].BodyToWorld = Transform(body->BodyLocalToWorld(Vector3::Zero()), body->Q);
			states[i].ShapeToWorld = body->GetGeometryTransform(geometries[i]);
		}
		std::vector<ContactManifold*> contacts;
		narrow->CollisionDetection(GeometryWorldStateSpan(pointers, 2), { OverlapPair(0, 1) }, &contacts);
		EXPECT(contacts.size() == 1);
		if (contacts.empty()) continue;
		EXPECT(contacts[0]->NumContactPointCount == 1);
		const Contact& contact = contacts[0]->ContactPoints[0];
		EXPECT_NEAR(contact.PenetrationDepth, 0.1f, 1.0e-5f);
		EXPECT_SAME(contact.Normal, (reverse ? -normal : normal));
		EXPECT_SAME((reverse ? contact.PositionWorldB : contact.PositionWorldA), origin);
		EXPECT_SAME((reverse ? contact.PositionWorldA : contact.PositionWorldB), (origin - normal * 0.1f));
		EXPECT_NEAR((contact.PositionWorldB - contact.PositionWorldA).Dot(contact.Normal), -0.1f, 1.0e-5f);
	}
}

void TestPBDScene()
{
	printf("Running TestPBDScene\n");
	TestSpherePlaneContactTransforms();
	SceneWorld scene;
	std::string error;
	const bool loaded = scene.LoadFromFile(TestContentsPath() + "Scenes/16_pbd_rigid.json", &error);
	EXPECT(loaded);
	if (!loaded) { printf("%s\n", error.c_str()); return; }
	EXPECT(scene.GetSimulation()->GetResolutionSolver() == ResolutionPhaseSolver::PBD);
	EXPECT(scene.GetSimulation()->GetSolverBackend() == SolverBackend::CPU);
	const PBDSolver* solver = scene.GetSimulation()->GetPositionSolver();
	EXPECT(solver != nullptr);
	if (solver)
	{
		EXPECT(solver->GetSettings().PositionIterations == 20);
		EXPECT(solver->GetSettings().VelocityIterations == 8);
		EXPECT_NEAR(solver->GetSettings().PenetrationSlop, 0.001f, 1.0e-7f);
	}
	EXPECT(scene.GetSimulation()->GetConstraintCount() == 1);
	RigidBody* sphere = nullptr;
	RigidBody* top = nullptr;
	for (const SceneObjectInstance& object : scene.GetObjects())
	{
		if (object.Id == "bounce") sphere = object.GeometryPtr->GetParent<RigidBody>();
		if (object.Id == "stack_0_3_0") top = object.GeometryPtr->GetParent<RigidBody>();
	}
	EXPECT(sphere && top);
	if (!sphere || !top) return;
	for (int frame = 0; frame < 30; ++frame) scene.Step(1.0f / 60.0f);
	EXPECT(sphere->X.y < 3.3f);
	for (int frame = 30; frame < 240; ++frame) scene.Step(1.0f / 60.0f);
	EXPECT(sphere->X.y > 0.45f && sphere->X.y < 2.0f);
	EXPECT(top->X.y > 3.0f && top->X.y < 3.8f);
	for (const SceneObjectInstance& object : scene.GetObjects())
	{
		RigidBody* body = object.GeometryPtr->GetParent<RigidBody>();
		if (!body) continue;
		EXPECT(std::isfinite(body->X.x) && std::isfinite(body->X.y) && std::isfinite(body->X.z));
		EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 0.001f);
	}
	const ConstraintControlState hinge = scene.GetConstraintControls()[0];
	EXPECT(std::abs(hinge.CurrentValue) < 76.0f);
	EXPECT(scene.LoadFromFile(TestContentsPath() + "Scenes/16_pbd_rigid.json", &error));
	EXPECT(scene.GetSimulation()->GetResolutionSolver() == ResolutionPhaseSolver::PBD);
}

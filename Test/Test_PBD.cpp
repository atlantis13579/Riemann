#include "Test.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Deformation/Rope.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include <fstream>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>
#include "../Src/Solver/PBDBackend.h"
#include "../Src/Solver/XPBDSolver.h"
#include <limits>

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

namespace Test_PBDScene_Cases
{
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
}
}

void TestPBDScene() { Test_PBDScene_Cases::TestPBDScene(); }

namespace Test_PBDBackends_Cases
{
using namespace Riemann;

namespace
{
	const SolverBackend Backends[] = { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA, SolverBackend::Metal };
	const char* BackendNames[] = { "CPU", "SIMD", "ComputeShader", "CUDA", "Metal" };

	PBDBackendVector BackendVector(const Vector3& value, float w = 0.0f)
	{
		return { value.x, value.y, value.z, w };
	}

	Vector3 BackendVector(const PBDBackendVector& value)
	{
		return Vector3(value.x, value.y, value.z);
	}

	PBDBackendBody BackendBody(const Vector3& position, bool dynamic)
	{
		PBDBackendBody result = {};
		result.Position = BackendVector(position, dynamic ? 1.0f : 0.0f);
		result.Rotation.w = 1.0f;
		result.LinearVelocity.w = 10000.0f;
		result.InverseInertia0.x = dynamic ? 1.5f : 0.0f;
		result.InverseInertia1.y = dynamic ? 2.0f : 0.0f;
		result.InverseInertia2.z = dynamic ? 2.5f : 0.0f;
		result.Flags.x = result.Flags.y = dynamic ? 1 : 0;
		return result;
	}

	PBDBackendContact BackendContact(const std::vector<PBDBackendBody>& bodies, uint32_t a, uint32_t b,
		const Vector3& point, const Vector3& normal, float depth)
	{
		PBDBackendContact contact = {};
		contact.Bodies.x = a;
		contact.Bodies.y = b;
		const Quaternion qa(bodies[a].Rotation.x, bodies[a].Rotation.y, bodies[a].Rotation.z, bodies[a].Rotation.w);
		const Quaternion qb(bodies[b].Rotation.x, bodies[b].Rotation.y, bodies[b].Rotation.z, bodies[b].Rotation.w);
		contact.AnchorA = BackendVector(qa.Conjugate() * (point - BackendVector(bodies[a].Position)));
		contact.AnchorB = BackendVector(qb.Conjugate() * (point - normal * depth - BackendVector(bodies[b].Position)));
		contact.Normal = BackendVector(normal);
		Vector3 tangent, binormal;
		normal.GetPerpVectors(tangent, binormal);
		contact.Tangent = BackendVector(tangent);
		contact.Binormal = BackendVector(binormal);
		contact.Material = { 0.5f, 0.3f, 0.6f, 0.0f };
		return contact;
	}

	struct BackendRecords
	{
		std::vector<PBDBackendBody> Bodies;
		std::vector<PBDBackendContact> Contacts;
	};

	BackendRecords IndependentRecords(int count = 17)
	{
		BackendRecords result;
		result.Bodies.push_back(BackendBody(Vector3::Zero(), false));
		for (int i = 0; i < count; ++i)
		{
			const int pattern = i % 17;
			const float x = pattern * 2.0f, z = (i / 17) * 2.0f;
			PBDBackendBody body = BackendBody(Vector3(x, 0.4f, z), true);
			body.LinearVelocity = BackendVector(Vector3(1.0f + 0.1f * pattern, -2.0f - 0.03f * pattern, 0.3f), 10000.0f);
			body.AngularVelocity = BackendVector(Vector3(0.1f, -0.2f, 0.3f));
			Quaternion rotation;
			rotation.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.025f * pattern);
			body.Rotation = { rotation.x, rotation.y, rotation.z, rotation.w };
			result.Bodies.push_back(body);
			result.Contacts.push_back(BackendContact(result.Bodies, 0, (uint32_t)i + 1,
				Vector3(x + 0.11f, 0.0f, z + 0.07f), Vector3::UnitY(), 0.1f + pattern * 0.001f));
		}
		return result;
	}

	BackendRecords DependentRecords()
	{
		BackendRecords result;
		result.Bodies.push_back(BackendBody(Vector3::Zero(), false));
		for (int i = 0; i < 4; ++i)
		{
			PBDBackendBody body = BackendBody(Vector3(i * 0.6f, 0.4f + i * 0.2f, i * 0.1f), true);
			body.LinearVelocity = BackendVector(Vector3(0.8f - i * 0.4f, -2.0f - i * 0.3f, 0.2f), 10000.0f);
			body.AngularVelocity = BackendVector(Vector3(0.2f, 0.1f * i, -0.1f));
			result.Bodies.push_back(body);
		}
		PBDBackendBody kinematic = BackendBody(Vector3(0.1f, 0.0f, 0.1f), false);
		kinematic.Flags.y = 1;
		kinematic.LinearVelocity = BackendVector(Vector3(0.7f, 0.0f, 0.2f), 10000.0f);
		kinematic.AngularVelocity = BackendVector(Vector3(0.0f, 0.3f, 0.0f));
		result.Bodies.push_back(kinematic);
		result.Bodies.push_back(BackendBody(Vector3(0.0f, -1.0f, 0.0f), false));
		result.Contacts.push_back(BackendContact(result.Bodies, 0, 1, Vector3(0.1f, 0, 0.1f), Vector3::UnitY(), 0.12f));
		result.Contacts.push_back(BackendContact(result.Bodies, 1, 2, Vector3(0.4f, 0.5f, 0.1f), Vector3::UnitX(), 0.08f));
		result.Contacts.push_back(BackendContact(result.Bodies, 0, 3, Vector3(1.3f, 0, 0.2f), Vector3::UnitY(), 0.1f));
		result.Contacts.push_back(BackendContact(result.Bodies, 2, 3, Vector3(1.0f, 0.6f, 0.2f), Vector3(1, 1, 0).Unit(), 0.09f));
		result.Contacts.push_back(BackendContact(result.Bodies, 3, 4, Vector3(1.5f, 0.7f, 0.2f), Vector3::UnitX(), 0.07f));
		result.Contacts.push_back(BackendContact(result.Bodies, 5, 1, Vector3(-0.1f, 0.1f, 0.0f), Vector3::UnitY(), 0.08f));
		result.Contacts.push_back(BackendContact(result.Bodies, 2, 0, Vector3(0.5f, -0.08f, 0.0f), -Vector3::UnitY(), 0.08f));
		result.Contacts.push_back(BackendContact(result.Bodies, 6, 4, Vector3(1.8f, 0, 0.3f), Vector3::UnitY(), 0.11f));
		return result;
	}

	void CheckDispatch(const PBDBackendStatistics& stats, SolverBackend requested, bool fallback, bool allStages)
	{
		EXPECT(stats.RequestedBackend == requested);
		EXPECT(stats.Backend == (fallback ? SolverBackend::CPU : requested));
		EXPECT(stats.UsedFallback == fallback);
		EXPECT(stats.PositionDispatches > 0);
		if (allStages)
		{
			EXPECT(stats.ReconstructionDispatches > 0);
			EXPECT(stats.VelocityDispatches > 0);
		}
		EXPECT(stats.ContactRows > 0);
	}

	void CompareRecords(const BackendRecords& actual, const BackendRecords& expected, float tolerance)
	{
		EXPECT(actual.Bodies.size() == expected.Bodies.size());
		EXPECT(actual.Contacts.size() == expected.Contacts.size());
		for (size_t i = 0; i < std::min(actual.Bodies.size(), expected.Bodies.size()); ++i)
		{
			const PBDBackendBody& a = actual.Bodies[i];
			const PBDBackendBody& e = expected.Bodies[i];
			EXPECT((BackendVector(a.Position) - BackendVector(e.Position)).Length() < tolerance);
			EXPECT((BackendVector(a.LinearVelocity) - BackendVector(e.LinearVelocity)).Length() < tolerance);
			EXPECT((BackendVector(a.AngularVelocity) - BackendVector(e.AngularVelocity)).Length() < tolerance);
			const Quaternion qa(a.Rotation.x, a.Rotation.y, a.Rotation.z, a.Rotation.w);
			const Quaternion qe(e.Rotation.x, e.Rotation.y, e.Rotation.z, e.Rotation.w);
			EXPECT_NEAR(std::abs(qa.Dot(qe)), 1.0f, tolerance);
			EXPECT_NEAR(a.Position.w, e.Position.w, 1.0e-7f);
		}
		for (size_t i = 0; i < std::min(actual.Contacts.size(), expected.Contacts.size()); ++i)
		{
			const PBDBackendVector& a = actual.Contacts[i].Lambda;
			const PBDBackendVector& e = expected.Contacts[i].Lambda;
			EXPECT_NEAR(a.x, e.x, tolerance);
			EXPECT_NEAR(a.y, e.y, tolerance);
			EXPECT_NEAR(a.z, e.z, tolerance);
			EXPECT_NEAR(a.w, e.w, tolerance);
		}
	}

	BackendRecords ExecuteRecords(const BackendRecords& input, SolverBackend backend, float alpha, bool independent)
	{
		BackendRecords result = input;
		PBDBackend dispatcher;
		EXPECT(dispatcher.Configure(backend, false));
		PBDBackendParameters parameters;
		parameters.Alpha = alpha;
		parameters.PenetrationSlop = 0.0f;
		// A single sweep exposes ordering errors that many iterations could conceal.
		EXPECT(dispatcher.Execute(result.Bodies, result.Contacts, parameters, PBDBackendStage::Position));
		EXPECT(dispatcher.Execute(result.Bodies, result.Contacts, parameters, PBDBackendStage::Reconstruct));
		EXPECT(dispatcher.Execute(result.Bodies, result.Contacts, parameters, PBDBackendStage::Velocity));
		CheckDispatch(dispatcher.GetStatistics(), backend, false, true);
		if (independent) EXPECT(dispatcher.GetStatistics().MaxBatchSize >= 17);
		for (size_t i = 0; i < input.Bodies.size(); ++i)
			if (!input.Bodies[i].Flags.x)
				EXPECT(std::memcmp(&result.Bodies[i], &input.Bodies[i], sizeof(PBDBackendBody)) == 0);
		return result;
	}

	void TestBackendCapacityChanges(SolverBackend backend)
	{
		PBDBackend dispatcher, reference;
		EXPECT(dispatcher.Configure(backend, false));
		EXPECT(reference.Configure(SolverBackend::CPU, false));
		PBDBackendParameters parameters;
		parameters.PenetrationSlop = 0.0f;
		// Reuse both contexts across growth, multiple thread groups, and shrinkage.
		for (int count : { 17, 263, 9 })
		for (float alpha : { 0.0f, 0.72f })
		{
			BackendRecords actual = IndependentRecords(count);
			BackendRecords expected = actual;
			const PBDBackendBody staticPlane = actual.Bodies[0];
			parameters.Alpha = alpha;
			for (PBDBackendStage stage : { PBDBackendStage::Position, PBDBackendStage::Reconstruct, PBDBackendStage::Velocity })
			{
				EXPECT(reference.Execute(expected.Bodies, expected.Contacts, parameters, stage));
				EXPECT(dispatcher.Execute(actual.Bodies, actual.Contacts, parameters, stage));
				CompareRecords(actual, expected, 3.0e-4f);
				EXPECT(std::memcmp(&actual.Bodies[0], &staticPlane, sizeof(PBDBackendBody)) == 0);
			}
			EXPECT(dispatcher.GetStatistics().MaxBatchSize >= (uint32_t)count);
		}
		CheckDispatch(dispatcher.GetStatistics(), backend, false, true);
		EXPECT(dispatcher.GetStatistics().MaxBatchSize >= 263);
	}

	void TestBackendContactOracles(SolverBackend backend)
	{
		for (float alpha : { 0.0f, 1.0f })
		{
			PBDBackend dispatcher;
			EXPECT(dispatcher.Configure(backend, false));
			BackendRecords records;
			records.Bodies = { BackendBody(Vector3::Zero(), false), BackendBody(Vector3(0, 0.4f, 0), true) };
			records.Bodies[1].InverseInertia0 = records.Bodies[1].InverseInertia1 = records.Bodies[1].InverseInertia2 = {};
			records.Contacts = { BackendContact(records.Bodies, 0, 1, Vector3::Zero(), Vector3::UnitY(), 0.1f) };
			records.Contacts[0].Material = {};
			PBDBackendParameters parameters;
			parameters.DeltaTime = 0.1f;
			parameters.PenetrationSlop = 0.0f;
			parameters.Alpha = alpha;
			for (int i = 0; i < 30; ++i)
				EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, parameters, PBDBackendStage::Position));
			const float correction = 0.1f / (1.0f + alpha);
			EXPECT_NEAR(records.Bodies[1].Position.y, 0.4f + correction, 2.0e-6f);
			EXPECT_NEAR(records.Contacts[0].Lambda.x, correction, 2.0e-6f);
			EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, parameters, PBDBackendStage::Reconstruct));
			EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, parameters, PBDBackendStage::Velocity));
			EXPECT(BackendVector(records.Bodies[1].LinearVelocity).Length() < 2.0e-5f);
			CheckDispatch(dispatcher.GetStatistics(), backend, false, true);
		}
		for (float alpha : { 0.0f, 1.0f })
		{
			PBDBackend dispatcher;
			EXPECT(dispatcher.Configure(backend, false));
			BackendRecords records;
			records.Bodies = { BackendBody(Vector3::Zero(), false), BackendBody(Vector3(0.4f, 0, 0), true), BackendBody(Vector3(1.3f, 0, 0), true) };
			for (PBDBackendBody& body : records.Bodies) body.InverseInertia0 = body.InverseInertia1 = body.InverseInertia2 = {};
			records.Contacts = {
				BackendContact(records.Bodies, 0, 1, Vector3::Zero(), Vector3::UnitX(), 0.1f),
				BackendContact(records.Bodies, 1, 2, Vector3(0.9f, 0, 0), Vector3::UnitX(), 0.1f)
			};
			PBDBackendParameters parameters;
			parameters.PenetrationSlop = 0.0f;
			parameters.Alpha = alpha;
			EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, parameters, PBDBackendStage::Position));
			// The second row must read the first row's updated shared body pose.
			const float first = 0.1f / (1.0f + alpha);
			const float second = (0.1f + first) / (2.0f + alpha);
			EXPECT_NEAR(records.Bodies[1].Position.x, 0.4f + first - second, 2.0e-6f);
			EXPECT_NEAR(records.Bodies[2].Position.x, 1.3f + second, 2.0e-6f);
			EXPECT_NEAR(records.Contacts[1].Lambda.x, second, 2.0e-6f);
		}
		for (int bounce = 0; bounce < 2; ++bounce)
		{
			PBDBackend dispatcher;
			EXPECT(dispatcher.Configure(backend, false));
			BackendRecords records;
			records.Bodies = { BackendBody(Vector3::Zero(), false), BackendBody(Vector3(0, 0.5f, 0), true) };
			records.Bodies[1].InverseInertia0 = records.Bodies[1].InverseInertia1 = records.Bodies[1].InverseInertia2 = {};
			records.Bodies[1].LinearVelocity = BackendVector(Vector3(2, -2, 0), 10000.0f);
			records.Contacts = { BackendContact(records.Bodies, 0, 1, Vector3::Zero(), Vector3::UnitY(), 0.0f) };
			records.Contacts[0].Material = { bounce ? 2.0f : 0.0f, bounce ? 0.0f : 0.5f, bounce ? 0.0f : 0.5f, 0.0f };
			PBDBackendParameters parameters;
			EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, parameters, PBDBackendStage::Velocity));
			EXPECT_NEAR(records.Bodies[1].LinearVelocity.y, bounce ? 2.0f : 0.0f, 1.0e-5f);
			EXPECT_NEAR(records.Bodies[1].LinearVelocity.x, bounce ? 2.0f : 1.0f, 1.0e-5f);
		}
	}

	struct SolverSnapshot
	{
		Vector3 Position, Velocity, AngularVelocity;
		Quaternion Rotation;
		float Lambda, NormalImpulse;
		PBDBackendStatistics Statistics;
	};

	SolverSnapshot RunSolverFixture(SolverBackend backend, bool xpbd, bool kinematicContact, bool offsetCOM = true)
	{
		PhysicsWorld world{ PhysicsWorldParam() };
		RigidBodyParam dynamic;
		dynamic.rigidType = RigidType::Dynamic;
		dynamic.disableGravity = true;
		dynamic.maxContactImpulse = 10000.0f;
		RigidBodyDynamic* body = world.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(1, 0.4f, -0.3f), Vector3(0.5f)), dynamic)->CastDynamic();
		body->CM = offsetCOM ? Vector3(0.3f, -0.1f, 0.2f) : Vector3::Zero();
		body->Q.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.4f);
		body->InvMass = 0.7f;
		body->InvInertia = Matrix3(1.3f, 1.9f, 2.4f);
		body->V = Vector3(1, -3, 0.3f);
		body->W = Vector3(0.1f, -0.2f, 0.4f);
		body->SetRestitution(0.6f);
		RigidBodyParam fixed;
		RigidBody* ground = world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), fixed);
		ground->SetRestitution(0.0f);
		RigidBodyKinematics kinematic;
		std::unique_ptr<Geometry> kinematicShape(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()));
		kinematicShape->SetParent(&kinematic);
		kinematic.SetPosition(Vector3(0.2f, 0, -0.1f));
		kinematic.V = Vector3(0.8f, 0.1f, -0.3f);
		kinematic.W = Vector3(0.1f, 0, -0.1f);
		kinematic.SetRestitution(0.0f);
		const Vector3 kinematicPosition = kinematic.X, kinematicVelocity = kinematic.V, kinematicAngular = kinematic.W;
		XPBDSolverSettings settings;
		settings.Backend = backend;
		settings.AllowBackendFallback = false;
		settings.ContactCompliance = xpbd ? 0.0002f : 0.0f;
		std::unique_ptr<PBDSolver> solver(xpbd ? static_cast<PBDSolver*>(new XPBDSolver(settings)) : new PBDSolver(settings));
		const float dt = 1.0f / 60.0f;
		solver->BeginStep({ body }, dt);
		solver->PredictPoses(dt);
		std::vector<Geometry*> geometries = { kinematicContact ? kinematicShape.get() : ground->Geometries()[0], body->Geometries()[0] };
		ContactManifold manifold;
		manifold.indexA = 0;
		manifold.indexB = 1;
		manifold.NumContactPointCount = 2;
		for (int i = 0; i < 2; ++i)
		{
			Contact& point = manifold.ContactPoints[i];
			point.PositionWorldA = Vector3(0.75f + i * 0.5f, 0, -0.2f);
			point.PositionWorldB = point.PositionWorldA - Vector3(0, 0.08f, 0);
			point.Normal = Vector3::UnitY();
			point.PenetrationDepth = 0.08f;
		}
		solver->AddContacts(geometries, { &manifold });
		EXPECT(solver->GetContactCount() == 2);
		for (int i = 0; i < 8; ++i) solver->SolvePositions(dt);
		solver->ReconstructVelocities(dt);
		for (int i = 0; i < 8; ++i) solver->SolveVelocities(dt);
		solver->EndStep(dt);
		EXPECT_SAME(kinematic.X, kinematicPosition);
		EXPECT_SAME(kinematic.V, kinematicVelocity);
		EXPECT_SAME(kinematic.W, kinematicAngular);
		SolverSnapshot result;
		result.Position = body->X;
		result.Rotation = body->Q;
		result.Velocity = body->V;
		result.AngularVelocity = body->W;
		result.Lambda = solver->GetContactPositionLambda(0);
		result.NormalImpulse = 0.0f;
		for (const ContactManifold* output : solver->GetContactManifolds())
			for (int i = 0; i < output->NumContactPointCount; ++i) result.NormalImpulse += output->ContactPoints[i].totalImpulseNormal;
		result.Statistics = solver->GetBackendStatistics();
		CheckDispatch(result.Statistics, backend, false, true);
		EXPECT(solver->GetBackend() == backend);
		EXPECT(result.Lambda > 0.0f);
		EXPECT(result.NormalImpulse > 0.0f);
		EXPECT(result.AngularVelocity.Length() > 0.01f);
		return result;
	}

	void CompareSnapshots(const SolverSnapshot& actual, const SolverSnapshot& expected)
	{
		EXPECT((actual.Position - expected.Position).Length() < 2.0e-4f);
		EXPECT((actual.Velocity - expected.Velocity).Length() < 5.0e-4f);
		EXPECT((actual.AngularVelocity - expected.AngularVelocity).Length() < 5.0e-4f);
		EXPECT_NEAR(std::abs(actual.Rotation.Dot(expected.Rotation)), 1.0f, 1.0e-5f);
		EXPECT_NEAR(actual.Lambda, expected.Lambda, 2.0e-5f);
		EXPECT_NEAR(actual.NormalImpulse, expected.NormalImpulse, 5.0e-4f);
	}

	void TestBackendStack(SolverBackend backend, const char* name)
	{
		const auto started = std::chrono::steady_clock::now();
		PhysicsWorldParam settings;
		settings.resolutionSolver = ResolutionPhaseSolver::PBD;
		settings.solverBackend = backend;
		settings.allowBackendFallback = false;
		settings.gravityAcc = Vector3(0, -10, 0);
		settings.pbdSettings.PositionIterations = 20;
		settings.pbdSettings.VelocityIterations = 8;
		PhysicsWorld world(settings);
		RigidBodyParam fixed;
		fixed.rigidType = RigidType::Static;
		world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), fixed)
			->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Concrete);
		std::vector<RigidBodyDynamic*> boxes;
		for (int i = 0; i < 3; ++i)
		{
			RigidBodyParam dynamic;
			dynamic.rigidType = RigidType::Dynamic;
			dynamic.linearDamping = 1.0f;
			dynamic.angularDamping = 1.0f;
			dynamic.sleepThreshold = 0.0f;
			dynamic.maxContactImpulse = 100.0f;
			RigidBodyDynamic* body = world.CreateRigidBody(
				GeometryFactory::CreateOBB(Vector3(0, 0.5f + i, 0), Vector3(0.5f)), dynamic)->CastDynamic();
			body->InvMass = 1.0f;
			body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Concrete);
			boxes.push_back(body);
		}
		float maxSpeed = 0.0f, maxAngular = 0.0f, maxDrift = 0.0f;
		for (int frame = 0; frame < 240; ++frame)
		{
			world.Simulate();
			for (const RigidBodyDynamic* body : boxes)
			{
				EXPECT(std::isfinite(body->X.x) && std::isfinite(body->X.y) && std::isfinite(body->X.z));
				EXPECT(std::isfinite(body->V.SquareLength()) && std::isfinite(body->W.SquareLength()));
				EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1.0e-3f);
				maxSpeed = std::max(maxSpeed, body->V.Length());
				maxAngular = std::max(maxAngular, body->W.Length());
				maxDrift = std::max(maxDrift, std::sqrt(body->X.x * body->X.x + body->X.z * body->X.z));
			}
		}
		for (size_t i = 0; i < boxes.size(); ++i)
		{
			const RigidBodyDynamic* body = boxes[i];
			printf("  %s stack[%zu] X=(%.4f,%.4f,%.4f) speed=%.4f angular=%.4f up=%.4f\n",
				name, i, body->X.x, body->X.y, body->X.z, body->V.Length(), body->W.Length(), (body->Q * Vector3::UnitY()).y);
			// Use the original PBD stack's physical tolerances, not trajectory equality.
			EXPECT_NEAR(body->X.y, 0.5f + i, 0.12f);
			EXPECT(std::abs(body->X.x) < 0.15f && std::abs(body->X.z) < 0.15f);
			EXPECT((body->Q * Vector3::UnitY()).y > 0.95f);
			EXPECT(body->V.Length() < 0.3f);
		}
		EXPECT(world.GetSolverBackend() == backend);
		CheckDispatch(world.GetPositionSolver()->GetBackendStatistics(), backend, false, true);
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
		printf("  %s stack 240 frames: max speed=%.4f angular=%.4f lateral drift=%.4f, %.3f s\n",
			name, maxSpeed, maxAngular, maxDrift, seconds);
	}

	void TestBackendFailures()
	{
		for (SolverBackend backend : Backends)
		{
			PBDBackend dispatcher;
			BackendRecords records = IndependentRecords();
			const BackendRecords original = records;
			if (!IsPBDBackendAvailable(backend))
			{
				EXPECT_FALSE(dispatcher.Configure(backend, false));
				EXPECT_FALSE(dispatcher.Execute(records.Bodies, records.Contacts, PBDBackendParameters(), PBDBackendStage::Position));
				EXPECT(std::memcmp(records.Bodies.data(), original.Bodies.data(), records.Bodies.size() * sizeof(PBDBackendBody)) == 0);
				EXPECT(std::memcmp(records.Contacts.data(), original.Contacts.data(), records.Contacts.size() * sizeof(PBDBackendContact)) == 0);
				PBDSolverSettings settings;
				settings.Backend = backend;
				settings.AllowBackendFallback = false;
				bool threw = false;
				try { PBDSolver solver(settings); }
				catch (const std::runtime_error&) { threw = true; }
				EXPECT(threw);
				PBDSolver configured;
				threw = false;
				try { configured.SetSettings(settings); }
				catch (const std::runtime_error&) { threw = true; }
				EXPECT(threw);
				EXPECT(dispatcher.Configure(backend, true));
				EXPECT(dispatcher.Execute(records.Bodies, records.Contacts, PBDBackendParameters(), PBDBackendStage::Position));
				CheckDispatch(dispatcher.GetStatistics(), backend, true, false);
			}
			else
			{
				EXPECT(dispatcher.Configure(backend, false));
				records.Contacts[0].Bodies.y = (uint32_t)records.Bodies.size();
				const BackendRecords invalid = records;
				EXPECT_FALSE(dispatcher.Execute(records.Bodies, records.Contacts, PBDBackendParameters(), PBDBackendStage::Position));
				EXPECT(std::memcmp(records.Bodies.data(), invalid.Bodies.data(), records.Bodies.size() * sizeof(PBDBackendBody)) == 0);
				EXPECT(std::memcmp(records.Contacts.data(), invalid.Contacts.data(), records.Contacts.size() * sizeof(PBDBackendContact)) == 0);
				EXPECT(dispatcher.GetStatistics().PositionDispatches == 0);
				records = original;
				PBDBackendParameters invalidTime;
				invalidTime.DeltaTime = 0.0f;
				EXPECT_FALSE(dispatcher.Execute(records.Bodies, records.Contacts, invalidTime, PBDBackendStage::Position));
				EXPECT(std::memcmp(records.Bodies.data(), original.Bodies.data(), records.Bodies.size() * sizeof(PBDBackendBody)) == 0);
				EXPECT(std::memcmp(records.Contacts.data(), original.Contacts.data(), records.Contacts.size() * sizeof(PBDBackendContact)) == 0);
			}
		}
	}




}

void TestPBDBackends()
{
	printf("Running TestPBDBackends\n");
	EXPECT(IsPBDBackendAvailable(SolverBackend::CPU));
	const char* requireGPU = std::getenv("RIEMANN_REQUIRE_PBD_GPU");
	if (requireGPU && std::strcmp(requireGPU, "1") == 0)
	{
		EXPECT(IsPBDBackendAvailable(SolverBackend::ComputeShader));
		EXPECT(IsPBDBackendAvailable(SolverBackend::CUDA));
	}
	const BackendRecords independent = IndependentRecords();
	const BackendRecords dependent = DependentRecords();
	for (size_t index = 0; index < 5; ++index)
	{
		const SolverBackend backend = Backends[index];
		printf("  PBD backend %s: %s\n", BackendNames[index], IsPBDBackendAvailable(backend) ? "available" : "unavailable (strict/fallback tested)");
		if (!IsPBDBackendAvailable(backend)) continue;
		TestBackendContactOracles(backend);
		TestBackendCapacityChanges(backend);
		TestBackendStack(backend, BackendNames[index]);
		for (float alpha : { 0.0f, 0.72f })
		{
			CompareRecords(ExecuteRecords(independent, backend, alpha, true), ExecuteRecords(independent, SolverBackend::CPU, alpha, true), 3.0e-4f);
			CompareRecords(ExecuteRecords(dependent, backend, alpha, false), ExecuteRecords(dependent, SolverBackend::CPU, alpha, false), 3.0e-4f);
		}
		for (bool xpbd : { false, true })
		for (bool kinematic : { false, true })
			CompareSnapshots(RunSolverFixture(backend, xpbd, kinematic), RunSolverFixture(SolverBackend::CPU, xpbd, kinematic));
	}
	// Moving the body-local origin cannot change the same world-space contact response.
	for (bool xpbd : { false, true })
	for (bool kinematic : { false, true })
		CompareSnapshots(RunSolverFixture(SolverBackend::CPU, xpbd, kinematic), RunSolverFixture(SolverBackend::CPU, xpbd, kinematic, false));
	TestBackendFailures();


}
}

void TestPBDBackends() { Test_PBDBackends_Cases::TestPBDBackends(); }

namespace Test_XPBD_Cases
{
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
}

void TestXPBDRigidSolver() { Test_XPBD_Cases::TestXPBDRigidSolver(); }

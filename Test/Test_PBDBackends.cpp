#include "Test.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Deformation/PBDBackend.h"
#include "../Src/Deformation/PBDSolver.h"
#include "../Src/Deformation/XPBDSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	const SolverBackend Backends[] = { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA };
	const char* BackendNames[] = { "CPU", "SIMD", "ComputeShader", "CUDA" };

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

	std::string BackendSceneJson(const char* mode, const char* backend, bool fallback)
	{
		return std::string("{\"physics\":{\"gravity\":[0,0,0],\"resolutionSolver\":\"") + mode
			+ "\",\"solverBackend\":\"" + backend + "\",\"allowBackendFallback\":" + (fallback ? "true" : "false")
			+ ",\"pbd\":{\"positionIterations\":3,\"velocityIterations\":3},\"xpbd\":{\"positionIterations\":3,\"velocityIterations\":3,\"contactCompliance\":0.001}},"
			+ "\"objects\":[{\"id\":\"floor\",\"type\":\"plane\",\"body\":\"static\",\"position\":[0,0,0],\"normal\":[0,1,0]},"
			+ "{\"id\":\"ball\",\"type\":\"sphere\",\"body\":\"dynamic\",\"position\":[0,0.4,0],\"radius\":0.5,\"invMass\":1,\"linearVelocity\":[0,-2,0]}]}";
	}

	void CheckSceneFailurePreservesWorld(const char* mode, const std::string& invalidScene)
	{
		SceneWorld scene;
		std::string error;
		const bool loaded = scene.LoadFromText(BackendSceneJson(mode, "CPU", false), "backend_original.json", &error);
		EXPECT(loaded);
		if (!loaded) return;
		scene.Step(1.0f / 60.0f);
		PhysicsWorld* originalWorld = scene.GetSimulation();
		const std::vector<SceneObjectInstance> originalObjects = scene.GetObjects();
		const std::string originalName = scene.GetName();
		const uint32_t originalRevision = scene.GetRenderRevision();
		const uint64_t originalDispatches = originalWorld->GetPositionSolver()->GetBackendStatistics().PositionDispatches;
		error.clear();
		EXPECT_FALSE(scene.LoadFromText(invalidScene, "backend_invalid_reload.json", &error));
		EXPECT(!error.empty());
		EXPECT(scene.GetSimulation() == originalWorld);
		EXPECT(scene.GetObjects().size() == originalObjects.size());
		EXPECT(scene.GetName() == originalName);
		EXPECT(scene.GetRenderRevision() == originalRevision);
		for (size_t i = 0; i < std::min(scene.GetObjects().size(), originalObjects.size()); ++i)
		{
			EXPECT(scene.GetObjects()[i].Id == originalObjects[i].Id);
			EXPECT(scene.GetObjects()[i].GeometryPtr == originalObjects[i].GeometryPtr);
		}
		if (scene.GetSimulation() != originalWorld) return;
		EXPECT(scene.GetSimulation()->GetSolverBackend() == SolverBackend::CPU);
		EXPECT(scene.GetSimulation()->GetPositionSolver()->GetBackendStatistics().PositionDispatches == originalDispatches);
		// The retained world remains usable after a rejected replacement.
		scene.Step(1.0f / 60.0f);
		EXPECT(scene.GetSimulation() == originalWorld);
		EXPECT(scene.GetSimulation()->GetPositionSolver()->GetBackendStatistics().PositionDispatches >= originalDispatches);
	}

	void TestBackendSceneFailurePreservation()
	{
		for (const char* mode : { "pbd", "xpbd" })
		{
			CheckSceneFailurePreservesWorld(mode, BackendSceneJson(mode, "unknown_backend", true));
			std::string invalidType = BackendSceneJson(mode, "CPU", false);
			const std::string validFragment = "\"allowBackendFallback\":false";
			const size_t offset = invalidType.find(validFragment);
			EXPECT(offset != std::string::npos);
			if (offset != std::string::npos)
			{
				invalidType.replace(offset, validFragment.size(), "\"allowBackendFallback\":\"false\"");
				CheckSceneFailurePreservesWorld(mode, invalidType);
			}
			for (size_t i = 0; i < 4; ++i)
				if (!IsPBDBackendAvailable(Backends[i]))
					CheckSceneFailurePreservesWorld(mode, BackendSceneJson(mode, BackendNames[i], false));
		}
	}

	void TestBackendSceneSelection()
	{
		for (const char* mode : { "pbd", "xpbd" })
		for (size_t index = 0; index < 4; ++index)
		{
			const SolverBackend backend = Backends[index];
			const bool available = IsPBDBackendAvailable(backend);
			SceneWorld scene;
			std::string error;
			const bool loaded = scene.LoadFromText(BackendSceneJson(mode, BackendNames[index], false), "backend_scene.json", &error);
			EXPECT(loaded == available);
			if (!available)
			{
				EXPECT(!error.empty());
				EXPECT(scene.LoadFromText(BackendSceneJson(mode, BackendNames[index], true), "backend_fallback.json", &error));
			}
			if (scene.GetSimulation()->GetPositionSolver() == nullptr) continue;
			scene.Step(1.0f / 60.0f);
			const PBDSolver* solver = scene.GetSimulation()->GetPositionSolver();
			EXPECT(scene.GetSimulation()->GetSolverBackend() == (available ? backend : SolverBackend::CPU));
			CheckDispatch(solver->GetBackendStatistics(), backend, !available, true);
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
	for (size_t index = 0; index < 4; ++index)
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
	TestBackendSceneSelection();
	TestBackendSceneFailurePreservation();
}

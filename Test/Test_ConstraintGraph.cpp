#include "Test.h"
#include "../Src/RigidBodyDynamics/Island.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Solver/XPBDSolver.h"
#include "../Src/Solver/LCPSolver.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Viewer/SceneWorld.h"
#include <memory>

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

	void TestConfiguration()
	{
		SceneWorld Scene;
		std::string Error;
		EXPECT(Scene.LoadFromText(R"({"physics":{"unifiedConstraints":false},"objects":[]})", "legacy graph", &Error));
		EXPECT(!Scene.GetSimulation()->GetUnifiedConstraintSolving());
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
			ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			EXPECT(Scene.GetSimulation()->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			EXPECT(!Scene.GetSimulation()->GetUnifiedConstraintSolving());
			Scene.GetSimulation()->SetUnifiedConstraintSolving(true);
			EXPECT(Scene.GetSimulation()->GetUnifiedConstraintSolving());
			Scene.GetSimulation()->SetUnifiedConstraintSolving(false);
			EXPECT(!Scene.GetSimulation()->GetUnifiedConstraintSolving());
		}
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/06_constraints.json", &Error));
		EXPECT(Scene.GetSimulation()->GetUnifiedConstraintSolving());
		EXPECT(!Scene.LoadFromText(R"({"physics":{"unifiedConstraints":1}})", "invalid graph", &Error));
		EXPECT(Error.find("unifiedConstraints") != std::string::npos);
	}
}

void TestConstraintGraph()
{
	TestTopology(); TestVelocityCoupling(); TestPositionSolverCoupling(); TestConfiguration();
}

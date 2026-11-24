#include "Test.h"
#include <memory>
#include <stdexcept>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include "../Src/RigidBodyDynamics/Jacobian.h"
#include "../Src/RigidBodyDynamics/WarmStart.h"
#include "../Viewer/SceneWorld.h"

namespace
{
	using namespace Riemann;
	const float TimeStep = 1.0f / 60.0f;
	const float NumericalTolerance = 1.0e-5f;

	Contact MakeContact(float X, uint32_t Feature = 0)
	{
		Contact C;
		C.PositionWorldA = Vector3(X, 0.0f, 0.0f);
		C.PositionWorldB = Vector3(X, -0.01f, 0.0f);
		C.PositionLocalA = C.PositionWorldA;
		C.PositionLocalB = C.PositionWorldB;
		C.Normal = Vector3::UnitY();
		C.Tangent = Vector3::UnitX();
		C.Binormal = C.Normal.Cross(C.Tangent);
		C.PenetrationDepth = 0.01f;
		C.FeatureA = Feature;
		C.FeatureB = Feature;
		C.totalImpulseNormal = 2.0f;
		C.totalImpulseTangent = 0.2f;
		C.totalImpulseBinormal = -0.1f;
		return C;
	}

	void TestAnchorCache()
	{
		std::unique_ptr<Geometry> ShapeA(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		std::unique_ptr<Geometry> ShapeB(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		GeometryWorldState A, B;
		A.Geom = ShapeA.get(); B.Geom = ShapeB.get();
		A.Handle = GeometryHandle(0, 1); B.Handle = GeometryHandle(1, 1);
		ContactManifoldManager Cache;
		ContactManifold M;
		M.AddNewContact(0, 1, MakeContact(0.0f));
		M.AddNewContact(0, 1, MakeContact(0.1f, 1));
		Cache.BeginFrame(1);
		Cache.Store(A, B, M, TimeStep);
		const uint64_t Id = M.ContactPoints[0].PersistentId;
		EXPECT(Id != 0 && Cache.GetPairCount() == 1);
		Cache.BeginFrame(2);
		ContactManifold Refreshed;
		A.ShapeToWorld.pos = B.ShapeToWorld.pos = Vector3(100.0f, 20.0f, -40.0f);
		EXPECT(Cache.Refresh(A, B, &Refreshed));
		EXPECT(Refreshed.NumContactPointCount == 2);
		EXPECT(Refreshed.ContactPoints[0].PersistentId == Id);
		EXPECT_NEAR(Refreshed.ContactPoints[0].PositionWorldA.x, 100.0f, NumericalTolerance);
		ContactManifold Reversed;
		EXPECT(Cache.Refresh(B, A, &Reversed));
		EXPECT(Reversed.ContactPoints[0].Normal.Dot(Refreshed.ContactPoints[0].Normal) < 0.0f);
		EXPECT((Reversed.ContactPoints[0].PositionWorldA - Refreshed.ContactPoints[0].PositionWorldB).Length() < NumericalTolerance);
		const Contact& Forward = Refreshed.ContactPoints[0];
		const Contact& Backward = Reversed.ContactPoints[0];
		EXPECT((Forward.Normal.Cross(Forward.Tangent) - Forward.Binormal).Length() < NumericalTolerance);
		EXPECT((Backward.Normal.Cross(Backward.Tangent) - Backward.Binormal).Length() < NumericalTolerance);
		const Vector3 ForwardImpulse = Forward.Normal * Forward.totalImpulseNormal
			+ Forward.Tangent * Forward.totalImpulseTangent + Forward.Binormal * Forward.totalImpulseBinormal;
		const Vector3 BackwardImpulse = Backward.Normal * Backward.totalImpulseNormal
			+ Backward.Tangent * Backward.totalImpulseTangent + Backward.Binormal * Backward.totalImpulseBinormal;
		EXPECT((ForwardImpulse + BackwardImpulse).Length() < NumericalTolerance);

		A.ShapeToWorld = B.ShapeToWorld = Transform::Identity();
		ContactManifold Fresh;
		Fresh.AddNewContact(0, 1, MakeContact(0.001f));
		Fresh.AddNewContact(0, 1, MakeContact(0.002f));
		EXPECT(Cache.Match(A, B, &Fresh, true) == 1);
		EXPECT(Fresh.ContactPoints[0].PersistentId == Id);
		EXPECT_NEAR(Fresh.ContactPoints[0].totalImpulseNormal, 2.0f, NumericalTolerance);
		Cache.Store(A, B, Fresh, TimeStep);
		EXPECT(Cache.Match(A, B, &Fresh, true) == 0);
		++A.Handle.Generation;
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		ShapeA->MarkCollisionDirty();
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeA->UpdateVolumeProperties();
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeB->SetTriggerEnabled(true);
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeB->SetPosition(Vector3::UnitX());
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		Cache.Invalidate(ShapeB.get());
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		Cache.BeginFrame(2 + ContactPersistenceSettings().RetentionFrames + 1);
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		ContactManifold Empty;
		Cache.Store(A, B, Empty, TimeStep);
		EXPECT(Cache.GetPairCount() == 0);

		ContactPersistenceSettings Invalid;
		Invalid.MatchDistance = -1.0f;
		bool Rejected = false;
		try { ContactManifoldManager InvalidCache(Invalid); }
		catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
	}

	void TestScaledBoxFeatures()
	{
		// Changing length units must not turn a face witness into an edge or corner.
		for (float Scale : { 0.0001f, 1.0f, 10000.0f })
		for (int Axis = 0; Axis < 3; ++Axis)
		for (float Sign : { -1.0f, 1.0f })
		for (bool Reverse : { false, true })
		for (bool Persistent : { false, true })
		{
			Vector3 Center = Vector3::Zero(); Center[Axis] = Sign * Scale;
			std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f * Scale)));
			std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Center, 0.6f * Scale));
			Box->SetContactOffsets(0.01f * Scale, 0);
			GeometryWorldState States[2];
			States[0].Geom = Reverse ? Sphere.get() : Box.get();
			States[1].Geom = Reverse ? Box.get() : Sphere.get();
			for (int Index = 0; Index < 2; ++Index)
			{
				States[Index].Handle = GeometryHandle(Index, 1);
				States[Index].ShapeToWorld = *States[Index].Geom->GetTransform();
				States[Index].WorldBounds = States[Index].Geom->GetBounds();
			}
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
			std::unique_ptr<NarrowPhase> Narrow(Persistent ? NarrowPhase::Create_PCM() : NarrowPhase::Create_GJKEPA());
			ContactGenerationSettings Settings; Settings.LengthScale = Scale;
			Narrow->SetContactGeneration(Settings, TimeStep); Narrow->BeginFrame(1);
			std::vector<ContactManifold*> Contacts;
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			EXPECT(!Contacts.empty());
			const uint32_t Face = 1u << (2 * Axis + (Sign > 0 ? 1 : 0));
			for (const auto* Manifold : Contacts)
			for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
			{
				const auto& Point = Manifold->ContactPoints[Index];
				EXPECT((Reverse ? Point.FeatureB : Point.FeatureA) == Face);
				EXPECT((Reverse ? Point.FeatureA : Point.FeatureB) == 0);
			}
		}
	}

	void TestSameFrameAccumulation()
	{
		std::unique_ptr<Geometry> ShapeA(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		std::unique_ptr<Geometry> ShapeB(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		GeometryWorldState A, B;
		A.Geom = ShapeA.get(); B.Geom = ShapeB.get();
		ContactManifoldManager Cache;
		Cache.BeginFrame(1);
		ContactManifold First;
		First.AddNewContact(0, 1, MakeContact(0.0f));
		Cache.Store(A, B, First, TimeStep);
		ContactManifold Second;
		Second.AddNewContact(0, 1, MakeContact(0.001f));
		Second.AddNewContact(0, 1, MakeContact(0.002f));
		Cache.Store(A, B, Second, TimeStep, true, true);
		Cache.BeginFrame(2);
		EXPECT(Cache.Match(A, B, &Second, true) == 2);
		EXPECT_NEAR(Second.ContactPoints[0].totalImpulseNormal, 4.0f, NumericalTolerance);
		EXPECT_NEAR(Second.ContactPoints[1].totalImpulseNormal, 2.0f, NumericalTolerance);
		Cache.BeginFrame(1);
		EXPECT(Cache.GetPairCount() == 0);
	}

	void TestReductionAndImpulseRetraction()
	{
		ContactManifold M;
		for (int Index = 0; Index < 12; ++Index)
		{
			Contact C = MakeContact((float)Index);
			C.PenetrationDepth = (float)Index;
			M.AddNewContact(0, 1, C);
		}
		EXPECT(M.NumContactPointCount == MAX_CONTACT_POINTS);
		bool Deepest = false;
		for (int Index = 0; Index < M.NumContactPointCount; ++Index)
			Deepest = Deepest || M.ContactPoints[Index].PenetrationDepth == 11.0f;
		EXPECT(Deepest);

		RigidBodyParam StaticSettings, DynamicSettings;
		DynamicSettings.rigidType = RigidType::Dynamic;
		std::unique_ptr<RigidBody> A(RigidBody::CreateRigidBody(StaticSettings, Transform::Identity()));
		std::unique_ptr<RigidBody> B(RigidBody::CreateRigidBody(DynamicSettings, Transform::Identity()));
		GeneralizedVelocity Velocities[2] = {};
		Contact C = MakeContact(0.0f);
		C.PositionLocalA = C.PositionLocalB = Vector3::Zero();
		C.PenetrationDepth = 0.0f;
		C.CachedDt = TimeStep;
		ContactJacobianSolver Constraint(Velocities, 0, 1, A.get(), B.get());
		Constraint.SetupPositionPass(&C, TimeStep);
		WarmStart::Apply(Constraint, TimeStep, ContactPersistenceSettings());
		EXPECT(Constraint.m_jN.m_totalLambda > 0.0f);
		EXPECT(Velocities[1].v.y > 0.0f);
		Constraint.Solve();
		EXPECT_NEAR(Constraint.m_jN.m_totalLambda, 0.0f, NumericalTolerance);
		EXPECT(Velocities[1].v.Length() < NumericalTolerance);
	}

	void TestWarmWorldAndSwitches()
	{
		for (NarrowPhaseSolver Mode : { NarrowPhaseSolver::GJKEPA, NarrowPhaseSolver::PCM })
		{
			PhysicsWorldParam Settings;
			Settings.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
			Settings.narrowphase = Mode;
			PhysicsWorld World(Settings);
			RigidBodyParam StaticSettings, DynamicSettings;
			DynamicSettings.rigidType = RigidType::Dynamic;
			DynamicSettings.sleepThreshold = 0.0f;
			World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), StaticSettings);
			RigidBody* Ball = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0.0f, 0.49f, 0.0f), 0.5f), DynamicSettings);
			int WarmFrames = 0;
			for (int Frame = 0; Frame < 120; ++Frame)
			{
				World.Simulate();
				WarmFrames += World.GetWarmStartContactCount() > 0;
				EXPECT(std::isfinite(Ball->X.y) && Ball->X.y > 0.4f && Ball->X.y < 0.6f);
			}
			EXPECT(WarmFrames > 0);
			World.SetSequentialImpulseWarmStarting(false);
			World.Simulate();
			EXPECT(World.GetWarmStartContactCount() == 0);
			EXPECT(World.SetNarrowPhaseSolver(NarrowPhaseSolver::PCM));
			World.Simulate();
			EXPECT(World.GetCachedContactPairCount() > 0);
			EXPECT(World.RemoveRigidBody(Ball));
			Ball->ReleaseGeometries(); delete Ball;
			EXPECT(World.GetCachedContactPairCount() == 0);
			World.Reset();
			EXPECT(World.GetCachedContactPairCount() == 0);
		}
	}

	void TestExistingContactScenes()
	{
		struct SceneCase { const char* Name; float MinimumHeight; const char* Preset = ""; };
		const SceneCase Scenes[] = {
			{ "01_stack_boxes.json", -5.5f, "grid" }, { "03_domino.json", -0.5f, "classic" },
			{ "09_ragdoll_stairs.json", -3.0f }, { "14_terrain_editing.json", -0.5f }
		};
		for (const SceneCase& Example : Scenes)
		{
			for (NarrowPhaseSolver Mode : { NarrowPhaseSolver::GJKEPA, NarrowPhaseSolver::PCM })
			{
				for (ResolutionPhaseSolver Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
					ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
				{
					printf("Contact scene %s, solver %d, narrow phase %d\n", Example.Name, (int)Solver, (int)Mode);
					SceneWorld Scene;
					std::string Error;
					EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/" + Example.Name, &Error, Example.Preset));
					PhysicsWorld* World = Scene.GetSimulation();
					if (!World) continue;
					EXPECT(World->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
					EXPECT(World->SetNarrowPhaseSolver(Mode));
					float MinimumHeight = FLT_MAX;
					for (int Frame = 0; Frame < 90; ++Frame)
					{
						Scene.Step(TimeStep);
						std::vector<Geometry*> Shapes;
						World->GetAllGeometries(&Shapes);
						for (Geometry* Shape : Shapes)
						{
							RigidBody* Body = Shape->GetParent<RigidBody>();
							if (!Body || !Body->CastDynamic()) continue;
							EXPECT(std::isfinite(Body->X.SquareLength()) && std::isfinite(Body->V.SquareLength()));
							MinimumHeight = std::min(MinimumHeight, Body->X.y);
						}
					}
					printf("Minimum height: %g\n", MinimumHeight);
					EXPECT(MinimumHeight > Example.MinimumHeight);
				}
			}
		}
	}
}

void TestContactPersistence()
{
	TestAnchorCache();
	TestScaledBoxFeatures();
	TestSameFrameAccumulation();
	TestReductionAndImpulseRetraction();
	TestWarmWorldAndSwitches();
	TestExistingContactScenes();
}

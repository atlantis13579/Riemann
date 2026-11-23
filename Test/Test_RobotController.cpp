#include "Test.h"
#include "../Src/Modules/Robotics/ArticulatedRobotController.h"
#include "../Src/Modules/Robotics/SpiderRobot.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Modules/Robotics/SparseJointSpaceDynamics.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/DistanceFieldShape.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/LinearSystem/NeuralNetwork.h"
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 1.0f / 60.0f;
	RigidBody* Box(PhysicsWorld& World, const Vector3& Position, const Vector3& Extent, const Quaternion& Rotation = Quaternion::One())
	{
		return World.CreateRigidBody(GeometryFactory::CreateOBB(Position, Extent, Rotation), RigidBodyParam());
	}
	void Floor(PhysicsWorld& World)
	{
		World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
	}
	void Step(PhysicsWorld& World, int Frames) { for (int I = 0; I < Frames; ++I) World.Simulate(Dt); }
	float Height(const RobotControllerSettings& S) { return S.Shape == RobotShape::Capsule ? S.HalfHeight + S.Radius : S.HalfExtent.y; }

	void TestGroundAndSlide(RobotShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
		RobotControllerSettings S; S.Shape = Shape;
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, 2, 0));
		Step(World, 90);
		printf("Character shape=%d floor=(%.5f, %.5f, %.5f) grounded=%d\n", (int)Shape, C->GetPosition().x, C->GetPosition().y, C->GetPosition().z, C->IsGrounded());
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.005f);
		Box(World, Vector3(2, 2, 0), Vector3(0.2f, 2, 10));
		const RobotMoveResult Hit = C->Move(Vector3(4, 0, 2), Dt);
		EXPECT(Hit.HitSide); EXPECT(C->GetPosition().x < 1.46f); EXPECT_NEAR(C->GetPosition().z, 2, 0.03f);
		EXPECT(C->IsGrounded());
		EXPECT(C->Jump(4)); EXPECT(!C->Jump(4)); Step(World, 10); EXPECT(C->GetPosition().y > Height(S) + 0.3f);
		Step(World, 90); EXPECT(C->IsGrounded());
		C->SetPosition(Vector3(0, Height(S) - 0.15f, 0));
		const RobotMoveResult Recovered = C->Move(Vector3::Zero(), Dt);
		EXPECT(Recovered.RecoveryDistance > 0.1f); EXPECT(!Recovered.RecoveryIncomplete);
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.005f);
		C->SetPosition(Vector3(0, -2, 0));
		const RobotMoveResult Deep = C->Move(Vector3(1, 0, 0), Dt);
		EXPECT(Deep.RecoveryIncomplete); EXPECT(Deep.RecoveryDistance <= S.MaximumRecoveryDistance + 0.001f);
		EXPECT_NEAR(C->GetPosition().x, 0, 0.001f);
	}

	void TestBallisticJump(RobotShape Shape)
	{
		for (int Substeps : { 1, 4 })
		{
			PhysicsWorldParam P; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			PhysicsWorld World(P);
			RobotControllerSettings S; S.Shape = Shape; S.Gravity = 10;
			S.Up = Vector3::UnitZ();
			World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), S.Up), RigidBodyParam());
			SweepRobotController* C = World.CreateRobotController(S, S.Up * (Height(S) + S.SkinWidth));
			Step(World, 2);
			const float Start = C->GetPosition().z;
			EXPECT(C->Jump(5));
			C->SetMoveVelocity(Vector3(2, 0, 0));
			for (int Frame = 1; Frame < 10; ++Frame)
			{
				const float T = Frame * 0.1f;
				World.Simulate(0.1f);
				EXPECT_NEAR(C->GetPosition().z, Start + 5 * T - 5 * T * T, 0.001f);
				EXPECT_NEAR(C->GetPosition().x, 2 * T, 0.001f);
				EXPECT(!C->IsGrounded() && !C->Jump(5));
			}
			World.Simulate(0.1f); EXPECT(C->IsGrounded());
			EXPECT_NEAR(C->GetPosition().z, Start, 0.001f);
			EXPECT(C->Jump(5));
		}

		// The endpoint is below the ceiling, but the apex crosses it within the step.
		PhysicsWorldParam P; P.temporalSubsteps.Enabled = false;
		PhysicsWorld World(P); Floor(World);
		RobotControllerSettings S; S.Shape = Shape; S.Gravity = 10;
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(World, 2);
		Box(World, Vector3(0, C->GetPosition().y + Height(S) + S.SkinWidth + 0.2f, 0), Vector3(2, 0.1f, 2));
		EXPECT(C->Jump(2));
		World.Simulate(0.35f);
		EXPECT(C->GetLastMove().HitAbove && C->IsGrounded());
	}

	void TestSteps(RobotShape Shape)
	{
		for (float HeightOfStep : { 0.2f, 0.6f })
		{
			PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
			Box(World, Vector3(2, HeightOfStep / 2, 0), Vector3(1, HeightOfStep / 2, 2));
			RobotControllerSettings S; S.Shape = Shape;
			SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
			C->SetMoveVelocity(Vector3(2, 0, 0));
			bool Stepped = false;
			for (int I = 0; I < 70; ++I) { World.Simulate(Dt); Stepped |= C->GetLastMove().Stepped; }
			printf("Character step shape=%d height=%.2f x=%.5f y=%.5f stepped=%d\n", (int)Shape, HeightOfStep, C->GetPosition().x, C->GetPosition().y, Stepped);
			if (HeightOfStep < S.StepHeight)
			{
				EXPECT(Stepped);
				EXPECT(C->GetPosition().x > 2);
				EXPECT_NEAR(C->GetPosition().y, Height(S) + HeightOfStep + S.SkinWidth, 0.03f);
			}
			else
			{
				EXPECT(C->GetPosition().x < 1);
				EXPECT(!Stepped);
			}
		}
	}

	void TestPlatforms(RobotShape Shape, SceneQueryType Type)
	{
		PhysicsWorldParam P; P.sceneQueryType = Type; PhysicsWorld World(P);
		RigidBody* Platform = Box(World, Vector3(0, -0.2f, 0), Vector3(3, 0.2f, 3));
		RobotControllerSettings S; S.Shape = Shape;
		SweepRobotController* C = World.CreateRobotController(S, Vector3(1, Height(S) + S.SkinWidth, 0));
		Step(World, 2); EXPECT(C->IsGrounded());
		Platform->X += Vector3(0.3f, 0.2f, 0.4f); World.Simulate(Dt);
		EXPECT_NEAR(C->GetPosition().x, 1.3f, 0.01f); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth + 0.2f, 0.01f);
		EXPECT_NEAR(C->GetPosition().z, 0.4f, 0.01f);
		Platform->Q.FromRotateY(0.5f); World.Simulate(Dt);
		const Vector3 Expected = Platform->X + Platform->Q * Vector3(1, Height(S) + S.SkinWidth + 0.2f, 0);
		printf("platform rotation actual=(%.6f,%.6f) expected=(%.6f,%.6f) carry=(%.6f,%.6f) ground=%d\n",C->GetPosition().x,C->GetPosition().z,Expected.x,Expected.z,C->GetLastMove().PlatformDisplacement.x,C->GetLastMove().PlatformDisplacement.z,C->IsGrounded());
		EXPECT_NEAR(C->GetPosition().x, Expected.x, 0.01f); EXPECT_NEAR(C->GetPosition().z, Expected.z, 0.01f);
		EXPECT(C->IsGrounded());
		EXPECT(World.RemoveRigidBody(Platform)); Platform->ReleaseGeometries(); delete Platform;
		// Reusing a world slot must not turn the old ground handle into a new platform.
		Box(World, Vector3(100, 20, 0), Vector3(3, 0.2f, 3));
		Step(World, 5); EXPECT(!C->IsGrounded());
		EXPECT_NEAR(C->GetPosition().x, Expected.x, 0.01f);
		EXPECT(World.RemoveRobotController(C)); EXPECT(World.GetRobotControllerCount() == 0);
		World.CreateRobotController(S, Vector3::Zero()); World.Reset(); EXPECT(World.GetRobotControllerCount() == 0);
	}

	void TestPlatformObstructionAndDrop(RobotShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		RigidBody* Platform = Box(World, Vector3(0, -0.2f, 0), Vector3(3, 0.2f, 3));
		Box(World, Vector3(1.5f, 2, 0), Vector3(0.1f, 2, 3));
		RobotControllerSettings S; S.Shape = Shape;
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(World, 2); Platform->X.x += 2; World.Simulate(Dt);
		EXPECT(C->GetLastMove().HitSide); EXPECT(C->GetPosition().x < 1.05f);
		EXPECT(C->IsGrounded()); EXPECT(C->GetLastMove().PlatformDisplacement.x < 1.05f);
		for (float Drop : { 0.15f, 0.6f })
		{
			PhysicsWorld Steps{ PhysicsWorldParam() }; Floor(Steps);
			Box(Steps, Vector3(0, Drop / 2, 0), Vector3(1, Drop / 2, 2));
			C = Steps.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth + Drop, 0));
			Step(Steps, 2); EXPECT(C->IsGrounded());
			C->Move(Vector3(2, 0, 0), Dt);
			if (Drop < S.GroundSnapDistance)
			{
				EXPECT(C->IsGrounded());
				EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.01f);
			}
			else
			{
				EXPECT(!C->IsGrounded());
				EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth + Drop, 0.01f);
			}
		}
	}

	void TestSlopes(RobotShape Shape)
	{
		for (float Angle : { 0.3f, 1.0f })
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			const Vector3 Normal(-std::sin(Angle), std::cos(Angle), 0);
			World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Normal), RigidBodyParam());
			RobotControllerSettings S; S.Shape = Shape; S.StepHeight = 0;
			SweepRobotController* C = World.CreateRobotController(S, Vector3(0, 2, 0)); Step(World, 60);
			const Vector3 Start = C->GetPosition(); C->SetMoveVelocity(Vector3(2, 0, 0)); Step(World, 60);
			printf("Character slope shape=%d angle=%.2f from=(%.4f,%.4f) to=(%.4f,%.4f) grounded=%d\n", (int)Shape, Angle, Start.x, Start.y, C->GetPosition().x, C->GetPosition().y, C->IsGrounded());
			if (Angle < 0.7f)
			{
				EXPECT(C->IsGrounded());
				EXPECT(C->GetPosition().x > Start.x + 1);
				EXPECT(C->GetPosition().y > Start.y + 0.3f);
			}
			else
			{
				EXPECT(!C->IsGrounded());
				EXPECT(C->GetPosition().y <= Start.y + 0.03f);
			}
		}
	}

	void TestShortSweeps(RobotShape Shape)
	{
		for (SceneQueryType Type : { SceneQueryType::StaticAABB, SceneQueryType::DynamicAABB })
		{
			PhysicsWorldParam P; P.sceneQueryType = Type;
			PhysicsWorld World(P); Floor(World);
			Box(World, Vector3(8, 4, 8), Vector3(2));
			RobotControllerSettings S; S.Shape = Shape;
			SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
			SweepOption O; O.MaxDist = S.SkinWidth * 2;
			O.PreFilter = [&](const Geometry* G) { return G != C->GetGeometry(); };
			SweepResult H;
			// The cast begins inside the tree root. Its broad-phase entry is zero,
			// not the distance at which it exits that enclosing node.
			EXPECT(World.GetGeometryQuery()->SweepGeometry(C->GetGeometry(), -Vector3::UnitY(), O, &H));
			EXPECT_NEAR(H.hitTimeMin, S.SkinWidth, 0.001f);
			EXPECT(H.hitNormal.y > 0.99f); EXPECT_NEAR(H.hitPosition.y, 0, 0.001f);
			Step(World, 3); EXPECT(C->IsGrounded());
		}
	}

	Geometry* Surface(int Kind)
	{
		if (Kind == 0)
		{
			Geometry* G = GeometryFactory::CreateTriangleMesh();
			TriangleMesh* M = G->GetShapeObj<TriangleMesh>();
			for (Vector3 P : { Vector3(-4, 0, -4), Vector3(-4, 0, 4), Vector3(4, 0, 4), Vector3(4, 0, -4) }) M->AddVertex(P);
			M->AddTriangle(0, 1, 2); M->AddTriangle(0, 2, 3);
			M->Compact(); M->BuildBVH(); G->UpdateVolumeProperties(); return G;
		}
		if (Kind == 1)
		{
			Geometry* G = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4), Vector3(4, 1, 4)), 8, 8);
			HeightField3* H = G->GetShapeObj<HeightField3>(); H->HeightScale = 0.001f; H->AllocMemory();
			for (int Z = 0; Z < 8; ++Z) for (int X = 0; X < 8; ++X) EXPECT(H->SetHeight(X, Z, 0));
			G->UpdateVolumeProperties(); return G;
		}
		constexpr int Cells = 16;
		SignedDistanceField3 Grid;
		EXPECT(Grid.Reset(Box3(Vector3(-4), Vector3(4)), Cells, Cells, Cells, 8.0f / Cells));
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
		{
			const Vector3 P = Grid.GridPointToWorld(X, Y, Z);
			const Vector3 D(std::abs(P.x) - 3, std::abs(P.y + 1) - 1, std::abs(P.z) - 3);
			Grid.At(X, Y, Z) = D.Max(Vector3::Zero()).Length() + std::min(0.0f, std::max(D.x, std::max(D.y, D.z)));
		}
		RuntimeDistanceField Field; EXPECT(Field.Build(Grid));
		DistanceFieldShape S; EXPECT(S.Build(Field)); return GeometryFactory::CreateDistanceField(S);
	}

	void TestSurfaces(RobotShape Shape)
	{
		for (int Kind = 0; Kind < 3; ++Kind)
		{
			PhysicsWorld World{ PhysicsWorldParam() }; World.CreateRigidBody(Surface(Kind), RigidBodyParam());
			RobotControllerSettings S; S.Shape = Shape;
			SweepRobotController* C = World.CreateRobotController(S, Vector3(-1, 2, -0.5f)); Step(World, 45);
			EXPECT(C->IsGrounded());
			C->SetMoveVelocity(Vector3(2, 0, 1)); Step(World, 60);
			printf("Character surface shape=%d kind=%d position=(%.5f,%.5f,%.5f)\n", (int)Shape, Kind, C->GetPosition().x, C->GetPosition().y, C->GetPosition().z);
			EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().x, 1, 0.03f);
			EXPECT_NEAR(C->GetPosition().z, 0.5f, 0.03f); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.01f);
			C->SetPosition(Vector3(0, Height(S) - 0.1f, 0));
			const RobotMoveResult Recovery = C->Move(Vector3::Zero(), Dt);
			EXPECT(!Recovery.RecoveryIncomplete); EXPECT(Recovery.RecoveryDistance > 0.09f);
			EXPECT(C->IsGrounded());
		}
	}

	void TestFilters(RobotShape Shape)
	{
		PhysicsWorldParam P; P.collisionFilter.reset(CollisionFilter::CreateLayerMask());
		PhysicsWorld World(P); Floor(World);
		Box(World, Vector3(1, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetFilterData(CollisionData::ForLayer(2));
		Box(World, Vector3(2, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetTriggerEnabled(true);
		Box(World, Vector3(3, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetQueryEnabled(false);
		Box(World, Vector3(4, 2, 0), Vector3(0.2f, 2, 2));
		RobotControllerSettings S; S.Shape = Shape; S.FilterData = CollisionData::ForLayer(1, 1u);
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		C->Move(Vector3(6, 0, 0), Dt);
		EXPECT(C->GetPosition().x > 3.35f && C->GetPosition().x < 3.45f); EXPECT(C->IsGrounded());
		const float X = C->GetPosition().x;
		C->SetFilterData(CollisionData::ForLayer(1, 0)); C->Move(Vector3(2, 0, 0), Dt);
		EXPECT_NEAR(C->GetPosition().x, X + 2, 0.001f); EXPECT(!C->IsGrounded());
	}

	void TestCeilingAndCorners(RobotShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
		Box(World, Vector3(0, 2.5f, 0), Vector3(5, 0.2f, 5));
		Box(World, Vector3(2, 1, 0), Vector3(0.2f, 1, 5));
		Box(World, Vector3(0, 1, 2), Vector3(5, 1, 0.2f));
		RobotControllerSettings S; S.Shape = Shape;
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(World, 2); EXPECT(C->Jump(10));
		bool Ceiling = false;
		for (int I = 0; I < 20; ++I)
		{
			World.Simulate(Dt); Ceiling |= C->GetLastMove().HitAbove;
			EXPECT(C->GetPosition().y + Height(S) <= 2.3f);
		}
		EXPECT(Ceiling); Step(World, 45); EXPECT(C->IsGrounded());
		C->Move(Vector3(10, 0, 10), Dt);
		EXPECT(C->GetPosition().x > 1.2f && C->GetPosition().x < 1.45f);
		EXPECT(C->GetPosition().z > 1.2f && C->GetPosition().z < 1.45f);

		PhysicsWorld Low{ PhysicsWorldParam() }; Floor(Low);
		Box(Low, Vector3(1.5f, 0.1f, 0), Vector3(0.5f, 0.1f, 2));
		Box(Low, Vector3(0, 2.2f, 0), Vector3(5, 0.2f, 5));
		C = Low.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(Low, 2); const RobotMoveResult Move = C->Move(Vector3(2, 0, 0), Dt);
		EXPECT(!Move.Stepped); EXPECT(C->GetPosition().x < 1);
	}

	void TestManualAndSubsteps(RobotShape Shape)
	{
		for (ResolutionPhaseSolver Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
			ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (int Substeps : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			PhysicsWorld World(P); Floor(World);
			RobotControllerSettings S; S.Shape = Shape; S.AutomaticUpdate = false;
			SweepRobotController* C = World.CreateRobotController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
			C->Move(Vector3(1, 0, 0), Dt); const Vector3 Position = C->GetPosition();
			C->SetMoveVelocity(Vector3(10, 0, 0)); Step(World, 10);
			EXPECT((C->GetPosition() - Position).Length() < 0.001f);
			S.AutomaticUpdate = true;
			SweepRobotController* Other = World.CreateRobotController(S, Vector3(-2, Height(S) + S.SkinWidth, 2));
			Other->SetMoveVelocity(Vector3(2, 0, 0)); Step(World, 60);
			EXPECT_NEAR(Other->GetPosition().x, 0, 0.01f); EXPECT(Other->IsGrounded());
			EXPECT(World.RemoveRobotController(Other)); EXPECT(!World.RemoveRobotController(nullptr));
		}
		PhysicsWorld World{ PhysicsWorldParam() };
		World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitZ()), RigidBodyParam());
		RobotControllerSettings S; S.Shape = Shape; S.Up = Vector3::UnitZ();
		SweepRobotController* C = World.CreateRobotController(S, Vector3(0, 0, 2)); Step(World, 60);
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().z, Height(S) + S.SkinWidth, 0.01f);
		bool Rejected = false; S.Up = Vector3::Zero();
		try { World.CreateRobotController(S, Vector3::Zero()); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT(World.GetRobotControllerCount() == 1);
		Rejected = false;
		try { C->Move(Vector3(std::numeric_limits<float>::quiet_NaN()), Dt); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT(C->IsGrounded());
		Rejected = false;
		try { C->Move(Vector3(std::numeric_limits<float>::max()), Dt); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT(C->IsGrounded());
	}

	void TestDemo()
	{
		SceneWorld Scene; std::string Error; const std::string Path = TestContentsPath() + "Scenes/06_constraints.json";
		EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(!Scene.GetDemoCharacter());
		const size_t OriginalObjects = Scene.GetObjects().size();
		for (bool BoxShape : { false, true })
		{
			Scene.SetCharacterDemo(true, BoxShape); EXPECT(Scene.LoadFromFile(Path, &Error));
			SweepRobotController* C = Scene.GetDemoCharacter();
			EXPECT(C != nullptr);
			if (!C)
				continue;
			EXPECT(C->GetSettings().Shape == (BoxShape ? RobotShape::Box : RobotShape::Capsule));
			EXPECT(Scene.GetObjects().size() > OriginalObjects);
			for (int I = 0; I < 45; ++I) Scene.Step(Dt);
			EXPECT(C->IsGrounded()); const Vector3 Start = C->GetPosition();
			C->SetMoveVelocity(Vector3(3, 0, 0));
			for (int I = 0; I < 80; ++I) Scene.Step(Dt);
			EXPECT(C->GetPosition().x > Start.x + 3.5f);
			EXPECT(C->GetPosition().y > Start.y + 0.5f);
			bool Rendered = false;
			for (const SceneObjectInstance& Object : Scene.GetObjects()) Rendered |= Object.GeometryPtr == C->GetGeometry() && Object.RenderEnabled;
			EXPECT(Rendered);
		}
		Scene.SetCharacterDemo(false, false); EXPECT(Scene.LoadFromFile(Path, &Error));
		EXPECT(!Scene.GetDemoCharacter()); EXPECT(Scene.GetObjects().size() == OriginalObjects);
	}
}

void TestRobotController()
{
	for (Riemann::RobotShape Shape : { Riemann::RobotShape::Capsule, Riemann::RobotShape::Box })
	{
		TestGroundAndSlide(Shape); TestBallisticJump(Shape); TestSteps(Shape); TestSlopes(Shape);
		for (Riemann::SceneQueryType Type : { Riemann::SceneQueryType::StaticAABB, Riemann::SceneQueryType::DynamicAABB }) TestPlatforms(Shape, Type);
		TestPlatformObstructionAndDrop(Shape);
		TestShortSweeps(Shape); TestSurfaces(Shape); TestFilters(Shape);
		TestCeilingAndCorners(Shape); TestManualAndSubsteps(Shape);
	}
	TestDemo();
}

void TestRobotBodyContact()
{
	using namespace Riemann;
	PhysicsWorld World{PhysicsWorldParam()};
	Box(World, Vector3(0, -0.15f, 0), Vector3(2, 0.15f, 2));
	ArticulatedRobotSettings Settings;
	RagdollLinkSettings Root;
	Root.Joint = RagdollReducedJointType::Floating;
	Settings.Links.push_back(Root);
	Settings.Pose.resize(1);
	Settings.Pose[0].Translation.y = 1;
	ArticulatedRobotController* Robot = World.CreateRobotController(Settings);
	Robot->AddLinkCollider(0, Transform(), std::unique_ptr<Geometry>(GeometryFactory::CreateSphere(Vector3::Zero(), 0.25f)));
	Step(World, 90);
	EXPECT_NEAR(Robot->GetPosition().y, 0.25f, 0.003f);
	EXPECT(World.RemoveRobotController(Robot));
	EXPECT(World.GetRobotControllerCount() == 0);
}

void TestRobotSpider()
{
	using namespace Riemann;
	TestRobotBodyContact();
	{
		auto Control = std::make_shared<SpiderRobotControl>();
		ArticulatedRobotSettings Settings = CreateSpiderRobotSettings(Control);
		EXPECT(Settings.Links.size() == 25 && Settings.Feet.size() == 8 && Settings.Pose.size() == 25);
		FeatherstoneSolver Tree(Settings.Links);
		Tree.SetConfiguration(Settings.Pose);
		std::vector<double> Torque(Tree.GetDofCount(), 0);
		for (double& Offset : Control->TargetOffset) Offset = 0.2;
		Settings.Actuate(Tree, Vector3::Zero(), 0, Torque);
		for (size_t M = 0; M < SpiderMotorCount; ++M)
			EXPECT(std::abs(Torque[Tree.GetLinkDofOffset(M + 1)]) <= Control->TorqueLimit[M]);
		Control->TargetOffset.fill(0);
		for (size_t Leg = 0; Leg < SpiderLegCount; ++Leg)
		{
			const size_t M = Leg * 3;
			const RobotFootContact& Foot = Settings.Feet[Leg];
			const Transform Base = Tree.GetLinkTransform(0);
			const Vector3 Home = Base.WorldToLocal(Tree.GetLinkTransform(Foot.Link) * Foot.Center);
			const Vector3 RaisedJoint = Base.WorldToLocal(Tree.GetLinkTransform(Foot.Link).pos);
			EXPECT(RaisedJoint.z > 0.18f);
			EXPECT_NEAR((Tree.GetLinkTransform(Foot.Link) * Foot.Center).y, Foot.Radius, 0.02f);
			for (size_t Joint = 0; Joint < 3; ++Joint)
			{
				std::vector<RagdollConfiguration> Pose = Settings.Pose;
				Pose[1 + M + Joint].Position += 0.05;
				Tree.SetConfiguration(Pose);
				const Vector3 Moved = Base.WorldToLocal(Tree.GetLinkTransform(Foot.Link) * Foot.Center);
				if (Joint == 0)
				{
					const float Angle = float(Leg * 6.283185307179586 / SpiderLegCount);
					EXPECT((Moved - Home).Dot(Vector3(-std::sin(Angle), std::cos(Angle), 0)) > 0.015f);
				}
				else EXPECT(Moved.z < Home.z - 0.005f);
				Tree.SetConfiguration(Settings.Pose);
			}
		}
		const std::string SpiderScene = R"({"walkingRobot":true,"robotModel":"Spider","objects":[{"id":"floor","type":"box","body":"static","position":[0,-0.1,0],"halfExtent":[12,0.1,12]}]})";
		SceneWorld Spider;
		std::string Error;
		EXPECT(Spider.LoadFromText(SpiderScene, TestContentsPath() + "Scenes/07_robot_walking.json", &Error));
		if (Spider.GetWalkingRobot())
		{
			int Grounded = 0;
			float MinHeight = 1;
			for (int Frame = 0; Frame < 300; ++Frame)
			{
				Spider.Step(1.0f / 60);
				const ArticulatedRobotController* Robot = Spider.GetWalkingRobot();
				Grounded += Robot->IsGrounded();
				MinHeight = std::min(MinHeight, Robot->GetPosition().y);
			}
			EXPECT(Grounded > 250 && MinHeight > 0.35f);
			int ContactFeet = 0;
			for (const RobotFootContact& Foot : Settings.Feet)
			{
				const Vector3 Center = Spider.GetWalkingRobot()->GetDynamics().GetLinkTransform(Foot.Link) * Foot.Center;
				SweepOption Option; Option.MaxDist = 0.07f;
				SweepResult Hit;
				ContactFeet += Spider.GetSimulation()->GetGeometryQuery()->SphereCastQuery(
					Center + Vector3::UnitY() * 0.06f, Foot.Radius, -Vector3::UnitY(), Option, &Hit)
					&& Hit.hitTimeMin <= 0.065f;
			}
			EXPECT(ContactFeet == 8);
		}
	}
	}

void TestRobotSpiderPolicy()
{
	using namespace Riemann;
	const auto CheckLandmarks = [](SceneWorld& World, bool Course)
	{
		int Obstacles = 0, Markings = 0;
		for (const SceneObjectInstance& Object : World.GetObjects())
		{
			Geometry* Shape = Object.GeometryPtr;
			if (Object.Id.find("robot_marking_") == 0)
			{
				++Markings;
				EXPECT(!Shape->IsSimulationEnabled() && !Shape->IsQueryEnabled());
			}
			if (Object.Id.find("robot_obstacle_") != 0) continue;
			++Obstacles;
			EXPECT(Shape->IsSimulationEnabled() && Shape->IsQueryEnabled());
			RigidBody* Body = Shape->GetParent<RigidBody>();
			EXPECT(Body && Body->mRigidType == RigidType::Static);
			if (!Body) continue;
			const Box3 Bounds = Body->GetGeometryBounds(Shape);
			EXPECT(Bounds.Min.z >= 3 || Bounds.Max.z <= -3);
			const Vector3 P = Body->GetGeometryTransform(Shape).pos;
			RayCastOption Option; Option.MaxDist = 12;
			RayCastResult Hit;
			EXPECT(World.GetSimulation()->GetGeometryQuery()->RayCastQuery(
				Vector3(P.x, 10, P.z), -Vector3::UnitY(), Option, &Hit));
			EXPECT(Hit.hitGeom == Shape);
			// Ignore the obstacle itself to check that its base rests on the terrain.
			Option.SourceBody = Body;
			EXPECT(World.GetSimulation()->GetGeometryQuery()->RayCastQuery(
				Vector3(P.x, 10, P.z), -Vector3::UnitY(), Option, &Hit));
			EXPECT_NEAR(Bounds.Min.y, 10 - Hit.hitTimeMin, 0.002f);
		}
		EXPECT(Obstacles >= 30);
		EXPECT(Markings >= (Course ? 40 : 80));
	};
	SceneWorld Scene;
	std::string Error;
	const bool Loaded = Scene.LoadFromFile(TestContentsPath() + "Scenes/07_robot_walking.json", &Error);
	EXPECT(Loaded);
	if (!Loaded) { printf("%s\n", Error.c_str()); return; }
	CheckLandmarks(Scene, true);
	const auto TerrainHeight = [&Scene](float X, float Z)
	{
		float Height = -100;
		for (const auto& Object : Scene.GetObjects())
		{
			if (Object.Id.find("course_") != 0) continue;
			const Geometry* Tile = Object.GeometryPtr;
			RayCastOption Option; Option.MaxDist = 12;
			RayCastResult Hit;
			if (Tile->RayCast(Tile->GetParent<RigidBody>()->GetGeometryTransform(Tile),
				Vector3(X, 10, Z), -Vector3::UnitY(), &Option, &Hit))
				Height = std::max(Height, 10 - Hit.hitTime);
		}
		return Height;
	};
	for (float X : {0.0f, 5.0f, 20.0f, 35.0f})
		for (float Z : {-3.0f, 0.0f, 3.0f})
		{
			RayCastOption Option; Option.MaxDist = 12;
			RayCastResult Hit;
			EXPECT(Scene.GetSimulation()->GetGeometryQuery()->RayCastQuery(
				Vector3(X, 10, Z), -Vector3::UnitY(), Option, &Hit));
			EXPECT_NEAR(10 - Hit.hitTimeMin, 0, 0.003f);
		}
	EXPECT(TerrainHeight(5, 5) > TerrainHeight(10, 5) + 0.25f);
	EXPECT(TerrainHeight(15, -5) > TerrainHeight(20, -5) + 0.25f);
	for (float Z : {-5.0f, 5.0f})
	{
		RayCastOption Option; Option.MaxDist = 12;
		RayCastResult Hit;
		EXPECT(Scene.GetSimulation()->GetGeometryQuery()->RayCastQuery(
			Vector3(15, 10, Z), -Vector3::UnitY(), Option, &Hit));
		EXPECT_NEAR(10 - Hit.hitTimeMin, TerrainHeight(15, Z), 0.005f);
	}
	Scene.SetRobotAutoWalk(true);
	const ArticulatedRobotController* Robot = Scene.GetWalkingRobot();
	EXPECT(Robot != nullptr);
	if (!Robot) return;
	float MinHeight = Robot->GetPosition().y, MinUp = 1, X25 = 0;
	int Grounded = 0;
	for (int Frame = 0; Frame < 1800; ++Frame)
	{
		Scene.Step(1.0f / 60);
		MinHeight = std::min(MinHeight, Robot->GetPosition().y);
		MinUp = std::min(MinUp, (Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitZ()).y);
		Grounded += Robot->IsGrounded();
		if (Frame == 1499) X25 = Robot->GetPosition().x;
		if (Frame % 300 == 299)
			printf("Spider level course t=%.1f x=%.3f z=%.3f facing=%.3f\n", Robot->GetTime(),
				Robot->GetPosition().x, Robot->GetPosition().z,
				(Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitX()).x);
	}
	printf("Spider policy t=%.3f x=%.3f min-height=%.3f min-up=%.3f supported=%d\n",
		Robot->GetTime(), Robot->GetPosition().x, MinHeight, MinUp, Grounded);
	EXPECT(Robot->GetPosition().x > 24.0f);
	EXPECT(Robot->GetPosition().x - X25 >= 2.0f);
	EXPECT(std::abs(Robot->GetPosition().z) <= 2.0f);
	EXPECT((Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitX()).x >= 0.7f);
	EXPECT(MinHeight > 0.35f && MinUp > 0.9f && Grounded > 1400);
	for (int Surface = 0; Surface < 2; ++Surface)
	{
		SceneWorld Comparison;
		const std::string Text = Surface == 0
			? R"({"walkingRobot":true,"robotModel":"SpiderPolicy","objects":[{"id":"floor","type":"plane","body":"static","position":[0,0,0],"normal":[0,1,0]}]})"
			: R"({"walkingRobot":true,"robotModel":"SpiderPolicy","objects":[{"id":"floor","type":"heightfield","body":"static","position":[14,0,0],"halfExtent":[28,0,8],"rows":129,"columns":225,"heightScale":0.002,"baseHeight":0}]})";
		const bool ComparisonLoaded = Comparison.LoadFromText(Text, TestContentsPath() + "Scenes/07_robot_walking.json", &Error);
		EXPECT(ComparisonLoaded);
		if (!ComparisonLoaded)
		{
			printf("Spider comparison load: %s\n", Error.c_str());
			continue;
		}
		Comparison.SetRobotAutoWalk(true);
		const auto* ComparisonRobot = Comparison.GetWalkingRobot();
		float PreviousX = 0;
		for (int Frame = 0; Frame < 1800; ++Frame)
		{
			Comparison.Step(Dt);
			if (Frame == 1499) PreviousX = ComparisonRobot->GetPosition().x;
		}
		printf("Spider comparison %s x=%.3f last5=%.3f z=%.3f\n",
			Surface == 0 ? "plane" : "heightfield", ComparisonRobot->GetPosition().x,
			ComparisonRobot->GetPosition().x - PreviousX, ComparisonRobot->GetPosition().z);
		EXPECT(ComparisonRobot->GetPosition().x > (Surface == 0 ? 24.0f : 20.0f));
		EXPECT(ComparisonRobot->GetPosition().x - PreviousX >= 2.0f);
	}
}

void TestRobotSpiderControls()
{
	using namespace Riemann;
	SceneWorld Scene;
	std::string Error;
	const std::string Path = TestContentsPath() + "Scenes/07_robot_walking.json";
	const std::string Flat = R"({"walkingRobot":true,"robotModel":"SpiderPolicy","robotLandmarks":true,"objects":[{"id":"floor","type":"plane","body":"static","position":[0,0,0],"normal":[0,1,0]}]})";
	Scene.SetRobotWalkMode(RobotWalkMode::Sweep);
	const bool Loaded = Scene.LoadFromText(Flat, Path, &Error);
	EXPECT(Loaded);
	if (!Loaded) { printf("%s\n", Error.c_str()); return; }
	EXPECT(Scene.IsSpiderWalkingDemo());
	EXPECT(Scene.GetWalkingRobot() == nullptr);
	EXPECT(Scene.GetSimulation()->GetRobotControllerCount() == 1);
	auto* Walker = dynamic_cast<SweepRobotController*>(Scene.GetRobotDemoController());
	EXPECT(Walker);
	if (!Walker) return;
	Scene.SetRobotAutoWalk(false);
	const auto Advance = [&Scene](int Frames) { for (int I = 0; I < Frames; ++I) Scene.Step(Dt); };
	Advance(60);
	EXPECT(Walker->IsGrounded());
	const Vector3 Start = Walker->GetPosition();
	Scene.SetRobotDriveInput(1, 0, 0); Advance(180);
	EXPECT_NEAR(Walker->GetPosition().x - Start.x, 2.4f, 0.03f);
	Scene.SetRobotDriveInput(-1, 0, 0); Advance(60);
	EXPECT_NEAR(Walker->GetPosition().x - Start.x, 1.6f, 0.03f);
	Scene.SetRobotDriveInput(0, 0, 1); Advance(60);
	EXPECT_NEAR(Walker->GetPosition().z - Start.z, 0.8f, 0.03f);
	Scene.SetRobotDriveInput(0, 0, -1); Advance(60);
	EXPECT_NEAR(Walker->GetPosition().z, Start.z, 0.03f);
	Scene.SetRobotDriveInput(0, 0, 0);
	const Vector3 Stopped = Walker->GetPosition(); Advance(60);
	EXPECT((Walker->GetPosition() - Stopped).Length() < 0.002f);
	Scene.SetRobotDriveInput(0, 1, 0); Advance(60);
	EXPECT_NEAR(Scene.GetRobotDriveHeading(), 1, 0.001f);
	EXPECT((Walker->GetPosition() - Stopped).Length() < 0.002f);
	Scene.SetRobotDriveInput(1, 0, 0); Advance(60);
	EXPECT_NEAR(Walker->GetPosition().x - Stopped.x, 0.8f * std::cos(1.0f), 0.03f);
	EXPECT_NEAR(Walker->GetPosition().z - Stopped.z, -0.8f * std::sin(1.0f), 0.03f);
	Scene.SetRobotDriveInput(0, -1, 0); Advance(60);
	EXPECT_NEAR(Scene.GetRobotDriveHeading(), 0, 0.001f);
	Scene.SetRobotDriveInput(1, 0, 1); Advance(1);
	EXPECT_NEAR(Scene.GetRobotDriveCommand().Length(), 0.8f, 1e-6);

	// Both side exercise areas leave the central path unobstructed.
	int Stairs = 0, Hills = 0;
	for (const auto& Object : Scene.GetObjects())
		if (Object.Id.find("robot_stair_") == 0 || Object.Id == "robot_terrain_hills")
		{
			const auto* Body = Object.GeometryPtr->GetParent<RigidBody>();
			const Box3 Bounds = Body->GetGeometryBounds(Object.GeometryPtr);
			EXPECT(Bounds.Min.z >= 4.7f || Bounds.Max.z <= -4.7f);
			EXPECT(Object.GeometryPtr->IsSimulationEnabled());
			Stairs += Object.Id.find("robot_stair_") == 0;
			Hills += Object.Id == "robot_terrain_hills";
		}
	EXPECT(Stairs == 16 && Hills == 1);
	Walker->SetPosition(Vector3(2, 0.51f, 6));
	Scene.SetRobotDriveInput(1, 0, 0);
	float Peak = 0;
	for (int I = 0; I < 960; ++I) { Scene.Step(Dt); Peak = std::max(Peak, Walker->GetPosition().y); }
	printf("Spider sweep stairs x=%.3f height=%.3f peak=%.3f\n", Walker->GetPosition().x, Walker->GetPosition().y, Peak);
	EXPECT(Walker->GetPosition().x > 13.5f);
	EXPECT(Peak > 1.65f && Walker->GetPosition().y < 0.6f);
	Walker->SetPosition(Vector3(2, 0.51f, -6.1f)); Peak = 0;
	for (int I = 0; I < 900; ++I) { Scene.Step(Dt); Peak = std::max(Peak, Walker->GetPosition().y); }
	printf("Spider sweep hills x=%.3f peak=%.3f\n", Walker->GetPosition().x, Peak);
	EXPECT(Walker->GetPosition().x > 13.5f && Peak > 0.95f);
	Walker->SetPosition(Vector3(16, 0.51f, -6)); Advance(300);
	printf("Spider sweep wall x=%.3f\n", Walker->GetPosition().x);
	EXPECT(Walker->GetPosition().x > 17 && Walker->GetPosition().x < 17.5f);
	Scene.SetRobotDriveInput(0, 0, 1); Advance(150);
	Scene.SetRobotDriveInput(1, 0, 0); Advance(180);
	EXPECT(Walker->GetPosition().x > 19);

	// Reloading switches implementations without leaving an old controller behind.
	Scene.SetRobotWalkMode(RobotWalkMode::Policy);
	EXPECT(Scene.LoadFromText(Flat, Path, &Error));
	EXPECT(Scene.GetWalkingRobot() != nullptr);
	EXPECT(Scene.GetSimulation()->GetRobotControllerCount() == 1);
	Scene.SetRobotAutoWalk(false);
	Scene.SetRobotDriveInput(0, 0, 0); Advance(120);
	EXPECT(Scene.GetRobotDemoController()->GetPosition().Length() < 0.7f);
	const float IdleX = Scene.GetRobotDemoController()->GetPosition().x;
	Scene.SetRobotDriveInput(1, 0, 0); Advance(180);
	printf("Spider manual policy forward x=%.3f\n", Scene.GetRobotDemoController()->GetPosition().x - IdleX);
	EXPECT(Scene.GetRobotDemoController()->GetPosition().x - IdleX > 0.5f);
	Scene.SetRobotDriveInput(-1, 0, 0); Advance(1);
	EXPECT(Scene.GetRobotDriveCommand().x < -0.79f);
	Scene.SetRobotDriveInput(0, 0, 1); Advance(1);
	EXPECT(Scene.GetRobotDriveCommand().z > 0.79f);
	for (const Vector3 Input : {Vector3(-1, 0, 0), Vector3(0, 0, 1), Vector3(0, 0, -1)})
	{
		EXPECT(Scene.LoadFromText(Flat, Path, &Error));
		Scene.SetRobotAutoWalk(false);
		Scene.SetRobotDriveInput(Input.x, 0, Input.z); Advance(300);
		const Vector3 P = Scene.GetRobotDemoController()->GetPosition();
		printf("Spider manual policy input=(%.0f,%.0f) position=(%.3f,%.3f,%.3f)\n", Input.x, Input.z, P.x, P.y, P.z);
		EXPECT(P.Dot(Input) > 1.0f);
		EXPECT(P.y > 0.3f);
		Scene.SetRobotDriveInput(0, 0, 0); Advance(60);
		const Vector3 Rest = Scene.GetRobotDemoController()->GetPosition(); Advance(60);
		EXPECT((Scene.GetRobotDemoController()->GetPosition() - Rest).Length() < 0.15f);
		EXPECT(Scene.GetRobotDemoController()->GetPosition().y > 0.3f);
	}
	Scene.SetRobotWalkMode(RobotWalkMode::Sweep);
	EXPECT(Scene.LoadFromText(Flat, Path, &Error));
	EXPECT(Scene.GetWalkingRobot() == nullptr && Scene.GetRobotDemoController() != nullptr);
	EXPECT(Scene.GetSimulation()->GetRobotControllerCount() == 1);
	Scene.SetRobotDriveInput(0, 0, 0);
	Advance(60);
	const float BeforeStart = Scene.GetRobotDemoController()->GetPosition().x;
	Scene.SetRobotAutoWalk(true);
	Advance(120);
	const float AfterStart = Scene.GetRobotDemoController()->GetPosition().x;
	EXPECT_NEAR(AfterStart - BeforeStart, 1.6f, 0.03f);
	Scene.SetRobotAutoWalk(false);
	Advance(60);
	EXPECT_NEAR(Scene.GetRobotDemoController()->GetPosition().x, AfterStart, 0.002f);
}

void TestRobotNetworks()
{
	using namespace Maths::LinearAlgebra;
	// A rectangular affine map, with and without bias, independent of the walking policy.
	const std::vector<float> Matrix = {1, -2, 3, 0, 2, -1};
	const std::vector<float> X = {2, -1, 0.5f};
	Linear Affine(3, 2, Matrix, {0.5f, -1});
	EXPECT_NEAR(Affine.Evaluate(X)[0], 6, 1e-6);
	EXPECT_NEAR(Affine.Evaluate(X)[1], -3.5f, 1e-6);
	Linear NoBias(3, 2, Matrix);
	EXPECT_NEAR(NoBias.Evaluate(X)[0], 5.5f, 1e-6);
	NeuralNetwork Classifier(3);
	Classifier.Append(std::make_unique<Linear>(3, 2, Matrix));
	Classifier.Append(std::make_unique<Activation>(2, NeuralLayerType::ReLU));
	Classifier.Append(std::make_unique<Linear>(2, 2, std::vector<float>{0, 0, 0, 0}, std::vector<float>{1000, 1001}));
	Classifier.Append(std::make_unique<Activation>(2, NeuralLayerType::Softmax));
	EXPECT(Classifier.GetOutputSize() == 2);
	EXPECT_NEAR(Classifier.Evaluate(X)[1], 0.73105858f, 1e-6);

	using Type = NeuralLayerType;
	const Type Functions[] = {Type::Identity, Type::ReLU, Type::ELU, Type::Sigmoid, Type::Tanh};
	const float Expected[] = {-1, 0, -0.63212056f, 0.26894142f, -0.76159416f};
	for (size_t I = 0; I < sizeof(Functions) / sizeof(Functions[0]); ++I)
	{
		NeuralNetwork Unary(2, {{Functions[I]}}, {});
		EXPECT_NEAR(Unary.Evaluate({-1, 3})[0], Expected[I], 1e-6);
	}
	LSTM Memory(1, 1, {0, 0, 1, 0}, {0, 0, 0, 0});
	const float Cell = 0.5f * std::tanh(1.0f);
	EXPECT_NEAR(Memory.Evaluate({1})[0], 0.5f * std::tanh(Cell), 1e-6);
	EXPECT_NEAR(Memory.Evaluate({0})[0], 0.5f * std::tanh(Cell / 2), 1e-6);
	Memory.Reset();
	EXPECT_NEAR(Memory.Evaluate({0})[0], 0.0f, 1e-6);
	bool Rejected = false;
	try { Classifier.Append(std::make_unique<Activation>(3, Type::ReLU)); }
	catch (const std::invalid_argument&) { Rejected = true; }
	EXPECT(Rejected);


}

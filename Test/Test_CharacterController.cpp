#include "Test.h"
#include "../Src/Modules/CharacterController/CharacterController.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/DistanceFieldShape.h"
#include "../Viewer/SceneWorld.h"
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
	float Height(const CharacterControllerSettings& S) { return S.Shape == CharacterShape::Capsule ? S.HalfHeight + S.Radius : S.HalfExtent.y; }

	void TestGroundAndSlide(CharacterShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
		CharacterControllerSettings S; S.Shape = Shape;
		auto* C = World.CreateCharacterController(S, Vector3(0, 2, 0));
		Step(World, 90);
		printf("Character shape=%d floor=(%.5f, %.5f, %.5f) grounded=%d\n", (int)Shape, C->GetPosition().x, C->GetPosition().y, C->GetPosition().z, C->IsGrounded());
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.005f);
		Box(World, Vector3(2, 2, 0), Vector3(0.2f, 2, 10));
		const auto Hit = C->Move(Vector3(4, 0, 2), Dt);
		EXPECT(Hit.HitSide); EXPECT(C->GetPosition().x < 1.46f); EXPECT_NEAR(C->GetPosition().z, 2, 0.03f);
		EXPECT(C->IsGrounded());
		EXPECT(C->Jump(4)); EXPECT(!C->Jump(4)); Step(World, 10); EXPECT(C->GetPosition().y > Height(S) + 0.3f);
		Step(World, 90); EXPECT(C->IsGrounded());
		C->SetPosition(Vector3(0, Height(S) - 0.15f, 0));
		const auto Recovered = C->Move(Vector3::Zero(), Dt);
		EXPECT(Recovered.RecoveryDistance > 0.1f); EXPECT(!Recovered.RecoveryIncomplete);
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.005f);
		C->SetPosition(Vector3(0, -2, 0));
		const auto Deep = C->Move(Vector3(1, 0, 0), Dt);
		EXPECT(Deep.RecoveryIncomplete); EXPECT(Deep.RecoveryDistance <= S.MaximumRecoveryDistance + 0.001f);
		EXPECT_NEAR(C->GetPosition().x, 0, 0.001f);
	}

	void TestSteps(CharacterShape Shape)
	{
		for (float HeightOfStep : { 0.2f, 0.6f })
		{
			PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
			Box(World, Vector3(2, HeightOfStep / 2, 0), Vector3(1, HeightOfStep / 2, 2));
			CharacterControllerSettings S; S.Shape = Shape;
			auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
			C->SetMoveVelocity(Vector3(2, 0, 0));
			bool Stepped = false;
			for (int I = 0; I < 70; ++I) { World.Simulate(Dt); Stepped |= C->GetLastMove().Stepped; }
			printf("Character step shape=%d height=%.2f x=%.5f y=%.5f stepped=%d\n", (int)Shape, HeightOfStep, C->GetPosition().x, C->GetPosition().y, Stepped);
			if (HeightOfStep < S.StepHeight)
			{ EXPECT(Stepped); EXPECT(C->GetPosition().x > 2); EXPECT_NEAR(C->GetPosition().y, Height(S) + HeightOfStep + S.SkinWidth, 0.03f); }
			else { EXPECT(C->GetPosition().x < 1); EXPECT(!Stepped); }
		}
	}

	void TestPlatforms(CharacterShape Shape, SceneQueryType Type)
	{
		PhysicsWorldParam P; P.sceneQueryType = Type; PhysicsWorld World(P);
		auto* Platform = Box(World, Vector3(0, -0.2f, 0), Vector3(3, 0.2f, 3));
		CharacterControllerSettings S; S.Shape = Shape;
		auto* C = World.CreateCharacterController(S, Vector3(1, Height(S) + S.SkinWidth, 0));
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
		EXPECT(World.RemoveCharacterController(C)); EXPECT(World.GetCharacterControllerCount() == 0);
		World.CreateCharacterController(S, Vector3::Zero()); World.Reset(); EXPECT(World.GetCharacterControllerCount() == 0);
	}

	void TestPlatformObstructionAndDrop(CharacterShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Platform = Box(World, Vector3(0, -0.2f, 0), Vector3(3, 0.2f, 3));
		Box(World, Vector3(1.5f, 2, 0), Vector3(0.1f, 2, 3));
		CharacterControllerSettings S; S.Shape = Shape;
		auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(World, 2); Platform->X.x += 2; World.Simulate(Dt);
		EXPECT(C->GetLastMove().HitSide); EXPECT(C->GetPosition().x < 1.05f);
		EXPECT(C->IsGrounded()); EXPECT(C->GetLastMove().PlatformDisplacement.x < 1.05f);
		for (float Drop : { 0.15f, 0.6f })
		{
			PhysicsWorld Steps{ PhysicsWorldParam() }; Floor(Steps);
			Box(Steps, Vector3(0, Drop / 2, 0), Vector3(1, Drop / 2, 2));
			C = Steps.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth + Drop, 0));
			Step(Steps, 2); EXPECT(C->IsGrounded());
			C->Move(Vector3(2, 0, 0), Dt);
			if (Drop < S.GroundSnapDistance)
			{ EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.01f); }
			else
			{ EXPECT(!C->IsGrounded()); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth + Drop, 0.01f); }
		}
	}

	void TestSlopes(CharacterShape Shape)
	{
		for (float Angle : { 0.3f, 1.0f })
		{
			PhysicsWorld World{ PhysicsWorldParam() };
			const Vector3 Normal(-std::sin(Angle), std::cos(Angle), 0);
			World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Normal), RigidBodyParam());
			CharacterControllerSettings S; S.Shape = Shape; S.StepHeight = 0;
			auto* C = World.CreateCharacterController(S, Vector3(0, 2, 0)); Step(World, 60);
			const Vector3 Start = C->GetPosition(); C->SetMoveVelocity(Vector3(2, 0, 0)); Step(World, 60);
			printf("Character slope shape=%d angle=%.2f from=(%.4f,%.4f) to=(%.4f,%.4f) grounded=%d\n", (int)Shape, Angle, Start.x, Start.y, C->GetPosition().x, C->GetPosition().y, C->IsGrounded());
			if (Angle < 0.7f) { EXPECT(C->IsGrounded()); EXPECT(C->GetPosition().x > Start.x + 1); EXPECT(C->GetPosition().y > Start.y + 0.3f); }
			else { EXPECT(!C->IsGrounded()); EXPECT(C->GetPosition().y <= Start.y + 0.03f); }
		}
	}

	void TestShortSweeps(CharacterShape Shape)
	{
		for (auto Type : { SceneQueryType::StaticAABB, SceneQueryType::DynamicAABB })
		{
			PhysicsWorldParam P; P.sceneQueryType = Type;
			PhysicsWorld World(P); Floor(World);
			Box(World, Vector3(8, 4, 8), Vector3(2));
			CharacterControllerSettings S; S.Shape = Shape;
			auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
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
			auto* G = GeometryFactory::CreateTriangleMesh();
			auto* M = G->GetShapeObj<TriangleMesh>();
			for (auto P : { Vector3(-4, 0, -4), Vector3(-4, 0, 4), Vector3(4, 0, 4), Vector3(4, 0, -4) }) M->AddVertex(P);
			M->AddTriangle(0, 1, 2); M->AddTriangle(0, 2, 3);
			M->Compact(); M->BuildBVH(); G->UpdateVolumeProperties(); return G;
		}
		if (Kind == 1)
		{
			auto* G = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4), Vector3(4, 1, 4)), 8, 8);
			auto* H = G->GetShapeObj<HeightField3>(); H->HeightScale = 0.001f; H->AllocMemory();
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

	void TestSurfaces(CharacterShape Shape)
	{
		for (int Kind = 0; Kind < 3; ++Kind)
		{
			PhysicsWorld World{ PhysicsWorldParam() }; World.CreateRigidBody(Surface(Kind), RigidBodyParam());
			CharacterControllerSettings S; S.Shape = Shape;
			auto* C = World.CreateCharacterController(S, Vector3(-1, 2, -0.5f)); Step(World, 45);
			EXPECT(C->IsGrounded());
			C->SetMoveVelocity(Vector3(2, 0, 1)); Step(World, 60);
			printf("Character surface shape=%d kind=%d position=(%.5f,%.5f,%.5f)\n", (int)Shape, Kind, C->GetPosition().x, C->GetPosition().y, C->GetPosition().z);
			EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().x, 1, 0.03f);
			EXPECT_NEAR(C->GetPosition().z, 0.5f, 0.03f); EXPECT_NEAR(C->GetPosition().y, Height(S) + S.SkinWidth, 0.01f);
			C->SetPosition(Vector3(0, Height(S) - 0.1f, 0));
			const auto Recovery = C->Move(Vector3::Zero(), Dt);
			EXPECT(!Recovery.RecoveryIncomplete); EXPECT(Recovery.RecoveryDistance > 0.09f);
			EXPECT(C->IsGrounded());
		}
	}

	void TestFilters(CharacterShape Shape)
	{
		PhysicsWorldParam P; P.collisionFilter.reset(CollisionFilter::CreateLayerMask());
		PhysicsWorld World(P); Floor(World);
		Box(World, Vector3(1, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetFilterData(CollisionData::ForLayer(2));
		Box(World, Vector3(2, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetTriggerEnabled(true);
		Box(World, Vector3(3, 2, 0), Vector3(0.1f, 2, 2))->mGeometries[0]->SetQueryEnabled(false);
		Box(World, Vector3(4, 2, 0), Vector3(0.2f, 2, 2));
		CharacterControllerSettings S; S.Shape = Shape; S.FilterData = CollisionData::ForLayer(1, 1u);
		auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		C->Move(Vector3(6, 0, 0), Dt);
		EXPECT(C->GetPosition().x > 3.35f && C->GetPosition().x < 3.45f); EXPECT(C->IsGrounded());
		const float X = C->GetPosition().x;
		C->SetFilterData(CollisionData::ForLayer(1, 0)); C->Move(Vector3(2, 0, 0), Dt);
		EXPECT_NEAR(C->GetPosition().x, X + 2, 0.001f); EXPECT(!C->IsGrounded());
	}

	void TestCeilingAndCorners(CharacterShape Shape)
	{
		PhysicsWorld World{ PhysicsWorldParam() }; Floor(World);
		Box(World, Vector3(0, 2.5f, 0), Vector3(5, 0.2f, 5));
		Box(World, Vector3(2, 1, 0), Vector3(0.2f, 1, 5));
		Box(World, Vector3(0, 1, 2), Vector3(5, 1, 0.2f));
		CharacterControllerSettings S; S.Shape = Shape;
		auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
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
		C = Low.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
		Step(Low, 2); const auto Move = C->Move(Vector3(2, 0, 0), Dt);
		EXPECT(!Move.Stepped); EXPECT(C->GetPosition().x < 1);
	}

	void TestManualAndSubsteps(CharacterShape Shape)
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
			ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (int Substeps : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			PhysicsWorld World(P); Floor(World);
			CharacterControllerSettings S; S.Shape = Shape; S.AutomaticUpdate = false;
			auto* C = World.CreateCharacterController(S, Vector3(0, Height(S) + S.SkinWidth, 0));
			C->Move(Vector3(1, 0, 0), Dt); const Vector3 Position = C->GetPosition();
			C->SetMoveVelocity(Vector3(10, 0, 0)); Step(World, 10);
			EXPECT((C->GetPosition() - Position).Length() < 0.001f);
			S.AutomaticUpdate = true;
			auto* Other = World.CreateCharacterController(S, Vector3(-2, Height(S) + S.SkinWidth, 2));
			Other->SetMoveVelocity(Vector3(2, 0, 0)); Step(World, 60);
			EXPECT_NEAR(Other->GetPosition().x, 0, 0.01f); EXPECT(Other->IsGrounded());
			EXPECT(World.RemoveCharacterController(Other)); EXPECT(!World.RemoveCharacterController(nullptr));
		}
		PhysicsWorld World{ PhysicsWorldParam() };
		World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitZ()), RigidBodyParam());
		CharacterControllerSettings S; S.Shape = Shape; S.Up = Vector3::UnitZ();
		auto* C = World.CreateCharacterController(S, Vector3(0, 0, 2)); Step(World, 60);
		EXPECT(C->IsGrounded()); EXPECT_NEAR(C->GetPosition().z, Height(S) + S.SkinWidth, 0.01f);
		bool Rejected = false; S.Up = Vector3::Zero();
		try { World.CreateCharacterController(S, Vector3::Zero()); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT(World.GetCharacterControllerCount() == 1);
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
			auto* C = Scene.GetDemoCharacter(); EXPECT(C != nullptr); if (!C) continue;
			EXPECT(C->GetSettings().Shape == (BoxShape ? CharacterShape::Box : CharacterShape::Capsule));
			EXPECT(Scene.GetObjects().size() > OriginalObjects);
			for (int I = 0; I < 45; ++I) Scene.Step(Dt);
			EXPECT(C->IsGrounded()); const Vector3 Start = C->GetPosition();
			C->SetMoveVelocity(Vector3(3, 0, 0));
			for (int I = 0; I < 80; ++I) Scene.Step(Dt);
			EXPECT(C->GetPosition().x > Start.x + 3.5f);
			EXPECT(C->GetPosition().y > Start.y + 0.5f);
			bool Rendered = false;
			for (const auto& Object : Scene.GetObjects()) Rendered |= Object.GeometryPtr == C->GetGeometry() && Object.RenderEnabled;
			EXPECT(Rendered);
		}
		Scene.SetCharacterDemo(false, false); EXPECT(Scene.LoadFromFile(Path, &Error));
		EXPECT(!Scene.GetDemoCharacter()); EXPECT(Scene.GetObjects().size() == OriginalObjects);
	}
}

void TestCharacterController()
{
	for (auto Shape : { Riemann::CharacterShape::Capsule, Riemann::CharacterShape::Box })
	{
		TestGroundAndSlide(Shape); TestSteps(Shape); TestSlopes(Shape);
		for (auto Type : { Riemann::SceneQueryType::StaticAABB, Riemann::SceneQueryType::DynamicAABB }) TestPlatforms(Shape, Type);
		TestPlatformObstructionAndDrop(Shape);
		TestShortSweeps(Shape); TestSurfaces(Shape); TestFilters(Shape);
		TestCeilingAndCorners(Shape); TestManualAndSubsteps(Shape);
	}
	TestDemo();
}

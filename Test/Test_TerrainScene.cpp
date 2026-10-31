#include "Test.h"

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	const char* TerrainSceneText = R"json({
		"name": "Terrain scene regression",
		"physics": { "gravity": [0, 0, 0] },
		"objects": [{
			"id": "terrain", "type": "heightfield", "body": "static",
			"position": [3, 1, -2], "halfExtent": [2, 1, 2],
			"rows": 5, "columns": 5, "heightScale": 0.001, "baseHeight": 0.25
		}]
	})json";

	Geometry* FindTerrainSceneObject(const SceneWorld& Scene, const std::string& Id)
	{
		for (const SceneObjectInstance& Object : Scene.GetObjects())
		{
			if (Object.Id == Id) return Object.GeometryPtr;
		}
		return nullptr;
	}

	bool LoadTerrainText(SceneWorld& Scene, const std::string& Text = TerrainSceneText)
	{
		std::string Error;
		const bool Loaded = Scene.LoadFromText(Text, "terrain_scene_test.json", &Error);
		if (!Loaded) printf("Terrain scene load failed: %s\n", Error.c_str());
		EXPECT(Loaded);
		return Loaded;
	}

	void ExpectTerrainRayHeight(SceneWorld& Scene, Geometry* Terrain, float X, float Z, float Height)
	{
		RayCastOption Option;
		Option.MaxDist = 50.0f;
		RayCastResult Result;
		const bool Hit = Scene.GetSimulation()->GetGeometryQuery()->RayCastQuery(
			Vector3(X, 20.0f, Z), -Vector3::UnitY(), Option, &Result);
		EXPECT(Hit);
		if (Hit)
		{
			EXPECT(Result.hitGeom == Terrain);
			EXPECT_NEAR(Result.hitPoint.y, Height, 0.003f);
			EXPECT_NEAR(Result.hitTimeMin, 20.0f - Height, 0.003f);
		}
	}

	void TestTerrainSceneBrushes()
	{
		printf("Running TestTerrainSceneBrushes\n");
		SceneWorld Scene;
		if (!LoadTerrainText(Scene)) return;
		EXPECT(Scene.GetTerrainObjectIds() == std::vector<std::string>({ "terrain" }));
		Geometry* Terrain = FindTerrainSceneObject(Scene, "terrain");
		HeightField3* Field = Terrain ? Terrain->GetShapeObj<HeightField3>() : nullptr;
		EXPECT(Field != nullptr);
		if (!Field) return;
		EXPECT(Field->nX == 5 && Field->nZ == 5);
		EXPECT_NEAR(Field->GetHeight(12), 0.25f, 0.001f);
		ExpectTerrainRayHeight(Scene, Terrain, 3.0f, -2.0f, 1.25f);
		const Vector3 Center(3.0f, 1.0f, -2.0f);
		uint32_t RenderRevision = Scene.GetRenderRevision();
		uint64_t FieldRevision = Field->GetRevision();
		EXPECT(Scene.EditTerrain("terrain", Center, 0.4f, 2.0f, 0) == 1);
		EXPECT(Scene.GetRenderRevision() > RenderRevision);
		EXPECT(Field->GetRevision() > FieldRevision);
		EXPECT_NEAR(Field->GetHeight(12), 2.25f, 0.001f);
		ExpectTerrainRayHeight(Scene, Terrain, 3.0f, -2.0f, 3.25f);
		RenderRevision = Scene.GetRenderRevision();
		Scene.Step(1.0f / 60.0f);
		EXPECT(Scene.GetRenderRevision() == RenderRevision);

		// Brush centers use world coordinates; target heights use terrain-local Y.
		EXPECT(Scene.EditTerrain("terrain", Center, 0.4f, 0.5f, 1, 1.0f) == 1);
		EXPECT(Scene.GetRenderRevision() > RenderRevision);
		EXPECT_NEAR(Field->GetHeight(12), 0.5f, 0.001f);
		ExpectTerrainRayHeight(Scene, Terrain, 3.0f, -2.0f, 1.5f);
		EXPECT(Scene.EditTerrain("terrain", Center, 0.4f, 2.0f, 0) == 1);
		const float Peak = Field->GetHeight(12);
		RenderRevision = Scene.GetRenderRevision();
		EXPECT(Scene.EditTerrain("terrain", Center, 1.5f, 0.0f, 2, 1.0f) > 0);
		EXPECT(Scene.GetRenderRevision() > RenderRevision);
		EXPECT(Field->GetHeight(12) < Peak - 0.01f);
		EXPECT(Field->GetHeight(12) >= 0.25f);
		ExpectTerrainRayHeight(Scene, Terrain, 3.0f, -2.0f, 1.0f + Field->GetHeight(12));

		RenderRevision = Scene.GetRenderRevision();
		FieldRevision = Field->GetRevision();
		EXPECT(Scene.EditTerrain("missing", Center, 1.0f, 1.0f, 0) == 0);
		EXPECT(Scene.EditTerrain("terrain", Center, 1.0f, 1.0f, 99) == 0);
		EXPECT(Scene.EditTerrain("terrain", Center, 0.0f, 1.0f, 0) == 0);
		EXPECT(Scene.GetRenderRevision() == RenderRevision);
		EXPECT(Field->GetRevision() == FieldRevision);

		EXPECT(Scene.GetSimulation()->SetHeightFieldHeight(Terrain, 2, 2, 1.0f));
		EXPECT(Field->GetRevision() > FieldRevision);
		EXPECT(Scene.GetRenderRevision() == RenderRevision);
		ExpectTerrainRayHeight(Scene, Terrain, 3.0f, -2.0f, 2.0f);
		Scene.Step(1.0f / 60.0f);
		EXPECT(Scene.GetRenderRevision() > RenderRevision);
		RenderRevision = Scene.GetRenderRevision();
		Scene.Step(1.0f / 60.0f);
		EXPECT(Scene.GetRenderRevision() == RenderRevision);
		Scene.Reset();
		EXPECT(Scene.GetTerrainObjectIds().empty());
		EXPECT(Scene.GetObjects().empty());
		Scene.Step(1.0f / 60.0f);
	}

	void TestTerrainSceneSamplesAndValidation()
	{
		printf("Running TestTerrainSceneSamplesAndValidation\n");
		SceneWorld Scene;
		const char* Samples = R"json({
			"objects": [{
				"id": "terrain", "type": "heightfield", "body": "static",
				"halfExtent": [1, 1, 0.5], "rows": 2, "columns": 3,
				"heightScale": 1, "baseHeight": 9,
				"heights": [0, 1, 2, 3, 4, 5]
			}]
		})json";
		if (LoadTerrainText(Scene, Samples))
		{
			Geometry* Terrain = FindTerrainSceneObject(Scene, "terrain");
			HeightField3* Field = Terrain ? Terrain->GetShapeObj<HeightField3>() : nullptr;
			EXPECT(Field != nullptr);
			if (Field)
			{
				EXPECT(Field->nX == 3 && Field->nZ == 2);
				for (int Z = 0; Z < 2; ++Z)
				{
					for (int X = 0; X < 3; ++X) EXPECT_NEAR(Field->GetHeight(Z * 3 + X), (float)(Z * 3 + X), 1.0e-5f);
				}
				std::vector<Vector3> Vertices, Normals;
				std::vector<uint16_t> Indices;
				Field->GetMesh(Vertices, Indices, Normals);
				EXPECT(Vertices.size() == 6);
				if (Vertices.size() == 6)
				{
					EXPECT_SAME(Vertices[1], Vector3(0.0f, 1.0f, -0.5f));
					EXPECT_SAME(Vertices[4], Vector3(0.0f, 4.0f, 0.5f));
				}
				ExpectTerrainRayHeight(Scene, Terrain, 0.0f, 0.0f, 2.5f);
			}
		}

		const char* ValidParts[] = {
			"\"rows\": 5", "\"columns\": 5", "\"rows\": 5", "\"columns\": 5", "\"rows\": 5",
			"\"baseHeight\": 0.25", "\"baseHeight\": 0.25", "\"body\": \"static\"", "\"heightScale\": 0.001"
		};
		const char* InvalidParts[] = {
			"\"rows\": 1", "\"columns\": 1", "\"rows\": 256", "\"columns\": 256", "\"rows\": 2.5",
			"\"heights\": [0, 1]", "\"heights\": \"invalid\"", "\"body\": \"dynamic\"", "\"heightScale\": 0"
		};
		for (size_t i = 0; i < sizeof(ValidParts) / sizeof(ValidParts[0]); ++i)
		{
			std::string Invalid = TerrainSceneText;
			const size_t Offset = Invalid.find(ValidParts[i]);
			EXPECT(Offset != std::string::npos);
			if (Offset == std::string::npos) continue;
			Invalid.replace(Offset, std::string(ValidParts[i]).size(), InvalidParts[i]);
			std::string Error;
			EXPECT_FALSE(Scene.LoadFromText(Invalid, "invalid_terrain.json", &Error));
			EXPECT(Error.find("invalid_terrain.json") != std::string::npos);
			EXPECT(Error.find("heightfield") != std::string::npos);
			EXPECT(Scene.GetTerrainObjectIds().empty());
		}
		std::string InvalidSamples = Samples;
		const size_t Offset = InvalidSamples.find("3, 4, 5");
		EXPECT(Offset != std::string::npos);
		if (Offset != std::string::npos)
		{
			InvalidSamples.replace(Offset, std::string("3, 4, 5").size(), "3, \"bad\", 5");
			std::string Error;
			EXPECT_FALSE(Scene.LoadFromText(InvalidSamples, "invalid_samples.json", &Error));
			EXPECT(Error.find("heightfield") != std::string::npos);
		}
		EXPECT(LoadTerrainText(Scene));
		EXPECT(Scene.GetTerrainObjectIds().size() == 1);
	}

	void TestTerrainDemoScene()
	{
		printf("Running TestTerrainDemoScene\n");
		SceneWorld Scene;
		std::string Error;
		const bool Loaded = Scene.LoadFromFile(TestContentsPath() + "Scenes/14_terrain_editing.json", &Error);
		if (!Loaded) printf("Terrain demo load failed: %s\n", Error.c_str());
		EXPECT(Loaded);
		if (!Loaded) return;
		EXPECT(Scene.GetName() == "Terrain Editing");
		EXPECT(Scene.GetTerrainObjectIds() == std::vector<std::string>({ "editable_terrain" }));
		EXPECT(Scene.GetObjects().size() == 25);
		RigidBody* Falling = nullptr;
		for (const SceneObjectInstance& Object : Scene.GetObjects())
		{
			RigidBody* Body = Object.GeometryPtr ? Object.GeometryPtr->GetParent<RigidBody>() : nullptr;
			if (Body && Body->CastDynamic()) { Falling = Body; break; }
		}
		EXPECT(Falling != nullptr);
		if (!Falling) return;
		const float InitialHeight = Falling->X.y;
		for (int Frame = 0; Frame < 10; ++Frame) Scene.Step(1.0f / 60.0f);
		EXPECT(std::isfinite(Falling->X.y));
		EXPECT(Falling->X.y < InitialHeight - 0.05f);
		Geometry* Terrain = FindTerrainSceneObject(Scene, "editable_terrain");
		EXPECT(Terrain != nullptr);
		if (Terrain) ExpectTerrainRayHeight(Scene, Terrain, 12.0f, 12.0f, 0.0f);
	}
}

void TestTerrainScene()
{
	TestTerrainSceneBrushes();
	TestTerrainSceneSamplesAndValidation();
	TestTerrainDemoScene();
}

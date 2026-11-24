#include "Test.h"

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

#include <algorithm>

using namespace Riemann;

namespace
{
	bool IsFiniteSceneVector(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	Geometry* FindSceneGeometry(const SceneWorld& scene, const char* id)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
		{
			if (object.Id == id) return object.GeometryPtr;
		}
		return nullptr;
	}

	Transform GetSceneGeometryTransform(const Geometry* geometry)
	{
		const RigidBody* body = geometry->GetParent<RigidBody>();
		return body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
	}

	bool LoadTestScene(SceneWorld& scene, const char* fileName, const char* configurationId = "")
	{
		const std::string path = TestContentsPath() + "Scenes/" + fileName;
		std::string error;
		const bool loaded = scene.LoadFromFile(path, &error, configurationId);
		if (!loaded) printf("Scene load failed: %s\n", error.c_str());
		EXPECT(loaded);
		if (loaded)
		{
			EXPECT(scene.GetSimulation() != nullptr);
			EXPECT(!scene.GetObjects().empty());
		}
		return loaded;
	}

	void StepTestScene(SceneWorld& scene, int frameCount)
	{
		for (int frame = 0; frame < frameCount; ++frame)
		{
			scene.Step(1.0f / 60.0f);
			for (const SceneObjectInstance& object : scene.GetObjects())
			{
				Geometry* geometry = object.GeometryPtr;
				EXPECT(geometry != nullptr);
				if (!geometry) continue;
				const Transform worldTransform = GetSceneGeometryTransform(geometry);
				EXPECT(IsFiniteSceneVector(worldTransform.pos));
				EXPECT_NEAR(worldTransform.quat.SquareLength(), 1.0f, 1e-3f);
				const TriangleMesh* mesh = geometry->GetShapeObj<TriangleMesh>();
				if (mesh)
				{
					for (uint32_t vertex = 0; vertex < mesh->GetVertexCount(); ++vertex)
					{
						EXPECT(IsFiniteSceneVector(mesh->Vertices[vertex]));
					}
				}
				RigidBody* body = geometry->GetParent<RigidBody>();
				if (body)
				{
					EXPECT(IsFiniteSceneVector(body->X));
					EXPECT(IsFiniteSceneVector(body->V));
					EXPECT(IsFiniteSceneVector(body->W));
					EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1e-3f);
				}
			}
		}
	}

	void TestStackConfigurations()
	{
		SceneWorld scene;
		std::string error;
		const std::string path = TestContentsPath() + "Scenes/01_stack_boxes.json";
		EXPECT(scene.LoadFromFile(path, &error));
		EXPECT(scene.GetConfigurations().size() == 3);
		EXPECT(scene.GetActiveConfigurationId() == "column");
		EXPECT(scene.GetObjects().size() == 6);
		EXPECT_NEAR((scene.GetCamera().Eye - scene.GetCamera().At).Length(), 15.0f, 1e-4f);
		EXPECT_NEAR(scene.GetCamera().FovY, CameraDesc().FovY, 1e-6f);

		EXPECT(scene.LoadFromFile(path, &error, "grid"));
		EXPECT(scene.GetActiveConfigurationId() == "grid");
		EXPECT(scene.GetObjects().size() == 126);
		const CameraDesc gridCamera = scene.GetCamera();
		EXPECT_NEAR((gridCamera.Eye - gridCamera.At).Length(), 24.0f, 1e-4f);
		EXPECT(!scene.GetSimulation()->GetTemporalSubstepSettings().Enabled);

		EXPECT(scene.LoadFromFile(path, &error, "pyramid"));
		EXPECT(scene.GetActiveConfigurationId() == "pyramid");
		EXPECT(scene.GetObjects().size() == 56);
		EXPECT_SAME(scene.GetCamera().Eye, gridCamera.Eye);
		EXPECT_SAME(scene.GetCamera().At, gridCamera.At);
		EXPECT_NEAR(scene.GetCamera().FovY, gridCamera.FovY, 1e-6f);
		EXPECT(scene.GetSimulation()->GetTemporalSubstepSettings().Substeps == 4);
		std::vector<Vector3> initial;
		int rows[10] = {};
		for (const SceneObjectInstance& object : scene.GetObjects())
		{
			RigidBodyDynamic* body = object.GeometryPtr->GetParent<RigidBody>()->CastDynamic();
			if (!body) continue;
			initial.push_back(body->X);
			const int row = (int)std::lround((body->X.y - 1.5f) / 2.0f);
			EXPECT(row >= 0 && row < 10);
			if (row >= 0 && row < 10) ++rows[row];
			EXPECT_NEAR(body->X.z, 0.0f, 1e-6f);
		}
		EXPECT(initial.size() == 55);
		for (int row = 0; row < 10; ++row) EXPECT(rows[row] == 10 - row);
		StepTestScene(scene, 300);
		size_t index = 0;
		int sleeping = 0;
		float maximumDrift = 0, maximumHeightError = 0, maximumSpeed = 0;
		for (const SceneObjectInstance& object : scene.GetObjects())
		{
			RigidBodyDynamic* body = object.GeometryPtr->GetParent<RigidBody>()->CastDynamic();
			if (!body) continue;
			if (index >= initial.size()) { EXPECT(false); break; }
			const Vector3 start = initial[index++];
			const float expectedHeight = 0.5f + (start.y - 1.5f) * 0.5f;
			maximumDrift = std::max(maximumDrift, std::max(std::abs(body->X.x - start.x), std::abs(body->X.z)));
			maximumHeightError = std::max(maximumHeightError, std::abs(body->X.y - expectedHeight));
			maximumSpeed = std::max(maximumSpeed, body->V.Length());
			EXPECT((body->Q * Vector3::UnitY()).y > 0.999f);
			sleeping += body->Sleeping ? 1 : 0;
		}
		printf("  Pyramid: sleeping=%d drift=%.6f heightError=%.6f speed=%.6f\n",
			sleeping, maximumDrift, maximumHeightError, maximumSpeed);
		EXPECT(sleeping == 55);
		EXPECT(maximumDrift < 0.02f && maximumHeightError < 0.03f && maximumSpeed < 0.01f);

		// A rejected selection must preserve the loaded layout and its metadata.
		EXPECT_FALSE(scene.LoadFromFile(path, &error, "missing"));
		EXPECT(error.find("unknown configuration") != std::string::npos);
		EXPECT(scene.GetActiveConfigurationId() == "pyramid" && scene.GetObjects().size() == 56);
		EXPECT(scene.LoadFromFile(path, &error, "column"));
		EXPECT(scene.GetObjects().size() == 6);
		EXPECT(!scene.GetSimulation()->GetTemporalSubstepSettings().Enabled);
		EXPECT(!scene.GetSimulation()->GetIslandSleepingSettings().Enabled);
	}

	void TestDominoConfigurations()
	{
		SceneWorld scene;
		const std::string path = TestContentsPath() + "Scenes/03_domino.json";
		std::string error;
		if (!LoadTestScene(scene, "03_domino.json")) return;
		EXPECT(scene.GetActiveConfigurationId() == "classic" && scene.GetConfigurations().size() == 2);
		EXPECT(scene.GetName() == "Domino" && scene.GetObjects().size() == 23);
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 0);
		EXPECT_SAME(FindSceneGeometry(scene, "starter_sphere")->GetParent<RigidBody>()->X, Vector3(0, 20, -20));
		EXPECT_SAME(FindSceneGeometry(scene, "domino_line_0_0_0")->GetParent<RigidBody>()->X, Vector3(0, 3, -10));
		StepTestScene(scene, 90);

		for (bool release : { false, true })
		{
			if (!LoadTestScene(scene, "03_domino.json", "chain")) return;
			EXPECT(scene.GetActiveConfigurationId() == "chain" && scene.GetConfigurations().size() == 2);
			EXPECT(scene.GetSimulation()->GetConstraintCount() == 0);
			const CameraDesc camera = scene.GetCamera();
			EXPECT((camera.Eye - camera.At).Dot(scene.GetLight().Direction) < 0);
			RigidBody* roller = FindSceneGeometry(scene, "roller")->GetParent<RigidBody>();
			if (!release) roller->CastDynamic()->Freeze();
			std::vector<RigidBody*> upper, lower;
			for (int i = 0; i < 11; ++i) upper.push_back(FindSceneGeometry(scene, ("upper_domino_" + std::to_string(i)).c_str())->GetParent<RigidBody>());
			for (int i = 0; i < 10; ++i) lower.push_back(FindSceneGeometry(scene, ("lower_domino_" + std::to_string(i)).c_str())->GetParent<RigidBody>());
			RigidBody* transfer = FindSceneGeometry(scene, "transfer_weight")->GetParent<RigidBody>();
			RigidBody* finish = FindSceneGeometry(scene, "finish_ball")->GetParent<RigidBody>();
			std::vector<float> upperTimes(upper.size(), -1), lowerTimes(lower.size(), -1);
			float transferTime = -1, finishTime = -1;
			for (int frame = 0; frame < 720; ++frame)
			{
				StepTestScene(scene, 1);
				EXPECT_SAME(scene.GetCamera().Eye, camera.Eye);
				EXPECT_SAME(scene.GetCamera().At, camera.At);
				const float time = (frame + 1) / 60.0f;
				for (size_t i = 0; i < upper.size(); ++i)
					if (upperTimes[i] < 0 && (upper[i]->Q * Vector3::UnitY()).y < 0.9f) upperTimes[i] = time;
				for (size_t i = 0; i < lower.size(); ++i)
					if (lowerTimes[i] < 0 && (lower[i]->Q * Vector3::UnitY()).y < 0.9f) lowerTimes[i] = time;
				if (transferTime < 0 && transfer->X.y < 8.5f) transferTime = time;
				if (finishTime < 0 && finish->X.y < 1.5f) finishTime = time;
			}
			if (release)
			{
				EXPECT(upperTimes.front() > 1 && upperTimes.back() < 6);
				for (size_t i = 1; i < upperTimes.size(); ++i) EXPECT(upperTimes[i] > upperTimes[i - 1]);
				EXPECT(transferTime > upperTimes.back());
				EXPECT(lowerTimes.front() > transferTime);
				for (size_t i = 1; i < lowerTimes.size(); ++i) EXPECT(lowerTimes[i] > lowerTimes[i - 1]);
				EXPECT(finishTime > lowerTimes.back() && finishTime < 11);
				EXPECT(finish->X.x > 7.5f && finish->X.x < 9.7f && finish->X.y < 1.2f && std::abs(finish->X.z) < 1.3f);
				printf("  See saw chain: upper=%.2f..%.2f transfer=%.2f lower=%.2f..%.2f finish=%.2f\n",
					upperTimes.front(), upperTimes.back(), transferTime, lowerTimes.front(), lowerTimes.back(), finishTime);
			}
			else
			{
				// Holding the initial weight must prevent every downstream stage.
				for (float time : upperTimes) EXPECT(time < 0);
				for (float time : lowerTimes) EXPECT(time < 0);
				EXPECT(transferTime < 0 && finishTime < 0);
				EXPECT((finish->X - Vector3(9.6f, 3.85f, 0)).Length() < 0.02f);
			}
		}
		EXPECT(scene.LoadFromFile(path, &error, "classic"));
		EXPECT(scene.GetObjects().size() == 23);
		EXPECT(FindSceneGeometry(scene, "finish_ball") == nullptr);
		EXPECT(scene.GetCameraSidebarPadding() == 0);
	}

	void TestSimpleMachines()
	{
		SceneWorld scene;
		if (!LoadTestScene(scene, "02_machices.json")) return;
		const CameraDesc camera = scene.GetCamera();
		EXPECT(scene.GetCameraSidebarPadding() == 300);
		EXPECT((camera.Eye - camera.At).Dot(scene.GetLight().Direction) < 0);
		const char* ids[] = { "plank", "pendulum_bob_2", "crank_slider", "spring_mass" };
		RigidBody* bodies[4] = {};
		float minimum[4] = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
		float maximum[4] = { -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
		TriangleMesh* spring = nullptr;
		for (int i = 0; i < 4; ++i)
		{
			Geometry* shape = FindSceneGeometry(scene, ids[i]);
			bodies[i] = shape ? shape->GetParent<RigidBody>() : nullptr;
			EXPECT(bodies[i] && bodies[i]->CastDynamic());
			if (!bodies[i]) return;
		}
		for (const auto& object : scene.GetObjects())
			if (object.Id.find("spring_mass_spring_") == 0)
			{
				spring = object.GeometryPtr->GetShapeObj<TriangleMesh>();
				EXPECT(!object.GeometryPtr->IsSimulationEnabled());
			}
		EXPECT(spring != nullptr);
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 13);
		for (int frame = 0; frame < 300; ++frame)
		{
			StepTestScene(scene, 1);
			EXPECT_SAME(scene.GetCamera().Eye, camera.Eye);
			EXPECT_SAME(scene.GetCamera().At, camera.At);
			const float values[] = { (bodies[0]->Q * Vector3::UnitX()).y, bodies[1]->X.x, bodies[2]->X.x, bodies[3]->X.y };
			// Require visible motion late in the clip, after the initial release.
			if (frame >= 180) for (int i = 0; i < 4; ++i)
			{
				minimum[i] = std::min(minimum[i], values[i]);
				maximum[i] = std::max(maximum[i], values[i]);
			}
			EXPECT((bodies[0]->X - Vector3(-9, 2.3f, 0)).Length() < 0.01f);
			EXPECT(bodies[1]->X.x > -5.8f && bodies[1]->X.x < -0.2f && bodies[1]->X.y > 0.8f);
			EXPECT(bodies[2]->X.x > 3.3f && bodies[2]->X.x < 5.3f && std::abs(bodies[2]->X.y - 2.3f) < 0.01f);
			EXPECT(bodies[3]->X.y > 1.2f && bodies[3]->X.y < 3.7f && std::abs(bodies[3]->X.x - 9) < 0.01f);
			if (spring && spring->NumVertices >= 18)
			{
				// The two tube cap centers must follow the authored world anchors.
				EXPECT_SAME(spring->Vertices[spring->NumVertices - 18], Vector3(9, 5.3f, 0));
				EXPECT((spring->Vertices[spring->NumVertices - 9] - bodies[3]->X - Vector3(0, 0.38f, 0)).Length() < 0.001f);
			}
		}
		EXPECT(maximum[0] - minimum[0] > 0.3f);
		for (int i = 1; i < 4; ++i) EXPECT(maximum[i] - minimum[i] > 0.8f);
		printf("  Simple machines late motion: lever=%.3f pendulum=%.3f crank=%.3f spring=%.3f\n",
			maximum[0] - minimum[0], maximum[1] - minimum[1], maximum[2] - minimum[2], maximum[3] - minimum[3]);
		scene.Reset();
		EXPECT(scene.GetObjects().empty());
		EXPECT(scene.GetCameraSidebarPadding() == 0);

		const std::string prefix = R"({"objects":[{"id":"a","body":"static","position":[0,4,0]},
			{"id":"b","body":"dynamic","position":[0,1,0]}],"constraints":[{"type":"slider","bodyA":"a","bodyB":"b","springVisual":)";
		std::string error;
		for (const char* visual : { "null", R"({"radius":0})", R"({"wireRadius":0.4})", R"({"turns":0})",
			R"({"turns":65})", R"({"turns":1.5})", R"({"anchorA":[0,1]})", R"({"anchorB":[0,"bad",0]})" })
		{
			EXPECT_FALSE(scene.LoadFromText(prefix + visual + "}]}", "invalid_spring.json", &error));
			EXPECT(error.find("springVisual") != std::string::npos);
		}
		EXPECT(scene.LoadFromText(prefix + "{} }]}", "spring.json", &error));
		StepTestScene(scene, 10);
		for (const char* value : { "-1", "null", "true" })
		{
			EXPECT_FALSE(scene.LoadFromText(std::string("{\"objects\":[],\"camera\":{\"sidebarPadding\":") + value + "}}", "camera.json", &error));
			EXPECT(error.find("sidebarPadding") != std::string::npos);
		}
	}

	void TestSceneConfigurationSchema()
	{
		const std::string text = R"json({
			"name":"Custom layouts", "defaultConfiguration":"many",
			"camera":{"center":[3,4,5]},
			"objects":[{"id":"unused"}],
			"configurations":[
				{"id":"few","name":"Few boxes","scene":{"objects":[{"id":"few","repeat":{"count":[2,1,1],"spacing":[3,0,0]}}]}},
				{"id":"many","name":"Many boxes","scene":{"objects":[{"id":"many","repeat":{"count":[4,1,1],"spacing":[3,0,0]}}]}}
			]
		})json";
		SceneWorld scene;
		std::string error;
		EXPECT(scene.LoadFromText(text, "custom.json", &error));
		EXPECT(scene.GetName() == "Custom layouts");
		EXPECT(scene.GetActiveConfigurationId() == "many" && scene.GetObjects().size() == 4);
		EXPECT(FindSceneGeometry(scene, "unused") == nullptr);
		EXPECT_SAME(scene.GetCamera().At, Vector3(3, 4, 5));
		EXPECT(scene.LoadFromText(text, "custom.json", &error, "few"));
		EXPECT(scene.GetActiveConfigurationId() == "few" && scene.GetObjects().size() == 2);
		EXPECT(scene.GetConfigurations().size() == 2);
		if (scene.GetConfigurations().size() == 2) EXPECT(scene.GetConfigurations()[0].Name == "Few boxes");
		for (const char* invalid : {
			R"({"configurations":[]})", R"({"configurations":{}})", R"({"defaultConfiguration":"missing","objects":[]})",
			R"({"configurations":[{"id":"a","name":"A","scene":null}]})",
			R"({"configurations":[{"id":"a","scene":{"objects":[]}}]})",
			R"({"configurations":[{"id":"a","name":"A","scene":{}},{"id":"a","name":"B","scene":{}}]})",
			R"({"defaultConfiguration":"missing","configurations":[{"id":"a","name":"A","scene":{"objects":[]}}]})"
		})
		{
			EXPECT_FALSE(scene.LoadFromText(invalid, "invalid.json", &error));
			EXPECT(error.find("configuration") != std::string::npos || error.find("Configuration") != std::string::npos);
			EXPECT(scene.GetActiveConfigurationId() == "few" && scene.GetObjects().size() == 2);
		}
		EXPECT(scene.LoadFromText(R"({"objects":[]})", "plain.json", &error));
		EXPECT(scene.GetConfigurations().empty() && scene.GetActiveConfigurationId().empty());
		EXPECT(scene.LoadFromText(text, "custom.json", &error));
		scene.Reset();
		EXPECT(scene.GetConfigurations().empty() && scene.GetActiveConfigurationId().empty());
	}
}

void TestSceneLoading()
{
	printf("Running TestSceneLoading\n");
	TestStackConfigurations();
	TestSceneConfigurationSchema();
	TestSimpleMachines();
	TestDominoConfigurations();
	SceneWorld scene;
	if (LoadTestScene(scene, "10_destruction.json"))
	{
		EXPECT(scene.GetDestructionSetCount() > 0);
		StepTestScene(scene, 2);
	}
	if (LoadTestScene(scene, "06_constraints.json"))
	{
		EXPECT(scene.GetName() == "Constraints");
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 10);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
		EXPECT(scene.GetConstraintControls().size() == 10);
		StepTestScene(scene, 10);
		for (const ConstraintControlState& state : scene.GetConstraintControls())
		{
			EXPECT(state.Enabled);
			EXPECT(std::isfinite(state.CurrentValue));
		}
	}

	// Loading into the same instance must release the previous world's constraints.
	if (LoadTestScene(scene, "18_rope_towing.json"))
	{
		EXPECT(scene.GetName() == "Rope Towing Around a Post");
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 0);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 1);
		EXPECT(scene.GetConstraintControls().empty());
		Geometry* anchor = FindSceneGeometry(scene, "handle");
		Geometry* surface = FindSceneGeometry(scene, "tow_rope_surface");
		Geometry* box = FindSceneGeometry(scene, "crate");
		EXPECT(anchor != nullptr && surface != nullptr && box != nullptr);
		StepTestScene(scene, 10);
	}

	const char* invalidColliderScene = R"json({
		"name": "Invalid rope collider",
		"objects": [],
		"ropes": [{
			"id": "invalid_rope", "start": [0, 2, 0], "end": [2, 2, 0],
			"segments": 4, "colliders": ["missing_obstacle"]
		}]
	})json";
	std::string error;
	EXPECT_FALSE(scene.LoadFromText(invalidColliderScene, "invalid_collider.json", &error));
	EXPECT(error.find("missing_obstacle") != std::string::npos);
	EXPECT(error.find("collider") != std::string::npos);
	EXPECT(error.find("invalid_collider.json") != std::string::npos);

	const char* deformableScene = R"json({
		"name": "Deformable tetrahedron",
		"objects": [],
		"deformables": [{
			"id": "soft_tet",
			"particles": [[0, 2, 0], [1, 2, 0], [0, 3, 0], [0, 2, 1]],
			"tetrahedra": [[0, 1, 2, 3]],
			"triangles": [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
			"pins": [0], "inverseMass": 1,
			"compliance": 0.001, "volumeCompliance": 0.0001,
			"yieldStrain": 0.2, "plasticCreep": 0.0, "maxPlasticStrain": 0.5,
			"substeps": 2, "iterations": 12, "damping": 0.01,
			"gravity": [0, -9.8, 0], "floor": 0
		}]
	})json";
	const bool deformableLoaded = scene.LoadFromText(deformableScene, "deformable.json", &error);
	if (!deformableLoaded) printf("Deformable scene load failed: %s\n", error.c_str());
	EXPECT(deformableLoaded);
	if (deformableLoaded)
	{
		EXPECT(scene.GetSimulation()->GetDeformableBodyCount() == 1);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
		EXPECT(scene.GetObjects().size() == 1);
		Geometry* geometry = FindSceneGeometry(scene, "soft_tet");
		TriangleMesh* mesh = geometry ? geometry->GetShapeObj<TriangleMesh>() : nullptr;
		EXPECT(mesh != nullptr);
		if (mesh)
		{
			EXPECT(mesh->GetVertexCount() == 4);
			EXPECT(mesh->GetTriangleCount() == 4);
			if (mesh->GetVertexCount() == 4)
			{
				const Vector3 pinnedPosition = mesh->Vertices[0];
				const Vector3 freePosition = mesh->Vertices[1];
				const uint32_t revision = scene.GetRenderRevision();
				StepTestScene(scene, 10);
				// Deformed vertices stream into existing meshes without rebuilding the scene.
				EXPECT(scene.GetRenderRevision() == revision);
				EXPECT_SAME(mesh->Vertices[0], pinnedPosition);
				EXPECT((mesh->Vertices[1] - freePosition).Length() > 0.001f);
			}
		}
	}

	const char* validFragments[] = { "\"tetrahedra\": [[0, 1, 2, 3]", "\"triangles\": [[0, 2, 1]", "\"pins\": [0]" };
	const char* invalidFragments[] = { "\"tetrahedra\": [[0, 1, 2, 4]", "\"triangles\": [[0, 2, -1]", "\"pins\": [0.5]" };
	const char* expectedErrors[] = { "tetrahedron", "triangle", "pin" };
	for (int invalidCase = 0; invalidCase < 3; ++invalidCase)
	{
		std::string invalidScene = deformableScene;
		const size_t offset = invalidScene.find(validFragments[invalidCase]);
		EXPECT(offset != std::string::npos);
		if (offset == std::string::npos) continue;
		invalidScene.replace(offset, std::string(validFragments[invalidCase]).size(), invalidFragments[invalidCase]);
		error.clear();
		EXPECT_FALSE(scene.LoadFromText(invalidScene, "invalid_deformable.json", &error));
		EXPECT(error.find(expectedErrors[invalidCase]) != std::string::npos);
		EXPECT(error.find("soft_tet") != std::string::npos);
		EXPECT(error.find("invalid_deformable.json") != std::string::npos);
		EXPECT(scene.GetSimulation()->GetDeformableBodyCount() == 0);
	}

	// A rejected scene must not prevent a subsequent valid load or teardown.
	if (LoadTestScene(scene, "06_constraints.json"))
	{
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 10);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
		EXPECT(scene.GetSimulation()->GetDeformableBodyCount() == 0);
		StepTestScene(scene, 10);
	}
	scene.Reset();
	EXPECT(scene.GetObjects().empty());
	EXPECT(scene.GetSimulation()->GetConstraintCount() == 0);
	EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
	EXPECT(scene.GetSimulation()->GetDeformableBodyCount() == 0);
}

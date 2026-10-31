#include "Test.h"

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

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

	bool LoadTestScene(SceneWorld& scene, const char* fileName)
	{
		const std::string path = TestContentsPath() + "Scenes/" + fileName;
		std::string error;
		const bool loaded = scene.LoadFromFile(path, &error);
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
}

void TestSceneLoading()
{
	printf("Running TestSceneLoading\n");
	SceneWorld scene;
	if (LoadTestScene(scene, "06_constraints.json"))
	{
		EXPECT(scene.GetName() == "Constraints");
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 7);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
		EXPECT(scene.GetConstraintControls().size() == 7);
		StepTestScene(scene, 10);
		for (const ConstraintControlState& state : scene.GetConstraintControls())
		{
			EXPECT(state.Enabled);
			EXPECT(std::isfinite(state.CurrentValue));
		}
	}

	// Loading into the same instance must release the previous world's constraints.
	if (LoadTestScene(scene, "07_rope_box.json"))
	{
		EXPECT(scene.GetName() == "Rope");
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 0);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 1);
		EXPECT(scene.GetConstraintControls().empty());
		Geometry* anchor = FindSceneGeometry(scene, "anchor");
		Geometry* start = FindSceneGeometry(scene, "tow_rope_particle_0");
		Geometry* box = FindSceneGeometry(scene, "pulled_box");
		EXPECT(anchor != nullptr && start != nullptr && box != nullptr);
		const Vector3 initialBoxPosition = box ? GetSceneGeometryTransform(box).pos : Vector3::Zero();
		StepTestScene(scene, 10);
		if (anchor && start) EXPECT_SAME(GetSceneGeometryTransform(anchor).pos, GetSceneGeometryTransform(start).pos);
		if (box) EXPECT((GetSceneGeometryTransform(box).pos - initialBoxPosition).Length() > 0.01f);
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
				EXPECT(scene.GetRenderRevision() > revision);
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
		EXPECT(scene.GetSimulation()->GetConstraintCount() == 7);
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

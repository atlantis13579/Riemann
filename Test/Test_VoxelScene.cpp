#include "Test.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

void TestVoxelScene()
{
	printf("Running TestVoxelScene\n");
	SceneWorld scene;
	std::string error;
	const std::string prefix = R"json({"objects":[{"id":"voxels","type":"voxelMesh","body":"static",
		"position":[3,1,-2],"grid":[2,2,1],"voxelSize":0.5,"voxelHeight":0.25,"spans":)json";
	EXPECT(scene.LoadFromText(prefix + "[[0,0,0,1]]}]}", "voxel_test", &error));
	if (scene.GetObjects().empty()) return;
	Geometry* geometry = scene.GetObjects()[0].GeometryPtr;
	TriangleMesh* mesh = geometry->GetShapeObj<TriangleMesh>();
	EXPECT(mesh && mesh->GetBVH());
	if (mesh)
	{
		EXPECT(mesh->GetTriangleCount() == 20);
		EXPECT(mesh->mNormals.size() == mesh->GetVertexCount());
	}
	RayCastOption option;
	RayCastResult hit;
	option.MaxDist = 20;
	EXPECT(scene.GetSimulation()->GetGeometryQuery()->RayCastQuery(
		Vector3(3.25f, 10, -1.75f), -Vector3::UnitY(), option, &hit));
	EXPECT(hit.hitGeom == geometry);
	EXPECT_NEAR(hit.hitPoint.y, 1.5f, 0.001f);
	for (const char* invalid : { "[]", "[[2,0,0,1]]", "[[0,0,1,0]]", "[[0,0,0,2]]", "[[0.5,0,0,1]]", "[[0,0,0]]" })
	{
		EXPECT(!scene.LoadFromText(prefix + invalid + "}]}", "invalid_voxels", &error));
	}
	EXPECT(scene.LoadFromFile(TestContentsPath() + "Scenes/15_voxel_mesh.json", &error));
	RigidBody* sphere = nullptr;
	for (const SceneObjectInstance& object : scene.GetObjects())
	{
		if (object.Id == "falling_sphere") sphere = object.GeometryPtr->GetParent<RigidBody>();
	}
	EXPECT(sphere != nullptr);
	if (!sphere) return;
	const float initialHeight = sphere->X.y;
	for (int frame = 0; frame < 180; ++frame) scene.Step(1.0f / 60.0f);
	EXPECT(sphere->X.y < initialHeight - 1.0f);
	EXPECT_NEAR(sphere->X.y, 0.7f, 0.06f);
}

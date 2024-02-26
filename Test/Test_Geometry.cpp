
#include "Test.h"

#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/Polygon3d.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/DenseTensorField3d.h"

using namespace Riemann;

void TestMeshSimplify()
{
	Mesh mesh;
	mesh.LoadObj("../TestData/bunny.obj");
	mesh.Simplify(0.3f);
	mesh.ExportObj("../TestData/bunny2.obj");
}

void TestClip()
{
	printf("Running TestClip\n");

	Vector3 poly1[] = { Vector3(0, 0, 0), Vector3(4, 0, 0), Vector3(4, 4, 0), Vector3(0, 4, 0) };
	Vector3 poly2[] = { Vector3(-1, -1, 0), Vector3(-1, 1, 0), Vector3(1, 1, 0), Vector3(1, -1, 0) };
	Vector3 poly3[] = { Vector3(2, 5, -1), Vector3(2, 5, 1), Vector3(5, 2, 1), Vector3(5, 2, -1) };

	int c1 = 0, c2 = 0;
	Vector3 clip1[8], clip2[8];

	ClipPolygonByPlane3D(poly1, 4, Vector3(1, 1, 0), Vector3::UnitY(), clip1, &c1);
	EXPECT(c1 == 4);
	ClipPolygonByPlane3D(poly1, 4, Vector3(3, 3, 0), Vector3(3, 3, 0), clip1, &c1);
	EXPECT(c1 == 3);
	ClipPolygonByPlane3D(poly1, 4, Vector3(3, 3, 0), Vector3(-3, -3, 0), clip1, &c1);
	EXPECT(c1 == 5);
	ClipPolygonByProjectPolygon3D(poly1, 4, poly3, 4, Vector3(3, 3, 0), clip1, &c1);
	EXPECT(c1 == 6);
	ClipPolygonByProjectPolygon3D(poly1, 4, poly3, 4, Vector3(3, 3, 0), clip1, &c1);
	EXPECT(c1 == 6);
	ClipPolygonByAABB3D(poly1, 4, Vector3(2, 2, -1), Vector3(5, 5, 1), clip1, &c1);
	EXPECT(c1 == 4);

	bool succ = ClipPolygonAgainPolygon3D(poly1, 4, poly2, 4, Vector3::UnitZ(), 0.02f, clip1, &c1, clip2, &c2);
	EXPECT(succ);
	return;
}

void TestCatmullRom()
{
	printf("Running TestCatmullRom\n");
	std::vector<Vector3> paths;
	paths.emplace_back(0.0f, 0.0f, 0.0f);
	paths.emplace_back(0.0f, 1.0f, 0.0f);
	paths.emplace_back(2.0f, 1.0f, 0.0f);

	std::vector<SplineNode> smoothed = CatmullRom::Smoothing(paths, 0.2f);

	EXPECT(smoothed.size() == 16);
	EXPECT(smoothed[0].point == paths[0]);
	EXPECT(smoothed[5].point == paths[1]);
	EXPECT(smoothed[15].point == paths[2]);
	return;
}

void TestMesh1()
{
	printf("Running TestMesh1\n");
	Mesh mesh;
	mesh.AddAABB(Vector3(-1, -1, -1), Vector3(0.99f, 0.99f, 0.99f));
	mesh.CalculateBoundingBox();

	VoxelField field;
	field.MakeEmpty(Box3d::Unit(), 2, 2, 2, 1.0f, 2.0f);
	field.AddVoxel(0, 1, 2, 0);
	field.AddVoxel(0, 3, 5, 0);
	field.AddVoxel(0, 7, 8, 0);
	field.AddVoxel(0, 1, 10, 0);
	field.AddVoxel(1, 0, 0, 0);
	field.AddVoxel(2, 0, 1, 0);

	field.SerializeTo("../TestData/test.voxel");

	SparseVoxelField inference;
	inference.SerializeFrom("../TestData/test.voxel");

	Box3d v = field.GetVoxelBox(Vector3(-0.1f, -0.1f, -0.1f));
	EXPECT(Maths::FloatEqual(v.Min.x, -1.0f));
	EXPECT(Maths::FloatEqual(v.Max.x, 0.0f));

	v = field.GetVoxelBox(Vector3(0.01f, 0.01f, 0.01f));
	EXPECT(Maths::FloatEqual(v.Min.x, 0.0f));
	EXPECT(Maths::FloatEqual(v.Max.x, 1.0f));

	field.MakeComplementarySet();

	VoxelizationInfo info;
	info.BV.Min = Vector3(-2, -2, -2);
	info.BV.Max = Vector3(2, 2, 2);
	info.VoxelHeight = 0.5f;
	info.VoxelSize = 0.5f;

	field.VoxelizationTrianglesSet(info, &mesh);
	field.MakeComplementarySet();
	std::unordered_map<int, uint64_t> volumes;
	int space = field.SolveTopology(0.01f, &volumes);
	EXPECT(space == 2);
	return;
}


void TestGeometry()
{
	TestMeshSimplify();
	TestClip();
	TestCatmullRom();
	TestMesh1();
}

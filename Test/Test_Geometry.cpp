
#include "Test.h"

#include "../Src/CollisionPrimitive/AxisAlignedBox3d.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/CollisionPrimitive/OrientedBox3d.h"
#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/Polygon3d.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/SparseVoxelField.h"
#include "../Src/Geometry/DenseTensorField3d.h"

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

	field.SerializeTo("data/test.voxel");

	SparseVoxelField inference;
	inference.SerializeFrom("data/test.voxel");

	Box3d v = field.GetVoxelBox(Vector3(-0.1f, -0.1f, -0.1f));
	EXPECT(FloatEqual(v.mMin.x, -1.0f));
	EXPECT(FloatEqual(v.mMax.x, 0.0f));

	v = field.GetVoxelBox(Vector3(0.01f, 0.01f, 0.01f));
	EXPECT(FloatEqual(v.mMin.x, 0.0f));
	EXPECT(FloatEqual(v.mMax.x, 1.0f));

	field.MakeComplementarySet();

	VoxelizationInfo info;
	info.BV.mMin = Vector3(-2, -2, -2);
	info.BV.mMax = Vector3(2, 2, 2);
	info.VoxelHeight = 0.5f;
	info.VoxelSize = 0.5f;

	field.VoxelizationTrianglesSet(info, &mesh);
	field.MakeComplementarySet();
	std::unordered_map<int, uint64_t> volumes;
	int space = field.SolveTopology(0.01f, &volumes);
	EXPECT(space == 2);
	return;
}

void TestBuildOBB()
{
	printf("Running TestBuildOBB\n");
	
	AxisAlignedBox3d aabb(Vector3(-10.0f, -2.5f, -1.0f), Vector3(10.0f, 2.5f, 1.0f));
	Matrix3 mat;
	Vector3 center(1, 1, 2);
	
	mat.LoadRotateZ(PI_OVER_6);
	std::vector<Vector3> verties, normals;
	std::vector<uint16_t> Indices;
	
	aabb.GetMesh(verties, Indices, normals);
	for (size_t i = 0; i < verties.size(); ++i)
	{
		verties[i] = mat * verties[i] + center;
	}
	
	OrientedBox3d obb = OrientedBox3d::CalcBoundingOBB_PCA(verties.data(), (int)verties.size());
	
	EXPECT(fabsf(obb.Center.x - center.x) < 0.1f);
	EXPECT(fabsf(obb.Center.y - center.y) < 0.1f);
	EXPECT(fabsf(obb.Center.z - center.z) < 0.1f);
	
	Vector3 dir = obb.Rot.GetCol(0);
	float dp = dir.Dot(Vector3::UnitX());
	EXPECT(dp >= 0.8f);
	
	return;
}

void TestGeometry()
{
	TestClip();
	TestCatmullRom();
	TestMesh1();
	TestBuildOBB();
}

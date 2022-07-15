
#include "Test.h"

#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/SparseVoxelField.h"
#include "../Src/Geometry/DenseTensorField3d.h"

void TestCatmullRom()
{
	printf("Running TestCatmullRom\n");
	std::vector<Vector3d> paths;
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
	mesh.AddAABB(Vector3d(-1, -1, -1), Vector3d(0.99f, 0.99f, 0.99f));
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

	Box3d v = field.GetVoxelBox(Vector3d(-0.1f, -0.1f, -0.1f));
	EXPECT(FloatEqual(v.Min.x, -1.0f));
	EXPECT(FloatEqual(v.Max.x, 0.0f));

	v = field.GetVoxelBox(Vector3d(0.01f, 0.01f, 0.01f));
	EXPECT(FloatEqual(v.Min.x, 0.0f));
	EXPECT(FloatEqual(v.Max.x, 1.0f));

	field.MakeComplementarySet();

	VoxelizationInfo info;
	info.BV.Min = Vector3d(-2, -2, -2);
	info.BV.Max = Vector3d(2, 2, 2);
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
	TestCatmullRom();
	TestMesh1();
}

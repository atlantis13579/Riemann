
#include "Test.h"

#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Geometry/GeometryBoolean.h"
#include "../Src/Geometry/Polygon3.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/Delaunay.h"
#include "../Src/Geometry/Voronoi2.h"
#include "../Src/Geometry/Voronoi3.h"
#include "../Src/Geometry/DenseTensorField3.h"
#include "../Src/Geometry/SparseSpatialHash.h"
#include "../Src/Geometry/SparseOctree.h"

using namespace Riemann;

void TestParseObj()
{
	printf("Running TestParseObj\n");

	int num;
	int v[32], vt[32], vn[32];
	StaticMesh mesh;

	num = mesh.ParseFace(" 1 2 3", v, vt, vn, 32);
	EXPECT(num == 3);
	EXPECT(v[0] == 0 && v[1] == 1 && v[2] == 2);

	num = mesh.ParseFace(" 1 23 3 4333", v, vt, vn, 32);
	EXPECT(num == 4);
	EXPECT(v[0] == 0 && v[1] == 22 && v[2] == 2 && v[3] == 4332);

	num = mesh.ParseFace(" 1 23 3 4333 22", v, vt, vn, 32);
	EXPECT(num == 5);
	EXPECT(v[0] == 0 && v[1] == 22 && v[2] == 2 && v[3] == 4332 && v[4] == 21);

	num = mesh.ParseFace(" 1/1/1 22/21/2 33/3/34", v, vt, vn, 32);
	EXPECT(num == 3);
	EXPECT(v[0] == 0 && v[1] == 21 && v[2] == 32);
	EXPECT(vt[0] == 0 && vt[1] == 20 && vt[2] == 2);
	EXPECT(vn[0] == 0 && vn[1] == 1 && vn[2] == 33);

	num = mesh.ParseFace(" 1/1/1 22/21/2 33/3/34 4/4/5", v, vt, vn, 32);
	EXPECT(num == 4);
	EXPECT(v[0] == 0 && v[1] == 21 && v[2] == 32 && v[3] == 3);
	EXPECT(vt[0] == 0 && vt[1] == 20 && vt[2] == 2 && vt[3] == 3);
	EXPECT(vn[0] == 0 && vn[1] == 1 && vn[2] == 33 && vn[3] == 4);

	num = mesh.ParseFace(" 1/1 22/21 33/3 4/4", v, vt, vn, 32);
	EXPECT(num == 4);
	EXPECT(v[0] == 0 && v[1] == 21 && v[2] == 32 && v[3] == 3);
	EXPECT(vt[0] == 0 && vt[1] == 20 && vt[2] == 2 && vt[3] == 3);

	return;
}

void TestMeshSimplify1()
{
	printf("Running TestMeshSimplify1\n");

	SimplificationConfig cfg;
	cfg.rate = 0.1f;

	StaticMesh mesh;
	mesh.LoadObj("../TestData/bunny.obj");
	mesh.Simplify(cfg);
	mesh.ExportObj("../TestData/bunny2.obj");
}

void TestMeshSimplify2()
{
	printf("Running TestMeshSimplify2\n");

	SimplificationConfig cfg;
	cfg.rate = 0.1f;

	StaticMesh mesh;
	mesh.LoadObj("../TestData/dungeon.obj");
	mesh.Simplify(cfg);
	mesh.ExportObj("../TestData/dungeon2.obj");
}

void TestMeshSimplify3()
{
	SimplificationConfig cfg;
	cfg.faces = 20000;

	StaticMesh mesh;
	mesh.LoadObj("F:/Downloads/simplemesh/car_1.obj");
	mesh.Simplify(cfg);
	mesh.ExportObj("F:/Downloads/simplemesh/car_sim.obj");
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

void TestVoronoi2d()
{
	printf("Running TestVoronoi2d\n");

	std::vector<Vector2> points;
	points.emplace_back(0.0f, 0.0f);
	points.emplace_back(0.0f, -1.0f);
	points.emplace_back(1.0f, -1.0f);
	points.emplace_back(1.0f, -2.0f);
	points.emplace_back(2.0f, -2.0f);
	points.emplace_back(2.0f, -1.0f);
	points.emplace_back(3.0f, -1.0f);
	points.emplace_back(3.0f, 0.0f);
	points.emplace_back(2.0f, 0.0f);
	points.emplace_back(1.0f, 0.0f);

	std::vector<DelaunayEdge> edges;
	edges.emplace_back(0, 1);
	edges.emplace_back(1, 2);
	edges.emplace_back(2, 3);
	edges.emplace_back(3, 4);
	edges.emplace_back(4, 5);
	edges.emplace_back(5, 6);
	edges.emplace_back(6, 7);
	edges.emplace_back(7, 8);
	edges.emplace_back(8, 9);
	edges.emplace_back(9, 0);

	Delaunay d1;
	EXPECT(d1.Triangulate(points));
	EXPECT(d1.Triangles.size() == 10);

	ConstrainedDelaunay d2;
	EXPECT(d2.Triangulate(points, edges));
	EXPECT(d2.Triangles.size() == 8);

	return;
}

void TestVoronoi3d()
{
	printf("Running TestVoronoi3d\n");

	std::vector<Vector3> points;
	Voronoi3::GenerateRandomPoints(Box3::Unit(), 100, points);
	Voronoi3 v(points, Box3::Unit(), 1e-3f);

	std::vector<Voronoi3::Cell> cells;
	v.ComputeAllCells(cells, true);

	Voronoi3::GenerateRandomPoints(Box3::Unit(), 2, points);

	v.Set(points, Box3::Unit(), 1e-3f);
	v.Build();
	return;
}

void TestDynamicMesh()
{
	printf("Running TestDynamicMesh\n");

	AxisAlignedBox3 box1(Vector3(0.0f), Vector3(2.0f));
	AxisAlignedBox3 box2(Vector3(1.0f), Vector3(3.0f));
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	DynamicMesh mesh1;
	box1.GetMesh2(Vertices, Indices, Normals);
	std::swap(Indices[1], Indices[2]);
	std::swap(Indices[4], Indices[5]);
	mesh1.SetData(Vertices, Indices, Normals);
	EXPECT(2 == mesh1.FixTriangleOrientation(true));
	mesh1.CalculateWeightAverageNormals();
}

void TestGeometrySet()
{
	printf("Running TestGeometrySet\n");

	DynamicMesh set1;
	set1.LoadObj("../TestData/bunny.obj");
	Box3 box = set1.GetBounds();
	float x = box.GetCenter().x;
	float y = box.GetCenter().y;

	DynamicMesh set2;
	set2.AppendVertex(Vector3(box.Min.x, y, box.Min.z));
	set2.AppendVertex(Vector3(box.Min.x, y, box.Max.z));
	set2.AppendVertex(Vector3(box.Max.x, y, box.Max.z));
	set2.AppendVertex(Vector3(box.Max.x, y, box.Min.z));
	set2.AppendTriangle(Index3(0, 1, 2));
	set2.AppendTriangle(Index3(2, 3, 0));
	set2.BuildBounds();

	DynamicMesh set3;
	set3.AppendVertex(Vector3(x, box.Min.y, box.Min.z + 0.01f));
	set3.AppendVertex(Vector3(x, box.Min.y, box.Max.z + 0.01f));
	set3.AppendVertex(Vector3(x, box.Max.y, box.Max.z + 0.01f));
	set3.AppendTriangle(Index3(0, 1, 2));
	set3.BuildBounds();

	DynamicMeshAABBTree aabb1(&set1);
	DynamicMeshAABBTree aabb2(&set2);
	DynamicMeshAABBTree aabb3(&set3);

	IntersectionsQueryResult Result;
		
	Result = aabb1.FindAllIntersections(aabb2);
	Result = aabb2.FindAllIntersections(aabb3);

	return;
}


void TestGeometryBoolean1()
{
	printf("Running TestGeometryBoolean1\n");

	AxisAlignedBox3 box1(Vector3(0.0f), Vector3(2.0f));
	AxisAlignedBox3 box2(Vector3(1.0f), Vector3(3.0f));
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	DynamicMesh mesh1;
	box1.GetMesh2(Vertices, Indices, Normals);
	mesh1.SetData(Vertices, Indices, Normals);

	DynamicMesh mesh2;
	box2.GetMesh2(Vertices, Indices, Normals);
	mesh2.SetData(Vertices, Indices, Normals);

	GeometryBoolean b(&mesh1, &mesh2, GeometryBoolean::BooleanOp::Intersect);
	b.Compute();
	b.Result->FixTriangleOrientation(false);
	b.Result->ExportObj("../TestData/box_intersect.obj");

	return;
}

void TestGeometryBoolean2()
{
	printf("Running TestGeometryBoolean2\n");

	DynamicMesh mesh1;
	mesh1.LoadObj("../TestData/bunny.obj");

	DynamicMesh mesh2;
	Box3 box = mesh1.GetBounds();
	box.Max.y = box.GetCenter().y;
	AxisAlignedBox3 aabb(box.Min, box.Max);
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;
	aabb.GetMesh2(Vertices, Indices, Normals);
	mesh2.SetData(Vertices, Indices, Normals);

	GeometryBoolean b(&mesh2, &mesh1, GeometryBoolean::BooleanOp::Intersect);
	b.Compute();
	b.Result->FixTriangleOrientation(false);
	b.Result->ExportObj("../TestData/bunny_intersect.obj");

	return;
}

void TestHashGrid()
{
	printf("Running TestHashGrid\n");

	SparseSpatialHash2<int> s(1.0f, 1024);
	s.Insert(Vector2(0.0f, 0.0f), 1);
	s.Insert(Vector2(0.0f, 1.0f), 2);
	s.Insert(Vector2(1.0f, 1.0f), 3);
	s.Insert(Vector2(1.0f, 0.0f), 4);
	
	int v;
	EXPECT(s.FindNearest(Vector2(1.01f, 1.01f), 0.1f, v));
	EXPECT(v == 3);

	int sum_i = 0;
	s.RangeIteration(Vector2(0.00f, 0.00f), 1.1f, [&](const Vector2 &p, const int& i) { sum_i += i; });
	EXPECT(sum_i == 7);

	return;
}

void TestOctree()
{
	printf("Running TestOctree\n");

	SparseOctree tree(Vector3(0.0f), Vector3(4.0f));

	EXPECT(tree.InsertObject(1, Box3(Vector3(0.0f, 0.0f, 0.0f), 0.1f)));
	EXPECT(tree.InsertObject(2, Box3(Vector3(0.0f, 1.0f, 0.0f), 0.1f)));
	EXPECT(tree.InsertObject(3, Box3(Vector3(1.0f, 0.0f, 0.0f), 0.1f)));
	EXPECT(tree.InsertObject(4, Box3(Vector3(1.0f, 1.0f, 0.0f), 0.1f)));

	std::vector<int> Result;
	tree.RangeQuery(Box3(Vector3(1.0f, 1.0f, 0.0f), 0.1f), Result);
	EXPECT(Result.size() == 4);

	tree.Clear();
	EXPECT(tree.InsertObject(1, Box3(Vector3(0.0f, 0.0f, 0.0f), Vector3(2.0f, 2.0f, 0.0f))));
	EXPECT(tree.InsertObject(2, Box3(Vector3(1.0f, 1.0f, 0.0f), Vector3(3.0f, 3.0f, 0.0f))));
	EXPECT(tree.InsertObject(3, Box3(Vector3(2.0f, 2.0f, 0.0f), Vector3(4.0f, 4.0f, 0.0f))));

	Result.clear();
	tree.RangeQuery(Box3(Vector3(0.0f, 0.0f, 0.0f), 0.1f), Result);

	return;
}

void TestGeometry()
{
	TestHashGrid();
	TestOctree();
	TestParseObj();
	TestMeshSimplify1();
	TestMeshSimplify2();
	TestMeshSimplify3();
	TestClip();
	TestCatmullRom();
	TestVoronoi2d();
	TestVoronoi3d();
	TestDynamicMesh();
	TestGeometrySet();
	TestGeometryBoolean1();
	TestGeometryBoolean2();
}

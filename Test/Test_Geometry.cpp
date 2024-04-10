
#include "Test.h"

#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Geometry/GeometryBoolean.h"
#include "../Src/Geometry/Polygon3.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/Delaunay.h"
#include "../Src/Geometry/Voronoi2.h"
#include "../Src/Geometry/Voronoi3.h"
#include "../Src/Geometry/DenseTensorField3.h"

using namespace Riemann;

void TestMeshSimplify()
{
	DynamicMesh mesh;
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

void TestVoronoi2d()
{
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
	std::vector<Vector3> points;
	Voronoi3::GenerateRandomPoints(Box3::Unit(), 100, points);
	Voronoi3 v(points, Box3::Unit(), 1e-3f);

	std::vector<Voronoi3::Cell> cells;
	v.ComputeAllCells(cells, true);

	Voronoi3::GenerateRandomPoints(Box3::Unit(), 2, points);

	v.Set(points, Box3::Unit(), 1e-3f);
	v.Build();

	VoronoiMesh mesh(points, Box3::Unit(), 1e-3f);


	return;
}

void TestGeometrySet()
{
	DynamicMesh set1;
	set1.LoadObj("../TestData/bunny.obj");
	Box3 box = set1.Bounds;
	float x = box.GetCenter().x;
	float y = box.GetCenter().y;

	DynamicMesh set2;
	set2.VertexPositions.emplace_back(box.Min.x, y, box.Min.z);
	set2.VertexPositions.emplace_back(box.Min.x, y, box.Max.z);
	set2.VertexPositions.emplace_back(box.Max.x, y, box.Max.z);
	set2.VertexPositions.emplace_back(box.Max.x, y, box.Min.z);
	set2.Triangles.emplace_back(0, 1, 2);
	set2.Triangles.emplace_back(2, 3, 0);
	set2.BuildBounds();

	DynamicMesh set3;
	set3.VertexPositions.emplace_back(x, box.Min.y, box.Min.z + 0.01f);
	set3.VertexPositions.emplace_back(x, box.Min.y, box.Max.z + 0.01f);
	set3.VertexPositions.emplace_back(x, box.Max.y, box.Max.z + 0.01f);
	set3.Triangles.emplace_back(0, 1, 2);
	set3.BuildBounds();

	DynamicMeshAABBTree aabb1(&set1);
	DynamicMeshAABBTree aabb2(&set2);
	DynamicMeshAABBTree aabb3(&set3);

	IntersectionsQueryResult Result;
		
	Result = aabb1.FindAllIntersections(aabb2);
	Result = aabb2.FindAllIntersections(aabb3);

	return;
}

void TestGeometryBoolean()
{
	DynamicMesh set1;
	set1.LoadObj("../TestData/bunny.obj");

	Box3 box = set1.Bounds;
	float y = box.GetCenter().y;
	DynamicMesh set2;
	set2.VertexPositions.emplace_back(box.Min.x, y, box.Min.z);
	set2.VertexPositions.emplace_back(box.Min.x, y, box.Max.z);
	set2.VertexPositions.emplace_back(box.Max.x, y, box.Max.z);
	set2.VertexPositions.emplace_back(box.Max.x, y, box.Min.z);
	set2.Triangles.emplace_back(0, 1, 2);
	set2.Triangles.emplace_back(2, 3, 0);
	set2.BuildBounds();

	GeometryBoolean b(&set1, &set2, GeometryBoolean::BooleanOp::Intersect);
	b.Compute();

	return;
}

void TestGeometry()
{
	TestMeshSimplify();
	TestClip();
	TestCatmullRom();
	TestGeometrySet();
	TestGeometryBoolean();
	TestVoronoi2d();
	TestVoronoi3d();
}

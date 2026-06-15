
#include "Test.h"

#include <algorithm>
#include <cstdlib>

#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/OrientedBox3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
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
	mesh.LoadObj(TestDataPath("bunny.obj").c_str());
	mesh.Simplify(cfg);
	mesh.ExportObj(TestOutputPath("bunny2.obj").c_str());
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
	set1.LoadObj(TestDataPath("bunny.obj").c_str());
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


static Matrix3 TestRotationX(float angle)
{
	const float c = (float)std::cos(angle);
	const float s = (float)std::sin(angle);
	return Matrix3(1.0f, 0.0f, 0.0f,
		0.0f, c, -s,
		0.0f, s, c);
}

static Matrix3 TestRotationZ(float angle)
{
	const float c = (float)std::cos(angle);
	const float s = (float)std::sin(angle);
	return Matrix3(c, -s, 0.0f,
		s, c, 0.0f,
		0.0f, 0.0f, 1.0f);
}

static DynamicMesh MakeAABBMesh(const Vector3& bmin, const Vector3& bmax)
{
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	AxisAlignedBox3 box(bmin, bmax);
	box.GetMesh2(Vertices, Indices, Normals);

	DynamicMesh mesh;
	mesh.SetData(Vertices, Indices, Normals);
	return mesh;
}

static DynamicMesh MakeOBBMesh(const Vector3& center, const Vector3& extent, const Matrix3& rotation)
{
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	OrientedBox3 obb(center, extent, rotation);
	AxisAlignedBox3 box(-obb.Extent, obb.Extent);
	box.GetMesh2(Vertices, Indices, Normals);
	for (size_t i = 0; i < Vertices.size(); ++i)
	{
		Vertices[i] = obb.Center + obb.Rotation * Vertices[i];
		Normals[i] = obb.Rotation * Normals[i];
	}

	DynamicMesh mesh;
	mesh.SetData(Vertices, Indices, Normals);
	return mesh;
}

static DynamicMesh MakeSphereMesh(const Vector3& center, float radius)
{
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	Sphere3 sphere(center, radius);
	sphere.GetMesh(Vertices, Indices, Normals);

	DynamicMesh mesh;
	mesh.SetData(Vertices, Indices, Normals);
	return mesh;
}

static DynamicMesh MakeCapsuleMesh(const Vector3& x0, const Vector3& x1, float radius)
{
	std::vector<Vector3> Vertices;
	std::vector<uint16_t> Indices;
	std::vector<Vector3> Normals;

	Capsule3 capsule(x0, x1, radius);
	capsule.GetMesh(Vertices, Indices, Normals);

	DynamicMesh mesh;
	mesh.SetData(Vertices, Indices, Normals);
	return mesh;
}

static float GetBoundsMaxSize(const Box3& bounds)
{
	Vector3 size = bounds.Max - bounds.Min;
	return std::max(size.x, std::max(size.y, size.z));
}

static bool IsPointOnMeshVertex(const DynamicMesh& mesh, const Vector3& point, float tolerance)
{
	const float toleranceSqr = tolerance * tolerance;
	for (int i = 0; i < mesh.GetVertexCount(); ++i)
	{
		if (mesh.IsVertexFast(i) && (mesh.GetVertex(i) - point).SquareLength() <= toleranceSqr)
		{
			return true;
		}
	}
	return false;
}

static bool IntersectionsAvoidInputVertices(DynamicMesh& mesh1, DynamicMesh& mesh2, float tolerance = 1e-5f)
{
	DynamicMeshAABBTree aabb1(&mesh1);
	DynamicMeshAABBTree aabb2(&mesh2);
	IntersectionsQueryResult result = aabb1.FindAllIntersections(aabb2);
	bool hasIntersection = !result.Points.empty() || !result.Segments.empty() || !result.Polygons.empty();
	bool hitInputVertex = false;

	auto testPoint = [&](const Vector3& point)
	{
		if (IsPointOnMeshVertex(mesh1, point, tolerance) || IsPointOnMeshVertex(mesh2, point, tolerance))
		{
			hitInputVertex = true;
		}
	};

	for (const auto& intersection : result.Points)
	{
		testPoint(intersection.Point);
	}
	for (const auto& intersection : result.Segments)
	{
		testPoint(intersection.Point[0]);
		testPoint(intersection.Point[1]);
	}
	for (const auto& intersection : result.Polygons)
	{
		for (int i = 0; i < intersection.Quantity; ++i)
		{
			testPoint(intersection.Point[i]);
		}
	}

	return hasIntersection && !hitInputVertex;
}

static void AppendResultMeshToOutput(
	DynamicMesh& mergedResult,
	const DynamicMesh& resultMesh,
	int caseIndex,
	float outputScale = 1.0f,
	const Vector3& outputPivot = Vector3::Zero(),
	const Vector3& outputCenter = Vector3::Zero())
{
	FDynamicMeshEditor editor(&mergedResult);
	FMeshIndexMappings mappings;
	const Vector3 offset((float)caseIndex * 5.0f, 0.0f, 0.0f);
	editor.AppendMesh(&resultMesh, mappings,
		[offset, outputScale, outputPivot, outputCenter](int, const Vector3& position)
		{
			return outputCenter + (position - outputPivot) * outputScale + offset;
	});
}

static void TestGeometryBooleanAABB(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh aabb = MakeAABBMesh(Vector3(1.0f), Vector3(3.0f));
	EXPECT(IntersectionsAvoidInputVertices(box, aabb));

	GeometryBoolean boxAabbBoolean(&box, &aabb, GeometryBoolean::BooleanOp::Intersect);
	boxAabbBoolean.WeldSharedEdges = false;
	bool boxAabbComputed = boxAabbBoolean.Compute();
	EXPECT(boxAabbComputed);
	EXPECT(boxAabbBoolean.Result != nullptr);
	if (boxAabbComputed && boxAabbBoolean.Result != nullptr)
	{
		boxAabbBoolean.Result->FixTriangleOrientation(false);
		AppendResultMeshToOutput(mergedResult, *boxAabbBoolean.Result, 0);
	}
	delete boxAabbBoolean.Result;
	boxAabbBoolean.Result = nullptr;
}

static void TestGeometryBooleanOBB(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh obb = MakeOBBMesh(Vector3(1.45f, 1.2f, 1.15f), Vector3(0.95f, 0.75f, 0.85f), TestRotationZ(0.43f) * TestRotationX(0.29f));
	EXPECT(IntersectionsAvoidInputVertices(box, obb));

	GeometryBoolean boxObbBoolean(&box, &obb, GeometryBoolean::BooleanOp::Intersect);
	boxObbBoolean.WeldSharedEdges = false;
	bool boxObbComputed = boxObbBoolean.Compute();
	EXPECT(boxObbComputed);
	EXPECT(boxObbBoolean.Result != nullptr);
	if (boxObbComputed && boxObbBoolean.Result != nullptr)
	{
		boxObbBoolean.Result->FixTriangleOrientation(false);
		AppendResultMeshToOutput(mergedResult, *boxObbBoolean.Result, 1);
	}
	delete boxObbBoolean.Result;
	boxObbBoolean.Result = nullptr;
}

static void TestGeometryBooleanSphere(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh sphere = MakeSphereMesh(Vector3(1.23f, 1.07f, 1.31f), 1.18f);
	EXPECT(IntersectionsAvoidInputVertices(box, sphere));

	GeometryBoolean boxSphereBoolean(&box, &sphere, GeometryBoolean::BooleanOp::Intersect);
	boxSphereBoolean.WeldSharedEdges = false;
	bool boxSphereComputed = boxSphereBoolean.Compute();
	EXPECT(boxSphereComputed);
	EXPECT(boxSphereBoolean.Result != nullptr);
	if (boxSphereComputed && boxSphereBoolean.Result != nullptr)
	{
		boxSphereBoolean.Result->FixTriangleOrientation(false);
		AppendResultMeshToOutput(mergedResult, *boxSphereBoolean.Result, 2);
	}
	delete boxSphereBoolean.Result;
	boxSphereBoolean.Result = nullptr;
}

static void TestGeometryBooleanCapsule(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh capsule = MakeCapsuleMesh(Vector3(0.31f, -0.19f, 0.42f), Vector3(2.63f, 2.38f, 1.67f), 0.42f);
	EXPECT(IntersectionsAvoidInputVertices(box, capsule));

	GeometryBoolean boxCapsuleBoolean(&box, &capsule, GeometryBoolean::BooleanOp::Intersect);
	boxCapsuleBoolean.WeldSharedEdges = false;
	bool boxCapsuleComputed = boxCapsuleBoolean.Compute();
	EXPECT(boxCapsuleComputed);
	EXPECT(boxCapsuleBoolean.Result != nullptr);
	if (boxCapsuleComputed && boxCapsuleBoolean.Result != nullptr)
	{
		boxCapsuleBoolean.Result->FixTriangleOrientation(false);
		AppendResultMeshToOutput(mergedResult, *boxCapsuleBoolean.Result, 3);
	}
	delete boxCapsuleBoolean.Result;
	boxCapsuleBoolean.Result = nullptr;
}

static void TestGeometryBooleanBunny(DynamicMesh& mergedResult)
{
	DynamicMesh bunny;
	bool loaded = bunny.LoadObj(TestDataPath("bunny.obj").c_str());
	EXPECT(loaded);
	if (!loaded)
	{
		return;
	}

	Box3 clipBox = bunny.GetBounds();
	clipBox.Max.y = clipBox.GetCenter().y;
	DynamicMesh bunnyClip = MakeAABBMesh(clipBox.Min, clipBox.Max);
	float bunnyOutputScale = 2.0f / GetBoundsMaxSize(bunny.GetBounds());
	EXPECT(IntersectionsAvoidInputVertices(bunnyClip, bunny));
	GeometryBoolean bunnyAabbBoolean(&bunnyClip, &bunny, GeometryBoolean::BooleanOp::Intersect);
	bunnyAabbBoolean.WeldSharedEdges = false;
	bool bunnyAabbComputed = bunnyAabbBoolean.Compute();
	EXPECT(bunnyAabbComputed);
	EXPECT(bunnyAabbBoolean.Result != nullptr);
	if (bunnyAabbComputed && bunnyAabbBoolean.Result != nullptr)
	{
		bunnyAabbBoolean.Result->FixTriangleOrientation(false);
		AppendResultMeshToOutput(mergedResult, *bunnyAabbBoolean.Result, 4,
			bunnyOutputScale, bunny.GetBounds().GetCenter(), Vector3(1.0f, 1.0f, 1.0f));
	}
	delete bunnyAabbBoolean.Result;
	bunnyAabbBoolean.Result = nullptr;
}

void TestGeometryBoolean()
{
	printf("Running TestGeometryBoolean\n");

	DynamicMesh mergedResult;

	TestGeometryBooleanAABB(mergedResult);
	TestGeometryBooleanOBB(mergedResult);
	TestGeometryBooleanSphere(mergedResult);
	TestGeometryBooleanCapsule(mergedResult);
	TestGeometryBooleanBunny(mergedResult);

	mergedResult.BuildBounds();
	EXPECT(mergedResult.ExportObj(TestOutputPath("boolean_intersect.obj").c_str()));

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
	EXPECT(std::find(Result.begin(), Result.end(), 4) != Result.end());

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
	TestClip();
	TestCatmullRom();
	TestVoronoi2d();
	TestVoronoi3d();
	TestDynamicMesh();
	TestGeometrySet();
	TestGeometryBoolean();
}

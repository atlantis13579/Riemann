
#include "Test.h"

#include <algorithm>
#include <cstdlib>
#include <vector>

#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/OrientedBox3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Destruction/Fracture.h"
#include "../Src/Geometry/Spline.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Geometry/GeometryBoolean.h"
#include "../Src/Geometry/MeshCut.h"
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

	DynamicMesh square;
	square.AppendVertex(Vector3(0.0f, 0.0f, 0.0f));
	square.AppendVertex(Vector3(1.0f, 0.0f, 0.0f));
	square.AppendVertex(Vector3(1.0f, 1.0f, 0.0f));
	square.AppendVertex(Vector3(0.0f, 1.0f, 0.0f));
	square.AppendTriangle(0, 1, 2);
	square.AppendTriangle(0, 2, 3);

	EdgeCollapseInfo collapseInfo;
	EXPECT(square.CollapseEdge(1, 0, 0.0f, collapseInfo) == EMeshResult::Ok);
	EXPECT(!square.IsTriangle(0));
	EXPECT(square.IsTriangle(1));
	Index3 collapsedTri = square.GetTriangle(1);
	EXPECT(collapsedTri.Contains(1) && collapsedTri.Contains(2) && collapsedTri.Contains(3));

	DynamicMesh isolatedVertexMesh;
	isolatedVertexMesh.AppendVertex(Vector3(0.0f, 0.0f, 0.0f));
	FDynamicMeshEditor isolatedEditor(&isolatedVertexMesh);
	EXPECT(isolatedEditor.RemoveIsolatedVertices());
	EXPECT(!isolatedVertexMesh.IsVertex(0));

	class CountingAttribute : public DynamicMeshAttributeBase
	{
	public:
		void OnNewVertex(int, bool) override { ++NewVertexCount; }
		void OnNewTriangle(int, bool) override { ++NewTriangleCount; }

		int NewVertexCount = 0;
		int NewTriangleCount = 0;
	};

	DynamicMesh eventMesh;
	eventMesh.EnableAttributes();
	CountingAttribute* Counter = new CountingAttribute();
	eventMesh.Attributes()->AttachAttribute("counter", Counter);
	EXPECT(eventMesh.Attributes()->GetAttachedAttribute("missing") == nullptr);
	EXPECT(eventMesh.Attributes()->NumAttachedAttributes() == 1);
	eventMesh.AppendVertex(Vector3(0.0f, 0.0f, 0.0f));
	EXPECT(Counter->NewVertexCount == 1);
	EXPECT(Counter->NewTriangleCount == 0);

	DynamicMesh weightMesh;
	weightMesh.AppendVertex(Vector3(0.0f, 0.0f, 0.0f));
	weightMesh.AppendVertex(Vector3(1.0f, 0.0f, 0.0f));
	weightMesh.EnableAttributes();
	weightMesh.Attributes()->SetNumWeightLayers(1);
	float Weight = -1.0f;
	weightMesh.Attributes()->GetWeightLayer(0)->GetValue(1, &Weight);
	EXPECT(Weight == 0.0f);

	DynamicMesh uvMesh;
	uvMesh.AppendVertex(Vector3(0.0f, 0.0f, 0.0f));
	uvMesh.AppendVertex(Vector3(1.0f, 0.0f, 0.0f));
	uvMesh.AppendVertex(Vector3(0.0f, 1.0f, 0.0f));
	int TriID = uvMesh.AppendTriangle(0, 1, 2);
	uvMesh.EnableAttributes();
	DynamicMeshUVOverlay* UVs = uvMesh.Attributes()->PrimaryUV();
	int UV0 = UVs->AppendElement(Vector2(0.0f, 0.0f));
	int UV1 = UVs->AppendElement(Vector2(1.0f, 0.0f));
	int UV2 = UVs->AppendElement(Vector2(0.0f, 1.0f));
	EXPECT(UVs->SetTriangle(TriID, Index3(UV0, UV1, UV2)) == EMeshResult::Ok);
	EXPECT(UVs->CountVertexElements(0) == 1);
	EXPECT(UVs->CountVertexElements(0, true) == 1);
	Vector2 VertexUV = UVs->GetElementAtVertex(TriID, 1);
	EXPECT(VertexUV.x == 1.0f && VertexUV.y == 0.0f);
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

static int CountValidTriangles(const DynamicMesh& mesh)
{
	int count = 0;
	for (int tid = 0; tid < mesh.GetTriangleCount(); ++tid)
	{
		if (mesh.IsTriangleFast(tid))
		{
			++count;
		}
	}
	return count;
}

static bool HasRenderablePieces(const std::vector<FracturePiece>& pieces)
{
	for (const FracturePiece& piece : pieces)
	{
		if (CountValidTriangles(piece.Mesh) > 0)
		{
			return true;
		}
	}
	return false;
}

static bool PiecesMoveInPlane(const std::vector<FracturePiece>& pieces, const Vector3& normal)
{
	const Vector3 planeNormal = normal.SafeUnit();
	for (const FracturePiece& piece : pieces)
	{
		if (piece.Direction.SquareLength() > 1e-8f && std::fabs(piece.Direction.Dot(planeNormal)) > 1e-4f)
		{
			return false;
		}
	}
	return true;
}

static bool PiecesUseXYAxisSeparation(const std::vector<FracturePiece>& pieces, const Box3& bounds)
{
	const Vector3 center = bounds.GetCenter();
	const float halfX = std::max((bounds.Max.x - bounds.Min.x) * 0.5f, 1e-3f);
	const float halfY = std::max((bounds.Max.y - bounds.Min.y) * 0.5f, 1e-3f);
	bool foundNonUnitOffset = false;

	for (const FracturePiece& piece : pieces)
	{
		if (CountValidTriangles(piece.Mesh) == 0)
		{
			continue;
		}

		const float expectedX = std::max(-1.0f, std::min((piece.Center.x - center.x) / halfX, 1.0f));
		const float expectedY = std::max(-1.0f, std::min((piece.Center.y - center.y) / halfY, 1.0f));
		if (std::fabs(piece.Direction.x - expectedX) > 1e-3f ||
			std::fabs(piece.Direction.y - expectedY) > 1e-3f ||
			std::fabs(piece.Direction.z) > 1e-4f)
		{
			return false;
		}

		const float length = piece.Direction.Length();
		if (length > 1e-4f && std::fabs(length - 1.0f) > 1e-3f)
		{
			foundNonUnitOffset = true;
		}
	}

	return foundNonUnitOffset;
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

static void AppendBooleanMeshToOutput(
	DynamicMesh& mergedResult,
	const DynamicMesh& sourceMesh,
	int caseIndex,
	int rowIndex,
	float outputScale = 1.0f,
	const Vector3& outputPivot = Vector3::Zero(),
	const Vector3& outputCenter = Vector3::Zero(),
	bool reverseOrientationForOutput = false)
{
	DynamicMesh outputMesh(sourceMesh);
	if (reverseOrientationForOutput)
	{
		outputMesh.ReverseOrientation(false);
	}

	FDynamicMeshEditor editor(&mergedResult);
	FMeshIndexMappings mappings;
	constexpr float CaseSpacing = 2.5f;
	constexpr float RowSpacing = 5.0f;
	const Vector3 offset((float)caseIndex * CaseSpacing, (float)rowIndex * RowSpacing, 0.0f);
	editor.AppendMesh(&outputMesh, mappings,
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
	AppendBooleanMeshToOutput(mergedResult, box, 0, 0, 1.0f, Vector3::Zero(), Vector3::Zero(), true);
	AppendBooleanMeshToOutput(mergedResult, aabb, 0, 1, 1.0f, Vector3::Zero(), Vector3::Zero(), true);

	GeometryBoolean boxAabbBoolean(&box, &aabb, GeometryBoolean::BooleanOp::Intersect);
	boxAabbBoolean.WeldSharedEdges = false;
	bool boxAabbComputed = boxAabbBoolean.Compute();
	EXPECT(boxAabbComputed);
	EXPECT(boxAabbBoolean.Result != nullptr);
	if (boxAabbComputed && boxAabbBoolean.Result != nullptr)
	{
		boxAabbBoolean.Result->FixTriangleOrientation(false);
		boxAabbBoolean.Result->ReverseOrientation(false);
		AppendBooleanMeshToOutput(mergedResult, *boxAabbBoolean.Result, 0, 2);
	}
	delete boxAabbBoolean.Result;
	boxAabbBoolean.Result = nullptr;
}

static void TestGeometryBooleanOBB(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh obb = MakeOBBMesh(Vector3(1.45f, 1.2f, 1.15f), Vector3(0.95f, 0.75f, 0.85f), TestRotationZ(0.43f) * TestRotationX(0.29f));
	EXPECT(IntersectionsAvoidInputVertices(box, obb));
	AppendBooleanMeshToOutput(mergedResult, box, 1, 0, 1.0f, Vector3::Zero(), Vector3::Zero(), true);
	AppendBooleanMeshToOutput(mergedResult, obb, 1, 1, 1.0f, Vector3::Zero(), Vector3::Zero(), true);

	GeometryBoolean boxObbBoolean(&box, &obb, GeometryBoolean::BooleanOp::Intersect);
	boxObbBoolean.WeldSharedEdges = false;
	bool boxObbComputed = boxObbBoolean.Compute();
	EXPECT(boxObbComputed);
	EXPECT(boxObbBoolean.Result != nullptr);
	if (boxObbComputed && boxObbBoolean.Result != nullptr)
	{
		boxObbBoolean.Result->FixTriangleOrientation(false);
		boxObbBoolean.Result->ReverseOrientation(false);
		AppendBooleanMeshToOutput(mergedResult, *boxObbBoolean.Result, 1, 2);
	}
	delete boxObbBoolean.Result;
	boxObbBoolean.Result = nullptr;
}

static void TestGeometryBooleanSphere(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh sphere = MakeSphereMesh(Vector3(1.23f, 1.07f, 1.31f), 1.18f);
	EXPECT(IntersectionsAvoidInputVertices(box, sphere));
	AppendBooleanMeshToOutput(mergedResult, box, 2, 0, 1.0f, Vector3::Zero(), Vector3::Zero(), true);
	AppendBooleanMeshToOutput(mergedResult, sphere, 2, 1);

	GeometryBoolean boxSphereBoolean(&box, &sphere, GeometryBoolean::BooleanOp::Intersect);
	boxSphereBoolean.WeldSharedEdges = false;
	bool boxSphereComputed = boxSphereBoolean.Compute();
	EXPECT(boxSphereComputed);
	EXPECT(boxSphereBoolean.Result != nullptr);
	if (boxSphereComputed && boxSphereBoolean.Result != nullptr)
	{
		boxSphereBoolean.Result->FixTriangleOrientation(false);
		AppendBooleanMeshToOutput(mergedResult, *boxSphereBoolean.Result, 2, 2);
	}
	delete boxSphereBoolean.Result;
	boxSphereBoolean.Result = nullptr;
}

static void TestGeometryBooleanCapsule(DynamicMesh& mergedResult)
{
	DynamicMesh box = MakeAABBMesh(Vector3(0.0f), Vector3(2.0f));
	DynamicMesh capsule = MakeCapsuleMesh(Vector3(0.31f, -0.19f, 0.42f), Vector3(2.63f, 2.38f, 1.67f), 0.42f);
	EXPECT(IntersectionsAvoidInputVertices(box, capsule));
	AppendBooleanMeshToOutput(mergedResult, box, 3, 0, 1.0f, Vector3::Zero(), Vector3::Zero(), true);
	AppendBooleanMeshToOutput(mergedResult, capsule, 3, 1);

	GeometryBoolean boxCapsuleBoolean(&box, &capsule, GeometryBoolean::BooleanOp::Intersect);
	boxCapsuleBoolean.WeldSharedEdges = false;
	bool boxCapsuleComputed = boxCapsuleBoolean.Compute();
	EXPECT(boxCapsuleComputed);
	EXPECT(boxCapsuleBoolean.Result != nullptr);
	if (boxCapsuleComputed && boxCapsuleBoolean.Result != nullptr)
	{
		boxCapsuleBoolean.Result->FixTriangleOrientation(false);
		AppendBooleanMeshToOutput(mergedResult, *boxCapsuleBoolean.Result, 3, 2);
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
	const Vector3 bunnyOutputPivot = bunny.GetBounds().GetCenter();
	const Vector3 bunnyOutputCenter(1.0f, 1.0f, 1.0f);
	EXPECT(IntersectionsAvoidInputVertices(bunnyClip, bunny));
	AppendBooleanMeshToOutput(mergedResult, bunnyClip, 4, 0,
		bunnyOutputScale, bunnyOutputPivot, bunnyOutputCenter, true);
	AppendBooleanMeshToOutput(mergedResult, bunny, 4, 1,
		bunnyOutputScale, bunnyOutputPivot, bunnyOutputCenter);
	GeometryBoolean bunnyAabbBoolean(&bunnyClip, &bunny, GeometryBoolean::BooleanOp::Intersect);
	bunnyAabbBoolean.WeldSharedEdges = false;
	bool bunnyAabbComputed = bunnyAabbBoolean.Compute();
	EXPECT(bunnyAabbComputed);
	EXPECT(bunnyAabbBoolean.Result != nullptr);
	if (bunnyAabbComputed && bunnyAabbBoolean.Result != nullptr)
	{
		bunnyAabbBoolean.Result->FixTriangleOrientation(false);
		AppendBooleanMeshToOutput(mergedResult, *bunnyAabbBoolean.Result, 4, 2,
			bunnyOutputScale, bunnyOutputPivot, bunnyOutputCenter);
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

void TestMeshCutAlgorithms()
{
	printf("Running TestMeshCutAlgorithms\n");

	DynamicMesh wall;
	const bool loaded = wall.LoadObj(TestDataPath("wall_box.obj").c_str());
	EXPECT(loaded);
	if (!loaded)
	{
		return;
	}

	FractureOptions options;
	options.PieceCount = 6;
	options.PiecesX = 4;
	options.PiecesY = 3;
	options.PiecesZ = 2;
	options.Seed = 11;
	options.Normal = Vector3::UnitZ();

	std::vector<FracturePiece> pieces;
	EXPECT(Fracture::VoronoiFracture2D(wall, Vector3::UnitZ(), pieces, options));
	EXPECT(HasRenderablePieces(pieces));
	EXPECT(PiecesMoveInPlane(pieces, Vector3::UnitZ()));
	EXPECT(PiecesUseXYAxisSeparation(pieces, wall.GetBounds()));

	pieces.clear();
	EXPECT(Fracture::ClusterVoronoiFracture(wall, pieces, options));
	EXPECT(HasRenderablePieces(pieces));

	pieces.clear();
	EXPECT(Fracture::Voxel2D(wall, Vector3::UnitZ(), pieces, options));
	EXPECT(HasRenderablePieces(pieces));
	EXPECT(PiecesMoveInPlane(pieces, Vector3::UnitZ()));
	EXPECT(PiecesUseXYAxisSeparation(pieces, wall.GetBounds()));

	const Index2 voxel2DCounts[] =
	{
		Index2(1, 1),
		Index2(2, 2),
		Index2(4, 3),
		Index2(7, 5),
		Index2(10, 6),
	};
	for (const Index2& counts : voxel2DCounts)
	{
		FractureOptions stressOptions = options;
		stressOptions.PiecesX = counts.a;
		stressOptions.PiecesY = counts.b;
		stressOptions.PiecesZ = 1;
		stressOptions.Normal = Vector3::Zero();

		pieces.clear();
		EXPECT(Fracture::Voxel2D(wall, Vector3::Zero(), pieces, stressOptions));
		EXPECT(HasRenderablePieces(pieces));
		EXPECT(PiecesMoveInPlane(pieces, Vector3::UnitZ()));
	}

	pieces.clear();
	EXPECT(Fracture::Voxel3D(wall, pieces, options));
	EXPECT(HasRenderablePieces(pieces));
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
	TestMeshCutAlgorithms();
}

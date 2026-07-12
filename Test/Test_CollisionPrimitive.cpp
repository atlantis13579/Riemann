#include "Test.h"

#include <cmath>

#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/OrientedBox3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

static void ExpectSweepNormalOpposesDirection(const Vector3& normal, const Vector3& direction)
{
	EXPECT(normal.SquareLength() > 0.5f);
	EXPECT(normal.Dot(direction) <= 0.001f);
}

static void AddTriangleToMesh(TriangleMesh& mesh, const Vector3& a, const Vector3& b, const Vector3& c)
{
	const uint32_t base = mesh.NumVertices;
	mesh.AddVertex(a);
	mesh.AddVertex(b);
	mesh.AddVertex(c);
	mesh.AddTriangle(base, base + 1, base + 2);
}

static void InitFlatHeightField(HeightField3& hf, bool makeHole)
{
	hf.Init(Box3(Vector3(-2.0f, -0.1f, -2.0f), Vector3(2.0f, 0.1f, 2.0f)), 3, 3);
	hf.AllocMemory();
	hf.HeightScale = 1.0f;
	for (uint32_t i = 0; i < hf.nX * hf.nZ; ++i)
	{
		hf.Cells[i].HeightSample = 0;
		hf.Cells[i].Tessellation0 = makeHole ? 0x7F : 0;
		hf.Cells[i].Tessellation1 = makeHole ? 0x7F : 0;
	}
}

static void InitConvexBox(ConvexMesh& convex, const Vector3& bmin, const Vector3& bmax)
{
	Vector3 vertices[] =
	{
		Vector3(bmin.x, bmin.y, bmin.z),
		Vector3(bmax.x, bmin.y, bmin.z),
		Vector3(bmax.x, bmax.y, bmin.z),
		Vector3(bmin.x, bmax.y, bmin.z),
		Vector3(bmin.x, bmin.y, bmax.z),
		Vector3(bmax.x, bmin.y, bmax.z),
		Vector3(bmax.x, bmax.y, bmax.z),
		Vector3(bmin.x, bmax.y, bmax.z),
	};
	ConvexMeshFace faces[] =
	{
		ConvexMeshFace(Plane3(Vector3(-1.0f,  0.0f,  0.0f),  bmin.x), 4,  0),
		ConvexMeshFace(Plane3(Vector3( 1.0f,  0.0f,  0.0f), -bmax.x), 4,  4),
		ConvexMeshFace(Plane3(Vector3( 0.0f, -1.0f,  0.0f),  bmin.y), 4,  8),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  1.0f,  0.0f), -bmax.y), 4, 12),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f, -1.0f),  bmin.z), 4, 16),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f,  1.0f), -bmax.z), 4, 20),
	};
	uint8_t indices[] =
	{
		0, 3, 7, 4,
		1, 5, 6, 2,
		0, 4, 5, 1,
		3, 2, 6, 7,
		0, 1, 2, 3,
		4, 7, 6, 5,
	};
	convex.SetConvexData(vertices, 8, faces, 6, nullptr, 0, indices, 24, false);
}

static Matrix3 RotationZ(float angle)
{
	const float c = (float)std::cos(angle);
	const float s = (float)std::sin(angle);
	return Matrix3(c, -s, 0.0f,
		s, c, 0.0f,
		0.0f, 0.0f, 1.0f);
}

static float SupportOffsetAlong(const OrientedBox3& box, const Vector3& direction)
{
	return (box.GetSupport(direction) - box.Center).Dot(direction);
}

static void TestGeometryFactoryAxialShapes()
{
	printf("Running TestGeometryFactoryAxialShapes\n");

	const Vector3 x0(-2.0f, 1.0f, 3.0f);
	const Vector3 x1(2.0f, 4.0f, -1.0f);
	const float expectedHeight = (x1 - x0).Length();

	Geometry* capsuleGeometry = GeometryFactory::CreateCapsule(x0, x1, 0.25f);
	const Capsule3* capsule = capsuleGeometry->GetShapeObj<Capsule3>();
	EXPECT(capsule != nullptr);
	EXPECT_NEAR(capsule->GetHeight(), expectedHeight, 1.0e-5f);
	EXPECT((capsuleGeometry->GetTransform()->LocalToWorld(capsule->X0) - x0).SquareLength() < 1.0e-5f);
	EXPECT((capsuleGeometry->GetTransform()->LocalToWorld(capsule->X1) - x1).SquareLength() < 1.0e-5f);
	GeometryFactory::DeleteGeometry(capsuleGeometry);

	Geometry* cylinderGeometry = GeometryFactory::CreateCylinder(x0, x1, 0.25f);
	const Cylinder3* cylinder = cylinderGeometry->GetShapeObj<Cylinder3>();
	EXPECT(cylinder != nullptr);
	EXPECT_NEAR(cylinder->GetHeight(), expectedHeight, 1.0e-5f);
	EXPECT((cylinderGeometry->GetTransform()->LocalToWorld(cylinder->X0) - x0).SquareLength() < 1.0e-5f);
	EXPECT((cylinderGeometry->GetTransform()->LocalToWorld(cylinder->X1) - x1).SquareLength() < 1.0e-5f);
	GeometryFactory::DeleteGeometry(cylinderGeometry);
}

static void TestOrientedBoxTransforms()
{
	printf("Running TestOrientedBoxTransforms\n");

	const float halfPi = 1.5707963267948966f;
	const Matrix3 rotation = RotationZ(halfPi);
	const Vector3 center(3.0f, -2.0f, 1.0f);
	const Vector3 extent(2.0f, 0.5f, 1.0f);
	const OrientedBox3 box(center, extent, rotation);

	const Box3 bounds = box.CalculateBoundingVolume();
	EXPECT((bounds.Min - Vector3(2.5f, -4.0f, 0.0f)).SquareLength() < 1.0e-5f);
	EXPECT((bounds.Max - Vector3(3.5f, 0.0f, 2.0f)).SquareLength() < 1.0e-5f);

	const Vector3 outsideLocal(3.0f, 0.25f, 0.0f);
	const Vector3 outsideWorld = center + rotation * outsideLocal;
	const Vector3 expectedClosest = center + rotation * Vector3(2.0f, 0.25f, 0.0f);
	EXPECT((box.ClosestPointToPoint(outsideWorld) - expectedClosest).SquareLength() < 1.0e-5f);
	EXPECT_NEAR(box.SqrDistanceToPoint(outsideWorld), 1.0f, 1.0e-5f);
	float lineTime = 0.0f;
	EXPECT_NEAR(box.SqrDistanceToLine(
		center + rotation * Vector3(3.0f, 0.0f, 0.0f), rotation * Vector3::UnitY(), &lineTime), 1.0f, 1.0e-5f);
	EXPECT_NEAR(box.SqrDistanceToSegment(
		center + rotation * Vector3(3.0f, -0.25f, 0.0f),
		center + rotation * Vector3(3.0f, 0.25f, 0.0f)), 1.0f, 1.0e-5f);

	const Vector3 rayOrigin = center + rotation * Vector3(4.0f, 0.0f, 0.0f);
	const Vector3 rayDirection = rotation * -Vector3::UnitX();
	float rayTime = -1.0f;
	EXPECT(box.IntersectRay(rayOrigin, rayDirection, &rayTime));
	EXPECT_NEAR(rayTime, 2.0f, 1.0e-5f);

	EXPECT(box.IntersectSphere(center + rotation * Vector3(2.4f, 0.0f, 0.0f), 0.5f));
	EXPECT_FALSE(box.IntersectSphere(center + rotation * Vector3(2.6f, 0.0f, 0.0f), 0.5f));

	EXPECT(box.IntersectCapsule(
		center + rotation * Vector3(2.4f, -0.2f, 0.0f),
		center + rotation * Vector3(2.4f, 0.2f, 0.0f), 0.5f));
	EXPECT_FALSE(box.IntersectCapsule(
		center + rotation * Vector3(2.6f, -0.2f, 0.0f),
		center + rotation * Vector3(2.6f, 0.2f, 0.0f), 0.5f));

	const Vector3 triangleA = center + rotation * Vector3(1.5f, -0.25f, -0.25f);
	const Vector3 triangleB = center + rotation * Vector3(1.5f, 0.25f, -0.25f);
	const Vector3 triangleC = center + rotation * Vector3(1.5f, 0.0f, 0.25f);
	EXPECT(box.IntersectTriangle(triangleA, triangleB, triangleC));
	EXPECT_FALSE(box.IntersectTriangle(
		center + rotation * Vector3(3.0f, -0.25f, -0.25f),
		center + rotation * Vector3(3.0f, 0.25f, -0.25f),
		center + rotation * Vector3(3.0f, 0.0f, 0.25f)));

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, triangleA, triangleB, triangleC);
	mesh.BuildBVH();
	EXPECT(mesh.IntersectOBB(box.Center, box.Extent, box.Rotation));

	const OrientedBox3 planeCrossingBox(Vector3(0.0f, 0.1f, 0.0f),
		Vector3(0.3f, 0.2f, 0.2f), Matrix3::Identity());
	EXPECT(planeCrossingBox.IntersectTriangle(
		Vector3(-5.0f, 0.0f, -5.0f), Vector3(5.0f, 0.0f, -5.0f), Vector3(-5.0f, 0.0f, 5.0f)));
}

void TestAABBSweepTriangle()
{
	printf("Running TestAABBSweepTriangle\n");

	AxisAlignedBox3 box(Vector3(-0.5f), Vector3(0.5f));
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = box.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepTriangle(-Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);

	t = -1.0f;
	hit = box.SweepTriangle(Vector3::UnitX(),
		Vector3(2.0f, -2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 0.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.x, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitX());

	t = -1.0f;
	hit = box.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 0.0f, -2.0f),
		Vector3(2.0f, 0.0f, -2.0f),
		Vector3(0.0f, 0.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepTriangle(Vector3::UnitX(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);
}

void TestAABBSweepTriangleMesh()
{
	printf("Running TestAABBSweepTriangleMesh\n");

	AxisAlignedBox3 box(Vector3(-0.5f), Vector3(0.5f));
	Vector3 position, normal;
	float t = -1.0f;

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, Vector3(-2.0f, 4.0f, -2.0f), Vector3(2.0f, 4.0f, -2.0f), Vector3(0.0f, 4.0f, 2.0f));
	AddTriangleToMesh(mesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	mesh.BuildBVH();

	bool hit = box.SweepTriangleMesh(Vector3::UnitY(), &mesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh initialMesh;
	initialMesh.Flags = 0;
	AddTriangleToMesh(initialMesh, Vector3(-2.0f, 0.0f, -2.0f), Vector3(2.0f, 0.0f, -2.0f), Vector3(0.0f, 0.0f, 2.0f));
	initialMesh.BuildBVH();
	t = -1.0f;
	hit = box.SweepTriangleMesh(Vector3::UnitY(), &initialMesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh noBvhMesh;
	noBvhMesh.Flags = 0;
	AddTriangleToMesh(noBvhMesh, Vector3(2.0f, -2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(2.0f, 0.0f, 2.0f));
	t = -1.0f;
	hit = box.SweepTriangleMesh(Vector3::UnitX(), &noBvhMesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitX());

	TriangleMesh missMesh;
	missMesh.Flags = 0;
	AddTriangleToMesh(missMesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	missMesh.BuildBVH();
	t = -1.0f;
	hit = box.SweepTriangleMesh(Vector3::UnitX(), &missMesh, &position, &normal, &t);
	EXPECT(!hit);
}

void TestAABBSweepHeightField()
{
	printf("Running TestAABBSweepHeightField\n");

	HeightField3 hf;
	InitFlatHeightField(hf, false);

	AxisAlignedBox3 box(Vector3(-0.25f, 1.0f, -0.25f), Vector3(0.25f, 2.0f, 0.25f));
	Vector3 position, normal;
	float t = -1.0f;
	bool hit = box.SweepHeightField(-Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.0f, 0.001f);
	EXPECT_NEAR(position.y, 0.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, -Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepHeightField(Vector3::UnitX(), &hf, &position, &normal, &t);
	EXPECT(!hit);

	AxisAlignedBox3 overlappingBox(Vector3(-0.25f, -0.1f, -0.25f), Vector3(0.25f, 0.4f, 0.25f));
	t = -1.0f;
	hit = overlappingBox.SweepHeightField(Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	HeightField3 holeHf;
	InitFlatHeightField(holeHf, true);
	t = -1.0f;
	hit = box.SweepHeightField(-Vector3::UnitY(), &holeHf, &position, &normal, &t);
	EXPECT(!hit);
}

void TestAABBSweepPrimitives()
{
	printf("Running TestAABBSweepPrimitives\n");

	AxisAlignedBox3 box(Vector3(1.0f, -0.5f, -0.5f), Vector3(2.0f, 0.5f, 0.5f));
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = box.SweepPlane(Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = box.SweepAABB(Vector3::UnitY(), Vector3(1.25f, -0.25f, -0.25f), Vector3(1.75f, 0.25f, 0.25f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = box.SweepSphere(Vector3::UnitY(), Vector3(1.5f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = box.SweepCylinder(Vector3::UnitY(), Vector3(1.5f, -0.25f, 0.0f), Vector3(1.5f, 0.25f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = box.SweepCapsule(Vector3::UnitY(), Vector3(1.25f, 0.0f, 0.0f), Vector3(1.75f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	ConvexMesh convex;
	InitConvexBox(convex, Vector3(1.25f, -0.25f, -0.25f), Vector3(1.75f, 0.25f, 0.25f));
	hit = box.SweepConvex(Vector3::UnitY(), &convex, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = box.SweepPlane(-Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(!hit);
}

void TestSphereSweep()
{
	printf("Running TestSphereSweep\n");

	Sphere3 sphere(Vector3::Zero(), 0.5f);
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = sphere.SweepPlane(Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = sphere.SweepAABB(Vector3::UnitY(), Vector3(-0.25f), Vector3(0.25f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = sphere.SweepSphere(Vector3::UnitY(), Vector3::Zero(), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = sphere.SweepCylinder(Vector3::UnitY(), Vector3(0.0f, -0.25f, 0.0f), Vector3(0.0f, 0.25f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = sphere.SweepCapsule(Vector3::UnitY(), Vector3(-0.25f, 0.0f, 0.0f), Vector3(0.25f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	ConvexMesh convex;
	InitConvexBox(convex, Vector3(-0.25f), Vector3(0.25f));
	hit = sphere.SweepConvex(Vector3::UnitY(), &convex, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = sphere.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, Vector3(-2.0f, 4.0f, -2.0f), Vector3(2.0f, 4.0f, -2.0f), Vector3(0.0f, 4.0f, 2.0f));
	AddTriangleToMesh(mesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	mesh.BuildBVH();
	hit = sphere.SweepTriangleMesh(Vector3::UnitY(), &mesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	HeightField3 hf;
	InitFlatHeightField(hf, false);
	Sphere3 highSphere(Vector3(0.0f, 2.0f, 0.0f), 0.5f);
	hit = highSphere.SweepHeightField(-Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	EXPECT_NEAR(position.y, 0.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, -Vector3::UnitY());

	HeightField3 holeHf;
	InitFlatHeightField(holeHf, true);
	hit = highSphere.SweepHeightField(-Vector3::UnitY(), &holeHf, &position, &normal, &t);
	EXPECT(!hit);

	hit = sphere.SweepTriangle(-Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);

	hit = sphere.SweepTriangleMesh(Vector3::Zero(), &mesh, &position, &normal, &t);
	EXPECT(!hit);
}

void TestCapsuleSweep()
{
	printf("Running TestCapsuleSweep\n");

	Capsule3 capsule(Vector3(0.0f, -0.5f, 0.0f), Vector3(0.0f, 0.5f, 0.0f), 0.25f);
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = capsule.SweepPlane(Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.25f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = capsule.SweepAABB(Vector3::UnitY(), Vector3(-0.2f), Vector3(0.2f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = capsule.SweepSphere(Vector3::UnitY(), Vector3::Zero(), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = capsule.SweepCylinder(Vector3::UnitY(), Vector3(0.0f, -0.25f, 0.0f), Vector3(0.0f, 0.25f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = capsule.SweepCapsule(Vector3::UnitY(), Vector3(-0.25f, 0.0f, 0.0f), Vector3(0.25f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	ConvexMesh convex;
	InitConvexBox(convex, Vector3(-0.2f), Vector3(0.2f));
	hit = capsule.SweepConvex(Vector3::UnitY(), &convex, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = capsule.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.25f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, Vector3(-2.0f, 4.0f, -2.0f), Vector3(2.0f, 4.0f, -2.0f), Vector3(0.0f, 4.0f, 2.0f));
	AddTriangleToMesh(mesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	mesh.BuildBVH();
	hit = capsule.SweepTriangleMesh(Vector3::UnitY(), &mesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.25f, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	HeightField3 hf;
	InitFlatHeightField(hf, false);
	Capsule3 highCapsule(Vector3(0.0f, 1.0f, 0.0f), Vector3(0.0f, 2.0f, 0.0f), 0.25f);
	hit = highCapsule.SweepHeightField(-Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.75f, 0.001f);
	EXPECT_NEAR(position.y, 0.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, -Vector3::UnitY());

	HeightField3 holeHf;
	InitFlatHeightField(holeHf, true);
	hit = highCapsule.SweepHeightField(-Vector3::UnitY(), &holeHf, &position, &normal, &t);
	EXPECT(!hit);

	hit = capsule.SweepTriangle(-Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);

	hit = capsule.SweepTriangleMesh(Vector3::Zero(), &mesh, &position, &normal, &t);
	EXPECT(!hit);
}

void TestCylinderSweep()
{
	printf("Running TestCylinderSweep\n");

	Cylinder3 cylinder(Vector3(0.0f, -0.5f, 0.0f), Vector3(0.0f, 0.5f, 0.0f), 0.25f);
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = cylinder.SweepPlane(Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	hit = cylinder.SweepAABB(Vector3::UnitY(), Vector3(-0.2f), Vector3(0.2f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = cylinder.SweepSphere(Vector3::UnitY(), Vector3::Zero(), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = cylinder.SweepCylinder(Vector3::UnitY(), Vector3(0.0f, -0.25f, 0.0f), Vector3(0.0f, 0.25f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = cylinder.SweepCapsule(Vector3::UnitY(), Vector3(-0.25f, 0.0f, 0.0f), Vector3(0.25f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	ConvexMesh convex;
	InitConvexBox(convex, Vector3(-0.2f), Vector3(0.2f));
	hit = cylinder.SweepConvex(Vector3::UnitY(), &convex, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	hit = cylinder.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.01f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, Vector3(-2.0f, 4.0f, -2.0f), Vector3(2.0f, 4.0f, -2.0f), Vector3(0.0f, 4.0f, 2.0f));
	AddTriangleToMesh(mesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	mesh.BuildBVH();
	hit = cylinder.SweepTriangleMesh(Vector3::UnitY(), &mesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.5f, 0.01f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	HeightField3 hf;
	InitFlatHeightField(hf, false);
	Cylinder3 highCylinder(Vector3(0.0f, 1.0f, 0.0f), Vector3(0.0f, 2.0f, 0.0f), 0.25f);
	hit = highCylinder.SweepHeightField(-Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 1.0f, 0.01f);
	EXPECT_NEAR(position.y, 0.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, -Vector3::UnitY());

	HeightField3 holeHf;
	InitFlatHeightField(holeHf, true);
	hit = highCylinder.SweepHeightField(-Vector3::UnitY(), &holeHf, &position, &normal, &t);
	EXPECT(!hit);

	hit = cylinder.SweepTriangle(-Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);

	hit = cylinder.SweepTriangleMesh(Vector3::Zero(), &mesh, &position, &normal, &t);
	EXPECT(!hit);
}

void TestOBBSweepPrimitives()
{
	printf("Running TestOBBSweepPrimitives\n");

	const OrientedBox3 box(Vector3::Zero(), Vector3(0.5f), RotationZ(0.7853981633974483f));
	const float expectedT = 2.0f - SupportOffsetAlong(box, Vector3::UnitY());

	Vector3 position, normal;
	float t = -1.0f;
	bool hit = box.SweepPlane(Vector3::UnitY(), Vector3::UnitY(), -2.0f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedT, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepAABB(Vector3::UnitY(), Vector3(-1.0f, 2.0f, -1.0f), Vector3(1.0f, 3.0f, 1.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedT, 0.02f);

	t = -1.0f;
	hit = box.SweepSphere(Vector3::UnitY(), Vector3::Zero(), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	t = -1.0f;
	hit = box.SweepCylinder(Vector3::UnitY(), Vector3(0.0f, -0.25f, 0.0f), Vector3(0.0f, 0.25f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	t = -1.0f;
	hit = box.SweepCapsule(Vector3::UnitY(), Vector3(-0.25f, 0.0f, 0.0f), Vector3(0.25f, 0.0f, 0.0f), 0.25f, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	static Vector3 convexVertices[] =
	{
		Vector3(-0.25f, -0.25f, -0.25f),
		Vector3( 0.25f, -0.25f, -0.25f),
		Vector3( 0.25f,  0.25f, -0.25f),
		Vector3(-0.25f,  0.25f, -0.25f),
		Vector3(-0.25f, -0.25f,  0.25f),
		Vector3( 0.25f, -0.25f,  0.25f),
		Vector3( 0.25f,  0.25f,  0.25f),
		Vector3(-0.25f,  0.25f,  0.25f),
	};
	static ConvexMeshFace convexFaces[] =
	{
		ConvexMeshFace(Plane3(Vector3(-1.0f,  0.0f,  0.0f), -0.25f), 4,  0),
		ConvexMeshFace(Plane3(Vector3( 1.0f,  0.0f,  0.0f), -0.25f), 4,  4),
		ConvexMeshFace(Plane3(Vector3( 0.0f, -1.0f,  0.0f), -0.25f), 4,  8),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  1.0f,  0.0f), -0.25f), 4, 12),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f, -1.0f), -0.25f), 4, 16),
		ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f,  1.0f), -0.25f), 4, 20),
	};
	static uint8_t convexIndices[] =
	{
		0, 3, 7, 4,
		1, 5, 6, 2,
		0, 4, 5, 1,
		3, 2, 6, 7,
		0, 1, 2, 3,
		4, 7, 6, 5,
	};
	ConvexMesh convex;
	convex.SetConvexData(convexVertices, 8, convexFaces, 6, nullptr, 0, convexIndices, 24, true);

	t = -1.0f;
	hit = box.SweepConvex(Vector3::UnitY(), &convex, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);

	t = -1.0f;
	hit = box.SweepAABB(-Vector3::UnitY(), Vector3(-1.0f, 2.0f, -1.0f), Vector3(1.0f, 3.0f, 1.0f), &position, &normal, &t);
	EXPECT(!hit);
}

void TestOBBSweepTriangle()
{
	printf("Running TestOBBSweepTriangle\n");

	const OrientedBox3 box(Vector3::Zero(), Vector3(0.5f), RotationZ(0.7853981633974483f));
	const float expectedY = 2.0f - SupportOffsetAlong(box, Vector3::UnitY());
	const float expectedX = 2.0f - SupportOffsetAlong(box, Vector3::UnitX());
	Vector3 position, normal;
	float t = -1.0f;

	bool hit = box.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedY, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepTriangle(-Vector3::UnitY(),
		Vector3(-2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(0.0f, 2.0f, 2.0f), &position, &normal, &t);
	EXPECT(!hit);

	t = -1.0f;
	hit = box.SweepTriangle(Vector3::UnitX(),
		Vector3(2.0f, -2.0f, -2.0f),
		Vector3(2.0f, 2.0f, -2.0f),
		Vector3(2.0f, 0.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedX, 0.001f);
	EXPECT_NEAR(position.x, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitX());

	t = -1.0f;
	hit = box.SweepTriangle(Vector3::UnitY(),
		Vector3(-2.0f, 0.0f, -2.0f),
		Vector3(2.0f, 0.0f, -2.0f),
		Vector3(0.0f, 0.0f, 2.0f), &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());
}

void TestOBBSweepTriangleMesh()
{
	printf("Running TestOBBSweepTriangleMesh\n");

	const OrientedBox3 box(Vector3::Zero(), Vector3(0.5f), RotationZ(0.7853981633974483f));
	const float expectedT = 2.0f - SupportOffsetAlong(box, Vector3::UnitY());
	Vector3 position, normal;
	float t = -1.0f;

	TriangleMesh mesh;
	mesh.Flags = 0;
	AddTriangleToMesh(mesh, Vector3(-2.0f, 4.0f, -2.0f), Vector3(2.0f, 4.0f, -2.0f), Vector3(0.0f, 4.0f, 2.0f));
	AddTriangleToMesh(mesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	mesh.BuildBVH();

	bool hit = box.SweepTriangleMesh(Vector3::UnitY(), &mesh, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedT, 0.001f);
	EXPECT_NEAR(position.y, 2.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	TriangleMesh missMesh;
	missMesh.Flags = 0;
	AddTriangleToMesh(missMesh, Vector3(-2.0f, 2.0f, -2.0f), Vector3(2.0f, 2.0f, -2.0f), Vector3(0.0f, 2.0f, 2.0f));
	missMesh.BuildBVH();
	t = -1.0f;
	hit = box.SweepTriangleMesh(Vector3::UnitX(), &missMesh, &position, &normal, &t);
	EXPECT(!hit);

	t = -1.0f;
	hit = box.SweepTriangleMesh(Vector3::Zero(), &mesh, &position, &normal, &t);
	EXPECT(!hit);
}

void TestOBBSweepHeightField()
{
	printf("Running TestOBBSweepHeightField\n");

	HeightField3 hf;
	InitFlatHeightField(hf, false);

	const OrientedBox3 box(Vector3(0.0f, 2.0f, 0.0f), Vector3(0.5f), RotationZ(0.7853981633974483f));
	const float expectedT = box.GetSupport(-Vector3::UnitY()).y;
	Vector3 position, normal;
	float t = -1.0f;
	bool hit = box.SweepHeightField(-Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, expectedT, 0.001f);
	EXPECT_NEAR(position.y, 0.0f, 0.05f);
	ExpectSweepNormalOpposesDirection(normal, -Vector3::UnitY());

	t = -1.0f;
	hit = box.SweepHeightField(Vector3::UnitX(), &hf, &position, &normal, &t);
	EXPECT(!hit);

	OrientedBox3 overlappingBox(Vector3::Zero(), Vector3(0.5f), RotationZ(0.7853981633974483f));
	t = -1.0f;
	hit = overlappingBox.SweepHeightField(Vector3::UnitY(), &hf, &position, &normal, &t);
	EXPECT(hit);
	EXPECT_NEAR(t, 0.0f, 0.001f);
	ExpectSweepNormalOpposesDirection(normal, Vector3::UnitY());

	HeightField3 holeHf;
	InitFlatHeightField(holeHf, true);
	t = -1.0f;
	hit = box.SweepHeightField(-Vector3::UnitY(), &holeHf, &position, &normal, &t);
	EXPECT(!hit);

	t = -1.0f;
	hit = box.SweepHeightField(Vector3::Zero(), &hf, &position, &normal, &t);
	EXPECT(!hit);
}

void TestMutableTerrain()
{
	printf("Running TestMutableTerrain\n");

	{
		HeightField3 heightField;
		heightField.Init(Box3(Vector3(-4.0f, -1.0f, -4.0f), Vector3(4.0f, 1.0f, 4.0f)), 9, 9);
		heightField.AllocMemory();
		heightField.HeightScale = 0.01f;
		heightField.RecalculateBounds();
		const uint64_t initialRevision = heightField.GetRevision();
		EXPECT(heightField.SetHeight(0, 0, 1.25f));
		EXPECT_NEAR(heightField.GetHeight(0), 1.25f, 0.001f);
		EXPECT_FALSE(heightField.SetHeight(-1, 0, 0.0f));
		EXPECT(heightField.GetRevision() > initialRevision);

		const int modified = heightField.DigCrater(0.0f, 0.0f, 1.5f, 3.0f, false);
		EXPECT(modified == 9);
		EXPECT_NEAR(heightField.GetHeight(4 + 4 * 9), -3.0f, 0.001f);
		EXPECT_NEAR(heightField.GetHeight(0 + 4 * 9), 0.0f, 0.001f);
		EXPECT_NEAR(heightField.BV.Min.y, -3.0f, 0.001f);
		EXPECT_NEAR(heightField.BV.Max.y, 1.25f, 0.001f);

		HeightFieldHitOption option;
		option.hitBothSides = true;
		option.maxDist = 20.0f;
		HeightFieldHitResult result = { 0 };
		EXPECT(heightField.IntersectRay(Vector3(0.25f, 5.0f, 0.25f), -Vector3::UnitY(), option, &result));
		EXPECT_NEAR(result.hitTime, 8.0f, 0.01f);

		EXPECT(heightField.SetTriangleHole(4, 4, 0, true));
		EXPECT(heightField.SetTriangleHole(4, 4, 1, true));
		EXPECT(heightField.IsTriangleHole(4, 4, 0));
		EXPECT(heightField.IsTriangleHole(4, 4, 1));
		result = HeightFieldHitResult{ 0 };
		EXPECT_FALSE(heightField.IntersectRay(Vector3(0.5f, 5.0f, 0.5f), -Vector3::UnitY(), option, &result));
		std::vector<Vector3> vertices;
		std::vector<uint16_t> indices;
		std::vector<Vector3> normals;
		heightField.GetMesh(vertices, indices, normals);
		EXPECT(vertices.size() == 81);
		EXPECT(indices.size() == 126 * 3);
	}

	{
		PhysicsWorldParam param;
		param.sceneQueryType = SceneQueryType::DynamicAABB;
		PhysicsWorld world(param);
		Geometry* terrain = GeometryFactory::CreateHeightField(
			Box3(Vector3(-4.0f, -1.0f, -4.0f), Vector3(4.0f, 1.0f, 4.0f)), 9, 9);
		HeightField3* heightField = terrain->GetShapeObj<HeightField3>();
		heightField->AllocMemory();
		heightField->HeightScale = 0.01f;
		heightField->RecalculateBounds();
		terrain->SetShapeBounds(heightField->BV);
		RigidBodyParam terrainParam;
		terrainParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(terrain, terrainParam) != nullptr);

		RayCastOption rayOption;
		rayOption.MaxDist = 20.0f;
		RayCastResult rayResult;
		EXPECT(world.GetGeometryQuery()->RayCastQuery(Vector3(0.25f, 5.0f, 0.25f), -Vector3::UnitY(), rayOption, &rayResult));
		EXPECT_NEAR(rayResult.hitTimeMin, 5.0f, 0.01f);

		EXPECT(world.DigHeightFieldHole(terrain, Vector3(0.0f, 0.0f, 0.0f), 1.5f, 2.0f, false) == 9);
		rayResult.Reset();
		EXPECT(world.GetGeometryQuery()->RayCastQuery(Vector3(0.25f, 5.0f, 0.25f), -Vector3::UnitY(), rayOption, &rayResult));
		EXPECT_NEAR(rayResult.hitTimeMin, 7.0f, 0.01f);
		EXPECT(terrain->GetShapeBounds().Min.y <= -2.0f);

		EXPECT(world.SetHeightFieldTriangleHole(terrain, 4, 4, 0, true));
		EXPECT(world.SetHeightFieldTriangleHole(terrain, 4, 4, 1, true));
		rayResult.Reset();
		EXPECT_FALSE(world.GetGeometryQuery()->RayCastQuery(Vector3(0.5f, 5.0f, 0.5f), -Vector3::UnitY(), rayOption, &rayResult));
	}
}

void TestCollisionPrimitive()
{
	TestGeometryFactoryAxialShapes();
	TestOrientedBoxTransforms();
	TestAABBSweepPrimitives();
	TestAABBSweepTriangle();
	TestAABBSweepTriangleMesh();
	TestAABBSweepHeightField();
	TestSphereSweep();
	TestCapsuleSweep();
	TestCylinderSweep();
	TestOBBSweepPrimitives();
	TestOBBSweepTriangle();
	TestOBBSweepTriangleMesh();
	TestOBBSweepHeightField();
}

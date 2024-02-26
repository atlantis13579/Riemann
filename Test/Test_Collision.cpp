
#include "Test.h"

#include "../Src/CollisionPrimitive/AxisAlignedBox3d.h"
#include "../Src/CollisionPrimitive/Plane3d.h"
#include "../Src/CollisionPrimitive/Sphere3d.h"
#include "../Src/CollisionPrimitive/Ray3d.h"
#include "../Src/CollisionPrimitive/OrientedBox3d.h"
#include "../Src/CollisionPrimitive/Triangle3d.h"
#include "../Src/CollisionPrimitive/Cylinder3d.h"
#include "../Src/CollisionPrimitive/Capsule3d.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/CollisionPrimitive/MeshBVH4.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/DynamicAABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"
#include "../Src/Collision/GeometryDifference.h"
#include "../Src/Collision/GJK.h"
#include "../Src/Collision/EPAPenetration.h"
#include "../Src/Maths/Maths.h"

using namespace Riemann;

void TestPlane()
{
	Plane3d plane0, plane1, plane2;
	bool success;
	Vector3 p;

	plane0 = Plane3d(Vector3(6.51947312e-06f, -0.0494406559f, -0.998777092f), -7.89754963f);
	plane1 = Plane3d(Vector3(0.0f, 0.0f, -1.0f), -8.10573006f);
	plane2 = Plane3d(Vector3(7.3720279e-05f, -0.559060335f, -0.829126954f), -4.48326111f);
	success = Plane3d::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(!success);

	plane0 = Plane3d(Vector3::UnitX(), 0.0f);
	plane1 = Plane3d(Vector3::UnitY(), 0.0f);
	plane2 = Plane3d(Vector3::UnitZ(), 0.0f);

	success = Plane3d::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(success);
	EXPECT((p - Vector3::Zero()).SquareLength() < 1e-6f);

	plane0.Shift(0.1f);
	plane1.Shift(0.1f);
	plane2.Shift(0.1f);

	success = Plane3d::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(success);
	EXPECT((p - Vector3(0.1f, 0.1f, 0.1f)).SquareLength() < 1e-6f);
	return;
}

void TestTriangle()
{
	Triangle3d tri1(Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ());
	Vector3 p = Vector3::One();
	Vector3 bc1 = tri1.BaryCentric2D(p);
	Vector3 bc2 = tri1.BaryCentric3D(p);
	(void)bc1;
	(void)bc2;
	EXPECT(!tri1.IntersectPoint(p));
	EXPECT(tri1.IntersectPoint(tri1.A));

	Triangle3d tri2(Vector3(5.09916496f, 8.30379868f, 4.52991295f), Vector3(5.08997154f, 8.29810333f, 4.52174377f), Vector3(5.09456825f, 8.300951f, 4.52582836f));
	Vector3 length = tri2.GetSideLength();
	Vector3 normal = tri2.GetNormal();
	EXPECT(!tri2.IsValid());
	EXPECT(Triangle3d::IsColinear(tri2.A, tri2.B, tri2.C));
	return;
}

void TestSphere()
{
	Vector3 a = Vector3(0, 0, 0);
	Vector3 b = Vector3(0, 1, 0);
	Vector3 c = Vector3(1, 0, 0);
	Vector3 d = Vector3(1, 1, 0);
	Sphere3d s(a, b, c, d);

	EXPECT(s.IntersectPoint(a));
	EXPECT(s.IntersectPoint(b));
	EXPECT(s.IntersectPoint(b));
	EXPECT(s.IntersectPoint(c));
	
	int n = 1000;
	std::vector<Vector3> points;
	for (int i = 0 ;i < n; ++i)
	{
		points.push_back(10.0f * Vector3::Random());
	}
	Sphere3d s1 = Sphere3d::ComputeBoundingSphere_Welzl(points.data(), n);
	for (int i = 0 ;i < n; ++i)
	{
		EXPECT(s1.IntersectPoint(points[i]));
	}
	
	return;
}

void TestDynamicAABB()
{
	printf("Running TestDynamicAABB\n");
	
	DynamicAABBTree tree;
	
	struct Actor
	{
		Actor(Geometry *_geom, int _id)
		{
			p = _geom;
			id = _id;
		}
		Geometry *p;
		int		id;
	};
	
	std::vector<Actor> geoms;
	for (int i = 0; i < 128; ++i)
	{
		Geometry* obb = GeometryFactory::CreateOBB(Vector3::Random() * 10.0f, Vector3::One(), Quaternion::One());
		int id = tree.Add(obb->GetBoundingVolume_WorldSpace(), obb);
		EXPECT(tree.Validate());
		geoms.emplace_back(obb, id);
		
	}
	
	for (int k = 0; k < 1000; ++k)
	{
		for (size_t i = 0; i < geoms.size(); ++i)
		{
			int r = Maths::RandomInt(0, 5);
			if (geoms[i].id == -1)
			{
				if (r == 0)
				{
					geoms[i].id = tree.Add(geoms[i].p->GetBoundingVolume_WorldSpace(), geoms[i].p);
				}
			}
			else
			{
				if (r == 0)
				{
					tree.Remove(geoms[i].id);
					geoms[i].id = -1;
				}
				else
				{
					geoms[i].p->SetWorldPosition(Vector3::Random() * 10.0f);
					geoms[i].p->UpdateBoundingVolume();
					tree.Update(geoms[i].id, geoms[i].p->GetBoundingVolume_WorldSpace(), Vector3::UnitY());
				}
			}
			
			EXPECT(tree.Validate());
		}
	}
	
	tree.Rebuild();
	
	return;
}

void TestSupport()
{
	printf("Running TestSupport\n");
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	
	Vector3 support;

	support = obb2->GetSupport_WorldSpace(Vector3::UnitX());
	EXPECT(fabsf(support.x - 1.0f) < 0.1f);
	
	support = obb2->GetSupport_WorldSpace(-Vector3::UnitX());
	EXPECT(fabsf(support.x + 1.0f) < 0.1f);
	
	support = obb2->GetSupport_WorldSpace(Vector3::UnitY());
	EXPECT(fabsf(support.y - 1.0f) < 0.1f);
	
	obb2->SetWorldPosition(Vector3(5.0f, 0.0f, 0.0f));
	support = obb2->GetSupport_WorldSpace(Vector3::UnitX());
	EXPECT(fabsf(support.x - 6.0f) < 0.1f);
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitZ(), PI_OVER_4);
	obb2->SetWorldRotation(quat);
	support = obb2->GetSupport_WorldSpace(Vector3::UnitX());
	EXPECT(fabsf(support.x - (5.0f + SQRT_2)) < 0.1f);
	support = obb2->GetSupport_WorldSpace(Vector3::UnitY());
	EXPECT(fabsf(support.y - SQRT_2) < 0.1f);
	return;
}

bool GJK_Solve(Geometry *Geom1, Geometry* Geom2)
{
	GeometryDifference shape(Geom1, Geom2);
	GJKIntersection gjk;
	GJK_status gjk_status = gjk.Solve(&shape);
	if (gjk_status == GJK_status::Intersect)
	{
		return true;
	}
	return false;
}

float GJK_Solve_Distance(Geometry* Geom1, Geometry* Geom2)
{
	GeometryDifference shape(Geom1, Geom2);
	GJKClosestDistance gjk;
	return gjk.Solve(&shape);
}

void TestGJK()
{
	printf("Running TestGJK\n");
	
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	EXPECT(GJK_Solve(obb1, obb2));

	obb2->SetWorldPosition(Vector3(0.5f, 0.0f, 0.0f));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetWorldPosition(Vector3(0.0f, 2.1f, 0.0f));
	EXPECT(!GJK_Solve(obb1, obb2));
	
	EXPECT(GJK_Solve(obb1, plane1));
	EXPECT(GJK_Solve(plane1, obb1));
	EXPECT(!GJK_Solve(plane1, obb2));
	EXPECT(!GJK_Solve(obb2, plane1));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitX(), PI_OVER_4);
	obb2->SetWorldRotation(quat);
	EXPECT(GJK_Solve(obb2, obb1));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetWorldPosition(Vector3(0.0f, 1.1f, 0.0f));
	EXPECT(GJK_Solve(obb2, plane1));
	EXPECT(GJK_Solve(plane1, obb2));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	EXPECT(GJK_Solve(sp1, sp2));
	EXPECT(GJK_Solve(sp1, plane1));
	
	sp2->SetWorldPosition(Vector3(0.0f, 5.0f, 0.0f));
	EXPECT(!GJK_Solve(sp1, sp2));
	EXPECT(!GJK_Solve(plane1, sp2));

	float dist = GJK_Solve_Distance(sp1, sp2);
	EXPECT(fabsf(dist - 1.0f) < 0.01f);

	return;
}

void TestEPA()
{
	printf("Running TestEPA\n");

	// Geometry* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, -5.0f, 0.0f), Vector3::UnitY(), 1.0f);
	Geometry* plane1 = GeometryFactory::CreateOBB(Vector3(0.0f, -5.0f, 0.0f), Vector3(100.0f, 1.0f, 100.0f), Quaternion::One());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One() * 1.0f, Quaternion::One());
	obb1->SetWorldPosition(Vector3(0.0f, -3.7f, 0.0f));

	GJKIntersection gjk;
	GJK_status gjk_status;

	EPAPenetration epa;
	EPA_status epa_status;

	GeometryDifference shape(plane1, obb1);

	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);

	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);

	obb1->SetWorldPosition(Vector3(20.0f, -3.7f, 0.0f));
	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);
	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);

	obb1->SetWorldPosition(Vector3(-20.0f, -3.7f, 0.0f));
	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);
	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);
	
	Geometry *sp0 = GeometryFactory::CreateSphere(Vector3(1.0f, 1.0f, 0.0f), 1.5f);
	Geometry *sp1 = GeometryFactory::CreateSphere(Vector3(3.0f, 1.0f, 0.0f), 1.0f);
	shape = GeometryDifference(sp0, sp1);
	EXPECT(gjk.Solve(&shape) == GJK_status::Intersect);
	EXPECT(epa.Solve(gjk.result) != EPA_status::Failed);
	
	Vector3 normal;
	float depth;
	EXPECT(sp0->Penetration(sp1, &normal, &depth));
	
	EXPECT(normal.Dot(epa.penetration_normal) > 0.99f);
	EXPECT(fabsf(depth - epa.penetration_depth) < 0.001f);
	
	return;
}

void TestRTree1()
{
	printf("Running TestRTree1\n");
	TriangleMesh mesh;
	mesh.AddAABB(Vector3(-1, -1, -1), Vector3(1, 1, 1));
	mesh.AddAABB(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
	mesh.Compact();
	mesh.BuildBVH();
	float t;
	bool success;
	success = mesh.IntersectRay(Vector3(0.0f, 10.0f, 0.0f), -Vector3::UnitY(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, 9.0f));

	success = mesh.IntersectRay(Vector3(2.0f, 10.0f, 0.0f), -Vector3::UnitY(), &t);
	EXPECT(!success);

	success = mesh.IntersectRay(Vector3(0.0f, 0.75f, 0.0f), Vector3::UnitY(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, 0.25f));

	success = mesh.IntersectRay(Vector3(0.0f, 0.0f, 0.0f), Vector3(1, 1, 1).Unit(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, sqrtf(3.0f) * 0.5f));

	return;
}

void TestRTree2()
{
	printf("Running TestRTree2\n");
	TriangleMesh mesh;

	bool load_succ = mesh.LoadObj("../TestData/dungeon.obj");
	EXPECT(load_succ);
	if (load_succ)
	{
		mesh.Compact();
		mesh.BuildBVH();

		Vector3 Center;
		Center = (mesh(0, 0) + mesh(0, 1) + mesh(0, 2)) / 3.0f;
		Center.y = 0.0f;

		float t1, t2;
		bool success1, success2;
		Vector3 Dir = Vector3::UnitY();
		success1 = Triangle3d::RayIntersectTriangle(Center, Dir, mesh(0, 0), mesh(0, 1), mesh(0, 2), &t1);
		success2 = mesh.IntersectRay(Center, Dir, &t2);
		EXPECT(success1 == success2);
		EXPECT(Maths::FloatEqual(t1, t2));
	}
	return;
}

void TestAABB()
{
	printf("Running TestAABB\n");
	Box3d box1(Vector3(0, 0, 0), Vector3(10, 10, 10));
	Box3d box2(Vector3(4, 4, 4), Vector3(5, 5, 5));
	bool intersect1 = box1.Intersect(box2);
	bool intersect2 = box2.Intersect(box1);
	EXPECT(intersect1 && intersect2);
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
	
	OrientedBox3d obb = OrientedBox3d::ComputeBoundingOBB_PCA(verties.data(), (int)verties.size());
	
	EXPECT(fabsf(obb.Center.x - center.x) < 0.1f);
	EXPECT(fabsf(obb.Center.y - center.y) < 0.1f);
	EXPECT(fabsf(obb.Center.z - center.z) < 0.1f);
	
	Vector3 dir = obb.Rotation.GetCol(0);
	float dp = dir.Dot(Vector3::UnitX());
	EXPECT(dp >= 0.8f);
	
	return;
}

void TestOBB()
{
	printf("Running TestOBB\n");

	{
		OrientedBox3d obb1(Vector3(0, 0, 0), Vector3(1, 1, 1), Matrix3::Identity());
		OrientedBox3d obb2(Vector3(1, 1, 1), Vector3(5, 4, 4), Matrix3::Identity());
		obb1.Rotation.LoadRotateY(PI_OVER_3);
		obb2.Rotation.LoadRotateX(PI_OVER_3);
		bool intersect = obb1.IntersectOBB(obb2);
		EXPECT(intersect);

		obb2.Center.x = 10.0f;

		intersect = obb1.IntersectOBB(obb2);
		EXPECT(!intersect);
	}

	{
		Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
		Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(2.5f, 0.0, 0.0f), Vector3::One(), Quaternion::One());

		EXPECT(!obb1->Intersect(obb2));

		Quaternion quat;
		quat.FromRotateZ(PI_OVER_4);
		obb2->SetWorldRotation(quat);
		EXPECT(!obb1->Intersect(obb2));

		obb1->SetWorldRotation(quat);
		EXPECT(obb1->Intersect(obb2));
	}

	return;
}

void TestIntersect()
{
	printf("Running TestIntersect\n");
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));

	obb2->SetWorldPosition(Vector3(0.5f, 0.0f, 0.0f));
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));
	
	obb2->SetWorldPosition(Vector3(0.0f, 2.1f, 0.0f));
	EXPECT(!obb1->Intersect(obb2));
	EXPECT(!obb2->Intersect(obb1));
	
	EXPECT(obb1->Intersect(plane1));
	EXPECT(plane1->Intersect(obb1));
	EXPECT(!obb2->Intersect(plane1));
	EXPECT(!plane1->Intersect(obb2));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitX(), PI_OVER_4);
	obb2->SetWorldRotation(quat);
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));
	
	obb2->SetWorldPosition(Vector3(0.0f, 1.1f, 0.0f));
	EXPECT(plane1->Intersect(obb2));
	EXPECT(obb2->Intersect(plane1));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3(1.0f, 0.0, 0.0f), 2.0f);
	EXPECT(sp1->Intersect(sp2));
	EXPECT(sp2->Intersect(sp1));
	EXPECT(sp1->Intersect(plane1));
	EXPECT(plane1->Intersect(sp1));
	
	sp2->SetWorldPosition(Vector3(0.0f, 5.0f, 0.0f));
	EXPECT(!sp1->Intersect(sp2));
	EXPECT(!sp2->Intersect(sp1));
	EXPECT(!plane1->Intersect(sp2));
	EXPECT(!sp2->Intersect(plane1));
}

void TestRayAABB()
{
	printf("Running TestRayAABB\n");
	Vector3 Origin(-100, 0, 50);
	Vector3 Dir(1, 0, 0);
	float t0 = 0, t1 = 0;
	bool success;
	EXPECT(Ray3d::RayIntersectAABB2(Origin, Dir, Vector3::Zero(), Vector3(100, 100, 100), 0.00001f, 100000.0f, &t0, &t1));

	// Vector3 InPos = Origin + Dir * t0;
	// Vector3 OutPos = Origin + Dir * t1;

	// int x0 = (int)(InPos.x / 1.0f);
	// int x1 = (int)(OutPos.x / 1.0f);

	Triangle3d Tri(Vector3(0, 1, 0), Vector3(0, 0, 0), Vector3(1, 0, 0));
	success = Tri.IntersectAABB(Vector3(-1, -1, -1), Vector3(1, 1, 1));
	EXPECT(success);

	success = Tri.IntersectAABB(Vector3(1, 1, 1), Vector3(2, 2, 2));
	EXPECT(!success);

	return;
}

void TestAABBTree()
{
	printf("Running TestAABBTree\n");
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(1, 1, 1));
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(1, 1, 3));
	boxes.emplace_back(Vector3(1, 1, 1), Vector3(1, 1, 2));

	AABBTree tree;

	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), 1);

	tree.Release();
	tree.StaticBuild(param);

	int p = tree.IntersectPoint(Vector3(-1.0f, -1.0f, -1.0f));
	EXPECT(p == -1);
	p = tree.IntersectPoint(Vector3(0.5f, 0.5f, 0.5f));
	EXPECT(p >= 0);
	p = tree.IntersectPoint(Vector3(0.5f, 0.5f, 2.5f));
	EXPECT(p == 1);

	RayCastOption Option;
	RayCastResult Result;
	Ray3d ray(Vector3(0.5f, 0.5f, 100.0f), Vector3(0.0f, 0.0f, -1.0f));
	bool hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	ray.Origin = Vector3(0.5f, 0.5f, 0.5f);
	hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(point1, point1 + point2);
	}

	AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);
	param2.splitter = SplitHeuristic::TreeBalance;
	
	tree.Release();
	tree.StaticBuild(param2);
	TreeStatistics stat;
	tree.Statistic(stat);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3 point = Vector3::Random() * 100.0f;
		p = tree.IntersectPoint(point);

		ray.Origin = point;
		hit = tree.RayCastBoundingBox(ray, Option, &Result);
		if (hit)
		{
			Box3d bb = boxes[hit];
			// float t;
			// EXPECT(ray.IntersectAABB(bb.mMin, bb.mMax, &t));		TODO
		}
	}

	return;
}

void TestGeometryQuery()
{
	printf("Running TestGeometryQuery\n");
	GeometryQuery scene;
	
	std::vector<Geometry*> objs;
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitZ()));
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 10.0f), Vector3::UnitZ()));
	objs.emplace_back(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	RayCastOption option;
	scene.RayCastQuery(Vector3(0.2f, 0.2f, 5.0f), Vector3(0.2f, 0.2f, -1.0f), option , &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 4.0f) < 0.001f);

	scene.RayCastQuery(Vector3(0.0f, 0.0f, -5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(!result.hit);

	scene.RayCastQuery(Vector3(0.0f, 0.0f, 15.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 5.0f) < 0.001f);

	option.Type = RayCastOption::RayCastType::RAYCAST_PENETRATE;
	scene.RayCastQuery(Vector3(0.0f, 0.0f, 15.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(result.hit);
	EXPECT(result.hitGeometries.size() == 3);

	return;
}



class BVProxy2 : public SAP::BoundingVolumeProxy
{
public:
	BVProxy2(std::vector<Box3d>* objs)
	{
		m_objs = objs;
	}

	virtual int	 GetBoundingVolumeCount() const override
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override
	{
		const Box3d& box = m_objs->at(bv_i);
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const override
	{
		const Box3d& box1 = m_objs->at(bv_i);
		const Box3d& box2 = m_objs->at(bv_j);
		return box1.Intersect(box2);
	}

	virtual uint64_t	CalculateBoundingVolumeHash() const override
	{
		return 0;
	}

	std::vector<Box3d>* m_objs;
};

void TestSAP()
{
	printf("Running TestSAP\n");
	std::set<OverlapKey> overlaps;
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(2, 2, 2));
	boxes.emplace_back(Vector3(1, 1, 1), Vector3(3, 3, 3));
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(15, 15, 15));

	BVProxy2 P(&boxes);
	SAP sap(&P, { 0, 1, 2 });

	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 3);

	boxes[2] = Box3d(Vector3(10, 10, 10), Vector3(15, 15, 15));
	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 1);

	for (int i = 0; i < 200; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(point1, point1 + point2);
	}
	sap.Prune(&overlaps);

	for (size_t i = 0; i < boxes.size(); ++i)
	for (size_t j = 0; j < boxes.size(); ++j)
	{
		if (i == j) continue;
		OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
		if (boxes[i].Intersect(boxes[j]))
		{
			EXPECT(overlaps.count(key) == 1);
		}
		else
		{
			EXPECT(overlaps.count(key) == 0);
		}
	}

	return;
}

class BVProxy : public SAP::BoundingVolumeProxy
{
public:
	BVProxy(std::vector<Geometry*>* objs)
	{
		m_objs = objs;
	}

	virtual int	 GetBoundingVolumeCount() const override
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override
	{
		const Box3d& box = m_objs->at(bv_i)->GetBoundingVolume_WorldSpace();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const override
	{
		const Box3d& box1 = m_objs->at(bv_i)->GetBoundingVolume_WorldSpace();
		const Box3d& box2 = m_objs->at(bv_j)->GetBoundingVolume_WorldSpace();
		return box1.Intersect(box2);
	}

	virtual uint64_t	CalculateBoundingVolumeHash() const override
	{
		return 0;
	}

	std::vector<Geometry*>* m_objs;
};

void TestSAPInc()
{
	printf("Running TestSAPInc\n");
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(2.5f, 2.5f, 2.5f), Vector3(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(15, 15, 15), Vector3(5, 5, 5)));

	BVProxy P(&boxes);
	IncrementalSAP sap(&P, { 0, 1, 2 });

	std::set<OverlapKey> overlaps;

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	boxes[2]->SetWorldPosition(boxes[2]->GetWorldPosition() + Vector3(-10, -10, -10));
	boxes[2]->UpdateBoundingVolume();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	boxes[2]->SetWorldPosition(boxes[2]->GetWorldPosition() + Vector3(10, 10, 10));
	boxes[2]->UpdateBoundingVolume();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	for (int i = 0; i < 100; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(GeometryFactory::CreateOBB(point1, point2));
	}
	sap.SetDirty();

	for (int k = 0; k < 10; ++k)
	{
		boxes[2]->SetWorldPosition(boxes[2]->GetWorldPosition() + Vector3::Random() * 20.0f);
		boxes[2]->UpdateBoundingVolume();
		sap.IncrementalPrune(&overlaps);

		for (size_t i = 0; i < boxes.size(); ++i)
		for (size_t j = 0; j < boxes.size(); ++j)
		{
			if (i == j) continue;
			OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
			if (boxes[i]->GetBoundingVolume_WorldSpace().Intersect(boxes[j]->GetBoundingVolume_WorldSpace()))
			{
				EXPECT(overlaps.count(key) == 1);
			}
			else
			{
				EXPECT(overlaps.count(key) == 0);
			}
		}
	}


	return;
}


void TestCollision()
{
	TestPlane();
	TestTriangle();
	TestSphere();
	TestBuildOBB();
	TestOBB();
	TestIntersect();
	TestDynamicAABB();
	TestSupport();
	TestGJK();
	TestEPA();
	TestAABB();
	TestRayAABB();
	TestRTree1();
	TestRTree2();
	TestAABBTree();
	TestGeometryQuery();
	TestSAP();
	TestSAPInc();
	return;
}


#include "Test.h"

#include "../Src/CollisionPrimitive/AxisAlignedBox3d.h"
#include "../Src/CollisionPrimitive/Plane3d.h"
#include "../Src/CollisionPrimitive/Sphere3d.h"
#include "../Src/CollisionPrimitive/Ray3d.h"
#include "../Src/CollisionPrimitive/Triangle3d.h"
#include "../Src/CollisionPrimitive/Cylinder3d.h"
#include "../Src/CollisionPrimitive/Capsule3d.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/CollisionPrimitive/MeshBVH4.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"
#include "../Src/Collision/GeometryDifference.h"
#include "../Src/Collision/GJK.h"
#include "../Src/Collision/EPA.h"

void TestSupport()
{
	printf("Running TestSupport\n");
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	
	Vector3d support;

	support = obb2->GetSupport_WorldSpace(Vector3d::UnitX());
	EXPECT(fabsf(support.x - 1.0f) < 0.1f);
	
	support = obb2->GetSupport_WorldSpace(-Vector3d::UnitX());
	EXPECT(fabsf(support.x + 1.0f) < 0.1f);
	
	support = obb2->GetSupport_WorldSpace(Vector3d::UnitY());
	EXPECT(fabsf(support.y - 1.0f) < 0.1f);
	
	obb2->SetPosition(Vector3d(5.0f, 0.0f, 0.0f));
	support = obb2->GetSupport_WorldSpace(Vector3d::UnitX());
	EXPECT(fabsf(support.x - 6.0f) < 0.1f);
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3d::UnitZ(), PI_OVER_4);
	obb2->SetRotationQuat(quat);
	support = obb2->GetSupport_WorldSpace(Vector3d::UnitX());
	EXPECT(fabsf(support.x - (5.0f + SQRT_2)) < 0.1f);
	support = obb2->GetSupport_WorldSpace(Vector3d::UnitY());
	EXPECT(fabsf(support.y - SQRT_2) < 0.1f);
	return;
}

bool GJK_Solve(Geometry *Geom1, Geometry* Geom2)
{
	GeometryDifference shape(Geom1, Geom2);
	Vector3d guess = shape.GetCenter();

	GJK gjk;
	GJK_status gjk_status = gjk.Solve(&shape, -guess);
	if (gjk_status == GJK_status::Inside)
	{
		return true;
	}
	return false;
}

void TestGJK()
{
	printf("Running TestGJK\n");
	
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 0.0f), Vector3d::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	EXPECT(GJK_Solve(obb1, obb2));

	obb2->SetPosition(Vector3d(0.5f, 0.0f, 0.0f));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetPosition(Vector3d(0.0f, 2.1f, 0.0f));
	EXPECT(!GJK_Solve(obb1, obb2));
	
	EXPECT(GJK_Solve(obb1, plane1));
	EXPECT(GJK_Solve(plane1, obb1));
	EXPECT(!GJK_Solve(plane1, obb2));
	EXPECT(!GJK_Solve(obb2, plane1));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3d::UnitX(), PI_OVER_4);
	obb2->SetRotationQuat(quat);
	EXPECT(GJK_Solve(obb2, obb1));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetPosition(Vector3d(0.0f, 1.1f, 0.0f));
	EXPECT(GJK_Solve(obb2, plane1));
	EXPECT(GJK_Solve(plane1, obb2));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3d(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3d(0.0f, 0.0, 0.0f), 2.0f);
	EXPECT(GJK_Solve(sp1, sp2));
	EXPECT(GJK_Solve(sp1, plane1));
	
	sp2->SetPosition(Vector3d(0.0f, 5.0f, 0.0f));
	EXPECT(!GJK_Solve(sp1, sp2));
	EXPECT(!GJK_Solve(plane1, sp2));

	return;
}

void TestEPA()
{
	printf("Running TestEPA\n");

	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3d(0.0f, -5.0f, 0.0f), Vector3d::UnitY(), 1.0f);
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	obb1->SetPosition(Vector3d(0.0f, -3.7f, 0.0f));
	obb1->UpdateBoundingVolume();

	GeometryDifference shape(plane1, obb1);
	Vector3d guess = shape.GetCenter();

	GJK gjk;
	GJK_status gjk_status = gjk.Solve(&shape, -guess);
	EXPECT(gjk_status == GJK_status::Inside);

	EPA epa;
	EPA_status epa_status = epa.Solve(gjk.simplex, &shape, -guess);
	EXPECT(epa_status == EPA_status::AccuraryReached);

	return;
}

void TestRTree1()
{
	printf("Running TestRTree1\n");
	TriangleMesh mesh;
	mesh.AddAABB(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
	mesh.AddAABB(Vector3d(-0.5f, -0.5f, -0.5f), Vector3d(0.5f, 0.5f, 0.5f));
	mesh.Compact();
	mesh.BuildBVH();
	float t;
	bool success;
	success = mesh.IntersectRay(Vector3d(0.0f, 10.0f, 0.0f), -Vector3d::UnitY(), &t);
	EXPECT(success);
	EXPECT(FloatEqual(t, 9.0f));

	success = mesh.IntersectRay(Vector3d(2.0f, 10.0f, 0.0f), -Vector3d::UnitY(), &t);
	EXPECT(!success);

	success = mesh.IntersectRay(Vector3d(0.0f, 0.75f, 0.0f), Vector3d::UnitY(), &t);
	EXPECT(success);
	EXPECT(FloatEqual(t, 0.25f));

	success = mesh.IntersectRay(Vector3d(0.0f, 0.0f, 0.0f), Vector3d(1, 1, 1).Unit(), &t);
	EXPECT(success);
	EXPECT(FloatEqual(t, sqrtf(3.0f) * 0.5f));

	return;
}

void TestRTree2()
{
	printf("Running TestRTree2\n");
	TriangleMesh mesh;

	bool load_succ = mesh.LoadObj("data/dungeon.obj");
	EXPECT(load_succ);
	if (load_succ)
	{
		mesh.Compact();
		mesh.BuildBVH();

		Vector3d Center;
		Center = (mesh(0, 0) + mesh(0, 1) + mesh(0, 2)) / 3.0f;
		Center.y = 0.0f;

		float t1, t2;
		bool success1, success2;
		Vector3d Dir = Vector3d::UnitY();
		success1 = Triangle3d::RayIntersectTriangle(Center, Dir, mesh(0, 0), mesh(0, 1), mesh(0, 2), &t1);
		success2 = mesh.IntersectRay(Center, Dir, &t2);
		EXPECT(success1 == success2);
		EXPECT(FloatEqual(t1, t2));
	}
	return;
}

void TestOverlap()
{
	printf("Running TestOverlap\n");
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 0.0f), Vector3d::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0, 0.0f), Vector3d::One(), Quaternion::One());
	EXPECT(obb1->Overlap(obb2));
	EXPECT(obb2->Overlap(obb1));

	obb2->SetPosition(Vector3d(0.5f, 0.0f, 0.0f));
	EXPECT(obb1->Overlap(obb2));
	EXPECT(obb2->Overlap(obb1));
	
	obb2->SetPosition(Vector3d(0.0f, 2.1f, 0.0f));
	EXPECT(!obb1->Overlap(obb2));
	EXPECT(!obb2->Overlap(obb1));
	
	EXPECT(obb1->Overlap(plane1));
	EXPECT(plane1->Overlap(obb1));
	EXPECT(!obb2->Overlap(plane1));
	EXPECT(!plane1->Overlap(obb2));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3d::UnitX(), PI_OVER_4);
	obb2->SetRotationQuat(quat);
	EXPECT(obb1->Overlap(obb2));
	EXPECT(obb2->Overlap(obb1));
	
	obb2->SetPosition(Vector3d(0.0f, 1.1f, 0.0f));
	EXPECT(plane1->Overlap(obb2));
	EXPECT(obb2->Overlap(plane1));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3d(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3d(1.0f, 0.0, 0.0f), 2.0f);
	EXPECT(sp1->Overlap(sp2));
	EXPECT(sp2->Overlap(sp1));
	EXPECT(sp1->Overlap(plane1));
	EXPECT(plane1->Overlap(sp1));
	
	sp2->SetPosition(Vector3d(0.0f, 5.0f, 0.0f));
	EXPECT(!sp1->Overlap(sp2));
	EXPECT(!sp2->Overlap(sp1));
	EXPECT(!plane1->Overlap(sp2));
	EXPECT(!sp2->Overlap(plane1));
}

void TestRayAABB()
{
	printf("Running TestRayAABB\n");
	Vector3d Origin(-100, 0, 50);
	Vector3d Dir(1, 0, 0);
	float t0 = 0, t1 = 0;
	bool success;
	EXPECT(Ray3d::RayIntersectAABB2(Origin, Dir, Vector3d::Zero(), Vector3d(100, 100, 100), 0.00001f, 100000.0f, &t0, &t1));

	// Vector3d InPos = Origin + Dir * t0;
	// Vector3d OutPos = Origin + Dir * t1;

	// int x0 = (int)(InPos.x / 1.0f);
	// int x1 = (int)(OutPos.x / 1.0f);

	Triangle3d Tri(Vector3d(0, 1, 0), Vector3d(0, 0, 0), Vector3d(1, 0, 0));
	success = Tri.IntersectAABB(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
	EXPECT(success);

	success = Tri.IntersectAABB(Vector3d(1, 1, 1), Vector3d(2, 2, 2));
	EXPECT(!success);

	return;
}

void TestAABBTree()
{
	printf("Running TestAABBTree\n");
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 3));
	boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(1, 1, 2));

	AABBTree tree;

	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), 1);

	tree.Release();
	tree.StaticBuild(param);

	int p = tree.IntersectPoint(Vector3d(-1.0f, -1.0f, -1.0f));
	EXPECT(p == -1);
	p = tree.IntersectPoint(Vector3d(0.5f, 0.5f, 0.5f));
	EXPECT(p >= 0);
	p = tree.IntersectPoint(Vector3d(0.5f, 0.5f, 2.5f));
	EXPECT(p == 1);

	RayCastOption Option;
	RayCastResult Result;
	Ray3d ray(Vector3d(0.5f, 0.5f, 100.0f), Vector3d(0.0f, 0.0f, -1.0f));
	bool hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	ray.Origin = Vector3d(0.5f, 0.5f, 0.5f);
	hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3d point1 = Vector3d::Random() * 100.0f;
		Vector3d point2 = Vector3d::Random() * 100.0f;
		boxes.emplace_back(point1, point1 + point2);
	}

	AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);

	tree.Release();
	tree.StaticBuild(param2);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3d point = Vector3d::Random() * 100.0f;
		p = tree.IntersectPoint(point);

		ray.Origin = point;
		hit = tree.RayCastBoundingBox(ray, Option, &Result);
		if (hit)
		{
			Box3d bb = boxes[hit];
			// EXPECT(ray.IntersectAABB(bb.Min, bb.Max, &t));	  // TODO
		}
	}

	return;
}

void TestGeometryQuery()
{
	printf("Running TestGeometryQuery\n");
	GeometryQuery scene;
	
	std::vector<Geometry*> objs;
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 0.0f), Vector3d::UnitZ()));
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 10.0f), Vector3d::UnitZ()));
	objs.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	RayCastOption option;
	scene.RayCast(Vector3d(0.2f, 0.2f, 5.0f), Vector3d(0.2f, 0.2f, -1.0f), option , &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 4.0f) < 0.001f);

	scene.RayCast(Vector3d(0.0f, 0.0f, -5.0f), Vector3d(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(!result.hit);

	scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 5.0f) < 0.001f);

	option.Type = RayCastOption::RayCastType::RAYCAST_PENETRATE;
	scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), option, &result);
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

	virtual int	 GetBoundingVolumeCount() const
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const
	{
		const Box3d& box = m_objs->at(bv_i);
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const
	{
		const Box3d& box1 = m_objs->at(bv_i);
		const Box3d& box2 = m_objs->at(bv_j);
		return box1.Intersect(box2);
	}

	std::vector<Box3d>* m_objs;
};

void TestSAP()
{
	printf("Running TestSAP\n");
	std::set<OverlapKey> overlaps;
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(2, 2, 2));
	boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(15, 15, 15));

	BVProxy2 P(&boxes);
	SAP sap(&P, { 0, 1, 2 });

	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 3);

	boxes[2] = Box3d(Vector3d(10, 10, 10), Vector3d(15, 15, 15));
	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 1);

	for (int i = 0; i < 100; ++i)
	{
		Vector3d point1 = Vector3d::Random() * 100.0f;
		Vector3d point2 = Vector3d::Random() * 100.0f;
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

	virtual int	 GetBoundingVolumeCount() const
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const
	{
		const Box3d& box = m_objs->at(bv_i)->GetBoundingVolume_WorldSpace();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const
	{
		const Box3d& box1 = m_objs->at(bv_i)->GetBoundingVolume_WorldSpace();
		const Box3d& box2 = m_objs->at(bv_j)->GetBoundingVolume_WorldSpace();
		return box1.Intersect(box2);
	}

	std::vector<Geometry*>* m_objs;
};

void TestSAPInc()
{
	printf("Running TestSAPInc\n");
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(0.5f, 0.5f, 0.5f), Vector3d(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(2.5f, 2.5f, 2.5f), Vector3d(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(15, 15, 15), Vector3d(5, 5, 5)));

	BVProxy P(&boxes);
	IncrementalSAP sap(&P, { 0, 1, 2 });

	std::set<OverlapKey> overlaps;

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3d(-10, -10, -10));
	boxes[2]->UpdateBoundingVolume();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3d(10, 10, 10));
	boxes[2]->UpdateBoundingVolume();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	for (int i = 0; i < 100; ++i)
	{
		Vector3d point1 = Vector3d::Random() * 100.0f;
		Vector3d point2 = Vector3d::Random() * 100.0f;
		boxes.emplace_back(GeometryFactory::CreateOBB(point1, point2));
	}
	sap.SetDirty();

	for (int k = 0; k < 10; ++k)
	{
		boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3d::Random() * 20.0f);
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
	//TestSupport();
	//TestGJK();
	TestEPA();
	TestOverlap();
	TestRayAABB();
	TestRTree1();
	TestRTree2();
	TestAABBTree();
	TestGeometryQuery();
	TestSAP();
	TestSAPInc();
	return;
}

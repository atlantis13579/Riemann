
#include <assert.h>
#include <string>
#include <vector>

#include "../Src/Tools/SimpleBmp.h"
#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/Maths/Tensor.h"
#include "../Src/Maths/Box3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Matrix2d.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Frustum.h"
#include "../Src/Maths/SIMD.h"
#include "../Src/ImageSpace/ContinuousBitmap.h"
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
#include "../Src/Collision/GJK.h"
#include "../Src/Collision/EPA.h"

#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/SparseVoxelField.h"
#include "../Src/Geometry/DenseTensorField3d.h"
#include "../Src/Tools/PhysxBinaryParser.h"

void TestPhysxBin()
{
	std::vector<Geometry*> collection;
	LoadPhysxBinary("e:/temp/physx/fighting.bin", &collection);

	GeometryQuery query;
	query.BuildStaticGeometry(collection, 1);

	TreeStatistics stat;
	query.GetStaticTree()->Statistic(stat);

	RayCastOption Option;
	RayCastResult Result;

	OverlapOption OOption;
	OverlapResult OResult;

	bool ret = query.RayCast(Vector3d(-569, 0, 427), Vector3d(1, -1, 1).Unit(), Option, &Result);
	assert(ret);
	assert(Result.hitGeom->GetGuid() == 2926462965280);

	Vector3d Pos = Result.hitPoint;

	ret = query.OverlapBox(Vector3d(Pos.x, Pos.y + 15.0f, Pos.z), Vector3d::One(), OOption, &OResult);
	assert(!ret);

	OOption.maxOverlaps = 1;
	ret = query.OverlapBox(Vector3d(Pos.x, Pos.y, Pos.z), 1 * Vector3d::One(), OOption, &OResult);
	assert(ret);
	assert(OResult.overlapGeoms[0]->GetGuid() == 2926462965280);

	ret = query.RayCast(Vector3d(Pos.x + 0.01f, Pos.y - 10.0f, Pos.z + 0.01f), -Vector3d::UnitY(), Option, &Result);
	// assert(!ret);		// TODO filter the world box

	ret = query.RayCast(Vector3d(Pos.x + 0.01f, 0, Pos.z + 0.01f), -Vector3d::UnitY(), Option, &Result);
	assert(ret);
	assert(FloatDiff(Result.hitPoint.y, Pos.y) < 0.01f);
	assert(Result.hitGeom->GetGuid() == 2926462965280);

	ret = query.RayCast(Vector3d(-2222, 0, -773), -Vector3d::UnitY(), Option, &Result);
	assert(ret);
	assert(Result.hitGeom->GetGuid() == 2925373493328);

	ret = query.RayCast(Vector3d(-569, 0, 427), -Vector3d::UnitY(), Option, &Result);
	assert(ret);
	assert(Result.hitGeom->GetGuid() == 2926462965280);

	return;
}

void TestSIMD()
{
	Vec4V v1 = V4Load(1.2f);
	Vec4V v2 = V4Load(1.2f);
	Vec4V v3 = V4Mul(v1, v2);
	Vector3d vv = V4ReadXYZ(v3);
	return;
}

void TestRTree1()
{
	TriangleMesh mesh;
	mesh.AddAABB(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
	mesh.AddAABB(Vector3d(-0.5f, -0.5f, -0.5f), Vector3d(0.5f, 0.5f, 0.5f));
	mesh.Compact();
	mesh.BuildBVH();
	float t;
	bool success;
	success = mesh.IntersectRay(Vector3d(0.0f, 10.0f, 0.0f), -Vector3d::UnitY(), &t);
	assert(success);
	assert(FloatEqual(t, 9.0f));

	success = mesh.IntersectRay(Vector3d(2.0f, 10.0f, 0.0f), -Vector3d::UnitY(), &t);
	assert(!success);

	success = mesh.IntersectRay(Vector3d(0.0f, 0.75f, 0.0f), Vector3d::UnitY(), &t);
	assert(success);
	assert(FloatEqual(t, 0.25f));

	success = mesh.IntersectRay(Vector3d(0.0f, 0.0f, 0.0f), Vector3d(1, 1, 1).Unit(), &t);
	assert(success);
	assert(FloatEqual(t, sqrtf(3.0f) * 0.5f));

	return;
}

void TestRTree2()
{
	TriangleMesh mesh;

	mesh.LoadObj("e:/temp/dungeon.obj");
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
	assert(success1 == success2);
	assert(FloatEqual(t1, t2));
	return;
}

void TestBitmap()
{
	int a[] = { 1, 1, 0, 0, 1, 0, 1, 0,
				1, 1, 1, 1, 1, 1, 1, 1,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 1, 0, 1, 1 };
	ContinuousBitmap<uint16_t> bitmap;
	bitmap.Build<int>(a, 8, 4, -4, -4, 4, 4);
	bitmap.SerializeToFile("E://Temp//cbit.map");
	bitmap.SerializeFromFile("E://Temp//cbit.map");
	assert(bitmap.QueryBitmapSpace(3, 0) == false);
	assert(bitmap.QueryBitmapSpace(4, 0) == true);
	assert(bitmap.QueryBitmapSpace(3, 1) == true);
	assert(bitmap.QueryBitmapSpace(13, 1) == false);
	assert(bitmap.QueryBitmapSpace(7, 3) == true);
}

void TestRayAABB()
{
	Vector3d Origin(-100, 0, 50);
	Vector3d Dir(1, 0, 0);
	float t0, t1;
	bool success;
	assert(Ray3d::RayIntersectAABB2(Origin, Dir, Vector3d::Zero(), Vector3d(100, 100, 100), 0.00001f, 100000.0f, &t0, &t1));

	Vector3d InPos = Origin + Dir * t0;
	Vector3d OutPos = Origin + Dir * t1;

	int x0 = (int)(InPos.x / 1.0f);
	int x1 = (int)(OutPos.x / 1.0f);

	Triangle3d Tri(Vector3d(0, 1, 0), Vector3d(0, 0, 0), Vector3d(1, 0, 0));
	success = Tri.IntersectAABB(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
	assert(success);

	success = Tri.IntersectAABB(Vector3d(1, 1, 1), Vector3d(2, 2, 2));
	assert(!success);

	return;
}

void TestAABBTree()
{
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 3));
	boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(1, 1, 2));

	AABBTree tree;

	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), 1);

	tree.Release();
	tree.StaticBuild(param);

	int p = tree.IntersectPoint(Vector3d(-1.0f, -1.0f, -1.0f));
	assert(p == -1);
	p = tree.IntersectPoint(Vector3d(0.5f, 0.5f, 0.5f));
	assert(p >= 0);
	p = tree.IntersectPoint(Vector3d(0.5f, 0.5f, 2.5f));
	assert(p == 1);

	RayCastOption Option;
	RayCastResult Result;
	Ray3d ray(Vector3d(0.5f, 0.5f, 100.0f), Vector3d(0.0f, 0.0f, -1.0f));
	bool hit = tree.RayCastBoundingBox(ray, Option, &Result);
	assert(hit);

	ray.Origin = Vector3d(0.5f, 0.5f, 0.5f);
	hit = tree.RayCastBoundingBox(ray, Option, &Result);
	assert(hit);

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
			// assert(ray.IntersectAABB(bb.Min, bb.Max, &t));	  // TODO
		}
	}

	return;
}

void TestTensor()
{
	Tensor<float, 4> t(10, 20, 30, 40);
	t(0, 0, 0, 0) = 1.0f;
	t(9, 19, 29, 39) = 2.0f;

	auto t2 = t[9][19][29];

	assert(t2[39] == 2.0f);
}

void TestGeometryQuery()
{
	GeometryQuery scene;
	
	std::vector<Geometry*> objs;
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 0.0f), Vector3d::UnitZ()));
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d(0.0f, 0.0f, 10.0f), Vector3d::UnitZ()));
	objs.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	RayCastOption option;
	scene.RayCast(Vector3d(0.2f, 0.2f, 5.0f), Vector3d(0.2f, 0.2f, -1.0f), option , &result);
	assert(result.hit);
	assert(fabsf(result.hitTime - 4.0f) < 0.001f);

	scene.RayCast(Vector3d(0.0f, 0.0f, -5.0f), Vector3d(0.0f, 0.0f, -1.0f), option, &result);
	assert(!result.hit);

	scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), option, &result);
	assert(result.hit);
	assert(fabsf(result.hitTime - 5.0f) < 0.001f);

	for (auto obj : objs)
	{
		delete obj;
	}

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
	std::set<OverlapKey> overlaps;
	std::vector<Box3d> boxes;
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(2, 2, 2));
	boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(15, 15, 15));

	BVProxy2 P(&boxes);
	SAP sap(&P, { 0, 1, 2 });

	sap.Prune(&overlaps);
	assert(overlaps.size() == 3);

	boxes[2] = Box3d(Vector3d(10, 10, 10), Vector3d(15, 15, 15));
	sap.Prune(&overlaps);
	assert(overlaps.size() == 1);

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
			assert(overlaps.count(key) == 1);
		}
		else
		{
			assert(overlaps.count(key) == 0);
		}
	}

	return;
}

void TestMesh1()
{
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

	field.SerializeTo("e://temp//test.voxel");

	SparseVoxelField inference;
	inference.SerializeFrom("e://temp//test.voxel");

	Box3d v = field.GetVoxelBox(Vector3d(-0.1f, -0.1f, -0.1f));
	assert(FloatEqual(v.Min.x, -1.0f));
	assert(FloatEqual(v.Max.x, 0.0f));

	v = field.GetVoxelBox(Vector3d(0.01f, 0.01f, 0.01f));
	assert(FloatEqual(v.Min.x, 0.0f));
	assert(FloatEqual(v.Max.x, 1.0f));

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
	assert(space == 2);
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
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(0.5f, 0.5f, 0.5f), Vector3d(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(2.5f, 2.5f, 2.5f), Vector3d(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d(15, 15, 15), Vector3d(5, 5, 5)));

	BVProxy P(&boxes);
	IncrementalSAP sap(&P, { 0, 1, 2 });

	std::set<OverlapKey> overlaps;

	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 0);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3d(-10, -10, -10));
	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 2);

	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 2);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3d(10, 10, 10));
	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 0);

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
		sap.IncrementalPrune(&overlaps);

		for (size_t i = 0; i < boxes.size(); ++i)
		for (size_t j = 0; j < boxes.size(); ++j)
		{
			if (i == j) continue;
			OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
			if (boxes[i]->GetBoundingVolume_WorldSpace().Intersect(boxes[j]->GetBoundingVolume_WorldSpace()))
			{
				assert(overlaps.count(key) == 1);
			}
			else
			{
				assert(overlaps.count(key) == 0);
			}
		}

	}


	return;
}

void TestMainEntry()
{
	TestRayAABB();
	TestPhysxBin();
	// TestSIMD();
	// TestRTree1();
	TestRTree2();
	// TestBitmap();
	TestAABBTree();
	TestGeometryQuery();
	TestTensor();
	TestSAP();
	TestSAPInc();
	TestMesh1();
	return;
}

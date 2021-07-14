
#include <assert.h>
#include <string>
#include <vector>

#include "simple_bmp.h"
#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/Maths/Box3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Matrix2d.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Frustum.h"
#include "../Src/CollisionPrimitive/OrientedBox3d.h"
#include "../Src/CollisionPrimitive/Plane3d.h"
#include "../Src/CollisionPrimitive/Sphere3d.h"
#include "../Src/CollisionPrimitive/Ray3d.h"
#include "../Src/CollisionPrimitive/Triangle3d.h"
#include "../Src/CollisionPrimitive/Cylinder3d.h"
#include "../Src/CollisionPrimitive/Capsule3d.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"
#include "../Src/Collision/GJK.h"
#include "../Src/Collision/EPA.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Geometry/SparseVoxelField.h"

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

	int p = tree.Traverse(Vector3d(-1.0f, -1.0f, -1.0f));
	assert(p == -1);
	p = tree.Traverse(Vector3d(0.5f, 0.5f, 0.5f));
	assert(p >= 0);
	p = tree.Traverse(Vector3d(0.5f, 0.5f, 2.5f));
	assert(p == 1);

	float t;
	Ray3d ray(Vector3d(0.5f, 0.5f, 100.0f), Vector3d(0.0f, 0.0f, -1.0f));
	int hit = tree.RayCastBoundingBox(ray, &t);
	assert(hit >= 0);
	Vector3d hit_pos = ray.PointAt(t);
	
	ray.Origin = Vector3d(0.5f, 0.5f, 0.5f);
	hit = tree.RayCastBoundingBox(ray, &t);
	assert(hit >= 0);

	hit_pos = ray.PointAt(t);

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
		p = tree.Traverse(point);

		ray.Origin = point;
		hit = tree.RayCastBoundingBox(ray, &t);
		if (hit >= 0)
		{
			Box3d bb = boxes[hit];
			// assert(ray.IntersectAABB(bb.Min, bb.Max, &t));	  // TODO
		}
	}

	return;
}

void TestGeometryQuery()
{
	GeometryQuery scene;
	
	std::vector<Geometry*> objs;
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d::Zero(), Vector3d::UnitZ(), 0.0f));
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3d::Zero(), Vector3d::UnitZ(), -10.0f));
	objs.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(-1, -1, -1), Vector3d(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	scene.RayCast(Vector3d(0.0f, 0.0f, 5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
	assert(result.hit);
	assert(fabsf(result.hitTime - 4.0f) < 0.001f);

	scene.RayCast(Vector3d(0.0f, 0.0f, -5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
	assert(!result.hit);

	scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
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
	TriangleMesh mesh;
	mesh.AddVertex(Vector3d(-1, 1, -1));
	mesh.AddVertex(Vector3d(1, 1, -1));
	mesh.AddVertex(Vector3d(-1, 1, 1));
	mesh.AddVertex(Vector3d(1, 1, 1));
	mesh.AddVertex(Vector3d(-1, -1, -1));
	mesh.AddVertex(Vector3d(1, -1, -1));
	mesh.AddVertex(Vector3d(-1, -1, 1));
	mesh.AddVertex(Vector3d(1, -1, 1));
	mesh.AddTriangle(0, 1, 2);
	mesh.AddTriangle(3, 1, 2);
	mesh.AddTriangle(4, 5, 6);
	mesh.AddTriangle(7, 5, 6);

	mesh.AddTriangle(0, 1, 4);
	mesh.AddTriangle(5, 1, 4);
	mesh.AddTriangle(1, 3, 5);
	mesh.AddTriangle(7, 3, 5);
	mesh.AddTriangle(0, 2, 4);
	mesh.AddTriangle(6, 2, 4);
	mesh.AddTriangle(2, 3, 6);
	mesh.AddTriangle(7, 3, 6);

	mesh.CalculateBoundingBox();

	VoxelField field;
	field.InitField(Box3d::Unit(), 2, 2, 2, 1.0f, 2.0f);
	field.AddVoxel(0, 1, 2, 0);
	field.AddVoxel(0, 3, 5, 0);
	field.AddVoxel(0, 7, 8, 0);
	field.AddVoxel(0, 1, 10, 0);

	field.SerializeTo("D://home//test.voxel");

	SparseVoxelField inference;
	inference.SerializeFrom("D://home//test.voxel");

	Box3d v = field.GetVoxelBox(Vector3d(-0.1f, -0.1f, -0.1f));
	assert(FloatEqual(v.Min.x, -1.0f));
	assert(FloatEqual(v.Max.x, 0.0f));

	v = inference.GetVoxelBox(Vector3d(-0.1f, -0.1f, -0.1f));
	assert(FloatEqual(v.Min.x, -1.0f));
	assert(FloatEqual(v.Max.x, 0.0f));
	
	v = field.GetVoxelBox(Vector3d(0.01f, 0.01f, 0.01f));
	assert(FloatEqual(v.Min.x, 0.0f));
	assert(FloatEqual(v.Max.x, 1.0f));

	VoxelizationInfo info;
	info.BV.Min = Vector3d(-2, -2, -2);
	info.BV.Max = Vector3d(2, 2, 2);
	info.VoxelHeight = 0.5f;
	info.VoxelSize = 0.5f;

	field.VoxelizeTriangles(info, &mesh);
	field.MakeComplementarySet();
	int space = field.SolveSpatialTopology();
	assert(space == 2);
	return;
}

void TestMesh()
{
	VoxelField field;

	const int load_voxel = 1;
	if (load_voxel)
	{
		field.SerializeFrom("D://home//fighting.voxel");
	}
	else
	{
		TriangleMesh mesh;
		// mesh.LoadObj("D:/src/client/tools/RecastEditor/RecastDemo/Release/Meshes/dungeon.obj");
		// std::string file = "D:/src/client/tools/RecastEditor/RecastDemo/Release/Meshes/fighting.obj";
		// mesh.LoadObj("D:/src/client/tools/RecastEditor/RecastDemo/Release/Meshes/fighting_kou.obj");
		mesh.LoadFlat("D://home//fighting.flat");
		mesh.CalculateBoundingBox();

		/*
		int nFiltered = mesh.FilterTriangle([](const Vector3d& a, const Vector3d& b, const Vector3d& c) -> bool {
			if (fabsf(a.y + 102.0f) < 1.0f && fabsf(b.y + 102.0f) < 1.0f && fabsf(c.y + 102.0f) < 1.0f)
			{
				return true;
			}
			return false;
			});
		*/

		VoxelizationInfo info;
		info.BV.Min = mesh.BoundingBox.GetCenter() - mesh.BoundingBox.GetExtent() * 0.75;
		info.BV.Max = mesh.BoundingBox.GetCenter() + mesh.BoundingBox.GetExtent() * 0.75;
		info.VoxelHeight = 1.f;
		info.VoxelSize = 1.f;

		field.VoxelizeTriangles(info, &mesh);
		field.MakeComplementarySet();

		// field.SerializeTo("D://home//fighting.voxel");
		// field.SerializeFrom("D://home//fighting.voxel");
	}

	std::map<int, unsigned long long> volumes;
	int space = field.SolveSpatialTopology(&volumes);
	// field.FilterByData(4);
	// printf("space = %d\n", space);

	while (0)
	{
		printf("begin\n");
		int a, b, c;
		scanf("%d %d %d", &a, &b, &c);
		const Voxel* v = field.GetVoxel(Vector3d(a * 1.0f, b * 1.0f, c * 1.0f));
		printf("x=%d, y=%d, z=%d\n", a, b, c);
		while (v)
		{
			printf("[%1.f, %.1f] data=%d\n", field.GetVoxelY(v->ymin), field.GetVoxelY(v->ymax), v->data);
			v = v->next;
		}	
	}

	std::vector<int> levels;
	field.ExtractCutPlane(-100, levels);

	for (size_t i = 0; i < levels.size(); ++i)
	{
		int val = levels[i];
		if (val == 4)
			levels[i] = 1;
		else if (val == 36)
			levels[i] = 5;
		else
			levels[i] = 10;
	}

	BMPFile bitmap;
	bitmap.LoadBitmap(&levels[0], field.GetSizeX(), field.GetSizeZ(), 0.1f);
	bitmap.WriteToFile("D://home//fighting3.bmp");


	unsigned long long memory1 = field.EstimateMemoryUseage();
	unsigned long long memory2 = field.EstimateMemoryUseageEx();

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
		const Box3d& box = m_objs->at(bv_i)->GetBoundingBoxWorld();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const
	{
		const Box3d& box1 = m_objs->at(bv_i)->GetBoundingBoxWorld();
		const Box3d& box2 = m_objs->at(bv_j)->GetBoundingBoxWorld();
		return box1.Intersect(box2);
	}

	std::vector<Geometry*>* m_objs;
};

void TestSAPInc()
{
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(0, 0, 0), Vector3d(1, 1, 1)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(2, 2, 2), Vector3d(3, 3, 3)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), Vector3d(10, 10, 10), Vector3d(20, 20, 20)));

	BVProxy P(&boxes);
	IncrementalSAP sap(&P, { 0, 1, 2 });

	std::set<OverlapKey> overlaps;

	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 0);

	boxes[2]->SetPositionOffset(Vector3d(-10, -10, -10));
	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 2);

	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 2);

	boxes[2]->SetPositionOffset(Vector3d(10, 10, 10));
	sap.IncrementalPrune(&overlaps);
	assert(overlaps.size() == 0);

	for (int i = 0; i < 100; ++i)
	{
		Vector3d point1 = Vector3d::Random() * 100.0f;
		Vector3d point2 = point1 + Vector3d::Random() * 100.0f;
		boxes.emplace_back(GeometryFactory::CreateOBB(Vector3d::Zero(), point1, point2));
	}
	sap.SetDirty();

	for (int k = 0; k < 10; ++k)
	{
		boxes[2]->SetPositionOffset(Vector3d::Random() * 20.0f);
		sap.IncrementalPrune(&overlaps);

		for (size_t i = 0; i < boxes.size(); ++i)
		for (size_t j = 0; j < boxes.size(); ++j)
		{
			if (i == j) continue;
			OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
			if (boxes[i]->GetBoundingBoxWorld().Intersect(boxes[j]->GetBoundingBoxWorld()))
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
	TestAABBTree();
	TestGeometryQuery();
	TestSAP();
	TestSAPInc();
	TestMesh1();
	TestMesh();
	return;
}

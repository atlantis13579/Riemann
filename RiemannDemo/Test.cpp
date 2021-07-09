
#include <assert.h>
#include <vector>

#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/Maths/BoundingBox3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Transform.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox.h"
#include "../Src/CollisionPrimitive/Plane.h"
#include "../Src/CollisionPrimitive/Sphere.h"
#include "../Src/CollisionPrimitive/Ray.h"
#include "../Src/CollisionPrimitive/Triangle.h"
#include "../Src/CollisionPrimitive/Cylinder.h"
#include "../Src/CollisionPrimitive/Capsule.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"

void TestAABBTree()
{
	std::vector<BoundingBox3d> boxes;
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
	Ray ray(Vector3d(0.5f, 0.5f, 100.0f), Vector3d(0.0f, 0.0f, -1.0f));
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
			BoundingBox3d bb = boxes[hit];
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
	objs.emplace_back(GeometryFactory::CreateAABB(Vector3d::Zero(), Vector3d(-1, -1, -1), Vector3d(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	scene.RayCast(Vector3d(0.0f, 0.0f, 5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
	assert(result.hit);
	assert(fabsf(result.t - 4.0f) < 0.001f);

	scene.RayCast(Vector3d(0.0f, 0.0f, -5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
	assert(!result.hit);

	scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
	assert(result.hit);
	assert(fabsf(result.t - 5.0f) < 0.001f);

	for (auto obj : objs)
	{
		delete obj;
	}

	return;
}

class BVProxy2 : public SAP::BoundingVolumeProxy
{
public:
	BVProxy2(std::vector<BoundingBox3d>* objs)
	{
		m_objs = objs;
	}

	virtual int	 GetBoundingVolumeCount() const
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const
	{
		const BoundingBox3d& box = m_objs->at(bv_i);
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const
	{
		const BoundingBox3d& box1 = m_objs->at(bv_i);
		const BoundingBox3d& box2 = m_objs->at(bv_j);
		return box1.Intersect(box2);
	}

	std::vector<BoundingBox3d>* m_objs;
};

void TestSAP()
{
	std::set<OverlapKey> overlaps;
	std::vector<BoundingBox3d> boxes;
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(2, 2, 2));
	boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
	boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(15, 15, 15));

	BVProxy2 P(&boxes);
	SAP sap(&P, { 0, 1, 2 });

	sap.Prune(&overlaps);
	assert(overlaps.size() == 3);

	boxes[2] = BoundingBox3d(Vector3d(10, 10, 10), Vector3d(15, 15, 15));
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
		const BoundingBox3d& box = m_objs->at(bv_i)->GetBoundingBoxWorld();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const
	{
		const BoundingBox3d& box1 = m_objs->at(bv_i)->GetBoundingBoxWorld();
		const BoundingBox3d& box2 = m_objs->at(bv_j)->GetBoundingBoxWorld();
		return box1.Intersect(box2);
	}

	std::vector<Geometry*>* m_objs;
};

void TestSAPInc()
{
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateAABB(Vector3d::Zero(), Vector3d(0, 0, 0), Vector3d(1, 1, 1)));
	boxes.emplace_back(GeometryFactory::CreateAABB(Vector3d::Zero(), Vector3d(2, 2, 2), Vector3d(3, 3, 3)));
	boxes.emplace_back(GeometryFactory::CreateAABB(Vector3d::Zero(), Vector3d(10, 10, 10), Vector3d(20, 20, 20)));

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
		boxes.emplace_back(GeometryFactory::CreateAABB(Vector3d::Zero(), point1, point2));
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
	return;
}

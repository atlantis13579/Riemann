
#include "AABBTree.h"

#include <assert.h>

#include "AABBTreeOffline.h"
#include "AABBTreeInference.h"
#include "GeometryObject.h"

AABBTree::AABBTree()
{
	m_PrimitiveIndicesBase = nullptr;
	m_NumPrimitives = 0;
	m_AABBTreeInference = nullptr;
}

AABBTree::~AABBTree()
{
	Release();
}

void AABBTree::Release()
{
	m_NumPrimitives = 0;
	if (m_PrimitiveIndicesBase)
	{
		delete[]m_PrimitiveIndicesBase;
		m_PrimitiveIndicesBase = nullptr;
	}
	if (m_AABBTreeInference)
	{
		delete[]m_AABBTreeInference;
		m_AABBTreeInference = nullptr;
	}
}

void AABBTree::StaticBuild(AABBTreeBuildData& params)
{
	InitAABBTreeBuild(params);

	params.pAABBTree->Build(params);

	params.Release();

	assert(params.pAABBTree);
	m_AABBTreeInference = params.pAABBTree->BuildCompactTree();

	delete params.pAABBTree;
	params.pAABBTree = nullptr;
}

int AABBTree::Traverse(const Vector3d& Point) const
{
	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !p->BV.IsInside(Point))
	{
		return -1;
	}

	while (p)
	{
		if (p->IsLeafNode())
		{
			return *p->GetPrimitiveIndices(m_PrimitiveIndicesBase);
		}

		if (p->GetLeftNode(m_AABBTreeInference)->BV.IsInside(Point))
		{
			p = p->GetLeftNode(m_AABBTreeInference);
			continue;
		}
		else if (p->GetRightNode(m_AABBTreeInference)->BV.IsInside(Point))
		{
			p = p->GetRightNode(m_AABBTreeInference);
			continue;
		}

		return *p->GetPrimitiveIndices(m_PrimitiveIndicesBase);
	}

	return -1;
}

static int RayCastGeometry(const Ray& ray, int* prims, int numPrims, Geometry** ObjectCollection, RayCastResult* Result)
{
	assert(numPrims > 0);
	if (ObjectCollection == nullptr)
	{
		return *prims;
	}
	int min_idx = -1;
	float t, min_t = FLT_MAX;
	for (int i = 0; i < numPrims; ++i)
	{
		const int index = prims[i];
		Geometry *Geom = ObjectCollection[index];
		bool hit = Geom->RayCast(ray.Origin, ray.Dir, &t);
		if (hit)
		{
			if (t < min_t)
			{
				min_idx = i;
				min_t = t;
			}
		}
	}

	if (min_idx != -1)
	{
		Result->hit = true;
		Result->hitPoint = ray.PointAt(min_t);
		Result->hitGeom = ObjectCollection[min_idx];
		Result->hitTime = min_t;
	}
	return min_idx;
}

int  AABBTree::RayCast(const Ray& ray, Geometry **ObjectCollection, RayCastResult* Result) const
{
	Result->hit = false;

	float t1, t2;
	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !ray.IntersectAABB(p->BV.Min, p->BV.Max, &t1))
	{
		return -1;
	}
	Result->hit = true;
	Result->hitTime = t1;

	while (p)
	{
		if (p->IsLeafNode())
		{
			return RayCastGeometry(ray, p->GetPrimitiveIndices(m_PrimitiveIndicesBase), p->GetNumPrimitives(), ObjectCollection, Result);
		}

		AABBTreeNodeInference* p1 = p->GetLeftNode(m_AABBTreeInference);
		AABBTreeNodeInference* p2 = p->GetRightNode(m_AABBTreeInference);

		bool hit1 = ray.IntersectAABB(p1->BV.Min, p1->BV.Max, &t1);
		bool hit2 = ray.IntersectAABB(p2->BV.Min, p2->BV.Max, &t2);

		if (hit1 && hit2)
		{
			if (t1 < t2)
			{
				Result->hitTime = t1;
				p = p1;
			}
			else
			{
				Result->hitTime = t2;
				p = p2;
			}
			continue;
		}
		else if (hit1)
		{
			Result->hitTime = t1;
			p = p1;
			continue;
		}
		else if (hit2)
		{
			Result->hitTime = t2;
			p = p2;
			continue;
		}

		return RayCastGeometry(ray, p->GetPrimitiveIndices(m_PrimitiveIndicesBase), p->GetNumPrimitives(), ObjectCollection, Result);
	}

	return -1;
}

int  AABBTree::RayCastBoundingBox(const Ray& ray, float* t) const
{
	RayCastResult Result;
	int hit_obj = RayCast(ray, nullptr, &Result);
	if (hit_obj >= 0)
	{
		*t = Result.hitTime;
	}
	return hit_obj;
}

void AABBTree::InitAABBTreeBuild(AABBTreeBuildData& params)
{
	if (m_PrimitiveIndicesBase)
		return;

	m_PrimitiveIndicesBase = new int[params.NumPrimitives];
	for (int i = 0; i < params.NumPrimitives; ++i)
		m_PrimitiveIndicesBase[i] = (int)i;

	params.pAABBTree = new AABBTreeOffline();
	params.pAABBTree->Init(params.NumPrimitives, params.NumPrimitivesPerNode);

	params.pIndexBase = m_PrimitiveIndicesBase;
	params.pCenterBuffer = (Vector3d*) new float[(params.NumPrimitives + 1)*3];
	for (int i = 0; i < params.NumPrimitives; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


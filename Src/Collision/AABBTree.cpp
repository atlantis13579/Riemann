
#include "AABBTree.h"

#include <assert.h>

#include "../Maths/Stack.h"
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
	m_AABBTreeInference = params.pAABBTree->BuildInferenceTree();

	delete params.pAABBTree;
	params.pAABBTree = nullptr;
}

#define	GET_INDEX(_p)	(*_p->GetPrimitiveIndices(m_PrimitiveIndicesBase))
#define LEFT_NODE(_p)	(_p->GetLeftNode(m_AABBTreeInference))
#define RIGHT_NODE(_p)	(_p->GetRightNode(m_AABBTreeInference))

void AABBTree::Statistic(TreeStatistics& stat)
{
	memset(&stat, 0, sizeof(stat));

	FixedStack<AABBTreeNodeInference, 32> stack;
	stack.Push(m_AABBTreeInference);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				stat.Nodes += 1;
				stat.MaxDepth = std::max(stat.MaxDepth, stack.Depth());
				break;
			}

			AABBTreeNodeInference* Left = LEFT_NODE(p);
			AABBTreeNodeInference* Right = RIGHT_NODE(p);
			if (Left && Right)
			{
				p = Left;
				stack.Push(Right);
				continue;
			}
			else if (Left)
			{
				p = Left;
				continue;
			}
			else if (Right)
			{
				p = Right;
				continue;
			}

			assert(false);		// Should never goes here
		}
	}
}

int AABBTree::IntersectPoint(const Vector3d& Point) const
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
			return GET_INDEX(p);
		}

		AABBTreeNodeInference* Left = LEFT_NODE(p);
		AABBTreeNodeInference* Right = RIGHT_NODE(p);
		if (Left && Left->BV.IsInside(Point))
		{
			p = Left;
			continue;
		}
		if (Right && Right->BV.IsInside(Point))
		{
			p = Right;
			continue;
		}

		return GET_INDEX(p);
	}

	return -1;
}

static int IntersectGeometry(const Ray3d& ray, int* prims, int numPrims, Geometry** ObjectCollection, const Box3d& BV, const RayCastOption& Option, RayCastResult* Result)
{
	assert(numPrims > 0);
	if (ObjectCollection == nullptr)
	{
		float t;
		if (ray.IntersectAABB(BV.Min, BV.Max, &t))
		{
			Result->hit = true;
			if (t < Result->hitTime)
			{
				Result->hitPoint = ray.PointAt(t);
				Result->hitTime = t;
			}
			return *prims;
		}

		return -1;
	}
	int min_idx = -1;
	float t, min_t = FLT_MAX;
	for (int i = 0; i < numPrims; ++i)
	{
		const int index = prims[i];
		Geometry *Geom = ObjectCollection[index];
		bool hit = Geom->RayCast(ray.Origin, ray.Dir, &t);
		if (hit && t < Option.MaxDist)
		{
			if (Option.Type == RayCastOption::RAYCAST_ANY)
			{
				min_idx = i;
				min_t = t;
				break;
			}

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
		if (min_t < Result->hitTime)
		{
			Result->hitPoint = ray.PointAt(min_t);
			Result->hitGeom = ObjectCollection[min_idx];
			Result->hitTime = min_t;
		}
	}
	return min_idx;
}

#define SET_TIME(_t)	if (ObjectCollection == nullptr) Result->hitTime = (_t);

bool  AABBTree::RayCast(const Ray3d& ray, Geometry **ObjectCollection, const RayCastOption& Option, RayCastResult* Result) const
{
	Result->hit = false;
	Result->hitTime = FLT_MAX;

	float t1, t2;
	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !ray.IntersectAABB(p->BV.Min, p->BV.Max, &t1))
	{
		return false;
	}

	FixedStack<AABBTreeNodeInference, 32> stack;
	stack.Push(m_AABBTreeInference);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				int* PrimitiveIndices = p->GetPrimitiveIndices(m_PrimitiveIndicesBase);
				int	 nPrimitives = p->GetNumPrimitives();
				const Box3d& Box = p->GetBoundingVolume();
				int HitId =	IntersectGeometry(ray, PrimitiveIndices, nPrimitives, ObjectCollection, Box, Option, Result);
				if (HitId >= 0)
				{
					if (Option.Type == RayCastOption::RAYCAST_ANY)
					{
						return true;
					}
				}
				break;
			}

			AABBTreeNodeInference* Left = LEFT_NODE(p);
			AABBTreeNodeInference* Right = RIGHT_NODE(p);

			bool hit1 = ray.IntersectAABB(Left->BV.Min, Left->BV.Max, &t1) && t1 < Result->hitTime;
			bool hit2 = ray.IntersectAABB(Right->BV.Min, Right->BV.Max, &t2) && t2 < Result->hitTime;

			if (hit1 && hit2)
			{
				if (t1 < t2)
				{
					p = Left;
					stack.Push(Right);
				}
				else
				{
					p = Right;
					stack.Push(Left);
				}
				continue;
			}
			else if (hit1)
			{
				p = Left;
				continue;
			}
			else if (hit2)
			{
				p = Right;
				continue;
			}

			break;
		}

	}

	return Result->hit;
}

bool  AABBTree::RayCastBoundingBox(const Ray3d& ray, const RayCastOption& Option, RayCastResult* Result) const
{
	return RayCast(ray, nullptr, Option, Result);
}


void AABBTree::InitAABBTreeBuild(AABBTreeBuildData& params)
{
	if (m_PrimitiveIndicesBase)
		return;

	m_NumPrimitives = params.NumPrimitives;
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



#include "AABBTree.h"

#include <assert.h>

#include "../Core/StaticStack.h"
#include "AABBTreeOffline.h"
#include "AABBTreeInference.h"
#include "GeometryObject.h"

AABBTree::AABBTree()
{
	m_GeometryIndicesBase = nullptr;
	m_NumGeometries = 0;
	m_AABBTreeInference = nullptr;
}

AABBTree::~AABBTree()
{
	Release();
}

void AABBTree::Release()
{
	m_NumGeometries = 0;
	if (m_GeometryIndicesBase)
	{
		delete[]m_GeometryIndicesBase;
		m_GeometryIndicesBase = nullptr;
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

#define	GET_INDEX(_p)	(*_p->GetGeometryIndices(m_GeometryIndicesBase))
#define LEFT_NODE(_p)	(_p->GetLeftNode(m_AABBTreeInference))
#define RIGHT_NODE(_p)	(_p->GetRightNode(m_AABBTreeInference))

void AABBTree::Statistic(TreeStatistics& stat)
{
	memset(&stat, 0, sizeof(stat));

	StaticStack<uint32_t, RAYCAST_STACK_SIZE> stack;
	stack.Push(0);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = m_AABBTreeInference + stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				stat.NumNodes += 1;
				stat.MaxGeometriesAtLeaf = std::max(stat.MaxGeometriesAtLeaf, p->GetNumGeometries());
				stat.MaxDepth = std::max(stat.MaxDepth, stack.Depth());
				break;
			}

			AABBTreeNodeInference* Left = LEFT_NODE(p);
			AABBTreeNodeInference* Right = RIGHT_NODE(p);
			if (Left && Right)
			{
				p = Left;
				stack.Push((uint32_t)(Right - m_AABBTreeInference));
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

static int RayIntersectGeometry(const Ray3d& Ray, int* Geoms, int NumGeoms, Geometry** GeometryCollection, const Box3d& BV, const RayCastOption& Option, RayCastResult* Result)
{
	assert(NumGeoms > 0);
	if (GeometryCollection == nullptr)
	{
		float t;
		if (Ray.IntersectAABB(BV.Min, BV.Max, &t) && t < Option.MaxDist)
		{
			Result->hit = true;
			if (t < Result->hitTimeMin)
			{
				Result->hitGeom = nullptr;
				Result->hitTimeMin = t;
			}
			return *Geoms;
		}

		return -1;
	}

	int min_idx = -1;
	float min_t = FLT_MAX;
	for (int i = 0; i < NumGeoms; ++i)
	{
		const int index = Geoms[i];
		Geometry *Geom = GeometryCollection[index];
        assert(Geom);
		
		if (Geom == Option.Cache.prevhitGeom)
		{
			continue;
		}
		
		if (Option.Filter && !Option.Filter->IsCollidable(Option.FilterData, Geom->GetFilterData()))
		{
			continue;
		}
		
		bool hit = Geom->RayCast(Ray.Origin, Ray.Dir, &Option, Result);
		if (hit)
		{
			if (Option.Type == RayCastOption::RAYCAST_ANY)
			{
				min_idx = index;
				min_t = Result->hitTime;
				break;
			}
            else if (Option.Type == RayCastOption::RAYCAST_PENETRATE)
            {
                Result->hitGeometries.push_back(Geom);
            }

			if (Result->hitTime < min_t)
			{
				min_idx = index;
				min_t = Result->hitTime;
			}
		}
	}

	if (min_idx != -1)
	{
		Result->hit = true;
		if (min_t < Result->hitTimeMin)
		{
			Result->hitGeom = GeometryCollection[min_idx];
			Result->hitTimeMin = min_t;
		}
	}
	return min_idx;
}

bool RayIntersectCacheObj(const Ray3d& Ray, const RayCastOption& Option, RayCastResult* Result)
{
	const RayCastCache& Cache = Option.Cache;
	if (Cache.prevhitGeom)
	{
		if (Option.Filter && !Option.Filter->IsCollidable(Option.FilterData, Cache.prevhitGeom->GetFilterData()))
		{
			return false;
		}
		
		bool hit = Cache.prevhitGeom->RayCast(Ray.Origin, Ray.Dir, &Option, Result);
		if (hit)
		{
			Result->hit = true;
			Result->hitGeom = Cache.prevhitGeom;
			Result->hitTimeMin = Result->hitTime;
			return true;
		}
	}
	return false;
}

bool RestoreCacheStack(const RayCastOption& Option, StaticStack<uint32_t, RAYCAST_STACK_SIZE>* Stack)
{
	const RayCastCache& Cache = Option.Cache;
	if (!Cache.prevStack.Empty())
	{
		Stack->Restore(Option.Cache.prevStack);
		return true;
	}
	return false;
}

bool  AABBTree::RayCast(const Ray3d& Ray, Geometry** ObjectCollection, const RayCastOption& Option, RayCastResult* Result) const
{
	Result->hit = false;
	Result->hitTestCount = 0;
	Result->hitTimeMin = FLT_MAX;
	Result->hitGeom = nullptr;

	if (RayIntersectCacheObj(Ray, Option, Result))
	{
		if (Option.Type == RayCastOption::RAYCAST_ANY)
		{
			Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
			return true;
		}
		else if (Option.Type == RayCastOption::RAYCAST_PENETRATE)
		{
			Result->hitGeometries.push_back(Result->hitGeom);
		}
	}

	float t1, t2;
	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !Ray.IntersectAABB(p->BV.Min, p->BV.Max, &t1))
	{
		return false;
	}

	StaticStack<uint32_t, RAYCAST_STACK_SIZE> stack;
	if (!RestoreCacheStack(Option, &stack))
	{
		stack.Push(0);
	}

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = m_AABBTreeInference + stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				const Box3d& Box = p->GetBoundingVolume();
				int HitId =	RayIntersectGeometry(Ray, PrimitiveIndices, nPrimitives, ObjectCollection, Box, Option, Result);
				if (HitId >= 0)
				{
					if (Option.Type == RayCastOption::RAYCAST_ANY)
					{
						Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
						return true;
					}
				}
				break;
			}

			AABBTreeNodeInference* Left = LEFT_NODE(p);
			AABBTreeNodeInference* Right = Left + 1;

			Result->AddTestCount(2);

			bool hit1 = Ray.IntersectAABB(Left->BV.Min, Left->BV.Max, &t1);
			bool hit2 = Ray.IntersectAABB(Right->BV.Min, Right->BV.Max, &t2);

            if (Option.Type != RayCastOption::RAYCAST_PENETRATE)
            {
                hit1 = hit1 && t1 < Result->hitTimeMin && t1 < Option.MaxDist;
                hit2 = hit2 && t2 < Result->hitTimeMin && t2 < Option.MaxDist;
            }
            
			if (hit1 && hit2)
			{
				if (t1 < t2)
				{
					p = Left;
					stack.Push((uint32_t)(Right - m_AABBTreeInference));
				}
				else
				{
					p = Right;
					stack.Push((uint32_t)(Left - m_AABBTreeInference));
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

	if (Result->hit || Result->hitGeometries.size() > 0)
	{
		Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
		return true;
	}
	return false;
}

bool  AABBTree::RayCastBoundingBox(const Ray3d& ray, const RayCastOption& Option, RayCastResult* Result) const
{
	return RayCast(ray, nullptr, Option, Result);
}

static bool OverlapGeometry(Geometry *geometry, int* Indices, int NumIndices, Geometry** GeometryCollection, const OverlapOption& Option, OverlapResult* Result)
{
	assert(NumIndices > 0);
	if (GeometryCollection == nullptr)
	{
		return true;
	}

	for (int i = 0; i < NumIndices; ++i)
	{
		Result->AddTestCount(1);

		const int index = Indices[i];
		Geometry* candidate = GeometryCollection[index];

		if (Option.Filter && !Option.Filter->IsCollidable(Option.FilterData, candidate->GetFilterData()))
		{
			continue;
		}
		
		bool overlap = geometry->Overlap(candidate);
		if (overlap)
		{
			Result->overlaps = true;
			if (Result->overlapGeoms.size() < Option.maxOverlaps)
			{
				Result->overlapGeoms.push_back(candidate);
			}

			if (Result->overlapGeoms.size() >= Option.maxOverlaps)
			{
				return true;
			}
		}
	}
	return Result->overlaps;
}

bool AABBTree::Overlap(Geometry *geometry, Geometry** ObjectCollection, const OverlapOption& Option, OverlapResult* Result)
{
	Result->overlaps = false;
	Result->overlapGeoms.clear();

	const Box3d &aabb = geometry->GetBoundingVolume_WorldSpace();

	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !aabb.Intersect(p->BV.Min, p->BV.Max))
	{
		return false;
	}

	StaticStack<uint32_t, RAYCAST_STACK_SIZE> stack;
	stack.Push(0);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = m_AABBTreeInference + stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				bool overlap = OverlapGeometry(geometry, PrimitiveIndices, nPrimitives, ObjectCollection, Option, Result);
				if (overlap)
				{
					if (Result->overlapGeoms.size() >= Option.maxOverlaps)
					{
						return true;
					}
				}
				break;
			}

			Result->AddTestCount(2);

			AABBTreeNodeInference* Left = LEFT_NODE(p);
			AABBTreeNodeInference* Right = Left + 1;

			bool intersect1 = aabb.Intersect(Left->BV.Min, Left->BV.Max);
			bool intersect2 = aabb.Intersect(Right->BV.Min, Right->BV.Max);

			if (intersect1 && intersect2)
			{
				float d1 = (aabb.GetCenter() - Left->BV.GetCenter()).SquareLength();
				float d2 = (aabb.GetCenter() - Right->BV.GetCenter()).SquareLength();

				if (d1 < d2)
				{
					p = Left;
					stack.Push((uint32_t)(Right - m_AABBTreeInference));
				}
				else
				{
					p = Right;
					stack.Push((uint32_t)(Left - m_AABBTreeInference));
				}
				continue;
			}
			else if (intersect1)
			{
				p = Left;
				continue;
			}
			else if (intersect2)
			{
				p = Right;
				continue;
			}

			break;
		}

	}

	return Result->overlaps;
}

void AABBTree::InitAABBTreeBuild(AABBTreeBuildData& params)
{
	if (m_GeometryIndicesBase)
		return;

	m_NumGeometries = params.numGeometries;
	m_GeometryIndicesBase = new int[params.numGeometries];
	for (int i = 0; i < params.numGeometries; ++i)
		m_GeometryIndicesBase[i] = (int)i;

	params.pAABBTree = new AABBTreeOffline();
	params.pAABBTree->Init(params.numGeometries, params.numGeometriesPerNode);

	params.pIndicesBase = m_GeometryIndicesBase;
	params.pCenterBuffer = (Vector3d*) new float[(params.numGeometries + 1)*3];
	for (int i = 0; i < params.numGeometries; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


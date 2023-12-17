
#include "AABBTree.h"

#include <assert.h>

#include "AABBTreeOffline.h"
#include "AABBTreeInference.h"
#include "GeometryObject.h"
#include "GeometryQuery.h"

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
	m_AABBTreeInference = params.pAABBTree->BuildCacheFriendlyTree();

	delete  params.pAABBTree;
	params.pAABBTree = nullptr;
}

#define	GET_INDEX(_p)	(*_p->GetGeometryIndices(m_GeometryIndicesBase))
#define LEFT_NODE(_p)	(_p->GetLeftNode(m_AABBTreeInference))
#define RIGHT_NODE(_p)	(_p->GetRightNode(m_AABBTreeInference))

void AABBTree::Statistic(TreeStatistics& stat)
{
	memset(&stat, 0, sizeof(stat));

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	stack.Push(0);
	
	int curr_depth = 1;
	StaticStack<uint32_t, TREE_MAX_DEPTH> depth;
	depth.Push(curr_depth);
	
	while (!stack.Empty())
	{
		CacheFriendlyAABBTree* p = m_AABBTreeInference + stack.Pop();
		curr_depth = depth.Pop();
		while (p)
		{
			stat.NumNodes += 1;
			
			if (p->IsLeaf())
			{
				stat.NumLeafs += 1;
				stat.MaxGeometriesAtLeaf = std::max(stat.MaxGeometriesAtLeaf, p->GetNumGeometries());
				stat.MaxStack = std::max(stat.MaxStack, stack.Depth());
				stat.MaxDepth = std::max(stat.MaxDepth, curr_depth);
				break;
			}

			CacheFriendlyAABBTree* Left = LEFT_NODE(p);
			CacheFriendlyAABBTree* Right = RIGHT_NODE(p);
			if (Left && Right)
			{
				p = Left;
				stack.Push((uint32_t)(Right - m_AABBTreeInference));
				assert(!stack.Full());
				
				curr_depth += 1;
				depth.Push(curr_depth);
				continue;
			}

			assert(false);		// Should never goes here
		}
	}
}

int AABBTree::IntersectPoint(const Vector3& Point) const
{
	CacheFriendlyAABBTree* p = m_AABBTreeInference;
	if (p == nullptr || !p->aabb.IsInside(Point))
	{
		return -1;
	}

	while (p)
	{
		if (p->IsLeaf())
		{
			return GET_INDEX(p);
		}

		CacheFriendlyAABBTree* Left = LEFT_NODE(p);
		CacheFriendlyAABBTree* Right = RIGHT_NODE(p);
		if (Left && Left->aabb.IsInside(Point))
		{
			p = Left;
			continue;
		}
		if (Right && Right->aabb.IsInside(Point))
		{
			p = Right;
			continue;
		}

		return GET_INDEX(p);
	}

	return -1;
}

static int RayIntersectGeometries(const Ray3d& Ray, int* Geoms, int NumGeoms, Geometry** GeometryCollection, const Box3d& BV, const RayCastOption* Option, RayCastResult* Result)
{
	assert(NumGeoms > 0);
	if (GeometryCollection == nullptr)
	{
		float t;
		if (Ray.IntersectAABB(BV.mMin, BV.mMax, &t) && t < Option->MaxDist)
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
		
		if (Geom == Option->Cache.prevhitGeom)
		{
			continue;
		}

		bool hit = Geom->RayCast(Ray.Origin, Ray.Dir, Option, Result);
		if (hit)
		{
			if (Option->Type == RayCastOption::RAYCAST_ANY)
			{
				min_idx = index;
				min_t = Result->hitTime;
				break;
			}
            else if (Option->Type == RayCastOption::RAYCAST_PENETRATE)
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

bool RayIntersectCacheObj(const Ray3d& Ray, const RayCastOption* Option, RayCastResult* Result)
{
	const RayCastCache& Cache = Option->Cache;
	if (Cache.prevhitGeom)
	{
		bool hit = Cache.prevhitGeom->RayCast(Ray.Origin, Ray.Dir, Option, Result);
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

bool RestoreCacheStack(const RayCastOption* Option, StaticStack<uint32_t, TREE_MAX_DEPTH>* Stack)
{
	const RayCastCache& Cache = Option->Cache;
	if (!Cache.prevStack.Empty())
	{
		Stack->Restore(Option->Cache.prevStack);
		return true;
	}
	return false;
}

bool  AABBTree::RayCast(const Ray3d& Ray, Geometry** ObjectCollection, const RayCastOption* Option, RayCastResult* Result) const
{
	Result->hit = false;
	Result->hitTestCount = 0;
	Result->hitTimeMin = FLT_MAX;
	Result->hitGeom = nullptr;

	if (RayIntersectCacheObj(Ray, Option, Result))
	{
		if (Option->Type == RayCastOption::RAYCAST_ANY)
		{
			Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
			return true;
		}
		else if (Option->Type == RayCastOption::RAYCAST_PENETRATE)
		{
			Result->hitGeometries.push_back(Result->hitGeom);
		}
	}

	float t1, t2;
	CacheFriendlyAABBTree* p = m_AABBTreeInference;
	if (p == nullptr || !Ray.IntersectAABB(p->aabb.mMin, p->aabb.mMax, &t1) || t1 >= Option->MaxDist)
	{
		return false;
	}

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	if (!RestoreCacheStack(Option, &stack))
	{
		stack.Push(0);
	}

	while (!stack.Empty())
	{
		p = m_AABBTreeInference + stack.Pop();
		while (p)
		{
			if (p->IsLeaf())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				const Box3d& Box = p->GetBoundingVolume();
				int HitId =	RayIntersectGeometries(Ray, PrimitiveIndices, nPrimitives, ObjectCollection, Box, Option, Result);
				if (HitId >= 0)
				{
					if (Option->Type == RayCastOption::RAYCAST_ANY)
					{
						Result->hitPoint = Ray.PointAt(Result->hitTimeMin);
						return true;
					}
				}
				break;
			}

			CacheFriendlyAABBTree* Left = LEFT_NODE(p);
			CacheFriendlyAABBTree* Right = RIGHT_NODE(p);

			Result->AddTestCount(2);

			bool hit1 = Ray.IntersectAABB(Left->aabb.mMin, Left->aabb.mMax, &t1);
			bool hit2 = Ray.IntersectAABB(Right->aabb.mMin, Right->aabb.mMax, &t2);

            if (Option->Type != RayCastOption::RAYCAST_PENETRATE)
            {
                hit1 = hit1 && t1 < Result->hitTimeMin && t1 < Option->MaxDist;
                hit2 = hit2 && t2 < Result->hitTimeMin && t2 < Option->MaxDist;
            }
            
			assert(!stack.Full());
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
	return RayCast(ray, nullptr, &Option, Result);
}

static bool OverlapGeometries(const Geometry *geometry, int* Indices, int NumIndices, Geometry** GeometryCollection, const IntersectOption* Option, IntersectResult* Result)
{
	assert(NumIndices > 0);
	if (GeometryCollection == nullptr)
	{
		return true;
	}

	for (int i = 0; i < NumIndices; ++i)
	{
		const int index = Indices[i];
		Geometry* candidate = GeometryCollection[index];

		if (Option->Filter && !Option->Filter->IsCollidable(Option->FilterData, candidate->GetFilterData()))
		{
			continue;
		}
		
		Result->AddTestCount(1);

		bool overlap = geometry->Intersect(candidate);
		if (overlap)
		{
			Result->overlaps = true;
			if (Result->overlapGeoms.size() < Option->maxOverlaps)
			{
				Result->overlapGeoms.push_back(candidate);
			}

			if (Result->overlapGeoms.size() >= Option->maxOverlaps)
			{
				return true;
			}
		}
	}
	return Result->overlaps;
}

bool AABBTree::Intersect(const Geometry *geometry, Geometry** ObjectCollection, const IntersectOption* Option, IntersectResult* Result) const
{
	Result->overlaps = false;
	Result->overlapGeoms.clear();

	const Box3d &aabb = geometry->GetBoundingVolume_WorldSpace();

	CacheFriendlyAABBTree* p = m_AABBTreeInference;
	if (p == nullptr || !aabb.Intersect(p->aabb.mMin, p->aabb.mMax))
	{
		return false;
	}

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	stack.Push(0);

	while (!stack.Empty())
	{
		p = m_AABBTreeInference + stack.Pop();
		while (p)
		{
			if (p->IsLeaf())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				bool overlap = OverlapGeometries(geometry, PrimitiveIndices, nPrimitives, ObjectCollection, Option, Result);
				if (overlap)
				{
					if (Result->overlapGeoms.size() >= Option->maxOverlaps)
					{
						return true;
					}
				}
				break;
			}

			Result->AddTestCount(2);

			CacheFriendlyAABBTree* Left = LEFT_NODE(p);
			CacheFriendlyAABBTree* Right = RIGHT_NODE(p);

			bool intersect1 = aabb.Intersect(Left->aabb.mMin, Left->aabb.mMax);
			bool intersect2 = aabb.Intersect(Right->aabb.mMin, Right->aabb.mMax);

			assert(!stack.Full());
			if (intersect1 && intersect2)
			{
				p = Left;
				stack.Push((uint32_t)(Right - m_AABBTreeInference));
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

static int SweepGeometries(const Geometry *geometry, int* Geoms, int NumGeoms, Geometry** GeometryCollection, const Vector3 &Direction, const Box3d &BV, const SweepOption* Option, SweepResult* Result)
{
	assert(NumGeoms > 0);
	float t;
	Vector3 normal;
	if (GeometryCollection == nullptr)
	{
		if (geometry->SweepAABB(Direction, BV.mMin, BV.mMax, &normal, &t) && t < Option->MaxDist)
		{
			Result->hit = true;
			if (t < Result->hitTimeMin)
			{
				Result->hitGeom = nullptr;
				Result->hitNormal = normal;
				Result->hitTimeMin = t;
			}
			return *Geoms;
		}

		return -1;
	}

	int min_idx = -1;
	float min_t = FLT_MAX;
	Vector3 min_p;
	for (int i = 0; i < NumGeoms; ++i)
	{
		const int index = Geoms[i];
		Geometry *candidate = GeometryCollection[index];
		assert(candidate);

		bool hit = geometry->Sweep(Direction, candidate, &normal, &t);
		if (hit)
		{
			if (Option->Type == SweepOption::SWEEP_ANY)
			{
				min_idx = index;
				min_t = Result->hitTime;
				min_p = normal;
				break;
			}
			else if (Option->Type == SweepOption::SWEEP_PENETRATE)
			{
				Result->hitGeometries.push_back(candidate);
			}

			if (Result->hitTime < min_t)
			{
				min_idx = index;
				min_t = Result->hitTime;
				min_p = normal;
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
			Result->hitNormal = min_p;
		}
	}
	return min_idx;
}

bool AABBTree::Sweep(const Geometry *geometry, Geometry** ObjectCollection, const Vector3 &Direction, const SweepOption* Option, SweepResult *Result) const
{
	Result->hit = false;
	Result->hitTestCount = 0;
	Result->hitTimeMin = FLT_MAX;
	Result->hitGeom = nullptr;

	float t1, t2;
	Vector3 normal;
	CacheFriendlyAABBTree* p = m_AABBTreeInference;
	if (p == nullptr || !geometry->SweepAABB(Direction, p->aabb.mMin, p->aabb.mMax, &normal, &t1))
	{
		return false;
	}

	StaticStack<uint32_t, TREE_MAX_DEPTH> stack;
	stack.Push(0);

	while (!stack.Empty())
	{
		p = m_AABBTreeInference + stack.Pop();
		while (p)
		{
			if (p->IsLeaf())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				const Box3d& Box = p->GetBoundingVolume();
				int HitId =	SweepGeometries(geometry, PrimitiveIndices, nPrimitives, ObjectCollection, Direction, Box, Option, Result);
				if (HitId >= 0)
				{
					if (Option->Type == SweepOption::SWEEP_ANY)
					{
						return true;
					}
				}
				break;
			}

			CacheFriendlyAABBTree* Left = LEFT_NODE(p);
			CacheFriendlyAABBTree* Right = RIGHT_NODE(p);

			Result->AddTestCount(2);

			bool hit1 = geometry->SweepAABB(Direction, Left->aabb.mMin, Left->aabb.mMax, &normal, &t1);
			bool hit2 = geometry->SweepAABB(Direction, Right->aabb.mMin, Right->aabb.mMax, &normal, &t2);

			if (Option->Type != SweepOption::SWEEP_PENETRATE)
			{
				hit1 = hit1 && t1 < Result->hitTimeMin && t1 < Option->MaxDist;
				hit2 = hit2 && t2 < Result->hitTimeMin && t2 < Option->MaxDist;
			}
			
			assert(!stack.Full());
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
		return true;
	}
	return false;
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
	params.pCenterBuffer = new Vector3[params.numGeometries + 1];
	for (int i = 0; i < params.numGeometries; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


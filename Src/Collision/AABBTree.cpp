
#include "AABBTree.h"

#include <assert.h>

#include "../Maths/Stack.h"
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

	FixedStack<AABBTreeNodeInference, 32> stack;
	stack.Push(m_AABBTreeInference);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = stack.Pop();

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

static int RayIntersectGeometry(const Ray3d& Ray, int* Indices, int NumIndices, Geometry** GeometryCollection, const Box3d& BV, const RayCastOption& Option, RayCastResult* Result)
{
	assert(NumIndices > 0);
	if (GeometryCollection == nullptr)
	{
		float t;
		if (Ray.IntersectAABB(BV.Min, BV.Max, &t) && t < Option.MaxDist)
		{
			Result->hit = true;
			if (t < Result->hitTime)
			{
				Result->hitGeom = nullptr;
				Result->hitPoint = Ray.PointAt(t);
				Result->hitTime = t;
			}
			return *Indices;
		}

		return -1;
	}

	int min_idx = -1;
	float min_t = FLT_MAX;
	for (int i = 0; i < NumIndices; ++i)
	{
		const int index = Indices[i];
		Geometry *Geom = GeometryCollection[index];
		RayCastResult TempResult;
		bool hit = Geom->RayCast(Ray.Origin, Ray.Dir, &Option, &TempResult);

		#ifdef _DEBUG
		Result->TestCount += TempResult.TestCount;
		#endif // _DEBUG

		if (hit)
		{
			if (Option.Type == RayCastOption::RAYCAST_ANY)
			{
				min_idx = index;
				min_t = TempResult.hitTime;
				break;
			}

			if (TempResult.hitTime < min_t)
			{
				min_idx = index;
				min_t = TempResult.hitTime;
			}
		}
	}

	if (min_idx != -1)
	{
		Result->hit = true;
		if (min_t < Result->hitTime)
		{
			Result->hitPoint = Ray.PointAt(min_t);
			Result->hitGeom = GeometryCollection[min_idx];
			Result->hitTime = min_t;
		}
	}
	return min_idx;
}

bool  AABBTree::RayCast(const Ray3d& ray, Geometry **ObjectCollection, const RayCastOption& Option, RayCastResult* Result) const
{
	Result->hit = false;
	Result->hitTime = FLT_MAX;
	Result->hitGeom = nullptr;

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
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				const Box3d& Box = p->GetBoundingVolume();
				int HitId =	RayIntersectGeometry(ray, PrimitiveIndices, nPrimitives, ObjectCollection, Box, Option, Result);
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
			AABBTreeNodeInference* Right = Left + 1;

			#ifdef _DEBUG
			Result->TestCount += 2;
			#endif // _DEBUG

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

static bool OverlapGeometry(Geometry *geometry, int* Indices, int NumIndices, Geometry** GeometryCollection, const OverlapOption& Option, OverlapResult* Result)
{
	assert(NumIndices > 0);
	if (GeometryCollection == nullptr)
	{
		return true;
	}

	const Box3d& aabb = geometry->GetBoundingVolume_WorldSpace();

	for (int i = 0; i < NumIndices; ++i)
	{
		#ifdef _DEBUG
		Result->TestCount += 1;
		#endif // _DEBUG

		const int index = Indices[i];
		Geometry* candidate = GeometryCollection[index];

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

	FixedStack<AABBTreeNodeInference, 32> stack;
	stack.Push(m_AABBTreeInference);

	while (!stack.Empty())
	{
		AABBTreeNodeInference* p = stack.Pop();

		while (p)
		{
			if (p->IsLeafNode())
			{
				int* PrimitiveIndices = p->GetGeometryIndices(m_GeometryIndicesBase);
				int	 nPrimitives = p->GetNumGeometries();
				const Box3d& Box = p->GetBoundingVolume();
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

			#ifdef _DEBUG
			Result->TestCount += 2;
			#endif // _DEBUG

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
					stack.Push(Right);
				}
				else
				{
					p = Right;
					stack.Push(Left);
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

	m_NumGeometries = params.NumGeometries;
	m_GeometryIndicesBase = new int[params.NumGeometries];
	for (int i = 0; i < params.NumGeometries; ++i)
		m_GeometryIndicesBase[i] = (int)i;

	params.pAABBTree = new AABBTreeOffline();
	params.pAABBTree->Init(params.NumGeometries, params.NumGeometriesPerNode);

	params.pIndicesBase = m_GeometryIndicesBase;
	params.pCenterBuffer = (Vector3d*) new float[(params.NumGeometries + 1)*3];
	for (int i = 0; i < params.NumGeometries; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


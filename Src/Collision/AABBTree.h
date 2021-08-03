
#pragma once

#include "../Maths/Box3d.h"
#include "../CollisionPrimitive/Ray3d.h"
#include "AABBTreeOffline.h"

struct RayCastResult;
struct AABBTreeNodeInference;
class Geometry;

struct TreeStatistics
{
	int MaxDepth;
	int Nodes;
};

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void	Release();

	void	StaticBuild(AABBTreeBuildData& params);

	int		Traverse(const Vector3d& Point) const;
	int		RayCast(const Ray3d& ray, Geometry** ObjectCollection, RayCastResult *Result) const;
	int		RayCastBoundingBox(const Ray3d& ray, float* t) const;
	void	Statistic(TreeStatistics &stats);

private:
	void	InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_PrimitiveIndicesBase;
	int							m_NumPrimitives;

	AABBTreeNodeInference		*m_AABBTreeInference;
};
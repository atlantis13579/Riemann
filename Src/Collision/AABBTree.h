
#pragma once

#include "../Maths/Box3d.h"
#include "../CollisionPrimitive/Ray3d.h"
#include "AABBTreeOffline.h"
#include "GeometryQuery.h"

struct RayCastResult;
struct AABBTreeNodeInference;
class Geometry;

struct TreeStatistics
{
	int MaxDepth;
	int MaxGeometriesAtLeaf;
	int NumNodes;
};

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void	Release();

	void	StaticBuild(AABBTreeBuildData& params);
	void	Statistic(TreeStatistics &stat);

	bool	RayCast(const Ray3d& ray, Geometry** ObjectCollection, const RayCastOption* Option, RayCastResult *Result) const;
	bool	Overlap(Geometry *geometry, Geometry** ObjectCollection, const OverlapOption* Option, OverlapResult *Result) const;
	
	int		IntersectPoint(const Vector3& Point) const;
	bool	RayCastBoundingBox(const Ray3d& ray, const RayCastOption& Option, RayCastResult* Result) const;

private:
	void	InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_GeometryIndicesBase;
	int							m_NumGeometries;

	AABBTreeNodeInference		*m_AABBTreeInference;
};

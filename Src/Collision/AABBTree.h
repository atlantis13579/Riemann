
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

	int		IntersectPoint(const Vector3d& Point) const;
	bool	RayCast(const Ray3d& ray, Geometry** ObjectCollection, const RayCastOption& Option, RayCastResult *Result) const;
	bool	RayCastBoundingBox(const Ray3d& ray, const RayCastOption& Option, RayCastResult* Result) const;


private:
	void	InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_GeometryIndicesBase;
	int							m_NumGeometries;

	AABBTreeNodeInference		*m_AABBTreeInference;
};
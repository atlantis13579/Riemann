
#pragma once

#include "../Maths/Box3d.h"
#include "../CollisionPrimitive/Ray.h"
#include "AABBTreeOffline.h"

struct RayCastResult;
struct AABBTreeNodeInference;
class Geometry;

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void Release();

	void StaticBuild(AABBTreeBuildData& params);

	int	 Traverse(const Vector3d& Point) const;
	int  RayCast(const Ray& ray, Geometry** ObjectCollection, RayCastResult *Result) const;
	int  RayCastBoundingBox(const Ray& ray, float* t) const;

private:
	void InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_PrimitiveIndicesBase;
	int							m_NumPrimitives;

	AABBTreeNodeInference		*m_AABBTreeInference;
};
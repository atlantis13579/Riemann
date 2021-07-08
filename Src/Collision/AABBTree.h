
#pragma once

#include "../Maths/BoundingBox3d.h"
#include "../Geometry/Ray.h"
#include "AABBTreeOffline.h"

struct RayCastResult;
struct AABBTreeNodeInference;
class GeometryObject;

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void Release();

	void StaticBuild(AABBTreeBuildData& params);

	int	 Traverse(const Vector3d& Point) const;
	int  RayCast(const Ray& ray, GeometryObject** ObjectCollection, RayCastResult *Result) const;
	int  RayCastBoundingBox(const Ray& ray, float* t) const;

private:
	void InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_PrimitiveIndicesBase;
	int							m_NumPrimitives;

	AABBTreeNodeInference		*m_AABBTreeInference;
};
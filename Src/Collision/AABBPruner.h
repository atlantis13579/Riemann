
#pragma once

#include <vector>
#include "GeometryObject.h"

class AABBTree;
struct RayCastResult;

class AABBPruner
{
public:
	AABBPruner();
	~AABBPruner();

	void BuildAABB(const std::vector<GeometryObject*> &Objects, int nPrimitivePerNode);

	bool RayCast(const Vector3d& Origin, const Vector3d& Dir, RayCastResult* Result);

private:
	std::vector<GeometryObject*> m_Objects;

	AABBTree* m_staticAABB;
};
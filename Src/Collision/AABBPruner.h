
#pragma once

#include <vector>
#include "GeometryObject.h"

class AABBTree;

class AABBPruner
{
public:
	AABBPruner();
	~AABBPruner();

private:
	std::vector<GeometryObject> m_Objects;

	AABBTree* m_staticAABB;
};
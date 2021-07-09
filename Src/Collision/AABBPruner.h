
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "AABBTree.h"

class Geometry;
class AABBTree;
struct RayCastResult;

class AABBPruner : public AABBTree
{
public:
	AABBPruner();
	~AABBPruner();

	void BuildAABB(const std::vector<Geometry*> &Objects, int nPrimitivePerNode);

	bool RayCast(const Vector3d& Origin, const Vector3d& Dir, RayCastResult* Result);

private:
	std::vector<Geometry*> m_Objects;
};
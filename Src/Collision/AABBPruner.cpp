
#include "AABBPruner.h"

#include <assert.h>
#include "AABBTree.h"
#include "GeometryObject.h"

AABBPruner::AABBPruner()
{
}

AABBPruner::~AABBPruner()
{
}

void AABBPruner::BuildAABB(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
{
	m_Objects = std::move(Objects);
	std::vector<BoundingBox3d> boxes;
	boxes.resize(Objects.size());
	for (size_t i = 0; i < Objects.size(); ++i)
	{
		boxes[i] = Objects[i]->GetBoundingBoxWorld();
	}
	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), nPrimitivePerNode);
	
	StaticBuild(param);
}

bool AABBPruner::RayCast(const Vector3d& Origin, const Vector3d& Dir, RayCastResult* Result)
{
	Ray ray(Origin, Dir);
	Geometry** pp = &m_Objects[0];
	return AABBTree::RayCast(ray, pp, Result);
}
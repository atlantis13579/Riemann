
#include "AABBPruner.h"

#include <assert.h>
#include "AABBTree.h"


AABBPruner::AABBPruner()
{
	m_staticAABB = nullptr;
}

AABBPruner::~AABBPruner()
{
	if (m_staticAABB)
	{
		delete m_staticAABB;
		m_staticAABB = nullptr;
	}
}

void AABBPruner::BuildAABB(const std::vector<GeometryObject>& Objects, int nPrimitivePerNode)
{
	m_Objects = std::move(Objects);
	std::vector<BoundingBox3d> boxes;
	boxes.resize(Objects.size());
	for (size_t i = 0; i < Objects.size(); ++i)
	{
		GetBoundingBoxFunc func = GeometryObject::getboundingboxTable[Objects[i].Shape.Type];
		assert(func);
		boxes[i] = func(Objects[i].Shape.Object);
	}
	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), nPrimitivePerNode);
	
	m_staticAABB = new AABBTree();
	m_staticAABB->StaticBuild(param);
}

bool AABBPruner::RayCast(const Vector3d& Origin, const Vector3d& Dir, RayCastResult* Result)
{
	if (m_staticAABB)
	{
		Ray ray(Origin, Dir);
		return m_staticAABB->RayCast(ray, &m_Objects[0], Result);
	}
	return true;
}

#include "GeometryQuery.h"

#include <algorithm>
#include "AABBPruner.h"
#include "SparseSpatialHash.h"

GeometryQuery::GeometryQuery()
{
	m_staticGeometry = nullptr;
	m_dynamicPruner = nullptr;
	m_SpatialHashPruner = nullptr;
}

GeometryQuery::~GeometryQuery()
{
	if (m_staticGeometry)
	{
		delete m_staticGeometry;
		m_staticGeometry = nullptr;
	}

	if (m_dynamicPruner)
	{
		delete m_dynamicPruner;
		m_dynamicPruner = nullptr;
	}

	if (m_SpatialHashPruner)
	{
		delete m_SpatialHashPruner;
		m_SpatialHashPruner = nullptr;
	}
}

void GeometryQuery::BuildStaticGeometry(const std::vector<GeometryObject*>& Objects, int nPrimitivePerNode)
{
	if (m_staticGeometry == nullptr)
	{
		m_staticGeometry = new AABBPruner;
		m_staticGeometry->BuildAABB(std::move(Objects), nPrimitivePerNode);
	}
}

bool GeometryQuery::RayCast(const Vector3d& Origin, const Vector3d& Dir, RayCastResult* Result)
{
	if (m_staticGeometry)
	{
		return m_staticGeometry->RayCast(Origin, Dir, Result);
	}
	return false;
}
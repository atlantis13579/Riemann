
#include "GeometryQuery.h"

#include <algorithm>
#include "AABBPruner.h"
#include "SparseSpatialHash.h"
#include "BroadPhase.h"
#include "NarrowPhase.h"

GeometryQuery::GeometryQuery()
{
	m_staticGeometry = nullptr;
	m_dynamicPruner = nullptr;
	m_SpatialHashPruner = nullptr;
	m_BPhase = nullptr;
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

	if (m_BPhase)
	{
		delete m_BPhase;
		m_BPhase = nullptr;
	}
}

void GeometryQuery::Simulate(float dt)
{
	if (m_BPhase)
	{

	}
}

void GeometryQuery::CreateBroadPhaseFilter(const char* name)
{
	m_BPhase = BroadPhase::CreatSAP();
}

void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
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
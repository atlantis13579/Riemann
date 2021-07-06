
#include "GeometryQuery.h"

#include "AABBPruner.h"
#include "SparseSpatialHash.h"

GeometryQuery::GeometryQuery()
{
	m_staticPruner = nullptr;
	m_dynamicPruner = nullptr;
	m_SpatialHashPruner = nullptr;
}

GeometryQuery::~GeometryQuery()
{
	if (m_staticPruner)
	{
		delete m_staticPruner;
		m_staticPruner = nullptr;
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

void GeometryQuery::Init()
{

}
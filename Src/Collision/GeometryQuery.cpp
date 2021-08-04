
#include "GeometryQuery.h"

#include <algorithm>
#include "AABBTree.h"
#include "DynamicAABBTree.h"
#include "SparseSpatialHash.h"

GeometryQuery::GeometryQuery()
{
	m_staticGeometry = nullptr;
	m_dynamicPruner = nullptr;
	m_SpatialHash = nullptr;
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

	if (m_SpatialHash)
	{
		delete m_SpatialHash;
		m_SpatialHash = nullptr;
	}

}

void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
{
	if (m_staticGeometry == nullptr)
	{
		m_staticGeometry = new AABBTree;

		std::vector<Box3d> boxes;
		boxes.resize(Objects.size());
		for (size_t i = 0; i < Objects.size(); ++i)
		{
			boxes[i] = Objects[i]->GetBoundingVolume_WorldSpace();
		}
		m_Objects = Objects;
		AABBTreeBuildData param(&boxes[0], (int)boxes.size(), nPrimitivePerNode);

		m_staticGeometry->StaticBuild(param);
	}
}

bool GeometryQuery::RayCast(const Vector3d& Origin, const Vector3d& Dir, const RayCastOption& Option, RayCastResult* Result)
{
	if (m_staticGeometry)
	{
		Ray3d ray(Origin, Dir);
		Geometry** pp = &m_Objects[0];
		return m_staticGeometry->RayCast(ray, pp, Option, Result);
	}
	return false;
}

bool GeometryQuery::OverlapAABB(const Vector3d& Center, const Vector3d& Extent, const OverlapOption& Option, OverlapResult* Result)
{
	Geometry* Box = GeometryFactory::CreateOBB(Center, Extent);
	Geometry** pp = &m_Objects[0];
	m_staticGeometry->Overlap(Box, pp, Option, Result);
	GeometryFactory::DeleteGeometry(Box);
	return Result->overlaps;
}

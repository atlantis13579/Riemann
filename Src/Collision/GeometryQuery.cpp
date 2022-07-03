
#include "GeometryQuery.h"

#include <algorithm>
#include <string.h>
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
	if (m_staticGeometry == nullptr && Objects.size() > 0)
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
	Result->Reset();
	if (m_staticGeometry)
	{
		Ray3d ray(Origin, Dir);
		Geometry** pp = &m_Objects[0];
		return m_staticGeometry->RayCast(ray, pp, Option, Result);
	}
	return false;
}

bool GeometryQuery::OverlapBox(const Vector3d& Center, const Vector3d& Extent, const OverlapOption& Option, OverlapResult* Result)
{
	Result->Reset();
	Geometry* Box = GeometryFactory::CreateOBB(Center, Extent);
	Geometry** pp = &m_Objects[0];
	m_staticGeometry->Overlap(Box, pp, Option, Result);
	GeometryFactory::DeleteGeometry(Box);
	return Result->overlaps;
}

class DefaultCollisionFilter : public CollisionFilter
{
public:
	DefaultCollisionFilter(int n, unsigned char *pLayerData)
	{
		collisionTable.resize(n * n);
		memset(&collisionTable[0], 0, sizeof(collisionTable[0])*n*n);
		if (pLayerData)
		{
			memcpy(&collisionTable[0], pLayerData, sizeof(collisionTable[0])*n*n);
		}
	}
	
	virtual bool IsCollidable(const CollisionData &data0, const CollisionData& data1)
	{
		assert(0 <= data0.v0 && data0.v0 < nLayers);
		assert(0 <= data1.v0 && data1.v0 < nLayers);
		return collisionTable[data0.v0 * nLayers + data1.v0] == 0;
	}
	
private:
	int							nLayers;
	std::vector<unsigned char> 	collisionTable;
};

CollisionFilter *CollisionFilter::CreateDefault(int nLayers, unsigned char *pLayerData)
{
	return new DefaultCollisionFilter(nLayers, pLayerData);
}

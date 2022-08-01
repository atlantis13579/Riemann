
#include "GeometryQuery.h"

#include <assert.h>
#include <algorithm>
#include <string.h>
#include "../Core/Base.h"
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
	SAFE_DELETE(m_staticGeometry);
	SAFE_DELETE(m_dynamicPruner);
	SAFE_DELETE(m_SpatialHash);

	for (size_t i = 0; i < m_Objects.size(); ++i)
	{
		delete m_Objects[i];
	}
}

void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
{
	SAFE_DELETE(m_staticGeometry);
	
	if (Objects.size() > 0)
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

void GeometryQuery::CreateDynamicGeometry()
{
	SAFE_DELETE(m_dynamicPruner);
	m_dynamicPruner = new DynamicAABBTree;
}

bool GeometryQuery::RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption& Option, RayCastResult* Result)
{
	Result->Reset();
	
	bool hit = false;
	
	if (m_staticGeometry)
	{
		Ray3d ray(Origin, Dir);
		Geometry** pp = &m_Objects[0];
		hit = m_staticGeometry->RayCast(ray, pp, &Option, Result);
		
		if (Option.Type == RayCastOption::RAYCAST_ANY && hit)
		{
			return true;
		}
	}
	
	if (m_dynamicPruner)
	{
		Ray3d ray(Origin, Dir);
		RayCastResult Result2;
		bool hit_dynamic = m_dynamicPruner->RayCast(ray, &Option, &Result2);
		if (hit_dynamic)
		{
			if (!hit)
			{
				*Result = Result2;
			}
			else
			{
				Result->Merge(Result2);
			}
			return true;
		}
	}
	
	return hit;
}

bool GeometryQuery::OverlapBox(const Vector3& Center, const Vector3& Extent, const OverlapOption& Option, OverlapResult* Result)
{
	Result->Reset();
	
	if (m_staticGeometry)
	{
		// TODO, Box
		Geometry* Box = GeometryFactory::CreateOBB(Center, Extent);
		Geometry** pp = &m_Objects[0];
		m_staticGeometry->Overlap(Box, pp, &Option, Result);
		GeometryFactory::DeleteGeometry(Box);
		return Result->overlaps;
	}
	return false;
}

class DefaultCollisionFilter : public CollisionFilter
{
public:
	DefaultCollisionFilter() {}
	virtual ~DefaultCollisionFilter() {}

	virtual bool IsCollidable(const CollisionData& data0, const CollisionData& data1)
	{
		return data0.v0 == data1.v0;
	}
};

class CollisionTableFilter : public CollisionFilter
{
public:
	CollisionTableFilter(unsigned int n, unsigned char *pLayerData)
	{
		collisionTable.resize(n * n);
		memset(&collisionTable[0], 0, sizeof(collisionTable[0])*n*n);
		if (pLayerData)
		{
			memcpy(&collisionTable[0], pLayerData, sizeof(collisionTable[0])*n*n);
		}
	}

	virtual ~CollisionTableFilter() {}
	
	virtual bool IsCollidable(const CollisionData &data0, const CollisionData& data1)
	{
		assert(0 <= data0.v0 && data0.v0 < nLayers);
		assert(0 <= data1.v0 && data1.v0 < nLayers);
		return collisionTable[data0.v0 * nLayers + data1.v0] == 0;
	}
	
private:
	unsigned int				nLayers;
	std::vector<unsigned char> 	collisionTable;
};

CollisionFilter* CollisionFilter::CreateDefault()
{
	return new DefaultCollisionFilter();
}

CollisionFilter *CollisionFilter::CreateCollisionTable(unsigned int nLayers, unsigned char *pLayerData)
{
	return new CollisionTableFilter(nLayers, pLayerData);
}

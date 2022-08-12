
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

bool GeometryQuery::RayCastTest(const Vector3& Origin, const Vector3& Direction, const RayCastOption& Option, RayCastResult* Result)
{
	Result->Reset();
	
	Ray3d ray(Origin, Direction);
	bool hit = false;
	
	if (m_staticGeometry)
	{
		Geometry** pp = &m_Objects[0];
		hit = m_staticGeometry->RayCast(ray, pp, &Option, Result);
		
		if (Option.Type == RayCastOption::RAYCAST_ANY && hit)
		{
			return true;
		}
	}
	
	if (m_dynamicPruner)
	{
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

#define MAX_GEOMETRY_STACK_SIZE	(128)

bool GeometryQuery::IntersectTest_Box(const Vector3& Center, const Vector3& Extent, const IntersectOption& Option, IntersectResult* Result)
{
	char stack[MAX_GEOMETRY_STACK_SIZE];
	Geometry* Box = GeometryFactory::CreateOBB_placement(stack, Center, Extent);
	return IntersectTest_Impl(Box, Option, Result);
}

bool GeometryQuery::IntersectTest_Sphere(const Vector3& Center, float Radius, const IntersectOption& Option, IntersectResult* Result)
{
	char stack[MAX_GEOMETRY_STACK_SIZE];
	Geometry* Sphere = GeometryFactory::CreateSphere_placement(stack, Center, Radius);
	return IntersectTest_Impl(Sphere, Option, Result);
}

bool GeometryQuery::IntersectTest_Capsule(const Vector3& Center, float HalfHeight, float Radius, const IntersectOption& Option, IntersectResult* Result)
{
	char stack[MAX_GEOMETRY_STACK_SIZE];
	Geometry* Capsule = GeometryFactory::CreateCapsule_placement(stack, Center - Vector3(0, HalfHeight, 0), Center + Vector3(0, HalfHeight, 0), Radius);
	return IntersectTest_Impl(Capsule, Option, Result);
}

bool GeometryQuery::IntersectTest_Impl(const Geometry* geom, const IntersectOption& Option, IntersectResult* Result)
{
	Result->Reset();

	bool hit = false;

	if (m_staticGeometry)
	{
		Geometry** pp = &m_Objects[0];
		hit = m_staticGeometry->Intersect(geom, pp, &Option, Result);
		if (hit && Result->overlapGeoms.size() >= Option.maxOverlaps)
		{
			return hit;
		}
	}

	if (m_dynamicPruner)
	{
		IntersectResult Result2;
		bool hit_dynamic = m_dynamicPruner->Intersect(geom, &Option, &Result2);
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
		}
	}

	return hit;
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

#pragma once

#include <limits.h>
#include <vector>
#include "../Core/StaticStack.h"
#include "../Maths/Vector3.h"
#include "GeometryObject.h"

class AABBTree;
class DynamicAABBTree;
class SparseSpatialHash;
class Geometry;

#define RAYCAST_STACK_SIZE		(32)

struct RayCastCache
{
	RayCastCache()
	{
		prevhitGeom = nullptr;
		prevStack.Clear();
	}

	Geometry*									prevhitGeom;
	StaticStack<uint32_t, RAYCAST_STACK_SIZE>	prevStack;
};

class CollisionFilter
{
public:
	virtual ~CollisionFilter() {}
	virtual bool IsCollidable(const CollisionData &data0, const CollisionData& data1) = 0;
	static CollisionFilter* CreateDefault();
	static CollisionFilter* CreateCollisionTable(unsigned int nLayers, unsigned char *LayerData);
};

struct RayCastOption
{
	enum RayCastType
	{
		RAYCAST_NEAREST = 0,
        RAYCAST_ANY = 1,
        RAYCAST_PENETRATE = 2,
	};
	RayCastOption()
	{
		Type = RAYCAST_NEAREST;
		MaxDist = FLT_MAX;
        MaxObjects = INT_MAX;
		Filter = nullptr;
	}
	RayCastType		Type;
	RayCastCache	Cache;
	float			MaxDist;
	int             MaxObjects;
    CollisionData 	FilterData;
	CollisionFilter	*Filter;
};

struct RayCastResult
{
	RayCastResult()
	{
		Reset();
	}

	void Reset()
	{
		hit = false;
		hitTime = FLT_MAX;
		hitTimeMin = FLT_MAX;
		hitPoint = Vector3d::Zero();
		hitNormal = Vector3d::UnitY();
		hitGeom = nullptr;
        hitGeometries.clear();
        #ifdef _DEBUG
		hitTestCount = 0;
        #endif //_DEBUG
	}

	void AddTestCount(int Count)
	{
		#ifdef _DEBUG
		hitTestCount += Count;
		#endif // _DEBUG
	}

	bool		hit;
	float		hitTime;            // temp val
	float		hitTimeMin;         // result
	Vector3d	hitPoint;
	Vector3d	hitNormal;
	Geometry*	hitGeom;
    std::vector<Geometry*>  hitGeometries;
    
	int			hitTestCount;       // debug
};

struct SweepOption
{

};

struct SweepResult
{
	SweepResult()
	{
	}
};

struct OverlapOption
{
	enum OverlapTestType
	{
	};
	OverlapOption()
	{
		maxOverlaps = 1;
		Filter = nullptr;
	}
	unsigned int	maxOverlaps;
	CollisionData 	FilterData;
	CollisionFilter	*Filter;
};

struct OverlapResult
{
	OverlapResult()
	{
		Reset();
	}

	void AddTestCount(int Count)
	{
		#ifdef _DEBUG
		overlapTestCount += Count;
		#endif // _DEBUG
	}

	void Reset()
	{
		overlaps = true;
		overlapGeoms.clear();
		#ifdef _DEBUG
		overlapTestCount = 0;
		#endif // _DEBUG
	}

	bool					overlaps;
	std::vector<Geometry*>	overlapGeoms;
    
	int						overlapTestCount;       // debug
};

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

public:
	void			BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode);
	
	bool			RayCast(const Vector3d &Origin, const Vector3d& Dir, const RayCastOption& Option, RayCastResult *Result);
	bool			OverlapBox(const Vector3d &Center, const Vector3d& Extent, const OverlapOption& Option, OverlapResult* Result);

	AABBTree*		GetStaticTree()
	{
		return m_staticGeometry;
	}

private:
	std::vector<Geometry*>	m_Objects;

	AABBTree*				m_staticGeometry;
	DynamicAABBTree*		m_dynamicPruner;
	SparseSpatialHash*		m_SpatialHash;
};

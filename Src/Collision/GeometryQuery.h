#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "GeometryObject.h"

class AABBTree;
class DynamicAABBTree;
class SparseSpatialHash;
class Geometry;

struct RayCastOption
{
	enum RayCastType
	{
		RAYCAST_ANY,
		RAYCAST_NEAREST,
	};
	RayCastOption()
	{
		Type = RAYCAST_NEAREST;
		MaxDist = FLT_MAX;
	}
	RayCastType Type;
	float		MaxDist;
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
		hitPoint = Vector3d::Zero();
		hitNormal = Vector3d::UnitY();
		hitGeom = nullptr;
		#ifdef _DEBUG
		hitTestCount = 0;
		#endif // _DEBUG
	}

	void AddTestCount(int Count)
	{
		#ifdef _DEBUG
		hitTestCount += Count;
		#endif // _DEBUG
	}

	bool		hit;
	float		hitTime;
	Vector3d	hitPoint;
	Vector3d	hitNormal;
	Geometry*	hitGeom;
	#ifdef _DEBUG
	int			hitTestCount;
	#endif // _DEBUG
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
	}
	unsigned int		maxOverlaps;
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
	#ifdef _DEBUG
	int						overlapTestCount;
	#endif // _DEBUG
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
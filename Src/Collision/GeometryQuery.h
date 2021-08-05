#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "GeometryObject.h"

class AABBTree;
class DynamicAABBTree;
class SparseSpatialHash;

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
		hit = false;
		hitTime = FLT_MAX;
		hitGeom = nullptr;
	}

	bool		hit;
	float		hitTime;
	Vector3d	hitPoint;
	Vector3d	hitNormal;
	Geometry*	hitGeom;
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
		overlaps = true;
	}

	bool					overlaps;
	std::vector<Geometry*>	overlapGeoms;
};

class GeometryQuery
{
public:
	GeometryQuery();
	~GeometryQuery();

public:
	void			BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode);
	
	bool			RayCast(const Vector3d &Origin, const Vector3d& Dir, const RayCastOption& Option, RayCastResult *Result);
	bool			OverlapAABB(const Vector3d &Center, const Vector3d& Extent, const OverlapOption& Option, OverlapResult* Result);

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
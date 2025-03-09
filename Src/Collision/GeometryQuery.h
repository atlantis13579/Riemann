#pragma once

#include <limits.h>
#include <vector>
#include "../Core/StaticStack.h"
#include "../Maths/Vector3.h"
#include "GeometryObject.h"

namespace Riemann
{
	class AABBTree;
	class DynamicAABBTree;
	class SparseSpatialHash;
	class Geometry;

#define TREE_MAX_DEPTH			(32)
#define MAX_GEOMETRY_STACK_SIZE	(128)

	struct RayCastCache
	{
		RayCastCache()
		{
			prevhitGeom = nullptr;
			prevStack.clear();
		}

		Geometry* prevhitGeom;
		StaticStack<uint32_t, TREE_MAX_DEPTH>	prevStack;
	};

	class CollisionFilter
	{
	public:
		virtual ~CollisionFilter() {}
		virtual bool IsCollidable(const CollisionData& data0, const CollisionData& data1) = 0;
		static CollisionFilter* CreateDefault();
		static CollisionFilter* CreateCollisionTable(unsigned int nLayers, unsigned char* LayerData);
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
		bool			HitBothSides;
		float			MaxDist;
		int             MaxObjects;
		CollisionData 	FilterData;
		CollisionFilter* Filter;
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
			hitPoint = Vector3::Zero();
			hitNormal = Vector3::UnitY();
			hitGeom = nullptr;
			hitGeometries.clear();
			hitTestCount = 0;
		}

		void Merge(const RayCastResult& rhs)
		{
			if (!hit)
			{
				*this = rhs;
				return;
			}
			hit = hit || rhs.hit;
			hitPoint = hitTimeMin < rhs.hitTimeMin ? hitPoint : rhs.hitPoint;
			hitNormal = hitTimeMin < rhs.hitTimeMin ? hitNormal : rhs.hitNormal;
			hitGeom = hitTimeMin < rhs.hitTimeMin ? hitGeom : rhs.hitGeom;
			hitTime = hitTime < rhs.hitTime ? hitTime : rhs.hitTime;
			hitTimeMin = hitTimeMin < rhs.hitTimeMin ? hitTimeMin : rhs.hitTimeMin;
			hitGeometries.insert(hitGeometries.end(), rhs.hitGeometries.begin(), rhs.hitGeometries.end());
			hitTestCount += rhs.hitTestCount;
		}

		void AddTestCount(int Count)
		{
			hitTestCount += Count;
		}

		bool					hit;
		float					hitTime;            // temp val
		float					hitTimeMin;         // result
		Vector3					hitPoint;
		Vector3					hitNormal;
		Geometry* hitGeom;
		std::vector<Geometry*>  hitGeometries;

		int						hitTestCount;       // debug
	};

	struct IntersectOption
	{
		IntersectOption()
		{
			maxOverlaps = 1;
			Filter = nullptr;
		}

		unsigned int	maxOverlaps;
		CollisionData 	FilterData;
		CollisionFilter* Filter;
	};

	struct IntersectResult
	{
		IntersectResult()
		{
			Reset();
		}

		void AddTestCount(int Count)
		{
			intersectTestCount += Count;
		}

		void Reset()
		{
			overlaps = true;
			overlapGeoms.clear();
			intersectTestCount = 0;
		}

		void Merge(const IntersectResult& rhs)
		{
			if (!overlaps)
			{
				*this = rhs;
				return;
			}
			overlaps = overlaps || rhs.overlaps;
			overlapGeoms.insert(overlapGeoms.end(), rhs.overlapGeoms.begin(), rhs.overlapGeoms.end());
			intersectTestCount += rhs.intersectTestCount;
		}

		bool					overlaps;
		std::vector<Geometry*>	overlapGeoms;

		int						intersectTestCount;       // debug
	};

	struct SweepOption
	{
		enum SweepType
		{
			SWEEP_NEAREST = 0,
			SWEEP_ANY = 1,
			SWEEP_PENETRATE = 2,
		};
		SweepType		Type;
		bool			HitBothSides;
		float			MaxDist;
		CollisionData 	FilterData;
		CollisionFilter* Filter;
	};

	struct SweepResult
	{
		SweepResult()
		{
			Reset();
		}

		void Reset()
		{
			hit = false;
			hitTime = FLT_MAX;
			hitTimeMin = FLT_MAX;
			hitNormal = Vector3::Zero();
			hitGeom = nullptr;
			hitGeometries.clear();
			hitTestCount = 0;
		}

		void Merge(const SweepResult& rhs)
		{
			if (!hit)
			{
				*this = rhs;
				return;
			}
			hit = hit || rhs.hit;
			hitNormal = hitTimeMin < rhs.hitTimeMin ? hitNormal : rhs.hitNormal;
			hitGeom = hitTimeMin < rhs.hitTimeMin ? hitGeom : rhs.hitGeom;
			hitGeometries.insert(hitGeometries.end(), rhs.hitGeometries.begin(), rhs.hitGeometries.end());
			hitTime = hitTime < rhs.hitTime ? hitTime : rhs.hitTime;
			hitTimeMin = hitTimeMin < rhs.hitTimeMin ? hitTimeMin : rhs.hitTimeMin;
			hitTestCount += rhs.hitTestCount;
		}

		void AddTestCount(int Count)
		{
			hitTestCount += Count;
		}

		bool					hit;
		float					hitTime;            // temp val
		float					hitTimeMin;         // result
		Vector3					hitNormal;
		Vector3					hitPosition;
		Geometry*				hitGeom;
		std::vector<Geometry*>  hitGeometries;
		int						hitTestCount;       // debug
	};

	class GeometryQuery
	{
	public:
		GeometryQuery();
		~GeometryQuery();

	public:
		void		BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode);
		void		CreateDynamicGeometry();

		bool		RayCastQuery(const Vector3& Origin, const Vector3& Direction, const RayCastOption& Option, RayCastResult* Result);
		bool		IntersectQueryBox(const Vector3& Center, const Vector3& Extent, const IntersectOption& Option, IntersectResult* Result);
		bool		IntersectQuerySphere(const Vector3& Center, float Radius, const IntersectOption& Option, IntersectResult* Result);
		bool		IntersectQueryCapsule(const Vector3& x0, const Vector3& x1, float Radius, const IntersectOption& Option, IntersectResult* Result);
		bool		BoxCastQuery(const Vector3& Center, const Vector3& Extent, const Vector3& Direction, const SweepOption& Option, SweepResult* Result);
		bool		SphereCastQuery(const Vector3& Center, float Radius, const Vector3& Direction, const SweepOption& Option, SweepResult* Result);
		bool		CapsuleCastQuery(const Vector3& Center, float HalfH, float Radius, const Vector3& Direction, const SweepOption& Option, SweepResult* Result);

		AABBTree* GetStaticTree()
		{
			return m_staticGeometry;
		}

		DynamicAABBTree* GetDynamicTree()
		{
			return m_dynamicPruner;
		}

	private:
		bool		IntersectTest_Impl(const Geometry* geom, const IntersectOption& Option, IntersectResult* Result);
		bool		SweepTest_Impl(const Geometry* geom, const Vector3& Origin, const Vector3& Direction, const SweepOption& Option, SweepResult* Result);

	private:
		std::vector<Geometry*>	m_Objects;

		AABBTree* m_staticGeometry;
		DynamicAABBTree* m_dynamicPruner;
	};
}
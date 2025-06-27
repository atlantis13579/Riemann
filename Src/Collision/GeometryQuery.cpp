
#include "GeometryQuery.h"

#include <assert.h>
#include <algorithm>
#include "../Core/Base.h"
#include "AABBTree.h"
#include "DynamicAABBTree.h"

namespace Riemann
{
	GeometryQuery::GeometryQuery()
	{
		m_staticGeometry = nullptr;
		m_dynamicPruner = nullptr;
	}

	GeometryQuery::~GeometryQuery()
	{
		SAFE_DELETE(m_staticGeometry);
		SAFE_DELETE(m_dynamicPruner);

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

			std::vector<Box3> boxes;
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

	bool GeometryQuery::RayCastQuery(const Vector3& Origin, const Vector3& Direction, const RayCastOption& Option, RayCastResult* Result)
	{
		Result->Reset();

		Ray3 ray(Origin, Direction);
		bool hit = false;

		if (m_staticGeometry)
		{
			Geometry** pp = m_Objects.data();
			hit = pp && m_staticGeometry->RayCast(ray, pp, &Option, Result);

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
				Result->Merge(Result2);
				return true;
			}
		}

		return hit;
	}

	bool GeometryQuery::BoxCastQuery(const Vector3& Center, const Vector3& Extent, const Vector3& Direction, const SweepOption& Option, SweepResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Box = GeometryFactory::CreateOBB_placement(stack, Center, Extent);
		return SweepTest_Impl(Box, Direction, Option, Result);
	}

	bool GeometryQuery::SphereCastQuery(const Vector3& Center, float Radius, const Vector3& Direction, const SweepOption& Option, SweepResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Sphere = GeometryFactory::CreateSphere_placement(stack, Center, Radius);
		return SweepTest_Impl(Sphere, Direction, Option, Result);
	}

	bool GeometryQuery::CapsuleCastQuery(const Vector3& Center, float HalfH, float Radius, const Vector3& Direction, const SweepOption& Option, SweepResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Capsule = GeometryFactory::CreateCapsule_placement(stack, Center - Vector3(0, HalfH, 0), Center + Vector3(0, HalfH, 0), Radius);
		return SweepTest_Impl(Capsule, Direction, Option, Result);
	}

	bool GeometryQuery::IntersectQueryBox(const Vector3& Center, const Vector3& Extent, const IntersectOption& Option, IntersectResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Box = GeometryFactory::CreateOBB_placement(stack, Center, Extent);
		return IntersectTest_Impl(Box, Option, Result);
	}

	bool GeometryQuery::IntersectQuerySphere(const Vector3& Center, float Radius, const IntersectOption& Option, IntersectResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Sphere = GeometryFactory::CreateSphere_placement(stack, Center, Radius);
		return IntersectTest_Impl(Sphere, Option, Result);
	}

	bool GeometryQuery::IntersectQueryCapsule(const Vector3& x0, const Vector3& x1, float Radius, const IntersectOption& Option, IntersectResult* Result)
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		Geometry* Capsule = GeometryFactory::CreateCapsule_placement(stack, x0, x1, Radius);
		return IntersectTest_Impl(Capsule, Option, Result);
	}

	bool GeometryQuery::IntersectTest_Impl(const Geometry* geom, const IntersectOption& Option, IntersectResult* Result)
	{
		Result->Reset();

		bool hit = false;

		if (m_staticGeometry)
		{
			Geometry** pp = m_Objects.data();
			hit = pp && m_staticGeometry->Intersect(geom, pp, &Option, Result);
			if (hit && Result->overlapGeoms.size() >= Option.maxOverlaps)
			{
				return true;
			}
		}

		if (m_dynamicPruner)
		{
			IntersectResult Result2;
			bool hit_dynamic = m_dynamicPruner->Intersect(geom, &Option, &Result2);
			if (hit_dynamic)
			{
				Result->Merge(Result2);
				return true;
			}
		}

		return hit;
	}

	bool GeometryQuery::SweepTest_Impl(const Geometry* geom, const Vector3& Direction, const SweepOption& Option, SweepResult* Result)
	{
		Result->Reset();

		bool hit = false;

		if (m_staticGeometry)
		{
			Geometry** pp = m_Objects.data();
			hit = pp && m_staticGeometry->Sweep(geom, pp, Direction, &Option, Result);
			if (hit)
			{
				return true;
			}
		}

		if (m_dynamicPruner)
		{
			SweepResult Result2;
			bool hit_dynamic = m_dynamicPruner->Sweep(geom, Direction, &Option, &Result2);
			if (hit_dynamic)
			{
				Result->Merge(Result2);
				return true;
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
		CollisionTableFilter(unsigned int n, unsigned char* pLayerData)
		{
			nLayers = n;
			collisionTable.resize(n * n);
			memset(&collisionTable[0], 0, sizeof(collisionTable[0]) * n * n);
			if (pLayerData)
			{
				memcpy(&collisionTable[0], pLayerData, sizeof(collisionTable[0]) * n * n);
			}
		}

		virtual ~CollisionTableFilter() override {}

		virtual bool IsCollidable(const CollisionData& data0, const CollisionData& data1) override
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

	CollisionFilter* CollisionFilter::CreateCollisionTable(unsigned int nLayers, unsigned char* pLayerData)
	{
		return new CollisionTableFilter(nLayers, pLayerData);
	}
}


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
		m_dynamicGeometry = nullptr;
	}

	GeometryQuery::~GeometryQuery()
	{
		SAFE_DELETE(m_staticGeometry);
		SAFE_DELETE(m_dynamicGeometry);

		for (size_t i = 0; i < m_Objects.size(); ++i)
		{
			delete m_Objects[i];
		}
	}

	void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
	{
		SAFE_DELETE(m_staticGeometry);
		m_Objects.clear();

		std::vector<Geometry*> queryObjects;
		queryObjects.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			if (Object && Object->IsQueryEnabled())
			{
				queryObjects.push_back(Object);
			}
			else if (Object)
			{
				Object->SetNodeId(-1);
			}
		}

		if (queryObjects.size() > 0)
		{
			m_staticGeometry = new AABBTree;

			std::vector<Box3> boxes;
			boxes.resize(queryObjects.size());
			for (size_t i = 0; i < queryObjects.size(); ++i)
			{
				boxes[i] = queryObjects[i]->GetBoundingVolume_WorldSpace();
			}
			m_Objects = queryObjects;

			AABBTreeBuildData param(&boxes[0], (int)boxes.size(), nPrimitivePerNode);
			m_staticGeometry->StaticBuild(param);
		}
	}

	void GeometryQuery::CreateDynamicGeometry()
	{
		if (m_dynamicGeometry)
		{
			m_dynamicGeometry->Clear();
			return;
		}

		m_dynamicGeometry = new DynamicAABBTree;
	}

	void GeometryQuery::ClearDynamicGeometry()
	{
		if (m_dynamicGeometry)
		{
			m_dynamicGeometry->Clear();
		}
	}

	void GeometryQuery::BuildDynamicGeometry(const std::vector<Geometry*>& Objects)
	{
		m_Objects.clear();

		if (!m_dynamicGeometry)
		{
			m_dynamicGeometry = new DynamicAABBTree;
		}
		else
		{
			m_dynamicGeometry->Clear();
		}

		for (Geometry* Object : Objects)
		{
			if (AddDynamicGeometry(Object) >= 0)
			{
				m_Objects.push_back(Object);
			}
		}
	}

	int GeometryQuery::AddDynamicGeometry(Geometry* Object)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				Object->SetNodeId(-1);
			}
			return -1;
		}

		const int nodeId = m_dynamicGeometry->Add(Object->GetBoundingVolume_WorldSpace(), Object);
		Object->SetNodeId(nodeId);
		return nodeId;
	}

	void GeometryQuery::RemoveDynamicGeometry(Geometry* Object)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr)
		{
			return;
		}

		const int nodeId = Object->GetNodeId();
		if (nodeId < 0)
		{
			return;
		}

		m_dynamicGeometry->Remove(nodeId);
		Object->SetNodeId(-1);
	}

	bool GeometryQuery::UpdateDynamicGeometry(Geometry* Object, const Vector3& displacement)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || !Object->IsQueryEnabled())
		{
			return false;
		}

		const int nodeId = Object->GetNodeId();
		if (nodeId < 0)
		{
			return false;
		}

		return m_dynamicGeometry->Update(nodeId, Object->GetBoundingVolume_WorldSpace(), displacement);
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

		if (m_dynamicGeometry)
		{
			RayCastResult Result2;
			bool hit_dynamic = m_dynamicGeometry->RayCast(ray, &Option, &Result2);
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

	void GeometryQuery::CollectAABBs(std::vector<Box3>* aabbs) const
	{
		if (aabbs == nullptr)
		{
			return;
		}

		if (m_staticGeometry)
		{
			m_staticGeometry->CollectAABBs(aabbs);
		}
		if (m_dynamicGeometry)
		{
			m_dynamicGeometry->CollectAABBs(aabbs);
		}
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

		if (m_dynamicGeometry)
		{
			IntersectResult Result2;
			bool hit_dynamic = m_dynamicGeometry->Intersect(geom, &Option, &Result2);
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

		if (m_dynamicGeometry)
		{
			SweepResult Result2;
			bool hit_dynamic = m_dynamicGeometry->Sweep(geom, Direction, &Option, &Result2);
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

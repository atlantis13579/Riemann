
#include "GeometryQuery.h"

#include <assert.h>
#include <algorithm>
#include "../Core/Base.h"
#include "AABBPruner.h"
#include "AABBTree.h"
#include "DynamicAABBTree.h"

namespace Riemann
{
	GeometryQuery::GeometryQuery()
	{
		m_staticGeometry = nullptr;
		m_staticBucket = nullptr;
		m_staticPrimitivesPerNode = 1;
		m_dynamicGeometry = nullptr;
	}

	GeometryQuery::~GeometryQuery()
	{
		SAFE_DELETE(m_staticGeometry);
		SAFE_DELETE(m_staticBucket);
		SAFE_DELETE(m_dynamicGeometry);

		for (size_t i = 0; i < m_Objects.size(); ++i)
		{
			delete m_Objects[i];
		}
	}

	void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
	{
		m_Objects.clear();
		m_staticObjects.clear();
		m_staticPrimitivesPerNode = nPrimitivePerNode > 0 ? nPrimitivePerNode : 1;

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

		if (m_staticGeometry == nullptr)
		{
			m_staticGeometry = new AABBTree;
		}
		if (m_staticBucket == nullptr)
		{
			m_staticBucket = AABBPruner::CreateBucketStyle();
		}

		m_Objects = queryObjects;
		m_staticObjects = queryObjects;
		m_staticBucket->Clear();
		MarkStaticGeometryDirty();
		CommitStaticGeometry();
	}

	void GeometryQuery::ClearStaticGeometry()
	{
		if (m_staticGeometry)
		{
			m_staticGeometry->Release();
			m_staticGeometry->SetDirty(false);
		}
		if (m_staticBucket)
		{
			m_staticBucket->Clear();
		}
		m_staticObjects.clear();
		m_Objects.clear();
	}

	bool GeometryQuery::AddGeometry(Geometry* Object)
	{
		if (m_dynamicGeometry)
		{
			return AddToDynamic(Object) >= 0;
		}
		return AddToStatic(Object);
	}

	bool GeometryQuery::RemoveGeometry(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		bool removed = false;
		if (m_dynamicGeometry && Object->GetNodeId() >= 0)
		{
			RemoveFromDynamic(Object);
			removed = true;
		}
		if (ContainsStaticGeometry(Object))
		{
			removed = RemoveFromStatic(Object) || removed;
		}
		return removed;
	}

	bool GeometryQuery::UpdateGeometry(Geometry* Object, const Vector3& displacement)
	{
		if (Object == nullptr)
		{
			return false;
		}

		if (m_dynamicGeometry && Object->GetNodeId() >= 0)
		{
			return UpdateDynamicObject(Object, displacement);
		}
		if (ContainsStaticGeometry(Object) || m_dynamicGeometry == nullptr)
		{
			return UpdateStaticObject(Object);
		}
		return UpdateDynamicObject(Object, displacement);
	}

	bool GeometryQuery::AddToStatic(Geometry* Object)
	{
		if (m_staticGeometry == nullptr)
		{
			m_staticGeometry = new AABBTree;
		}
		if (m_staticBucket == nullptr)
		{
			m_staticBucket = AABBPruner::CreateBucketStyle();
		}

		if (Object == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				Object->SetNodeId(-1);
			}
			return false;
		}

		if (std::find(m_Objects.begin(), m_Objects.end(), Object) == m_Objects.end())
		{
			m_Objects.push_back(Object);
		}
		if (!ContainsStaticGeometry(Object))
		{
			m_staticObjects.push_back(Object);
		}

		m_staticBucket->Add(Object);
		MarkStaticGeometryDirty();
		return true;
	}

	bool GeometryQuery::RemoveFromStatic(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		const size_t oldStaticSize = m_staticObjects.size();
		m_staticObjects.erase(std::remove(m_staticObjects.begin(), m_staticObjects.end(), Object), m_staticObjects.end());
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), Object), m_Objects.end());
		if (m_staticBucket)
		{
			m_staticBucket->Remove(Object);
		}

		const bool removed = m_staticObjects.size() != oldStaticSize;
		if (removed)
		{
			Object->SetNodeId(-1);
			MarkStaticGeometryDirty();
		}
		return removed;
	}

	bool GeometryQuery::UpdateStaticObject(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		if (!Object->IsQueryEnabled())
		{
			return RemoveFromStatic(Object);
		}

		if (!ContainsStaticGeometry(Object))
		{
			return AddToStatic(Object);
		}

		if (m_staticBucket == nullptr)
		{
			m_staticBucket = AABBPruner::CreateBucketStyle();
		}
		m_staticBucket->Update(Object);
		MarkStaticGeometryDirty();
		return true;
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
		m_Objects.clear();
	}

	void GeometryQuery::BuildDynamicGeometry(const std::vector<Geometry*>& Objects)
	{
		ClearStaticGeometry();
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
			if (AddToDynamic(Object) >= 0)
			{
				m_Objects.push_back(Object);
			}
		}
	}

	int GeometryQuery::AddToDynamic(Geometry* Object)
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

	void GeometryQuery::RemoveFromDynamic(Geometry* Object)
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
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), Object), m_Objects.end());
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Vector3& displacement)
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

	AABBTree* GeometryQuery::GetStaticTree()
	{
		CommitStaticGeometry();
		return m_staticGeometry;
	}

	const AABBTree* GeometryQuery::GetStaticTree() const
	{
		const_cast<GeometryQuery*>(this)->CommitStaticGeometry();
		return m_staticGeometry;
	}

	bool GeometryQuery::CommitStaticGeometry()
	{
		if (m_staticGeometry == nullptr || !m_staticGeometry->IsDirty())
		{
			return true;
		}

		if (m_staticBucket)
		{
			m_staticBucket->Clear();
		}

		if (m_staticObjects.empty())
		{
			m_staticGeometry->Release();
			m_staticGeometry->SetDirty(false);
			return true;
		}

		std::vector<Box3> boxes;
		boxes.resize(m_staticObjects.size());
		for (size_t i = 0; i < m_staticObjects.size(); ++i)
		{
			boxes[i] = m_staticObjects[i]->GetBoundingVolume_WorldSpace();
		}

		AABBTreeBuildData param(boxes.data(), (int)boxes.size(), m_staticPrimitivesPerNode);
		m_staticGeometry->StaticBuild(param);
		return true;
	}

	void GeometryQuery::MarkStaticGeometryDirty()
	{
		if (m_staticGeometry == nullptr)
		{
			m_staticGeometry = new AABBTree;
		}
		m_staticGeometry->SetDirty(true);
	}

	bool GeometryQuery::ContainsStaticGeometry(const Geometry* Object) const
	{
		return std::find(m_staticObjects.begin(), m_staticObjects.end(), Object) != m_staticObjects.end();
	}

	bool GeometryQuery::RayCastQuery(const Vector3& Origin, const Vector3& Direction, const RayCastOption& Option, RayCastResult* Result)
	{
		Result->Reset();

		Ray3 ray(Origin, Direction);
		bool hit = false;

		if (m_staticGeometry)
		{
			CommitStaticGeometry();
			if (!m_staticObjects.empty())
			{
				RayCastResult staticResult;
				const bool hit_static = m_staticGeometry->RayCast(ray, m_staticObjects.data(), &Option, &staticResult);
				if (hit_static)
				{
					Result->Merge(staticResult);
					hit = true;
				}
			}

			if (Option.Type == RayCastOption::RAYCAST_ANY && hit)
			{
				return true;
			}
		}

		if (m_staticBucket)
		{
			RayCastResult bucketResult;
			const bool hit_bucket = m_staticBucket->RayCast(ray, &Option, &bucketResult);
			if (hit_bucket)
			{
				Result->Merge(bucketResult);
				hit = true;
				if (Option.Type == RayCastOption::RAYCAST_ANY)
				{
					return true;
				}
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
			const_cast<GeometryQuery*>(this)->CommitStaticGeometry();
			m_staticGeometry->CollectAABBs(aabbs);
		}
		if (m_staticBucket)
		{
			m_staticBucket->CollectAABBs(aabbs);
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
			CommitStaticGeometry();
			if (!m_staticObjects.empty())
			{
				IntersectResult staticResult;
				const bool hit_static = m_staticGeometry->Intersect(geom, m_staticObjects.data(), &Option, &staticResult);
				if (hit_static)
				{
					Result->Merge(staticResult);
					hit = true;
				}
			}
			if (hit && Result->overlapGeoms.size() >= Option.maxOverlaps)
			{
				return true;
			}
		}

		if (m_staticBucket)
		{
			IntersectResult bucketResult;
			const bool hit_bucket = m_staticBucket->Intersect(geom, &Option, &bucketResult);
			if (hit_bucket)
			{
				Result->Merge(bucketResult);
				hit = true;
				if (Result->overlapGeoms.size() >= Option.maxOverlaps)
				{
					return true;
				}
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
			CommitStaticGeometry();
			if (!m_staticObjects.empty())
			{
				SweepResult staticResult;
				const bool hit_static = m_staticGeometry->Sweep(geom, m_staticObjects.data(), Direction, &Option, &staticResult);
				if (hit_static)
				{
					Result->Merge(staticResult);
					hit = true;
				}
			}
			if (hit)
			{
				return true;
			}
		}

		if (m_staticBucket)
		{
			SweepResult bucketResult;
			const bool hit_bucket = m_staticBucket->Sweep(geom, Direction, &Option, &bucketResult);
			if (hit_bucket)
			{
				Result->Merge(bucketResult);
				hit = true;
				if (Option.Type == SweepOption::SWEEP_ANY)
				{
					return true;
				}
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

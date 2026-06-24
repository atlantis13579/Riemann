
#include "GeometryQuery.h"

#include <assert.h>
#include <algorithm>
#include "../Core/Base.h"
#include "AABBPruner.h"
#include "AABBTree.h"
#include "DynamicAABBTree.h"

namespace Riemann
{
	static bool RayCastProxyGeometry(const Ray3& ray, Geometry* geometry, const RayCastOption* option, RayCastResult* result, void* context)
	{
		GeometryQuery* query = static_cast<GeometryQuery*>(context);
		const GeometryQueryProxy* proxy = query ? query->GetProxy(geometry) : nullptr;
		const Transform transform = proxy ? proxy->ShapeToWorld : *geometry->GetTransform();
		return geometry->RayCast(transform, ray.Origin, ray.Dir, option, result);
	}

	static bool RayCastProxyGeometryDynamic(const Ray3& ray, void* userData, const RayCastOption* option, RayCastResult* result, void* context)
	{
		Geometry* geometry = static_cast<Geometry*>(userData);
		if (geometry == nullptr || !RayCastProxyGeometry(ray, geometry, option, result, context))
		{
			return false;
		}

		result->hit = true;
		if (option->Type == RayCastOption::RAYCAST_PENETRATE)
		{
			result->hitGeometries.push_back(geometry);
		}

		if (result->hitTime < result->hitTimeMin)
		{
			result->hitTimeMin = result->hitTime;
			result->hitGeom = geometry;
		}
		return true;
	}

	static bool IntersectProxyGeometry(const Geometry* queryGeometry, Geometry* geometry, const IntersectOption* option, IntersectResult* result, void* context)
	{
		(void)option;
		(void)result;
		GeometryQuery* query = static_cast<GeometryQuery*>(context);
		const GeometryQueryProxy* proxy = query ? query->GetProxy(geometry) : nullptr;
		const Transform queryTransform = *queryGeometry->GetTransform();
		const Transform geometryTransform = proxy ? proxy->ShapeToWorld : *geometry->GetTransform();
		return queryGeometry->Intersect(queryTransform, geometry, geometryTransform);
	}

	static bool IntersectProxyGeometryDynamic(const Geometry* queryGeometry, void* userData, const IntersectOption* option, IntersectResult* result, void* context)
	{
		Geometry* geometry = static_cast<Geometry*>(userData);
		if (geometry == nullptr)
		{
			return false;
		}
		if (option->Filter && !option->Filter->IsCollidable(option->FilterData, geometry->GetFilterData()))
		{
			return false;
		}

		result->AddTestCount(1);
		const bool overlap = IntersectProxyGeometry(queryGeometry, geometry, option, result, context);
		if (overlap)
		{
			result->overlaps = true;
			if (result->overlapGeoms.size() < option->maxOverlaps)
			{
				result->overlapGeoms.push_back(geometry);
			}
		}
		return overlap;
	}

	static bool SweepProxyGeometry(const Geometry* queryGeometry, Geometry* geometry, const Vector3& direction, const SweepOption* option, SweepResult* result, void* context)
	{
		(void)option;
		GeometryQuery* query = static_cast<GeometryQuery*>(context);
		const GeometryQueryProxy* proxy = query ? query->GetProxy(geometry) : nullptr;
		const Transform queryTransform = *queryGeometry->GetTransform();
		const Transform geometryTransform = proxy ? proxy->ShapeToWorld : *geometry->GetTransform();

		float t;
		Vector3 position;
		Vector3 normal;
		if (!queryGeometry->Sweep(queryTransform, direction, geometry, geometryTransform, &position, &normal, &t))
		{
			return false;
		}

		result->hit = true;
		result->hitTime = t;
		result->hitPosition = position;
		result->hitNormal = normal;
		return true;
	}

	static bool SweepProxyGeometryDynamic(const Geometry* queryGeometry, const Vector3& direction, void* userData, const SweepOption* option, SweepResult* result, void* context)
	{
		Geometry* geometry = static_cast<Geometry*>(userData);
		if (geometry == nullptr)
		{
			return false;
		}
		SweepResult candidate;
		if (!SweepProxyGeometry(queryGeometry, geometry, direction, option, &candidate, context))
		{
			return false;
		}

		result->hit = true;
		result->hitTime = candidate.hitTime;
		if (option->Type == SweepOption::SWEEP_PENETRATE)
		{
			result->hitGeometries.push_back(geometry);
		}
		if (candidate.hitTime < result->hitTimeMin)
		{
			result->hitTimeMin = candidate.hitTime;
			result->hitNormal = candidate.hitNormal;
			result->hitPosition = candidate.hitPosition;
			result->hitGeom = geometry;
		}
		return true;
	}

	GeometryQuery::GeometryQuery()
	{
		m_staticGeometry = nullptr;
		m_staticBucket = nullptr;
		m_staticPrimitivesPerNode = 1;
		m_dynamicGeometry = nullptr;
		m_FrameId = 0;
	}

	GeometryQuery::~GeometryQuery()
	{
		SAFE_DELETE(m_staticGeometry);
		SAFE_DELETE(m_staticBucket);
		SAFE_DELETE(m_dynamicGeometry);
		m_Objects.clear();
		m_staticObjects.clear();
		m_Proxies.clear();
	}

	void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, int nPrimitivePerNode)
	{
		std::vector<Box3> bounds;
		std::vector<Transform> shapeToWorld;
		std::vector<RigidBody*> bodies;
		bounds.reserve(Objects.size());
		shapeToWorld.reserve(Objects.size());
		bodies.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			bounds.push_back(Object ? Object->GetBounds() : Box3::Empty());
			shapeToWorld.push_back(Object ? *Object->GetTransform() : Transform::Identity());
			bodies.push_back(nullptr);
		}
		BuildStaticGeometry(Objects, bounds, shapeToWorld, bodies, nPrimitivePerNode);
	}

	void GeometryQuery::BuildStaticGeometry(const std::vector<Geometry*>& Objects, const std::vector<Box3>& Bounds, const std::vector<Transform>& ShapeToWorld, const std::vector<RigidBody*>& Bodies, int nPrimitivePerNode)
	{
		ClearStaticGeometry();
		m_staticPrimitivesPerNode = nPrimitivePerNode > 0 ? nPrimitivePerNode : 1;

		if (m_staticGeometry == nullptr)
		{
			m_staticGeometry = new AABBTree;
		}

		for (size_t i = 0; i < Objects.size(); ++i)
		{
			Geometry* Object = Objects[i];
			const Box3 bounds = i < Bounds.size() ? Bounds[i] : (Object ? Object->GetBounds() : Box3::Empty());
			const Transform transform = i < ShapeToWorld.size() ? ShapeToWorld[i] : (Object ? *Object->GetTransform() : Transform::Identity());
			RigidBody* body = i < Bodies.size() ? Bodies[i] : nullptr;
			AddToStatic(Object, bounds, transform, body);
		}

		CommitStaticGeometry();
	}

	void GeometryQuery::ClearStaticGeometry()
	{
		for (Geometry* Object : m_Objects)
		{
			if (Object)
			{
				Object->SetNodeId(-1);
			}
		}
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
		m_Proxies.clear();
	}

	GeometryQueryProxy* GeometryQuery::FindProxy(Geometry* Object)
	{
		auto iter = m_Proxies.find(Object);
		return iter == m_Proxies.end() ? nullptr : &iter->second;
	}

	const GeometryQueryProxy* GeometryQuery::FindProxy(const Geometry* Object) const
	{
		auto iter = m_Proxies.find(const_cast<Geometry*>(Object));
		return iter == m_Proxies.end() ? nullptr : &iter->second;
	}

	const GeometryQueryProxy* GeometryQuery::GetProxy(const Geometry* Object) const
	{
		return FindProxy(Object);
	}

	GeometryQueryProxy& GeometryQuery::UpdateProxy(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement, bool Dynamic)
	{
		GeometryQueryProxy& proxy = m_Proxies[Object];
		const bool isNew = proxy.Geom == nullptr;
		const bool changed = isNew
			|| proxy.Body != Body
			|| proxy.ShapeToWorld != ShapeToWorld
			|| !(proxy.WorldBounds == Bounds)
			|| proxy.Dynamic != Dynamic;

		proxy.Geom = Object;
		proxy.Body = Body;
		proxy.Displacement = displacement;
		proxy.ShapeToWorld = ShapeToWorld;
		proxy.WorldBounds = Bounds;
		proxy.Dynamic = Dynamic;
		proxy.FrameId = ++m_FrameId;
		proxy.Moved = changed || displacement != Vector3::Zero();
		proxy.Dirty = proxy.Moved;
		if (changed)
		{
			proxy.Version = proxy.Version == 0 ? 1 : proxy.Version + 1;
		}
		return proxy;
	}

	void GeometryQuery::RemoveProxy(Geometry* Object)
	{
		m_Proxies.erase(Object);
	}

	Transform GeometryQuery::GetProxyTransform(const Geometry* Object) const
	{
		const GeometryQueryProxy* proxy = FindProxy(Object);
		return proxy ? proxy->ShapeToWorld : *Object->GetTransform();
	}

	Box3 GeometryQuery::GetProxyBounds(const Geometry* Object) const
	{
		const GeometryQueryProxy* proxy = FindProxy(Object);
		return proxy ? proxy->WorldBounds : Object->GetBounds();
	}

	bool GeometryQuery::AddGeometry(Geometry* Object)
	{
		return AddGeometry(Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	bool GeometryQuery::AddGeometry(Geometry* Object, const Box3& Bounds)
	{
		return AddGeometry(Object, Bounds, Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	bool GeometryQuery::AddGeometry(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		if (m_dynamicGeometry)
		{
			return AddToDynamic(Object, Bounds, ShapeToWorld, Body) >= 0;
		}
		return AddToStatic(Object, Bounds, ShapeToWorld, Body);
	}

	bool GeometryQuery::RemoveGeometry(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		bool removed = false;
		const GeometryQueryProxy* proxy = FindProxy(Object);
		if (proxy && proxy->Dynamic)
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

		return UpdateGeometry(Object, Object->GetBounds(), *Object->GetTransform(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateGeometry(Geometry* Object, const Box3& Bounds, const Vector3& displacement)
	{
		if (Object == nullptr)
		{
			return false;
		}

		return UpdateGeometry(Object, Bounds, *Object->GetTransform(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateGeometry(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
	{
		if (Object == nullptr)
		{
			return false;
		}

		const GeometryQueryProxy* proxy = FindProxy(Object);
		if (m_dynamicGeometry && (!proxy || proxy->Dynamic))
		{
			return UpdateDynamicObject(Object, Bounds, ShapeToWorld, Body, displacement);
		}
		return UpdateStaticObject(Object, Bounds, ShapeToWorld, Body, displacement);
	}

	bool GeometryQuery::AddToStatic(Geometry* Object)
	{
		return AddToStatic(Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	bool GeometryQuery::AddToStatic(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		if (Object == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				Object->SetNodeId(-1);
				RemoveProxy(Object);
			}
			return false;
		}

		if (m_staticGeometry == nullptr)
		{
			m_staticGeometry = new AABBTree;
		}

		if (std::find(m_Objects.begin(), m_Objects.end(), Object) == m_Objects.end())
		{
			m_Objects.push_back(Object);
		}
		if (!ContainsStaticGeometry(Object))
		{
			m_staticObjects.push_back(Object);
		}

		GeometryQueryProxy& proxy = UpdateProxy(Object, Bounds, ShapeToWorld, Body, Vector3::Zero(), false);
		proxy.PrunerHandle = -1;
		proxy.Dirty = false;
		Object->SetNodeId(-1);
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
			RemoveProxy(Object);
			MarkStaticGeometryDirty();
		}
		return removed;
	}

	bool GeometryQuery::UpdateStaticObject(Geometry* Object)
	{
		return UpdateStaticObject(Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr, Vector3::Zero());
	}

	bool GeometryQuery::UpdateStaticObject(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
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
			return AddToStatic(Object, Bounds, ShapeToWorld, Body);
		}

		GeometryQueryProxy& proxy = UpdateProxy(Object, Bounds, ShapeToWorld, Body, displacement, false);
		proxy.PrunerHandle = -1;
		if (proxy.Dirty)
		{
			MarkStaticGeometryDirty();
			proxy.Dirty = false;
		}
		return true;
	}

	void GeometryQuery::CreateDynamicGeometry()
	{
		if (m_dynamicGeometry)
		{
			ClearDynamicGeometry();
			return;
		}

		m_dynamicGeometry = new DynamicAABBTree;
	}

	void GeometryQuery::ClearDynamicGeometry()
	{
		for (Geometry* Object : m_Objects)
		{
			if (Object)
			{
				Object->SetNodeId(-1);
			}
		}
		if (m_dynamicGeometry)
		{
			m_dynamicGeometry->Clear();
		}
		m_Objects.clear();
		m_Proxies.clear();
	}

	void GeometryQuery::BuildDynamicGeometry(const std::vector<Geometry*>& Objects)
	{
		std::vector<Box3> bounds;
		std::vector<Transform> shapeToWorld;
		std::vector<RigidBody*> bodies;
		bounds.reserve(Objects.size());
		shapeToWorld.reserve(Objects.size());
		bodies.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			bounds.push_back(Object ? Object->GetBounds() : Box3::Empty());
			shapeToWorld.push_back(Object ? *Object->GetTransform() : Transform::Identity());
			bodies.push_back(nullptr);
		}
		BuildDynamicGeometry(Objects, bounds, shapeToWorld, bodies);
	}

	void GeometryQuery::BuildDynamicGeometry(const std::vector<Geometry*>& Objects, const std::vector<Box3>& Bounds)
	{
		std::vector<Transform> shapeToWorld;
		std::vector<RigidBody*> bodies;
		shapeToWorld.reserve(Objects.size());
		bodies.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			shapeToWorld.push_back(Object ? *Object->GetTransform() : Transform::Identity());
			bodies.push_back(nullptr);
		}
		BuildDynamicGeometry(Objects, Bounds, shapeToWorld, bodies);
	}

	void GeometryQuery::BuildDynamicGeometry(const std::vector<Geometry*>& Objects, const std::vector<Box3>& Bounds, const std::vector<Transform>& ShapeToWorld, const std::vector<RigidBody*>& Bodies)
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

		for (size_t i = 0; i < Objects.size(); ++i)
		{
			Geometry* Object = Objects[i];
			const Box3 bounds = i < Bounds.size() ? Bounds[i] : (Object ? Object->GetBounds() : Box3::Empty());
			const Transform transform = i < ShapeToWorld.size() ? ShapeToWorld[i] : (Object ? *Object->GetTransform() : Transform::Identity());
			RigidBody* body = i < Bodies.size() ? Bodies[i] : nullptr;
			AddToDynamic(Object, bounds, transform, body);
		}
	}

	int GeometryQuery::AddToDynamic(Geometry* Object)
	{
		return AddToDynamic(Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	int GeometryQuery::AddToDynamic(Geometry* Object, const Box3& Bounds)
	{
		return AddToDynamic(Object, Bounds, Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	int GeometryQuery::AddToDynamic(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				Object->SetNodeId(-1);
				RemoveProxy(Object);
			}
			return -1;
		}

		GeometryQueryProxy* existing = FindProxy(Object);
		if (existing && existing->Dynamic && existing->PrunerHandle >= 0)
		{
			UpdateDynamicObject(Object, Bounds, ShapeToWorld, Body, Vector3::Zero());
			return existing->PrunerHandle;
		}

		const int nodeId = m_dynamicGeometry->Add(Bounds, Object);
		GeometryQueryProxy& proxy = UpdateProxy(Object, Bounds, ShapeToWorld, Body, Vector3::Zero(), true);
		proxy.PrunerHandle = nodeId;
		proxy.Dirty = false;
		Object->SetNodeId(nodeId);
		if (std::find(m_Objects.begin(), m_Objects.end(), Object) == m_Objects.end())
		{
			m_Objects.push_back(Object);
		}
		return nodeId;
	}

	void GeometryQuery::RemoveFromDynamic(Geometry* Object)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr)
		{
			return;
		}

		GeometryQueryProxy* proxy = FindProxy(Object);
		const int nodeId = proxy ? proxy->PrunerHandle : Object->GetNodeId();
		if (nodeId < 0)
		{
			Object->SetNodeId(-1);
			RemoveProxy(Object);
			return;
		}

		m_dynamicGeometry->Remove(nodeId);
		Object->SetNodeId(-1);
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), Object), m_Objects.end());
		RemoveProxy(Object);
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Vector3& displacement)
	{
		return UpdateDynamicObject(Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Box3& Bounds, const Vector3& displacement)
	{
		return UpdateDynamicObject(Object, Bounds, Object ? *Object->GetTransform() : Transform::Identity(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || !Object->IsQueryEnabled())
		{
			if (Object && !Object->IsQueryEnabled())
			{
				RemoveFromDynamic(Object);
			}
			return false;
		}

		GeometryQueryProxy* proxy = FindProxy(Object);
		if (proxy == nullptr || !proxy->Dynamic || proxy->PrunerHandle < 0)
		{
			return AddToDynamic(Object, Bounds, ShapeToWorld, Body) >= 0;
		}

		const int nodeId = proxy->PrunerHandle;
		if (nodeId < 0)
		{
			return false;
		}

		GeometryQueryProxy& updatedProxy = UpdateProxy(Object, Bounds, ShapeToWorld, Body, displacement, true);
		updatedProxy.PrunerHandle = nodeId;
		Object->SetNodeId(nodeId);
		if (!updatedProxy.Dirty)
		{
			return true;
		}

		m_dynamicGeometry->Update(nodeId, Bounds, displacement);
		updatedProxy.Dirty = false;
		return true;
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
			boxes[i] = GetProxyBounds(m_staticObjects[i]);
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
				const bool hit_static = m_staticGeometry->RayCast(ray, m_staticObjects.data(), &Option, &staticResult, RayCastProxyGeometry, this);
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

		if (m_dynamicGeometry)
		{
			RayCastResult Result2;
			bool hit_dynamic = m_dynamicGeometry->RayCast(ray, &Option, &Result2, RayCastProxyGeometryDynamic, this);
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
				const bool hit_static = m_staticGeometry->Intersect(geom, m_staticObjects.data(), &Option, &staticResult, IntersectProxyGeometry, this);
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

		if (m_dynamicGeometry)
		{
			IntersectResult Result2;
			bool hit_dynamic = m_dynamicGeometry->Intersect(geom, &Option, &Result2, IntersectProxyGeometryDynamic, this);
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
				const bool hit_static = m_staticGeometry->Sweep(geom, m_staticObjects.data(), Direction, &Option, &staticResult, SweepProxyGeometry, this);
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

		if (m_dynamicGeometry)
		{
			SweepResult Result2;
			bool hit_dynamic = m_dynamicGeometry->Sweep(geom, Direction, &Option, &Result2, SweepProxyGeometryDynamic, this);
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

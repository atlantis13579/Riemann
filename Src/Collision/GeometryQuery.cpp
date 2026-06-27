
#include "GeometryQuery.h"

#include <assert.h>
#include <algorithm>
#include "../Core/Base.h"
#include "AABBPruner.h"
#include "AABBTree.h"
#include "DynamicAABBTree.h"

namespace Riemann
{
	static void* EncodeProxyIdAsUserData(uint32_t index)
	{
		return reinterpret_cast<void*>((uintptr_t)index + 1);
	}

	static uint32_t DecodeProxyIdFromUserData(void* userData)
	{
		const uintptr_t encoded = reinterpret_cast<uintptr_t>(userData);
		return encoded == 0 ? kInvalidGeometryHandleIndex : (uint32_t)(encoded - 1);
	}

	static GeometryQueryProxy* GetQueryProxy(AABBTreePayloadId payloadId, void* context)
	{
		GeometryQuery* query = static_cast<GeometryQuery*>(context);
		if (query == nullptr)
		{
			return nullptr;
		}
		return query->GetProxyByIndex(payloadId);
	}

	static GeometryQueryProxy* GetQueryProxy(void* userData, void* context)
	{
		return GetQueryProxy(DecodeProxyIdFromUserData(userData), context);
	}

	static bool RayCastProxyGeometry(const Ray3& ray, AABBTreePayloadId payloadId, const RayCastOption* option, RayCastResult* result, void* context)
	{
		GeometryQueryProxy* proxy = GetQueryProxy(payloadId, context);
		Geometry* geometry = proxy ? proxy->Geom : nullptr;
		if (geometry == nullptr)
		{
			return false;
		}

		const Transform& transform = proxy && proxy->ShapeToWorld ? *proxy->ShapeToWorld : *geometry->GetTransform();
		if (!geometry->RayCast(transform, ray.Origin, ray.Dir, option, result))
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

	static bool RayCastProxyGeometry(const Ray3& ray, void* userData, const RayCastOption* option, RayCastResult* result, void* context)
	{
		return RayCastProxyGeometry(ray, DecodeProxyIdFromUserData(userData), option, result, context);
	}

	static bool IntersectProxyGeometry(const Geometry* queryGeometry, AABBTreePayloadId payloadId, const IntersectOption* option, IntersectResult* result, void* context)
	{
		GeometryQueryProxy* proxy = GetQueryProxy(payloadId, context);
		Geometry* geometry = proxy ? proxy->Geom : nullptr;
		if (geometry == nullptr)
		{
			return false;
		}
		if (option->Filter && !option->Filter->IsCollidable(option->FilterData, geometry->GetFilterData()))
		{
			return false;
		}

		result->AddTestCount(1);
		const Transform queryTransform = *queryGeometry->GetTransform();
		const Transform& geometryTransform = proxy && proxy->ShapeToWorld ? *proxy->ShapeToWorld : *geometry->GetTransform();
		const bool overlap = queryGeometry->Intersect(queryTransform, geometry, geometryTransform);
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

	static bool IntersectProxyGeometry(const Geometry* queryGeometry, void* userData, const IntersectOption* option, IntersectResult* result, void* context)
	{
		return IntersectProxyGeometry(queryGeometry, DecodeProxyIdFromUserData(userData), option, result, context);
	}

	static bool SweepProxyGeometry(const Geometry* queryGeometry, const Vector3& direction, AABBTreePayloadId payloadId, const SweepOption* option, SweepResult* result, void* context)
	{
		GeometryQueryProxy* proxy = GetQueryProxy(payloadId, context);
		Geometry* geometry = proxy ? proxy->Geom : nullptr;
		if (geometry == nullptr)
		{
			return false;
		}
		if (option->Filter && !option->Filter->IsCollidable(option->FilterData, geometry->GetFilterData()))
		{
			return false;
		}

		const Transform queryTransform = *queryGeometry->GetTransform();
		const Transform& geometryTransform = proxy && proxy->ShapeToWorld ? *proxy->ShapeToWorld : *geometry->GetTransform();

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
		if (option->Type == SweepOption::SWEEP_PENETRATE)
		{
			result->hitGeometries.push_back(geometry);
		}
		if (t < result->hitTimeMin)
		{
			result->hitTimeMin = t;
			result->hitNormal = normal;
			result->hitPosition = position;
			result->hitGeom = geometry;
		}
		return true;
	}

	static bool SweepProxyGeometry(const Geometry* queryGeometry, const Vector3& direction, void* userData, const SweepOption* option, SweepResult* result, void* context)
	{
		return SweepProxyGeometry(queryGeometry, direction, DecodeProxyIdFromUserData(userData), option, result, context);
	}

	GeometryQuery::GeometryQuery()
	{
		m_staticGeometry = nullptr;
		m_staticBucket = nullptr;
		m_staticPrimitivesPerNode = 1;
		m_dynamicGeometry = nullptr;
		m_NextLocalHandle = 0;
	}

	GeometryQuery::~GeometryQuery()
	{
		SAFE_DELETE(m_staticGeometry);
		SAFE_DELETE(m_staticBucket);
		SAFE_DELETE(m_dynamicGeometry);
		m_Objects.clear();
		m_staticObjects.clear();
		m_Proxies.clear();
		m_LocalStates.clear();
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
			GeometryQueryProxy* existing = FindProxy(Object);
			const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
			AddToStatic(handle, Object, bounds, transform, body);
		}

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
		m_staticProxyIds.clear();
		m_Objects.clear();
		m_Proxies.clear();
		m_LocalStates.clear();
		m_NextLocalHandle = 0;
	}

	GeometryQueryProxy* GeometryQuery::FindProxy(Geometry* Object)
	{
		for (GeometryQueryProxy& proxy : m_Proxies)
		{
			if (proxy.IsActive() && proxy.Geom == Object)
			{
				return &proxy;
			}
		}
		return nullptr;
	}

	const GeometryQueryProxy* GeometryQuery::FindProxy(const Geometry* Object) const
	{
		for (const GeometryQueryProxy& proxy : m_Proxies)
		{
			if (proxy.IsActive() && proxy.Geom == Object)
			{
				return &proxy;
			}
		}
		return nullptr;
	}

	const GeometryQueryProxy* GeometryQuery::GetProxy(const Geometry* Object) const
	{
		return FindProxy(Object);
	}

	const GeometryQueryProxy* GeometryQuery::GetProxy(GeometryHandle Handle) const
	{
		const GeometryQueryProxy* proxy = GetProxyByIndex(Handle.Index);
		return proxy && proxy->Active && proxy->Handle == Handle ? proxy : nullptr;
	}

	GeometryQueryProxy* GeometryQuery::GetProxyByIndex(uint32_t Index)
	{
		if (Index >= m_Proxies.size())
		{
			return nullptr;
		}
		return m_Proxies[Index].IsActive() ? &m_Proxies[Index] : nullptr;
	}

	const GeometryQueryProxy* GeometryQuery::GetProxyByIndex(uint32_t Index) const
	{
		if (Index >= m_Proxies.size())
		{
			return nullptr;
		}
		return m_Proxies[Index].IsActive() ? &m_Proxies[Index] : nullptr;
	}

	GeometryHandle GeometryQuery::AllocateLocalHandle()
	{
		while (m_NextLocalHandle < m_Proxies.size() && m_Proxies[m_NextLocalHandle].IsActive())
		{
			++m_NextLocalHandle;
		}
		return GeometryHandle(m_NextLocalHandle++, 1);
	}

	GeometryQueryLocalState* GeometryQuery::EnsureLocalState(GeometryHandle Handle)
	{
		if (!Handle.IsValid())
		{
			return nullptr;
		}
		if (Handle.Index >= m_LocalStates.size())
		{
			m_LocalStates.resize((size_t)Handle.Index + 1);
		}
		return &m_LocalStates[Handle.Index];
	}

	GeometryQueryProxy* GeometryQuery::EnsureProxy(GeometryHandle Handle)
	{
		if (!Handle.IsValid())
		{
			return nullptr;
		}
		if (Handle.Index >= m_Proxies.size())
		{
			m_Proxies.resize((size_t)Handle.Index + 1);
		}

		GeometryQueryProxy& proxy = m_Proxies[Handle.Index];
		if (proxy.IsActive() && proxy.Handle.Generation != Handle.Generation)
		{
			proxy = GeometryQueryProxy();
		}
		proxy.Handle = Handle;
		proxy.SetActive(true);
		return &proxy;
	}

	GeometryQueryProxy& GeometryQuery::UpdateProxy(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld, bool Dynamic)
	{
		GeometryQueryProxy* proxyPtr = EnsureProxy(Handle);
		assert(proxyPtr != nullptr);
		GeometryQueryProxy& proxy = *proxyPtr;

		proxy.Geom = Object;
		proxy.ShapeToWorld = ShapeToWorld;
		proxy.WorldBounds = Bounds;
		proxy.SetDynamic(Dynamic);
		proxy.SetDirty(true);
		return proxy;
	}

	void GeometryQuery::RemoveProxy(GeometryHandle Handle)
	{
		if (!Handle.IsValid() || Handle.Index >= m_Proxies.size())
		{
			return;
		}

		GeometryQueryProxy& proxy = m_Proxies[Handle.Index];
		if (!proxy.IsActive() || proxy.Handle != Handle)
		{
			return;
		}
		proxy = GeometryQueryProxy();
	}

	void GeometryQuery::RemoveProxy(Geometry* Object)
	{
		GeometryQueryProxy* proxy = FindProxy(Object);
		if (proxy)
		{
			RemoveProxy(proxy->Handle);
		}
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
		(void)Body;
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return AddGeometry(handle, Object, Bounds, ShapeToWorld, nullptr);
	}

	bool GeometryQuery::AddGeometry(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld)
	{
		if (m_dynamicGeometry)
		{
			return AddToDynamic(Handle, Object, Bounds, ShapeToWorld) >= 0;
		}
		return AddToStatic(Handle, Object, Bounds, ShapeToWorld);
	}

	bool GeometryQuery::AddGeometry(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		(void)Body;
		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return false;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return AddGeometry(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld);
	}

	bool GeometryQuery::RemoveGeometry(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		bool removed = false;
		GeometryQueryProxy* proxy = FindProxy(Object);
		if (proxy && proxy->IsDynamic())
		{
			RemoveFromDynamic(proxy->Handle);
			removed = true;
		}
		if (ContainsStaticGeometry(Object))
		{
			proxy = FindProxy(Object);
			removed = (proxy ? RemoveFromStatic(proxy->Handle) : RemoveFromStatic(Object)) || removed;
		}
		return removed;
	}

	bool GeometryQuery::RemoveGeometry(GeometryHandle Handle)
	{
		GeometryQueryProxy* proxy = GetProxyByIndex(Handle.Index);
		if (proxy == nullptr || proxy->Handle != Handle)
		{
			return false;
		}

		bool removed = false;
		if (proxy->IsDynamic())
		{
			RemoveFromDynamic(Handle);
			removed = true;
		}
		else
		{
			removed = RemoveFromStatic(Handle);
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
		(void)Body;
		if (Object == nullptr)
		{
			return false;
		}

		const GeometryQueryProxy* proxy = FindProxy(Object);
		const GeometryHandle handle = proxy ? proxy->Handle : AllocateLocalHandle();
		return UpdateGeometry(handle, Object, Bounds, ShapeToWorld, nullptr, displacement);
	}

	bool GeometryQuery::UpdateGeometry(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld, const Vector3& displacement)
	{
		if (Object == nullptr || Bounds == nullptr || ShapeToWorld == nullptr)
		{
			return false;
		}

		const GeometryQueryProxy* proxy = GetProxy(Handle);
		if (m_dynamicGeometry && (!proxy || proxy->IsDynamic()))
		{
			return UpdateDynamicObject(Handle, Object, Bounds, ShapeToWorld, displacement);
		}
		return UpdateStaticObject(Handle, Object, Bounds, ShapeToWorld, displacement);
	}

	bool GeometryQuery::UpdateGeometry(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
	{
		(void)Body;
		if (Object == nullptr)
		{
			return false;
		}

		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return false;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return UpdateGeometry(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld, displacement);
	}

	bool GeometryQuery::AddToStatic(Geometry* Object)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return AddToStatic(handle, Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	bool GeometryQuery::AddToStatic(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld)
	{
		if (Object == nullptr || Bounds == nullptr || ShapeToWorld == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				RemoveProxy(Handle);
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

		const bool isNewStatic = !ContainsStaticGeometry(Object);
		GeometryQueryProxy& proxy = UpdateProxy(Handle, Object, Bounds, ShapeToWorld, false);
		proxy.PrunerHandle = -1;
		proxy.SetDirty(false);
		if (isNewStatic)
		{
			m_staticObjects.push_back(Object);
			m_staticProxyIds.push_back(proxy.Handle.Index);
		}
		MarkStaticGeometryDirty();
		return true;
	}

	bool GeometryQuery::AddToStatic(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		(void)Body;
		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return false;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return AddToStatic(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld);
	}

	bool GeometryQuery::RemoveFromStatic(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return false;
		}

		bool removed = false;
		for (size_t i = 0; i < m_staticObjects.size();)
		{
			if (m_staticObjects[i] == Object)
			{
				m_staticObjects.erase(m_staticObjects.begin() + i);
				m_staticProxyIds.erase(m_staticProxyIds.begin() + i);
				removed = true;
				continue;
			}
			++i;
		}
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), Object), m_Objects.end());
		if (m_staticBucket)
		{
			m_staticBucket->Remove(Object);
		}

		if (removed)
		{
			RemoveProxy(Object);
			MarkStaticGeometryDirty();
		}
		return removed;
	}

	bool GeometryQuery::RemoveFromStatic(GeometryHandle Handle)
	{
		GeometryQueryProxy* proxy = GetProxyByIndex(Handle.Index);
		if (proxy == nullptr || proxy->Handle != Handle)
		{
			return false;
		}

		Geometry* Object = proxy->Geom;
		bool removed = false;
		for (size_t i = 0; i < m_staticObjects.size();)
		{
			if (m_staticObjects[i] == Object)
			{
				m_staticObjects.erase(m_staticObjects.begin() + i);
				m_staticProxyIds.erase(m_staticProxyIds.begin() + i);
				removed = true;
				continue;
			}
			++i;
		}
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), Object), m_Objects.end());
		if (m_staticBucket)
		{
			m_staticBucket->Remove(Object);
		}

		if (removed)
		{
			RemoveProxy(Handle);
			MarkStaticGeometryDirty();
		}
		return removed;
	}

	bool GeometryQuery::UpdateStaticObject(Geometry* Object)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return UpdateStaticObject(handle, Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr, Vector3::Zero());
	}

	bool GeometryQuery::UpdateStaticObject(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld, const Vector3& displacement)
	{
		(void)displacement;
		if (Object == nullptr || Bounds == nullptr || ShapeToWorld == nullptr)
		{
			return false;
		}

		if (!Object->IsQueryEnabled())
		{
			return RemoveFromStatic(Object);
		}

		if (!ContainsStaticGeometry(Object))
		{
			return AddToStatic(Handle, Object, Bounds, ShapeToWorld);
		}

		GeometryQueryProxy& proxy = UpdateProxy(Handle, Object, Bounds, ShapeToWorld, false);
		proxy.PrunerHandle = -1;
		if (proxy.IsDirty())
		{
			MarkStaticGeometryDirty();
			proxy.SetDirty(false);
		}
		return true;
	}

	bool GeometryQuery::UpdateStaticObject(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
	{
		(void)Body;
		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return false;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return UpdateStaticObject(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld, displacement);
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
		if (m_dynamicGeometry)
		{
			m_dynamicGeometry->Clear();
		}
		m_Objects.clear();
		m_Proxies.clear();
		m_LocalStates.clear();
		m_NextLocalHandle = 0;
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
			GeometryQueryProxy* existing = FindProxy(Object);
			const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
			AddToDynamic(handle, Object, bounds, transform, body);
		}
	}

	int GeometryQuery::AddToDynamic(Geometry* Object)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return AddToDynamic(handle, Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	int GeometryQuery::AddToDynamic(Geometry* Object, const Box3& Bounds)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return AddToDynamic(handle, Object, Bounds, Object ? *Object->GetTransform() : Transform::Identity(), nullptr);
	}

	int GeometryQuery::AddToDynamic(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || Bounds == nullptr || ShapeToWorld == nullptr || !Object->IsQueryEnabled())
		{
			if (Object)
			{
				RemoveProxy(Handle);
			}
			return -1;
		}

		GeometryQueryProxy* existing = GetProxyByIndex(Handle.Index);
		if (existing && existing->Handle != Handle)
		{
			existing = nullptr;
		}
		if (existing && existing->IsDynamic() && existing->PrunerHandle >= 0)
		{
			UpdateDynamicObject(Handle, Object, Bounds, ShapeToWorld, Vector3::Zero());
			return existing->PrunerHandle;
		}

		GeometryQueryProxy& proxy = UpdateProxy(Handle, Object, Bounds, ShapeToWorld, true);
		const int nodeId = m_dynamicGeometry->Add(*Bounds, EncodeProxyIdAsUserData(proxy.Handle.Index));
		proxy.PrunerHandle = nodeId;
		proxy.SetDirty(false);
		if (std::find(m_Objects.begin(), m_Objects.end(), Object) == m_Objects.end())
		{
			m_Objects.push_back(Object);
		}
		return nodeId;
	}

	int GeometryQuery::AddToDynamic(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body)
	{
		(void)Body;
		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return -1;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return AddToDynamic(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld);
	}

	void GeometryQuery::RemoveFromDynamic(Geometry* Object)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr)
		{
			return;
		}

		GeometryQueryProxy* proxy = FindProxy(Object);
		if (proxy)
		{
			RemoveFromDynamic(proxy->Handle);
			return;
		}
	}

	void GeometryQuery::RemoveFromDynamic(GeometryHandle Handle)
	{
		if (m_dynamicGeometry == nullptr)
		{
			return;
		}

		GeometryQueryProxy* proxy = GetProxyByIndex(Handle.Index);
		if (proxy && proxy->Handle != Handle)
		{
			proxy = nullptr;
		}
		const int nodeId = proxy ? proxy->PrunerHandle : -1;
		if (nodeId < 0)
		{
			RemoveProxy(Handle);
			return;
		}

		m_dynamicGeometry->Remove(nodeId);
		m_Objects.erase(std::remove(m_Objects.begin(), m_Objects.end(), proxy->Geom), m_Objects.end());
		RemoveProxy(Handle);
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Vector3& displacement)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return UpdateDynamicObject(handle, Object, Object ? Object->GetBounds() : Box3::Empty(), Object ? *Object->GetTransform() : Transform::Identity(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateDynamicObject(Geometry* Object, const Box3& Bounds, const Vector3& displacement)
	{
		GeometryQueryProxy* existing = FindProxy(Object);
		const GeometryHandle handle = existing ? existing->Handle : AllocateLocalHandle();
		return UpdateDynamicObject(handle, Object, Bounds, Object ? *Object->GetTransform() : Transform::Identity(), nullptr, displacement);
	}

	bool GeometryQuery::UpdateDynamicObject(GeometryHandle Handle, Geometry* Object, const Box3* Bounds, const Transform* ShapeToWorld, const Vector3& displacement)
	{
		if (m_dynamicGeometry == nullptr || Object == nullptr || Bounds == nullptr || ShapeToWorld == nullptr || !Object->IsQueryEnabled())
		{
			if (Object && !Object->IsQueryEnabled())
			{
				RemoveFromDynamic(Handle);
			}
			return false;
		}

		GeometryQueryProxy* proxy = GetProxyByIndex(Handle.Index);
		if (proxy && proxy->Handle != Handle)
		{
			proxy = nullptr;
		}
		if (proxy == nullptr || !proxy->IsDynamic() || proxy->PrunerHandle < 0)
		{
			return AddToDynamic(Handle, Object, Bounds, ShapeToWorld) >= 0;
		}

		const int nodeId = proxy->PrunerHandle;
		if (nodeId < 0)
		{
			return false;
		}

		GeometryQueryProxy& updatedProxy = UpdateProxy(Handle, Object, Bounds, ShapeToWorld, true);
		updatedProxy.PrunerHandle = nodeId;
		if (!updatedProxy.IsDirty())
		{
			return true;
		}

		m_dynamicGeometry->Update(nodeId, *Bounds, displacement);
		updatedProxy.SetDirty(false);
		return true;
	}

	bool GeometryQuery::UpdateDynamicObject(GeometryHandle Handle, Geometry* Object, const Box3& Bounds, const Transform& ShapeToWorld, RigidBody* Body, const Vector3& displacement)
	{
		(void)Body;
		GeometryQueryLocalState* localState = EnsureLocalState(Handle);
		if (localState == nullptr)
		{
			return false;
		}
		localState->WorldBounds = Bounds;
		localState->ShapeToWorld = ShapeToWorld;
		return UpdateDynamicObject(Handle, Object, &localState->WorldBounds, &localState->ShapeToWorld, displacement);
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
		boxes.resize(m_staticProxyIds.size());
		for (size_t i = 0; i < m_staticProxyIds.size(); ++i)
		{
			GeometryQueryProxy* proxy = GetProxyByIndex(m_staticProxyIds[i]);
			boxes[i] = proxy && proxy->WorldBounds ? *proxy->WorldBounds : Box3::Empty();
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
			if (!m_staticProxyIds.empty())
			{
				RayCastResult staticResult;
				const bool hit_static = m_staticGeometry->RayCast(ray, m_staticProxyIds.data(), &Option, &staticResult, RayCastProxyGeometry, this);
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
			bool hit_dynamic = m_dynamicGeometry->RayCast(ray, &Option, &Result2, RayCastProxyGeometry, this);
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
			if (!m_staticProxyIds.empty())
			{
				IntersectResult staticResult;
				const bool hit_static = m_staticGeometry->Intersect(geom, m_staticProxyIds.data(), &Option, &staticResult, IntersectProxyGeometry, this);
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
			bool hit_dynamic = m_dynamicGeometry->Intersect(geom, &Option, &Result2, IntersectProxyGeometry, this);
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
			if (!m_staticProxyIds.empty())
			{
				SweepResult staticResult;
				const bool hit_static = m_staticGeometry->Sweep(geom, m_staticProxyIds.data(), Direction, &Option, &staticResult, SweepProxyGeometry, this);
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
			bool hit_dynamic = m_dynamicGeometry->Sweep(geom, Direction, &Option, &Result2, SweepProxyGeometry, this);
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

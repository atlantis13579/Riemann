#include <assert.h>
#include <algorithm>

#include "PhysicsWorld.h"

#include "../Core/Base.h"
#include "../Core/File.h"
#include "../Core/LogSystem.h"
#include "../Core/JobSystem.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Destruction/DestructionSet.h"
#include "../Modules/Tools/PhysxBinaryParser.h"
#include "BroadPhase.h"
#include "CollidingContact.h"
#include "NarrowPhase.h"
#include "MotionIntegration.h"
#include "SequentialImpulseSolver.h"
#include "ForceField.h"
#include "KinematicsTree.h"

namespace Riemann
{
	void DefaultOnContact(const PhysicsContactInfo& ContactInfo)
	{
		GeometryAggregate* ParentA = ContactInfo.BodyA ? ContactInfo.BodyA->Parent : nullptr;
		GeometryAggregate* ParentB = ContactInfo.BodyB ? ContactInfo.BodyB->Parent : nullptr;
		if (ParentA)
		{
			ParentA->OnContact(ContactInfo);
		}
		if (ParentB && ParentB != ParentA)
		{
			ParentB->OnContact(ContactInfo);
		}
	}

	PhysicsWorld::PhysicsWorld(const PhysicsWorldParam& param)
	{
		m_GeometryQuery = new GeometryQuery;
		m_BPhase = nullptr;
		m_SceneQuery = param.sceneQueryType;
		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			m_GeometryQuery->CreateDynamicGeometry();
		}

		if (param.broadphase == BroadPhaseSolver::SAP)
		{
			m_BPhase = BroadPhase::Create_SAP();
		}
		else if (param.broadphase == BroadPhaseSolver::ABP)
		{
			m_BPhase = BroadPhase::Create_ABP();
		}
		else if (param.broadphase == BroadPhaseSolver::MBP)
		{
			m_BPhase = BroadPhase::Create_MBP();
		}
		else if (param.broadphase == BroadPhaseSolver::Bruteforce)
		{
			m_BPhase = BroadPhase::Create_Bruteforce();
		}
		else if (param.broadphase == BroadPhaseSolver::AllPairs)
		{
			m_BPhase = BroadPhase::Create_AllPairs();
		}
		else if (param.broadphase == BroadPhaseSolver::DynamicAABB)
		{
			m_BPhase = BroadPhase::Create_DynamicAABB();
		}
		if (m_BPhase == nullptr)
		{
			m_BPhase = BroadPhase::Create_SAP();
		}
		m_NPhase = NarrowPhase::Create_GJKEPA();
		m_Solver = ConstraintSolver::CreateSequentialImpulseSolver();
		m_IntegrateMethod = param.integrateMethod;
		m_OnContact = param.onContact ? param.onContact : OnContactCallback(DefaultOnContact);
		m_Fields.push_back(ForceField::CreateGrivityField(param.gravityAcc));
		m_Jobsystem = new JobSystem;
		if (param.workerThreads != 0)
		{
			m_Jobsystem->CreateWorkers(param.workerThreads);
		}
		m_SceneResource = nullptr;
	}

	PhysicsWorld::~PhysicsWorld()
	{
		for (DestructionSet* DestructSet : m_DestructionSets)
		{
			if (DestructSet)
			{
				DestructSet->ClearClusters();
				DestructSet->SetPhysicsWorld(nullptr);
			}
		}
		m_DestructionSets.clear();

		SAFE_DELETE(m_BPhase);
		SAFE_DELETE(m_GeometryQuery);
		SAFE_DELETE(m_NPhase);
		SAFE_DELETE(m_Solver);

		for (size_t i = 0; i < m_Fields.size(); ++i)
		{
			delete m_Fields[i];
		}
		m_Fields.clear();

		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			delete m_StaticBodies[i];
		}
		m_StaticBodies.clear();

		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			delete m_DynamicBodies[i];
		}
		m_DynamicBodies.clear();

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			delete m_Kinematics[i];
		}
		m_Kinematics.clear();

		if (m_Jobsystem)
		{
			m_Jobsystem->Terminate();
			delete m_Jobsystem;
		}

		if (m_SceneResource)
		{
			delete m_SceneResource;
			m_SceneResource = nullptr;
		}
	}

	void		PhysicsWorld::Simulate()
	{
		m_Clock.tick++;

		SimulateST(m_Clock.deltatime);
	}

	void PhysicsWorld::DispatchContactCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds)
	{
		if (!m_OnContact)
		{
			return;
		}

		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			ContactManifold* manifold = manifolds[i];
			if (manifold == nullptr || manifold->NumContactPointCount <= 0)
			{
				continue;
			}

			Geometry* geomA = geoms[manifold->indexA];
			Geometry* geomB = geoms[manifold->indexB];
			RigidBody* bodyA = geomA->GetParent<RigidBody>();
			RigidBody* bodyB = geomB->GetParent<RigidBody>();

			PhysicsContactPoint contactPoints[MAX_CONTACT_POINTS];
			const int contactPointCount = std::min(manifold->NumContactPointCount, MAX_CONTACT_POINTS);
			for (int j = 0; j < contactPointCount; ++j)
			{
				const Contact& contact = manifold->ContactPoints[j];
				PhysicsContactPoint& point = contactPoints[j];
				point.Position = (contact.PositionWorldA + contact.PositionWorldB) * 0.5f;
				point.PositionWorldA = contact.PositionWorldA;
				point.PositionWorldB = contact.PositionWorldB;
				point.PositionLocalA = contact.PositionLocalA;
				point.PositionLocalB = contact.PositionLocalB;
				point.Normal = contact.Normal;
				point.Tangent = contact.Tangent;
				point.Binormal = contact.Binormal;
				point.PenetrationDepth = contact.PenetrationDepth;
				point.NormalImpulse = contact.totalImpulseNormal;
				point.TangentImpulse = contact.totalImpulseTangent;
				point.BinormalImpulse = contact.totalImpulseBinormal;
				point.Impulse = contact.Normal * point.NormalImpulse
					+ contact.Tangent * point.TangentImpulse
					+ contact.Binormal * point.BinormalImpulse;
			}

			PhysicsContactInfo contactInfo;
			contactInfo.BodyA = bodyA;
			contactInfo.BodyB = bodyB;
			contactInfo.GeomA = geomA;
			contactInfo.GeomB = geomB;
			contactInfo.ContactPoints = contactPoints;
			contactInfo.NumContactPoints = contactPointCount;
			m_OnContact(contactInfo);
		}
	}

	void		PhysicsWorld::Reset()
	{
		for (DestructionSet* DestructSet : m_DestructionSets)
		{
			if (DestructSet)
			{
				DestructSet->ClearClusters();
				DestructSet->SetPhysicsWorld(nullptr);
			}
		}
		m_DestructionSets.clear();

		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			m_StaticBodies[i]->ReleaseGeometries();
			delete m_StaticBodies[i];
		}
		m_StaticBodies.clear();

		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			m_DynamicBodies[i]->ReleaseGeometries();
			delete m_DynamicBodies[i];
		}
		m_DynamicBodies.clear();

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			delete m_Kinematics[i];
		}
		m_Kinematics.clear();

		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			m_GeometryQuery->ClearDynamicGeometry();
		}
		else
		{
			m_GeometryQuery->ClearStaticGeometry();
		}

		m_GeometryWorldStates.clear();
		m_GeometryWorldRecords.clear();
		m_FreeGeometryWorldRecords.clear();
	}

	GeometryHandle PhysicsWorld::EnsureGeometryHandle(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return GeometryHandle();
		}

		RigidBody* body = Object->GetParent<RigidBody>();
		if (body)
		{
			const GeometryHandle existing = body->FindGeometryHandle(Object);
			if (IsValidGeometryHandle(existing))
			{
				return existing;
			}
		}
		else
		{
			for (size_t i = 0; i < m_GeometryWorldRecords.size(); ++i)
			{
				const GeometryWorldRecord& record = m_GeometryWorldRecords[i];
				if (record.Active && record.State.Geom == Object)
				{
					return GeometryHandle((uint32_t)i, record.Generation);
				}
			}
		}

		uint32_t index = 0;
		if (!m_FreeGeometryWorldRecords.empty())
		{
			index = m_FreeGeometryWorldRecords.back();
			m_FreeGeometryWorldRecords.pop_back();
		}
		else
		{
			index = (uint32_t)m_GeometryWorldRecords.size();
			m_GeometryWorldRecords.push_back(GeometryWorldRecord());
		}

		GeometryWorldRecord& record = m_GeometryWorldRecords[index];
		record.Active = true;
		record.HasState = false;
		record.State = GeometryWorldState();
		record.State.Geom = Object;
		record.State.Body = body;
		record.State.Handle = GeometryHandle(index, record.Generation);

		if (body)
		{
			body->SetGeometryHandle(body->FindGeometryIndex(Object), record.State.Handle);
		}

		return record.State.Handle;
	}

	bool PhysicsWorld::IsValidGeometryHandle(GeometryHandle Handle) const
	{
		return GetGeometryWorldRecord(Handle) != nullptr;
	}

	GeometryWorldRecord* PhysicsWorld::GetGeometryWorldRecord(GeometryHandle Handle)
	{
		if (!Handle.IsValid() || Handle.Index >= m_GeometryWorldRecords.size())
		{
			return nullptr;
		}

		GeometryWorldRecord& record = m_GeometryWorldRecords[Handle.Index];
		if (!record.Active || record.Generation != Handle.Generation)
		{
			return nullptr;
		}
		return &record;
	}

	const GeometryWorldRecord* PhysicsWorld::GetGeometryWorldRecord(GeometryHandle Handle) const
	{
		if (!Handle.IsValid() || Handle.Index >= m_GeometryWorldRecords.size())
		{
			return nullptr;
		}

		const GeometryWorldRecord& record = m_GeometryWorldRecords[Handle.Index];
		if (!record.Active || record.Generation != Handle.Generation)
		{
			return nullptr;
		}
		return &record;
	}

	GeometryWorldState PhysicsWorld::UpdateGeometryWorldState(GeometryHandle Handle, uint64_t FrameId)
	{
		GeometryWorldRecord* record = GetGeometryWorldRecord(Handle);
		if (record == nullptr)
		{
			GeometryWorldState State;
			State.Handle = Handle;
			State.FrameId = FrameId;
			return State;
		}

		Geometry* Object = record->State.Geom;
		GeometryWorldState State;
		State.Handle = Handle;
		State.Geom = Object;
		State.FrameId = FrameId;
		State.WorldBounds = Box3::Empty();

		if (Object == nullptr)
		{
			return State;
		}

		State.Body = Object->GetParent<RigidBody>();
		if (State.Body)
		{
			State.BodyToWorld = Transform(State.Body->BodyLocalToWorld(Vector3::Zero()), State.Body->Q);
			State.ShapeToWorld = State.Body->GetGeometryTransform(Object);
			State.WorldBounds = Box3::Transform(Object->GetShapeBounds(), State.ShapeToWorld.pos, State.ShapeToWorld.quat);
			State.MovingRigid = State.Body->mRigidType != RigidType::Static;
		}
		else
		{
			State.BodyToWorld = Transform::Identity();
			State.ShapeToWorld = *Object->GetTransform();
			State.WorldBounds = Object->GetBounds();
			State.MovingRigid = false;
		}

		if (!record->HasState)
		{
			State.Version = 1;
			State.Moved = true;
			State.Displacement = Vector3::Zero();
		}
		else
		{
			const GeometryWorldState& Prev = record->State;
			State.Displacement = State.ShapeToWorld.pos - Prev.ShapeToWorld.pos;
			State.Moved = State.Body != Prev.Body
				|| State.BodyToWorld != Prev.BodyToWorld
				|| State.ShapeToWorld != Prev.ShapeToWorld
				|| !(State.WorldBounds == Prev.WorldBounds)
				|| State.MovingRigid != Prev.MovingRigid;
			State.Version = State.Moved ? Prev.Version + 1 : Prev.Version;
		}

		record->State = State;
		record->HasState = true;
		return State;
	}

	GeometryWorldState PhysicsWorld::UpdateGeometryWorldState(Geometry* Object, uint64_t FrameId)
	{
		return UpdateGeometryWorldState(EnsureGeometryHandle(Object), FrameId);
	}

	void PhysicsWorld::UpdateGeometryWorldStates(const std::vector<Geometry*>& Objects)
	{
		m_GeometryWorldStates.clear();
		m_GeometryWorldStates.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			const GeometryWorldState State = UpdateGeometryWorldState(Object, m_Clock.tick);
			const GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
			if (record)
			{
				m_GeometryWorldStates.push_back(&record->State);
			}
		}
	}

	void PhysicsWorld::RemoveGeometryWorldState(GeometryHandle Handle)
	{
		if (!Handle.IsValid() || Handle.Index >= m_GeometryWorldRecords.size())
		{
			return;
		}

		GeometryWorldRecord& record = m_GeometryWorldRecords[Handle.Index];
		if (!record.Active || record.Generation != Handle.Generation)
		{
			return;
		}

		if (record.State.Body)
		{
			record.State.Body->SetGeometryHandle(record.State.Body->FindGeometryIndex(record.State.Geom), GeometryHandle());
		}

		record.State = GeometryWorldState();
		record.Active = false;
		record.HasState = false;
		++record.Generation;
		if (record.Generation == 0)
		{
			record.Generation = 1;
		}
		m_FreeGeometryWorldRecords.push_back(Handle.Index);
	}

	void PhysicsWorld::RemoveGeometryWorldState(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return;
		}

		RigidBody* body = Object->GetParent<RigidBody>();
		if (body)
		{
			RemoveGeometryWorldState(body->FindGeometryHandle(Object));
			return;
		}

		for (size_t i = 0; i < m_GeometryWorldRecords.size(); ++i)
		{
			const GeometryWorldRecord& record = m_GeometryWorldRecords[i];
			if (record.Active && record.State.Geom == Object)
			{
				RemoveGeometryWorldState(GeometryHandle((uint32_t)i, record.Generation));
				return;
			}
		}
	}

	void		PhysicsWorld::SimulateST(float dt)
	{
		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			m_Kinematics[i]->Simulate(dt);
		}

		std::vector<Geometry*> geoms;
		geoms.reserve(m_StaticBodies.size() + m_DynamicBodies.size());
		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			for (Geometry* geom : m_StaticBodies[i]->Geometries())
			{
				if (geom && geom->IsSimulationEnabled())
				{
					geoms.push_back(geom);
				}
			}
		}
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			for (Geometry* geom : m_DynamicBodies[i]->Geometries())
			{
				if (geom && geom->IsSimulationEnabled())
				{
					geoms.push_back(geom);
				}
			}
		}

		UpdateGeometryWorldStates(geoms);

		GeometryWorldStateSpan stateSpan(m_GeometryWorldStates.data(), m_GeometryWorldStates.size());
		std::vector<OverlapPair> overlaps;
		if (m_BPhase)
		{
			m_BPhase->ProduceOverlaps(stateSpan, &overlaps);
		}

		std::vector<ContactManifold*> manifolds;
		if (m_NPhase && !overlaps.empty())
		{
			m_NPhase->CollisionDetection(stateSpan, overlaps, &manifolds);
		}

		if (!manifolds.empty())
		{
			if (m_Solver)
			{
				m_Solver->PreResolve(geoms);

				const bool BuildIslands = true;
				if (BuildIslands)
				{
					ContactManifoldIslands manifold_islands;
					manifold_islands.BuildIslands(geoms, manifolds);
					manifolds.clear();

					for (size_t i = 0; i < manifold_islands.islands.size(); ++i)
					{
						m_Solver->ResolveContact(geoms, manifold_islands.islands[i], dt);
						DispatchContactCallbacks(geoms, manifold_islands.islands[i]);
					}
				}
				else
				{
					m_Solver->ResolveContact(geoms, manifolds, dt);
					DispatchContactCallbacks(geoms, manifolds);
				}

				m_Solver->PostResolve(geoms);
			}
			else
			{
				DispatchContactCallbacks(geoms, manifolds);
			}
		}

		PreIntegrate(dt);

		MotionIntegration::Integrate(m_DynamicBodies, dt, (uint8_t)m_IntegrateMethod);

		PostIntegrate(dt);
		UpdateDestructionSets(dt);

		return;
	}

	void PhysicsWorld::UpdateDestructionSets(float dt)
	{
		for (DestructionSet* DestructSet : m_DestructionSets)
		{
			DestructSet->Update(dt);
		}
	}

	void		PhysicsWorld::PreIntegrate(float dt)
	{
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			RigidBodyDynamic* Body = m_DynamicBodies[i];
			Body->ExtForce /= (dt * 2.0f);	 // Force Unit  1 Newton =  1 kg m / s^2
			Body->ExtTorque /= (dt * 2.0f);
			if (!Body->Sleeping)
				continue;
			if (Body->GetKinematicsEnergy() > Body->SleepThreshold)
			{
				Body->Wakeup();
			}
		}

		for (size_t j = 0; j < m_Fields.size(); ++j)
		{
			m_Fields[j]->Update(dt);

			for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
			{
				m_Fields[j]->ApplyForce(m_DynamicBodies[i]);
			}
		}
	}

	void		PhysicsWorld::PostIntegrate(float dt)
	{
		(void)dt;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			RigidBodyDynamic* Body = m_DynamicBodies[i];
			Body->ExtForce.SetZero();
			Body->ExtTorque.SetZero();
			if (Body->Sleeping)
				continue;
			Body->AutoSleep();
			UpdateSceneQuery(Body);
		}
	}

	void PhysicsWorld::BuildSceneQuery(const std::vector<Geometry*>& Objects)
	{
		UpdateGeometryWorldStates(Objects);
		BuildSceneQuery(GeometryWorldStateSpan(m_GeometryWorldStates.data(), m_GeometryWorldStates.size()));
	}

	void PhysicsWorld::BuildSceneQuery(GeometryWorldStateSpan States)
	{
		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			if (m_GeometryQuery->GetDynamicTree() == nullptr)
			{
				m_GeometryQuery->CreateDynamicGeometry();
			}
			else
			{
				m_GeometryQuery->ClearDynamicGeometry();
			}

			for (size_t i = 0; i < States.size(); ++i)
			{
				const GeometryWorldState& State = States[i];
				if (State.Geom == nullptr)
				{
					continue;
				}
				const GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
				if (record)
				{
					const GeometryWorldState& queryState = record->State;
					m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld);
				}
				else
				{
					m_GeometryQuery->AddGeometry(State.Handle, State.Geom, State.WorldBounds, State.ShapeToWorld, State.Body);
				}
			}
			return;
		}

		m_GeometryQuery->ClearStaticGeometry();
		for (size_t i = 0; i < States.size(); ++i)
		{
			const GeometryWorldState& State = States[i];
			if (State.Geom == nullptr)
			{
				continue;
			}
			const GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
			if (record)
			{
				const GeometryWorldState& queryState = record->State;
				m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld);
			}
			else
			{
				m_GeometryQuery->AddGeometry(State.Handle, State.Geom, State.WorldBounds, State.ShapeToWorld, State.Body);
			}
		}
		m_GeometryQuery->GetStaticTree();
	}

	void PhysicsWorld::AddGeometryToSceneQuery(Geometry* Object)
	{
		const GeometryWorldState State = UpdateGeometryWorldState(Object, m_Clock.tick);
		const GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
		if (record)
		{
			const GeometryWorldState& queryState = record->State;
			m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld);
		}
		else
		{
			m_GeometryQuery->AddGeometry(State.Handle, State.Geom, State.WorldBounds, State.ShapeToWorld, State.Body);
		}
	}

	void PhysicsWorld::RemoveGeometryFromSceneQuery(Geometry* Object)
	{
		const GeometryHandle handle = EnsureGeometryHandle(Object);
		m_GeometryQuery->RemoveGeometry(handle);
		RemoveGeometryWorldState(handle);
	}

	void PhysicsWorld::UpdateSceneQuery(const GeometryWorldState& State)
	{
		if (State.Geom == nullptr || !State.Moved)
		{
			return;
		}

		const GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
		if (record)
		{
			const GeometryWorldState& queryState = record->State;
			m_GeometryQuery->UpdateGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld, queryState.Displacement);
		}
		else
		{
			m_GeometryQuery->UpdateGeometry(State.Handle, State.Geom, State.WorldBounds, State.ShapeToWorld, State.Body, State.Displacement);
		}
	}

	void PhysicsWorld::UpdateSceneQuery(RigidBodyDynamic* Body)
	{
		if (Body == nullptr)
		{
			return;
		}

		for (Geometry* g : Body->Geometries())
		{
			const GeometryWorldState State = UpdateGeometryWorldState(g, m_Clock.tick);
			UpdateSceneQuery(State);
		}
	}

	bool         PhysicsWorld::LoadScene(const char* name, bool shared_mem)
	{
		std::vector<RigidBody*> collection;
		std::vector<Geometry*> geoms;
		if (shared_mem)
		{
			m_SceneResource = Riemann::LoadPhysxBinaryMmap(name, nullptr, &geoms);
			if (!m_SceneResource)
			{
				return false;
			}
		}
		else
		{
			bool load_succ = Riemann::LoadPhysxBinary(name, nullptr, &geoms);
			if (!load_succ)
			{
				return false;
			}
		}

		for (size_t i = 0; i < collection.size(); ++i)
		{
			RigidBody* body = collection[i];
			if (body->mRigidType == RigidType::Static)
			{
				m_StaticBodies.push_back(body->CastStatic());
			}
			else if (body->mRigidType == RigidType::Dynamic)
			{
				m_DynamicBodies.push_back(body->CastDynamic());
			}
		}

		assert(m_GeometryQuery);
		BuildSceneQuery(geoms);
		return true;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose)
	{
		RigidBody* body = RigidBody::CreateRigidBody(param, init_pose);
		if (body->mRigidType == RigidType::Static)
		{
			m_StaticBodies.push_back(body->CastStatic());
		}
		else if (body->mRigidType == RigidType::Dynamic)
		{
			m_DynamicBodies.push_back(body->CastDynamic());
		}

		return body;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
	{
		Transform init_pose(*Geom->GetTransform());
		Geom->SetTransform(Vector3::Zero(), Quaternion::One());
		RigidBody* body = CreateRigidBody(param, init_pose);
		body->AddGeometry(Geom);

		AddGeometryToSceneQuery(Geom);

		return body;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform)
	{
		if (Geoms.empty())
		{
			return nullptr;
		}

		RigidBody* body = CreateRigidBody(param, init_transform);
		if (!body)
		{
			return nullptr;
		}

		for (Geometry* Geom : Geoms)
		{
			if (!Geom)
			{
				continue;
			}

			const Transform current_transform = *Geom->GetTransform();
			const Transform local_transform = init_transform.TransformInv(current_transform);
			Geom->SetTransform(local_transform.pos, local_transform.quat);
			body->AddGeometry(Geom);

			AddGeometryToSceneQuery(Geom);
		}

		return body;
	}

	bool PhysicsWorld::RemoveRigidBody(RigidBody* Body)
	{
		if (Body->mRigidType == RigidType::Static)
		{
			for (size_t i = 0; i < m_StaticBodies.size(); ++i)
			{
				if (m_StaticBodies[i] == Body)
				{
					m_StaticBodies.erase(m_StaticBodies.begin() + i);
					for (Geometry* g : Body->Geometries())
					{
						RemoveGeometryFromSceneQuery(g);
					}
					return true;
				}
			}
		}
		else if (Body->mRigidType == RigidType::Dynamic)
		{
			for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
			{
				if (m_DynamicBodies[i] == Body)
				{
					m_DynamicBodies.erase(m_DynamicBodies.begin() + i);
					for (Geometry* g : Body->Geometries())
					{
						RemoveGeometryFromSceneQuery(g);
					}
					return true;
				}
			}
		}
		return false;
	}

	void PhysicsWorld::AddDestructionSet(DestructionSet* DestructSet)
	{
		if (DestructSet == nullptr)
		{
			return;
		}
		if (std::find(m_DestructionSets.begin(), m_DestructionSets.end(), DestructSet) == m_DestructionSets.end())
		{
			m_DestructionSets.push_back(DestructSet);
		}
		DestructSet->SetPhysicsWorld(this);
	}

	void PhysicsWorld::RemoveDestructionSet(DestructionSet* DestructSet)
	{
		m_DestructionSets.erase(std::remove(m_DestructionSets.begin(), m_DestructionSets.end(), DestructSet), m_DestructionSets.end());
	}

	bool PhysicsWorld::LoadAnimation(const std::string& resname, const std::string& filepath, float play_rate, bool begin_play)
	{
		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			if (m_Kinematics[i]->GetName() == filepath)
			{
				delete m_Kinematics[i];
				m_Kinematics.erase(m_Kinematics.begin() + i);
				break;
			}
		}

		KinematicsTree* tree = new KinematicsTree;
		if (!tree->Deserialize(filepath))
		{
			delete tree;
			return false;
		}
		tree->SetName(resname);
		tree->SetAnimationPlayRate(play_rate);
		tree->Pause(!begin_play);
		m_Kinematics.push_back(tree);
		return true;
	}

	KinematicsDriver* PhysicsWorld::FindKinematics(const std::string& resname)
	{
		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			if (m_Kinematics[i]->GetName() == resname)
			{
				return m_Kinematics[i];
			}
		}
		return nullptr;
	}

	void PhysicsWorld::GetAllGeometries(std::vector<Geometry*>* geoms)
	{
		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			m_StaticBodies[i]->GetGeometries(geoms);
		}
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			m_DynamicBodies[i]->GetGeometries(geoms);
		}
	}

	float PhysicsWorld::GetSystemTotalEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetKinematicsEnergy();
		}
		return Energy;
	}

	float PhysicsWorld::GetSystemTotalLinearKinematicsEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetLinearKinematicsEnergy();
		}
		return Energy;
	}

	float PhysicsWorld::GetSystemTotalAngularKinematicsEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetAngularKinematicsEnergy();
		}
		return Energy;
	}

	Vector3 PhysicsWorld::GetSystemTotalLinearMomentum() const
	{
		Vector3 Momentum(0.0f);
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Momentum += m_DynamicBodies[i]->GetLinearMomentum();
		}
		return Momentum;
	}

	Vector3 PhysicsWorld::GetSystemTotalAngularMomentum() const
	{
		Vector3 Momentum(0.0f);
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Momentum += m_DynamicBodies[i]->GetAngularMomentum();
		}
		return Momentum;
	}
}

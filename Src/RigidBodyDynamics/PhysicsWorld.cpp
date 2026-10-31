#include <assert.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "PhysicsWorld.h"

#include "../Core/Base.h"
#include "../Core/File.h"
#include "../Core/LogSystem.h"
#include "../Core/JobSystem.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../Destruction/DestructionSet.h"
#include "../Modules/Tools/PhysxBinaryParser.h"
#include "BroadPhase.h"
#include "CollidingContact.h"
#include "NarrowPhase.h"
#include "MotionIntegration.h"
#include "SequentialImpulseSolver.h"
#include "LCPSolver.h"
#include "ForceField.h"
#include "KinematicsTree.h"
#include "ContinuousCollisionDetection.h"
#include "../Deformation/Rope.h"
#include "../Deformation/DeformableBody.h"
#include "../Deformation/Cloth.h"
#include "Ragdoll.h"

namespace Riemann
{
	namespace
	{
		void CreateResolutionSolvers(ResolutionPhaseSolver Solver, SolverBackend Backend, bool AllowFallback,
			const PBDSolverSettings& PBDSettings, const XPBDSolverSettings& XPBDSettings,
			const LCPSolverSettings& LCPSettings, std::unique_ptr<ConstraintSolver>& ContactSolver,
			std::unique_ptr<PBDSolver>& PositionSolver)
		{
			if (static_cast<uint8_t>(Backend) > static_cast<uint8_t>(SolverBackend::CUDA))
				throw std::runtime_error("Invalid solver backend");
			switch (Solver)
			{
			case ResolutionPhaseSolver::SequentialImpulse:
				if (Backend != SolverBackend::CPU)
					throw std::runtime_error("Sequential impulse supports the CPU backend only");
				ContactSolver.reset(ConstraintSolver::CreateSequentialImpulseSolver());
				break;
			case ResolutionPhaseSolver::LCPGlobal:
			{
				LCPSolverSettings Settings = LCPSettings;
				Settings.Backend = Backend;
				Settings.AllowBackendFallback = AllowFallback;
				ContactSolver.reset(CreateGlobalLCPSolver(Settings));
				break;
			}
			case ResolutionPhaseSolver::PBD:
			{
				PBDSolverSettings Settings = PBDSettings;
				Settings.Backend = Backend;
				Settings.AllowBackendFallback = AllowFallback;
				PositionSolver.reset(new PBDSolver(Settings));
				break;
			}
			case ResolutionPhaseSolver::XPBD:
			{
				XPBDSolverSettings Settings = XPBDSettings;
				Settings.Backend = Backend;
				Settings.AllowBackendFallback = AllowFallback;
				PositionSolver.reset(new XPBDSolver(Settings));
				break;
			}
			default:
				throw std::runtime_error("Invalid resolution solver");
			}
		}
	}

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

	void DefaultOnTrigger(const PhysicsTriggerInfo& TriggerInfo)
	{
		GeometryAggregate* ParentA = TriggerInfo.BodyA ? TriggerInfo.BodyA->Parent : nullptr;
		GeometryAggregate* ParentB = TriggerInfo.BodyB ? TriggerInfo.BodyB->Parent : nullptr;
		if (ParentA)
		{
			ParentA->OnTrigger(TriggerInfo);
		}
		if (ParentB && ParentB != ParentA)
		{
			ParentB->OnTrigger(TriggerInfo);
		}
	}

	PhysicsWorld::PhysicsWorld(const PhysicsWorldParam& param)
	{
		m_PBDSettings = param.pbdSettings;
		m_XPBDSettings = param.xpbdSettings;
		m_LCPSettings.MaxIterations = std::max(1, param.contactSolverIterations);
		m_LCPSettings.Tolerance = std::max(0.0f, param.contactSolverTolerance);
		m_LCPSettings.Relaxation = std::max(0.01f, std::min(1.99f, param.contactSolverRelaxation));
		// Resolve strict backend requests before allocating world-owned raw resources.
		std::unique_ptr<ConstraintSolver> ContactSolver;
		std::unique_ptr<PBDSolver> PositionSolver;
		CreateResolutionSolvers(param.resolutionSolver, param.solverBackend, param.allowBackendFallback,
			m_PBDSettings, m_XPBDSettings, m_LCPSettings, ContactSolver, PositionSolver);
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
		m_Solver = ContactSolver.get();
		m_PositionSolver = PositionSolver.get();
		m_ResolutionSolver = param.resolutionSolver;
		m_RequestedSolverBackend = param.solverBackend;
		m_AllowBackendFallback = param.allowBackendFallback;
		m_IntegrateMethod = param.integrateMethod;
		m_OnContact = param.onContact ? param.onContact : OnContactCallback(DefaultOnContact);
		m_OnTrigger = param.onTrigger ? param.onTrigger : OnTriggerCallback(DefaultOnTrigger);
		m_Fields.push_back(ForceField::CreateGrivityField(param.gravityAcc));
		m_Jobsystem = new JobSystem;
		if (param.workerThreads != 0)
		{
			m_Jobsystem->CreateWorkers(param.workerThreads);
		}
		m_SceneResource = nullptr;
		m_LastCCDHitCount = 0;
		m_ConstraintIterations = std::max(1, param.constraintIterations);
		ContactSolver.release();
		PositionSolver.release();
	}

	PhysicsWorld::~PhysicsWorld()
	{
		for (Ragdoll* RagdollInstance : m_Ragdolls)
		{
			delete RagdollInstance;
		}
		m_Ragdolls.clear();

		for (ClothSystem* Cloth : m_Cloths)
		{
			delete Cloth;
		}
		m_Cloths.clear();

		for (DeformableBody* body : m_DeformableBodies) delete body;
		m_DeformableBodies.clear();

		for (RopeSystem* Rope : m_Ropes)
		{
			delete Rope;
		}
		m_Ropes.clear();

		for (PositionConstraint* Constraint : m_Constraints)
		{
			delete Constraint;
		}
		m_Constraints.clear();

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
		SAFE_DELETE(m_PositionSolver);

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
		Simulate(m_Clock.deltatime);
	}

	void PhysicsWorld::Simulate(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		++m_Clock.tick;
		SimulateST(Dt);
	}

	SolverBackend PhysicsWorld::GetSolverBackend() const
	{
		return m_PositionSolver ? m_PositionSolver->GetBackend()
			: (m_Solver ? m_Solver->GetBackend() : SolverBackend::CPU);
	}

	bool PhysicsWorld::IsSolverBackendAvailable(ResolutionPhaseSolver Solver, SolverBackend Backend)
	{
		switch (Solver)
		{
		case ResolutionPhaseSolver::SequentialImpulse: return Backend == SolverBackend::CPU;
		case ResolutionPhaseSolver::LCPGlobal: return LCPSolver::IsBackendAvailable(Backend);
		case ResolutionPhaseSolver::PBD:
		case ResolutionPhaseSolver::XPBD: return IsPBDBackendAvailable(Backend);
		default: return false;
		}
	}

	bool PhysicsWorld::SetSolverConfiguration(ResolutionPhaseSolver Solver, SolverBackend Backend,
		bool AllowFallback, std::string* ErrorMessage)
	{
		if (ErrorMessage) ErrorMessage->clear();
		if (Solver == m_ResolutionSolver && Backend == m_RequestedSolverBackend
			&& AllowFallback == m_AllowBackendFallback && Backend == GetSolverBackend()
			&& (!m_PositionSolver || m_PositionSolver->GetBackendStatistics().Message.empty())) return true;

		std::unique_ptr<ConstraintSolver> ContactSolver;
		std::unique_ptr<PBDSolver> PositionSolver;
		try
		{
			CreateResolutionSolvers(Solver, Backend, AllowFallback, m_PBDSettings, m_XPBDSettings,
				m_LCPSettings, ContactSolver, PositionSolver);
		}
		catch (const std::exception& Error)
		{
			if (ErrorMessage) *ErrorMessage = Error.what();
			return false;
		}

		// New solvers own fresh contact caches. Commit only after all device setup succeeds.
		delete m_Solver;
		delete m_PositionSolver;
		m_Solver = ContactSolver.release();
		m_PositionSolver = PositionSolver.release();
		m_ResolutionSolver = Solver;
		m_RequestedSolverBackend = Backend;
		m_AllowBackendFallback = AllowFallback;
		for (PositionConstraint* Constraint : m_Constraints)
		{
			if (Constraint) Constraint->BeginStep(m_Clock.deltatime, Solver == ResolutionPhaseSolver::XPBD);
		}
		// A sleeping body must be allowed to settle under the newly selected formulation.
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
		return true;
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

	void PhysicsWorld::DispatchTriggerCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds)
	{
		if (!m_OnTrigger)
		{
			return;
		}

		for (ContactManifold* manifold : manifolds)
		{
			if (manifold == nullptr || manifold->NumContactPointCount <= 0)
			{
				continue;
			}
			Geometry* geomA = geoms[(size_t)manifold->indexA];
			Geometry* geomB = geoms[(size_t)manifold->indexB];
			const Contact& contact = manifold->ContactPoints[0];
			PhysicsTriggerInfo triggerInfo;
			triggerInfo.BodyA = geomA->GetParent<RigidBody>();
			triggerInfo.BodyB = geomB->GetParent<RigidBody>();
			triggerInfo.GeomA = geomA;
			triggerInfo.GeomB = geomB;
			triggerInfo.Position = (contact.PositionWorldA + contact.PositionWorldB) * 0.5f;
			triggerInfo.Normal = contact.Normal;
			triggerInfo.Fraction = 0.0f;
			triggerInfo.IsCCD = false;
			m_OnTrigger(triggerInfo);
		}
	}

	void PhysicsWorld::DispatchTriggerCCDCallbacks(const std::vector<ContinuousCollisionHit>& hits)
	{
		if (!m_OnTrigger)
		{
			return;
		}
		for (const ContinuousCollisionHit& hit : hits)
		{
			PhysicsTriggerInfo triggerInfo;
			triggerInfo.BodyA = hit.Body;
			triggerInfo.BodyB = hit.OtherBody;
			triggerInfo.GeomA = hit.Shape;
			triggerInfo.GeomB = hit.OtherShape;
			triggerInfo.Position = hit.Position;
			triggerInfo.Normal = hit.Normal;
			triggerInfo.Fraction = hit.Fraction;
			triggerInfo.IsCCD = true;
			m_OnTrigger(triggerInfo);
		}
	}

	void		PhysicsWorld::Reset()
	{
		if (m_Solver)
		{
			m_Solver->ResetState();
		}
		if (m_PositionSolver) m_PositionSolver->ResetState();

		// The gravity field is permanent; additional fields belong to the cleared scene.
		for (size_t FieldIndex = 1; FieldIndex < m_Fields.size(); ++FieldIndex)
		{
			delete m_Fields[FieldIndex];
		}
		m_Fields.resize(std::min<size_t>(m_Fields.size(), 1));

		for (Ragdoll* RagdollInstance : m_Ragdolls)
		{
			delete RagdollInstance;
		}
		m_Ragdolls.clear();

		for (ClothSystem* Cloth : m_Cloths)
		{
			delete Cloth;
		}
		m_Cloths.clear();

		for (DeformableBody* body : m_DeformableBodies) delete body;
		m_DeformableBodies.clear();

		for (RopeSystem* Rope : m_Ropes)
		{
			delete Rope;
		}
		m_Ropes.clear();

		for (PositionConstraint* Constraint : m_Constraints)
		{
			delete Constraint;
		}
		m_Constraints.clear();

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
		m_LastCCDHitCount = 0;
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
			RigidBodyDynamic* DynamicBody = State.Body->CastDynamic();
			State.MovingRigid = DynamicBody ? !DynamicBody->Sleeping : State.Body->mRigidType != RigidType::Static;
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

	void PhysicsWorld::AddGeometryWorldState(Geometry* Object)
	{
		const GeometryWorldState State = UpdateGeometryWorldState(Object, m_Clock.tick);
		GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
		if (record == nullptr || record->State.Geom == nullptr)
		{
			return;
		}

		const GeometryWorldState* state = &record->State;
		if (std::find(m_GeometryWorldStates.begin(), m_GeometryWorldStates.end(), state) == m_GeometryWorldStates.end())
		{
			m_GeometryWorldStates.push_back(state);
		}
	}

	void PhysicsWorld::UpdateGeometryWorldStates(const std::vector<Geometry*>& Objects)
	{
		for (Geometry* Object : Objects)
		{
			AddGeometryWorldState(Object);
		}
	}

	void PhysicsWorld::UpdateGeometryWorldStates()
	{
		for (const GeometryWorldState* state : m_GeometryWorldStates)
		{
			if (state != nullptr)
			{
				UpdateGeometryWorldState(state->Handle, m_Clock.tick);
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

		const GeometryWorldState* state = &record.State;
		m_GeometryWorldStates.erase(
			std::remove(m_GeometryWorldStates.begin(), m_GeometryWorldStates.end(), state),
			m_GeometryWorldStates.end());

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

	void PhysicsWorld::GetActiveGeometryWorldStates(std::vector<const GeometryWorldState*>* States, std::vector<Geometry*>* Objects) const
	{
		if (States == nullptr || Objects == nullptr)
		{
			return;
		}

		States->clear();
		Objects->clear();
		States->reserve(m_GeometryWorldStates.size());
		Objects->reserve(m_GeometryWorldStates.size());
		for (const GeometryWorldState* state : m_GeometryWorldStates)
		{
			if (state != nullptr && state->Geom != nullptr && state->Body != nullptr
				&& (state->Geom->IsSimulationEnabled() || state->Geom->IsTriggerEnabled()))
			{
				States->push_back(state);
				Objects->push_back(state->Geom);
			}
		}
	}

	void PhysicsWorld::GetCurrentGeometryWorldStates(std::vector<GeometryWorldState>* StateStorage,
		std::vector<const GeometryWorldState*>* States, std::vector<Geometry*>* Objects) const
	{
		if (StateStorage == nullptr || States == nullptr || Objects == nullptr)
		{
			return;
		}
		StateStorage->clear();
		States->clear();
		Objects->clear();
		StateStorage->reserve(m_GeometryWorldStates.size());
		for (const GeometryWorldState* previousState : m_GeometryWorldStates)
		{
			if (previousState == nullptr || previousState->Geom == nullptr || previousState->Body == nullptr
				|| (!previousState->Geom->IsSimulationEnabled() && !previousState->Geom->IsTriggerEnabled()))
			{
				continue;
			}
			GeometryWorldState state = *previousState;
			state.BodyToWorld = Transform(state.Body->BodyLocalToWorld(Vector3::Zero()), state.Body->Q);
			state.ShapeToWorld = state.Body->GetGeometryTransform(state.Geom);
			state.WorldBounds = Box3::Transform(state.Geom->GetShapeBounds(), state.ShapeToWorld.pos, state.ShapeToWorld.quat);
			RigidBodyDynamic* dynamicBody = state.Body->CastDynamic();
			state.MovingRigid = dynamicBody ? !dynamicBody->Sleeping : state.Body->mRigidType != RigidType::Static;
			state.Displacement = state.ShapeToWorld.pos - previousState->ShapeToWorld.pos;
			state.Moved = state.BodyToWorld != previousState->BodyToWorld
				|| state.ShapeToWorld != previousState->ShapeToWorld
				|| !(state.WorldBounds == previousState->WorldBounds)
				|| state.MovingRigid != previousState->MovingRigid;
			StateStorage->push_back(state);
		}
		States->reserve(StateStorage->size());
		Objects->reserve(StateStorage->size());
		for (const GeometryWorldState& state : *StateStorage)
		{
			States->push_back(&state);
			Objects->push_back(state.Geom);
		}
	}

	void		PhysicsWorld::SimulateST(float dt)
	{
		if (m_Solver)
		{
			m_Solver->BeginFrame(m_Clock.tick);
		}

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			m_Kinematics[i]->Simulate(dt);
		}
		for (PositionConstraint* Constraint : m_Constraints)
		{
			if (Constraint) Constraint->BeginStep(dt, m_ResolutionSolver == ResolutionPhaseSolver::XPBD);
		}
		// Project joint drift before collision generation so joint correction cannot
		// move a body back into an already solved contact later in the frame.
		if (!m_PositionSolver) SolveConstraintPositions(dt);
		PreIntegrate(dt);
		MotionIntegration::IntegrateVelocities(m_DynamicBodies, dt);
		if (m_PositionSolver)
		{
			SimulatePositionBasedRigidBodies(dt);
			SimulateCloths(dt);
			for (DeformableBody* Body : m_DeformableBodies) Body->Simulate(dt);
			PostIntegrate(dt);
			UpdateDestructionSets(dt);
			return;
		}

		std::vector<const GeometryWorldState*> simulationStates;
		std::vector<Geometry*> geoms;
		UpdateGeometryWorldStates();
		GetActiveGeometryWorldStates(&simulationStates, &geoms);

		GeometryWorldStateSpan stateSpan(simulationStates.data(), simulationStates.size());
		std::vector<OverlapPair> overlaps;
		if (m_BPhase)
		{
			m_BPhase->ProduceOverlaps(stateSpan, &overlaps);
		}

		std::vector<OverlapPair> contactOverlaps;
		std::vector<OverlapPair> triggerOverlaps;
		for (const OverlapPair& overlap : overlaps)
		{
			Geometry* geomA = geoms[(size_t)overlap.index1];
			Geometry* geomB = geoms[(size_t)overlap.index2];
			if (geomA->IsTriggerEnabled() || geomB->IsTriggerEnabled())
			{
				triggerOverlaps.push_back(overlap);
			}
			else if (geomA->IsSimulationEnabled() && geomB->IsSimulationEnabled())
			{
				contactOverlaps.push_back(overlap);
			}
		}

		std::vector<ContactManifold*> manifolds;
		if (m_NPhase && !triggerOverlaps.empty())
		{
			std::vector<ContactManifold*> triggerManifolds;
			m_NPhase->CollisionDetection(stateSpan, triggerOverlaps, &triggerManifolds);
			DispatchTriggerCallbacks(geoms, triggerManifolds);
		}
		if (m_NPhase && !contactOverlaps.empty())
		{
			m_NPhase->CollisionDetection(stateSpan, contactOverlaps, &manifolds);
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

					for (size_t i = 0; i < manifold_islands.islands.size(); ++i)
					{
						m_Solver->ResolveContact(geoms, manifold_islands.islands[i], dt);
					}
					m_Solver->PostResolve(geoms);
					for (size_t i = 0; i < manifold_islands.islands.size(); ++i)
					{
						DispatchContactCallbacks(geoms, manifold_islands.islands[i]);
					}
				}
				else
				{
					m_Solver->ResolveContact(geoms, manifolds, dt);
					m_Solver->PostResolve(geoms);
					DispatchContactCallbacks(geoms, manifolds);
				}
			}
			else
			{
				DispatchContactCallbacks(geoms, manifolds);
			}
		}

		std::vector<RigidBody*> allBodies;
		allBodies.reserve(m_StaticBodies.size() + m_DynamicBodies.size());
		allBodies.insert(allBodies.end(), m_StaticBodies.begin(), m_StaticBodies.end());
		allBodies.insert(allBodies.end(), m_DynamicBodies.begin(), m_DynamicBodies.end());
		SolveConstraintVelocities(dt);
		// Rope attachments generate body velocity before CCD so fast mouse-driven
		// pulls are swept instead of teleporting bodies after collision handling.
		SimulateRopes(dt);
		std::vector<ContinuousCollisionHit> triggerHits;
		ContinuousCollisionDetection::FindTriggerHits(allBodies, dt, &triggerHits);
		DispatchTriggerCCDCallbacks(triggerHits);
		// CCD must use the velocities produced by the articulation solve. Running the
		// joint solver after CCD could give a connected bone new untested motion.
		m_LastCCDHitCount = ContinuousCollisionDetection::Resolve(m_DynamicBodies, allBodies, dt);

		// Contact impulses and joints affect the same articulated bodies. Finish the
		// velocity solve with contacts so a joint cannot reintroduce inward velocity
		// after the collision island has been solved.
		if (m_Solver && !m_Constraints.empty() && !manifolds.empty())
		{
			std::vector<RigidBody*> constrainedContactBodies;
			BuildConstrainedContactBodies(geoms, manifolds, &constrainedContactBodies);
			std::vector<ContactManifold*> constrainedManifolds;
			for (ContactManifold* manifold : manifolds)
			{
				if (manifold == nullptr || manifold->indexA < 0 || manifold->indexB < 0
					|| (size_t)manifold->indexA >= geoms.size() || (size_t)manifold->indexB >= geoms.size())
				{
					continue;
				}
				RigidBody* bodyA = geoms[(size_t)manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = geoms[(size_t)manifold->indexB]->GetParent<RigidBody>();
				if (std::find(constrainedContactBodies.begin(), constrainedContactBodies.end(), bodyA) != constrainedContactBodies.end()
					|| std::find(constrainedContactBodies.begin(), constrainedContactBodies.end(), bodyB) != constrainedContactBodies.end())
				{
					constrainedManifolds.push_back(manifold);
				}
			}
			if (!constrainedManifolds.empty())
			{
				m_Solver->PreResolve(geoms);
				ContactManifoldIslands manifoldIslands;
				manifoldIslands.BuildIslands(geoms, constrainedManifolds);
				for (std::vector<ContactManifold*>& island : manifoldIslands.islands)
				{
					m_Solver->ResolveContact(geoms, island, dt);
				}
				m_Solver->PostResolve(geoms);
			}
		}

		MotionIntegration::IntegratePoses(m_DynamicBodies, dt, (uint8_t)m_IntegrateMethod);
		SolvePostIntegrationPositions(dt);
		SimulateCloths(dt);
		for (DeformableBody* body : m_DeformableBodies) body->Simulate(dt);

		PostIntegrate(dt);
		UpdateDestructionSets(dt);

		return;
	}

	void PhysicsWorld::SimulatePositionBasedRigidBodies(float Dt)
	{
		m_PositionSolver->BeginStep(m_DynamicBodies, Dt);
		// Rope attachments and joint velocity limits must act before CCD, while
		// poses still describe the start of the interval being swept.
		SolveConstraintVelocities(Dt);
		SimulateRopes(Dt);
		std::vector<RigidBody*> AllBodies;
		AllBodies.reserve(m_StaticBodies.size() + m_DynamicBodies.size());
		AllBodies.insert(AllBodies.end(), m_StaticBodies.begin(), m_StaticBodies.end());
		AllBodies.insert(AllBodies.end(), m_DynamicBodies.begin(), m_DynamicBodies.end());
		std::vector<ContinuousCollisionHit> TriggerHits;
		ContinuousCollisionDetection::FindTriggerHits(AllBodies, Dt, &TriggerHits);
		DispatchTriggerCCDCallbacks(TriggerHits);
		m_LastCCDHitCount = ContinuousCollisionDetection::Resolve(m_DynamicBodies, AllBodies, Dt);
		m_PositionSolver->PredictPoses(Dt);

		const PBDSolverSettings& Settings = m_PositionSolver->GetSettings();
		for (int Iteration = 0; Iteration < Settings.PositionIterations; ++Iteration)
		{
			for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled()) Constraint->SolvePosition(Dt);
			}
			// Refresh candidates after joint projection. Keep owned contact rows in
			// the solver because NarrowPhase reuses its manifold pool on each call.
			UpdateGeometryWorldStates();
			std::vector<const GeometryWorldState*> States;
			std::vector<Geometry*> Geometries;
			GetActiveGeometryWorldStates(&States, &Geometries);
			const GeometryWorldStateSpan Span(States.data(), States.size());
			std::vector<OverlapPair> Overlaps, ContactOverlaps, TriggerOverlaps;
			if (m_BPhase) m_BPhase->ProduceOverlaps(Span, &Overlaps);
			for (const OverlapPair& Pair : Overlaps)
			{
				Geometry* A = Geometries[(size_t)Pair.index1];
				Geometry* B = Geometries[(size_t)Pair.index2];
				if (A->IsTriggerEnabled() || B->IsTriggerEnabled()) TriggerOverlaps.push_back(Pair);
				else if (A->IsSimulationEnabled() && B->IsSimulationEnabled()) ContactOverlaps.push_back(Pair);
			}
			if (Iteration == 0 && m_NPhase && !TriggerOverlaps.empty())
			{
				std::vector<ContactManifold*> TriggerManifolds;
				m_NPhase->CollisionDetection(Span, TriggerOverlaps, &TriggerManifolds);
				DispatchTriggerCallbacks(Geometries, TriggerManifolds);
			}
			if (m_NPhase && !ContactOverlaps.empty())
			{
				std::vector<ContactManifold*> Manifolds;
				m_NPhase->CollisionDetection(Span, ContactOverlaps, &Manifolds);
				m_PositionSolver->AddContacts(Geometries, Manifolds);
			}
			m_PositionSolver->SolvePositions(Dt);
		}
		m_PositionSolver->ReconstructVelocities(Dt);
		for (int Iteration = 0; Iteration < Settings.VelocityIterations; ++Iteration)
		{
			for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled()) Constraint->SolveVelocity(Dt);
			}
			m_PositionSolver->SolveVelocities(Dt);
		}
		m_PositionSolver->EndStep(Dt);
		DispatchContactCallbacks(m_PositionSolver->GetContactGeometries(), m_PositionSolver->GetContactManifolds());
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
			if (Body->Sleeping)
				continue;
			// Loads applied by callbacks after the velocity phase are queued for the
			// next frame. Keep the body awake so that integration can consume them.
			const bool HasPendingLoad = Body->ExtForce != Vector3::Zero()
				|| Body->ExtTorque != Vector3::Zero();
			bool HasCompliantJoint = false;
			if (m_ResolutionSolver == ResolutionPhaseSolver::XPBD)
			{
				for (const PositionConstraint* Constraint : m_Constraints)
				{
					if (Constraint && Constraint->IsEnabled()
						&& (Constraint->GetBodyA() == Body || Constraint->GetBodyB() == Body)
						&& (Constraint->GetLinearCompliance() > 0.0f || Constraint->GetAngularCompliance() > 0.0f))
					{
						HasCompliantJoint = true;
						break;
					}
				}
			}
			if (!HasPendingLoad && !HasCompliantJoint && (!m_PositionSolver || m_PositionSolver->CanSleep(Body)))
			{
				Body->AutoSleep();
			}
			UpdateSceneQuery(Body);
		}
	}

	void PhysicsWorld::BuildSceneQuery(const std::vector<Geometry*>& Objects)
	{
		UpdateGeometryWorldStates(Objects);
		std::vector<const GeometryWorldState*> queryStates;
		queryStates.reserve(Objects.size());
		for (Geometry* Object : Objects)
		{
			const GeometryWorldRecord* record = GetGeometryWorldRecord(EnsureGeometryHandle(Object));
			if (record && record->State.Geom)
			{
				queryStates.push_back(&record->State);
			}
		}
		BuildSceneQuery(GeometryWorldStateSpan(queryStates.data(), queryStates.size()));
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
		AddGeometryWorldState(Object);
		const GeometryWorldRecord* record = GetGeometryWorldRecord(EnsureGeometryHandle(Object));
		if (record)
		{
			const GeometryWorldState& queryState = record->State;
			m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld);
		}
	}

	void PhysicsWorld::RemoveGeometryFromSceneQuery(Geometry* Object)
	{
		if (Object == nullptr)
		{
			return;
		}

		GeometryHandle handle;
		RigidBody* body = Object->GetParent<RigidBody>();
		if (body)
		{
			handle = body->FindGeometryHandle(Object);
		}
		else
		{
			for (size_t i = 0; i < m_GeometryWorldRecords.size(); ++i)
			{
				const GeometryWorldRecord& record = m_GeometryWorldRecords[i];
				if (record.Active && record.State.Geom == Object)
				{
					handle = GeometryHandle((uint32_t)i, record.Generation);
					break;
				}
			}
		}
		if (!handle.IsValid())
		{
			return;
		}

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

	RigidBody* PhysicsWorld::CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform, bool RegisterDisabledGeometries)
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

		body->mGeometries.reserve(Geoms.size());
		body->mGeometryHandles.reserve(Geoms.size());
		for (Geometry* Geom : Geoms)
		{
			if (!Geom)
			{
				continue;
			}

			const Transform current_transform = *Geom->GetTransform();
			const Transform local_transform = init_transform.TransformInv(current_transform);
			Geom->SetTransform(local_transform.pos, local_transform.quat);
			body->mGeometries.push_back(Geom);
			body->mGeometryHandles.push_back(GeometryHandle());
			Geom->SetParent(body);
		}
		body->UpdateMassParameters();

		for (Geometry* Geom : body->mGeometries)
		{
			if (RegisterDisabledGeometries || Geom->IsQueryEnabled() || Geom->IsSimulationEnabled())
			{
				AddGeometryToSceneQuery(Geom);
			}
		}

		return body;
	}

	bool PhysicsWorld::RemoveRigidBody(RigidBody* Body)
	{
		for (ClothSystem* Cloth : m_Cloths)
		{
			if (Cloth) Cloth->RemoveSphereCollider(Body);
		}
		for (RopeSystem* Rope : m_Ropes)
		{
			if (Rope)
			{
				Rope->DetachRigidBody(Body);
				Rope->RemoveColliders(Body);
			}
		}
		for (size_t i = m_Constraints.size(); i > 0; --i)
		{
			PositionConstraint* Constraint = m_Constraints[i - 1];
			if (Constraint->GetBodyA() == Body || Constraint->GetBodyB() == Body)
			{
				delete Constraint;
				m_Constraints.erase(m_Constraints.begin() + i - 1);
			}
		}
		if (Body->mRigidType == RigidType::Static)
		{
			for (size_t i = 0; i < m_StaticBodies.size(); ++i)
			{
				if (m_StaticBodies[i] == Body)
				{
					if (m_Solver)
					{
						m_Solver->ResetState();
					}
					if (m_PositionSolver) m_PositionSolver->ResetState();
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
					if (m_Solver)
					{
						m_Solver->ResetState();
					}
					if (m_PositionSolver) m_PositionSolver->ResetState();
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

	PositionConstraint* PhysicsWorld::AddConstraint(PositionConstraint* Constraint)
	{
		if (Constraint == nullptr || Constraint->GetBodyA() == nullptr || Constraint->GetBodyB() == nullptr || Constraint->GetBodyA() == Constraint->GetBodyB())
		{
			delete Constraint;
			return nullptr;
		}
		m_Constraints.push_back(Constraint);
		Constraint->Setup();
		return Constraint;
	}

	ForceField* PhysicsWorld::AddForceField(ForceField* Field)
	{
		if (Field == nullptr)
		{
			return nullptr;
		}
		if (std::find(m_Fields.begin(), m_Fields.end(), Field) == m_Fields.end())
		{
			m_Fields.push_back(Field);
		}
		return Field;
	}

	bool PhysicsWorld::RemoveForceField(ForceField* Field)
	{
		const auto It = std::find(m_Fields.begin(), m_Fields.end(), Field);
		if (It == m_Fields.end())
		{
			return false;
		}
		delete *It;
		m_Fields.erase(It);
		return true;
	}

	bool PhysicsWorld::RemoveConstraint(PositionConstraint* Constraint)
	{
		const auto it = std::find(m_Constraints.begin(), m_Constraints.end(), Constraint);
		if (it == m_Constraints.end())
		{
			return false;
		}
		delete *it;
		m_Constraints.erase(it);
		return true;
	}

	FixedConstraint* PhysicsWorld::CreateFixedConstraint(RigidBody* BodyA, RigidBody* BodyB)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<FixedConstraint*>(AddConstraint(new FixedConstraint(BodyA, BodyB)));
	}

	PointConstraint* PhysicsWorld::CreatePointConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<PointConstraint*>(AddConstraint(new PointConstraint(BodyA, BodyB, WorldAnchor)));
	}

	DistanceConstraint* PhysicsWorld::CreateDistanceConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchorA, const Vector3& WorldAnchorB, float MinDistance, float MaxDistance)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<DistanceConstraint*>(AddConstraint(new DistanceConstraint(BodyA, BodyB, WorldAnchorA, WorldAnchorB, MinDistance, MaxDistance)));
	}

	HingeConstraint* PhysicsWorld::CreateHingeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinAngle, float MaxAngle)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<HingeConstraint*>(AddConstraint(new HingeConstraint(BodyA, BodyB, WorldAnchor, WorldAxis, MinAngle, MaxAngle)));
	}

	SliderConstraint* PhysicsWorld::CreateSliderConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinDistance, float MaxDistance)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<SliderConstraint*>(AddConstraint(new SliderConstraint(BodyA, BodyB, WorldAnchor, WorldAxis, MinDistance, MaxDistance)));
	}

	ConeConstraint* PhysicsWorld::CreateConeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxisA, const Vector3& WorldAxisB, float MaxAngle)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<ConeConstraint*>(AddConstraint(new ConeConstraint(BodyA, BodyB, WorldAnchor, WorldAxisA, WorldAxisB, MaxAngle)));
	}

	SixDOFConstraint* PhysicsWorld::CreateSixDOFConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const SixDOFConstraintSettings& Settings)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<SixDOFConstraint*>(AddConstraint(new SixDOFConstraint(BodyA, BodyB, WorldAnchor, Settings)));
	}

	void PhysicsWorld::SolveConstraintPositions(float dt)
	{
		for (int iteration = 0; iteration < m_ConstraintIterations; ++iteration)
		{
			for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled())
				{
					Constraint->SolvePosition(dt);
				}
			}
		}
	}

	void PhysicsWorld::SolveConstraintVelocities(float dt)
	{
		for (int iteration = 0; iteration < m_ConstraintIterations; ++iteration)
		{
			for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled())
				{
					Constraint->SolveVelocity(dt);
				}
			}
		}
	}

	void PhysicsWorld::BuildConstrainedContactBodies(const std::vector<Geometry*>& Geometries,
		const std::vector<ContactManifold*>& Manifolds, std::vector<RigidBody*>* Bodies) const
	{
		if (Bodies == nullptr)
		{
			return;
		}
		Bodies->clear();
		auto AddDynamicBody = [&](RigidBody* Body) -> bool
		{
			if (Body == nullptr || Body->CastDynamic() == nullptr
				|| std::find(Bodies->begin(), Bodies->end(), Body) != Bodies->end())
			{
				return false;
			}
			Bodies->push_back(Body);
			return true;
		};
		for (PositionConstraint* constraint : m_Constraints)
		{
			if (constraint && constraint->IsEnabled())
			{
				AddDynamicBody(constraint->GetBodyA());
				AddDynamicBody(constraint->GetBodyB());
			}
		}

		bool changed = true;
		while (changed)
		{
			changed = false;
			for (ContactManifold* manifold : Manifolds)
			{
				if (manifold == nullptr || manifold->indexA < 0 || manifold->indexB < 0
					|| (size_t)manifold->indexA >= Geometries.size() || (size_t)manifold->indexB >= Geometries.size())
				{
					continue;
				}
				RigidBody* bodyA = Geometries[(size_t)manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = Geometries[(size_t)manifold->indexB]->GetParent<RigidBody>();
				RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
				RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
				const bool containsA = dynamicA
					&& std::find(Bodies->begin(), Bodies->end(), bodyA) != Bodies->end();
				const bool containsB = dynamicB
					&& std::find(Bodies->begin(), Bodies->end(), bodyB) != Bodies->end();
				if (containsA && dynamicB)
				{
					changed = AddDynamicBody(bodyB) || changed;
				}
				if (containsB && dynamicA)
				{
					changed = AddDynamicBody(bodyA) || changed;
				}
			}
		}
	}

	void PhysicsWorld::SolvePostIntegrationPositions(float dt)
	{
		if (m_Constraints.empty())
		{
			return;
		}
		const int positionIterations = 4;
		const int jointIterationsPerContact = 3;
		const float penetrationSlop = 0.005f;
		for (int iteration = 0; iteration < positionIterations; ++iteration)
		{
			for (int jointIteration = 0; jointIteration < jointIterationsPerContact; ++jointIteration)
			{
				for (PositionConstraint* constraint : m_Constraints)
				{
					if (constraint && constraint->IsEnabled())
					{
						constraint->SolvePosition(dt);
					}
				}
			}

			std::vector<GeometryWorldState> stateStorage;
			std::vector<const GeometryWorldState*> states;
			std::vector<Geometry*> geoms;
			GetCurrentGeometryWorldStates(&stateStorage, &states, &geoms);
			if (m_BPhase == nullptr || m_NPhase == nullptr || states.empty())
			{
				continue;
			}

			std::vector<OverlapPair> overlaps;
			m_BPhase->ProduceOverlaps(GeometryWorldStateSpan(states.data(), states.size()), &overlaps);
			if (overlaps.empty())
			{
				continue;
			}

			std::vector<OverlapPair> contactOverlaps;
			for (const OverlapPair& overlap : overlaps)
			{
				Geometry* geomA = geoms[(size_t)overlap.index1];
				Geometry* geomB = geoms[(size_t)overlap.index2];
				if (!geomA->IsTriggerEnabled() && !geomB->IsTriggerEnabled()
					&& geomA->IsSimulationEnabled() && geomB->IsSimulationEnabled())
				{
					contactOverlaps.push_back(overlap);
				}
			}
			if (contactOverlaps.empty())
			{
				continue;
			}
			std::vector<ContactManifold*> manifolds;
			m_NPhase->CollisionDetection(GeometryWorldStateSpan(states.data(), states.size()), contactOverlaps, &manifolds);
			std::vector<RigidBody*> constrainedContactBodies;
			BuildConstrainedContactBodies(geoms, manifolds, &constrainedContactBodies);
			for (ContactManifold* manifold : manifolds)
			{
				if (manifold == nullptr || manifold->NumContactPointCount <= 0
					|| manifold->indexA < 0 || manifold->indexB < 0
					|| (size_t)manifold->indexA >= geoms.size() || (size_t)manifold->indexB >= geoms.size())
				{
					continue;
				}
				RigidBody* bodyA = geoms[(size_t)manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = geoms[(size_t)manifold->indexB]->GetParent<RigidBody>();
				if (std::find(constrainedContactBodies.begin(), constrainedContactBodies.end(), bodyA) == constrainedContactBodies.end()
					&& std::find(constrainedContactBodies.begin(), constrainedContactBodies.end(), bodyB) == constrainedContactBodies.end())
				{
					continue;
				}
				RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
				RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
				const float weightA = dynamicA && !dynamicA->Freezing ? dynamicA->InvMass : 0.0f;
				const float weightB = dynamicB && !dynamicB->Freezing ? dynamicB->InvMass : 0.0f;
				const float weightSum = weightA + weightB;
				if (weightSum <= kMinimumInvMass)
				{
					continue;
				}
				const Contact* deepestContact = &manifold->ContactPoints[0];
				for (int contactIndex = 1; contactIndex < manifold->NumContactPointCount; ++contactIndex)
				{
					if (manifold->ContactPoints[contactIndex].PenetrationDepth > deepestContact->PenetrationDepth)
					{
						deepestContact = &manifold->ContactPoints[contactIndex];
					}
				}
				const float depth = std::min(0.2f, deepestContact->PenetrationDepth - penetrationSlop);
				const Vector3 normal = deepestContact->Normal.SafeUnit();
				if (depth <= 0.0f || normal.IsZero())
				{
					continue;
				}
				if (dynamicA)
				{
					dynamicA->X -= normal * (depth * weightA / weightSum);
					dynamicA->Wakeup();
				}
				if (dynamicB)
				{
					dynamicB->X += normal * (depth * weightB / weightSum);
					dynamicB->Wakeup();
				}
			}
		}
		SolveConstraintVelocities(dt);
	}

	DeformableBody* PhysicsWorld::CreateDeformableBody(const DeformableBodySettings& Settings)
	{
		DeformableBody* body = new DeformableBody(Settings);
		m_DeformableBodies.push_back(body);
		return body;
	}

	bool PhysicsWorld::RemoveDeformableBody(DeformableBody* Body)
	{
		const auto it = std::find(m_DeformableBodies.begin(), m_DeformableBodies.end(), Body);
		if (it == m_DeformableBodies.end()) return false;
		delete *it;
		m_DeformableBodies.erase(it);
		return true;
	}

	RopeSystem* PhysicsWorld::CreateRope(const RopeSettings& Settings, const Vector3& Start, const Vector3& End)
	{
		RopeSystem* Rope = new RopeSystem(Settings, Start, End);
		m_Ropes.push_back(Rope);
		return Rope;
	}

	bool PhysicsWorld::RemoveRope(RopeSystem* Rope)
	{
		const auto it = std::find(m_Ropes.begin(), m_Ropes.end(), Rope);
		if (it == m_Ropes.end())
		{
			return false;
		}
		delete *it;
		m_Ropes.erase(it);
		return true;
	}

	void PhysicsWorld::SimulateRopes(float dt)
	{
		for (RopeSystem* Rope : m_Ropes)
		{
			if (Rope)
			{
				Rope->Simulate(dt);
			}
		}
	}

	ClothSystem* PhysicsWorld::CreateCloth(const ClothSettings& Settings, const Vector3& Center, const Vector3& HorizontalAxis, const Vector3& VerticalAxis)
	{
		ClothSystem* Cloth = new ClothSystem(Settings, Center, HorizontalAxis, VerticalAxis);
		m_Cloths.push_back(Cloth);
		return Cloth;
	}

	bool PhysicsWorld::RemoveCloth(ClothSystem* Cloth)
	{
		const auto it = std::find(m_Cloths.begin(), m_Cloths.end(), Cloth);
		if (it == m_Cloths.end())
		{
			return false;
		}
		delete *it;
		m_Cloths.erase(it);
		return true;
	}

	void PhysicsWorld::SimulateCloths(float dt)
	{
		for (ClothSystem* Cloth : m_Cloths)
		{
			if (Cloth) Cloth->Simulate(dt);
		}
	}

	Ragdoll* PhysicsWorld::CreateRagdoll(const RagdollSettings& Settings, const Transform& RootTransform)
	{
		Ragdoll* RagdollInstance = new Ragdoll(this, Settings, RootTransform);
		if (!RagdollInstance->IsValid())
		{
			RagdollInstance->Destroy();
			delete RagdollInstance;
			return nullptr;
		}
		m_Ragdolls.push_back(RagdollInstance);
		return RagdollInstance;
	}

	bool PhysicsWorld::RemoveRagdoll(Ragdoll* RagdollInstance)
	{
		const auto it = std::find(m_Ragdolls.begin(), m_Ragdolls.end(), RagdollInstance);
		if (it == m_Ragdolls.end())
		{
			return false;
		}
		RagdollInstance->Destroy();
		delete RagdollInstance;
		m_Ragdolls.erase(it);
		return true;
	}

	void PhysicsWorld::RefreshHeightFieldGeometry(Geometry* HeightFieldGeometry)
	{
		if (HeightFieldGeometry == nullptr) return;
		HeightField3* HeightField = HeightFieldGeometry->GetShapeObj<HeightField3>();
		if (HeightField == nullptr) return;
		HeightFieldGeometry->SetShapeBounds(HeightField->BV);
		GeometryWorldState State = UpdateGeometryWorldState(HeightFieldGeometry, m_Clock.tick);
		// A hole or interior edit can leave the overall bounds unchanged. Still
		// invalidate proxies and cached impulses associated with the old surface.
		GeometryWorldRecord* Record = GetGeometryWorldRecord(State.Handle);
		if (Record)
		{
			if (!State.Moved) ++Record->State.Version;
			Record->State.Moved = true;
			State = Record->State;
		}
		UpdateSceneQuery(State);
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		Box3 WorldDirtyBounds = Box3::Transform(HeightField->GetLastEditBounds(), State.ShapeToWorld.pos, State.ShapeToWorld.quat);
		WorldDirtyBounds.Thicken(0.05f);
		for (RigidBodyDynamic* Body : m_DynamicBodies)
		{
			for (const Geometry* Geom : Body->Geometries())
			{
				if (Geom != HeightFieldGeometry && WorldDirtyBounds.Intersect(Body->GetGeometryBounds(Geom)))
				{
					Body->Wakeup();
					break;
				}
			}
		}
	}

	bool PhysicsWorld::SetHeightFieldHeight(Geometry* HeightFieldGeometry, int X, int Z, float Height)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		if (HeightField == nullptr) return false;
		const uint64_t Revision = HeightField->GetRevision();
		if (!HeightField->SetHeight(X, Z, Height)) return false;
		if (HeightField->GetRevision() != Revision) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return true;
	}

	int PhysicsWorld::SetHeightFieldHeights(Geometry* HeightFieldGeometry, int X, int Z, int Width, int Height, const std::vector<float>& Heights)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		if (HeightField == nullptr) return 0;
		const int ModifiedCount = HeightField->SetHeightsRect(X, Z, Width, Height, Heights);
		if (ModifiedCount > 0) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return ModifiedCount;
	}

	namespace
	{
		bool HeightFieldBrushCenter(Geometry* GeometryObject, const Vector3& WorldCenter, Vector3& LocalCenter)
		{
			if (!std::isfinite(WorldCenter.x) || !std::isfinite(WorldCenter.y) || !std::isfinite(WorldCenter.z)) return false;
			RigidBody* Body = GeometryObject->GetParent<RigidBody>();
			const Transform ShapeTransform = Body ? Body->GetGeometryTransform(GeometryObject) : *GeometryObject->GetTransform();
			LocalCenter = ShapeTransform.WorldToLocal(WorldCenter);
			return std::isfinite(LocalCenter.x) && std::isfinite(LocalCenter.y) && std::isfinite(LocalCenter.z);
		}
	}

	int PhysicsWorld::ModifyHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float HeightDelta, bool LinearFalloff)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		if (HeightField == nullptr) return 0;
		Vector3 LocalCenter;
		if (!HeightFieldBrushCenter(HeightFieldGeometry, WorldCenter, LocalCenter)) return 0;
		const int ModifiedCount = HeightField->ModifyHeightCircle(LocalCenter.x, LocalCenter.z, Radius, HeightDelta, LinearFalloff);
		if (ModifiedCount > 0) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return ModifiedCount;
	}

	int PhysicsWorld::FillHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float TargetHeight, float Strength, bool LinearFalloff)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		Vector3 LocalCenter;
		if (HeightField == nullptr || !HeightFieldBrushCenter(HeightFieldGeometry, WorldCenter, LocalCenter)) return 0;
		const int ModifiedCount = HeightField->FillHeightCircle(LocalCenter.x, LocalCenter.z, Radius, TargetHeight, Strength, LinearFalloff);
		if (ModifiedCount > 0) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return ModifiedCount;
	}

	int PhysicsWorld::SmoothHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float Strength, int Iterations, bool LinearFalloff)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		Vector3 LocalCenter;
		if (HeightField == nullptr || !HeightFieldBrushCenter(HeightFieldGeometry, WorldCenter, LocalCenter)) return 0;
		const int ModifiedCount = HeightField->SmoothHeightCircle(LocalCenter.x, LocalCenter.z, Radius, Strength, Iterations, LinearFalloff);
		if (ModifiedCount > 0) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return ModifiedCount;
	}

	int PhysicsWorld::DigHeightFieldHole(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float Depth, bool LinearFalloff)
	{
		return ModifyHeightField(HeightFieldGeometry, WorldCenter, Radius, -std::abs(Depth), LinearFalloff);
	}

	bool PhysicsWorld::SetHeightFieldTriangleHole(Geometry* HeightFieldGeometry, int CellX, int CellZ, int TriangleIndex, bool Hole)
	{
		HeightField3* HeightField = HeightFieldGeometry ? HeightFieldGeometry->GetShapeObj<HeightField3>() : nullptr;
		if (HeightField == nullptr) return false;
		const uint64_t Revision = HeightField->GetRevision();
		if (!HeightField->SetTriangleHole(CellX, CellZ, TriangleIndex, Hole)) return false;
		if (HeightField->GetRevision() != Revision) RefreshHeightFieldGeometry(HeightFieldGeometry);
		return true;
	}

	void PhysicsWorld::AddDestructionSet(DestructionSet* DestructSet)
	{
		if (DestructSet == nullptr)
		{
			return;
		}
		PhysicsWorld* ExistingWorld = DestructSet->GetPhysicsWorld();
		if (ExistingWorld && ExistingWorld != this)
		{
			ExistingWorld->RemoveDestructionSet(DestructSet);
		}
		if (std::find(m_DestructionSets.begin(), m_DestructionSets.end(), DestructSet) == m_DestructionSets.end())
		{
			m_DestructionSets.push_back(DestructSet);
		}
		DestructSet->SetPhysicsWorld(this);
		if (ExistingWorld != this && !DestructSet->GetChunks().empty())
		{
			DestructSet->RebuildClustersFromGraph();
		}
	}

	void PhysicsWorld::RemoveDestructionSet(DestructionSet* DestructSet)
	{
		if (DestructSet && DestructSet->GetPhysicsWorld() == this)
		{
			DestructSet->ClearClusters();
			DestructSet->SetPhysicsWorld(nullptr);
		}
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

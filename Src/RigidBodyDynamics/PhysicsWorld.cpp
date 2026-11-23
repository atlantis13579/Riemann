#include "../Modules/Vehicle/VehicleDynamics.h"
#include "../Modules/Robotics/ArticulatedRobotController.h"
#include <assert.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "PhysicsWorld.h"
#include "../Deformation/DeformableStep.h"
#include "Ragdoll.h"
#include "PhysicalAnimation.h"
#include "Island.h"

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
#include "../Solver/LCPSolver.h"
#include "../Solver/PBDSolver.h"
#include "../Solver/XPBDSolver.h"
#include "ForceField.h"
#include "PhysicsField.h"
#include "KinematicsTree.h"
#include "ContinuousCollisionDetection.h"
#include "../Deformation/Rope.h"
#include "../Deformation/DeformableBody.h"
#include "../Deformation/StrandSystem.h"
#include "../Deformation/Cloth.h"
#include "../Fluid/AerodynamicsSimulation.h"
#include "../Fluid/AerodynamicsRigidCoupling.h"

namespace Riemann
{
	struct PhysicsWorld::FrameCollisionData : BroadPhaseResult
	{
		bool Built = false, ForcesReady = false;
		std::vector<const GeometryWorldState*> Pointers;
		std::vector<OverlapPair> Contacts, Triggers;
		std::vector<int> Indices;
		std::vector<GeometryHandle> Registered;
		std::vector<float> ProjectionTravel;
		std::vector<Box3> ContactBounds;
		float Dt = 0;
	};
	namespace
	{
		void CreateResolutionSolvers(ResolutionPhaseSolver Solver, SolverBackend Backend, bool AllowFallback,
			const PBDSolverSettings& PBDSettings, const XPBDSolverSettings& XPBDSettings,
			const LCPSolverSettings& LCPSettings, std::unique_ptr<ConstraintSolver>& ContactSolver,
			std::unique_ptr<PBDSolver>& PositionSolver, const ContactPersistenceSettings& Persistence,
			const FrictionPatchSettings& Friction, const SequentialImpulseSettings& Sequential)
		{
			if (!Sequential.IsValid()) throw std::invalid_argument("Invalid sequential impulse iteration settings");
			if (static_cast<uint8_t>(Backend) > static_cast<uint8_t>(SolverBackend::CUDA))
				throw std::runtime_error("Invalid solver backend");
			switch (Solver)
			{
			case ResolutionPhaseSolver::SequentialImpulse:
				if (Backend != SolverBackend::CPU)
					throw std::runtime_error("Sequential impulse supports the CPU backend only");
				ContactSolver.reset(ConstraintSolver::CreateSequentialImpulseSolver(&Persistence, &Sequential));
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
			if (ContactSolver) ContactSolver->SetFrictionPatchSettings(Friction);
			if (PositionSolver) PositionSolver->SetFrictionPatchSettings(Friction);
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
		if (!param.contactGeneration.IsValid()) throw std::invalid_argument("Invalid contact generation settings");
		if (!param.meshContactSettings.IsValid()) throw std::invalid_argument("Invalid mesh contact settings");
		if (!param.contactPersistence.IsValid()) throw std::invalid_argument("Invalid contact persistence settings");
		if (!param.frictionPatches.IsValid()) throw std::invalid_argument("Invalid friction patch settings");
		if (!param.islandSleeping.IsValid()) throw std::invalid_argument("Invalid island sleeping settings");
		if (!param.supportStabilization.IsValid()) throw std::invalid_argument("Invalid support stabilization settings");
		if (!param.temporalSubsteps.IsValid()) throw std::invalid_argument("Invalid temporal substep settings");
		if (!param.ccdContinuation.IsValid()) throw std::invalid_argument("Invalid CCD continuation settings");
		if (param.narrowphase != NarrowPhaseSolver::GJKEPA && param.narrowphase != NarrowPhaseSolver::PCM)
			throw std::invalid_argument("Invalid narrow phase solver");
		m_PBDSettings = param.pbdSettings;
		m_XPBDSettings = param.xpbdSettings;
		m_LCPSettings.MaxIterations = std::max(1, param.contactSolverIterations);
		m_LCPSettings.Tolerance = std::max(0.0f, param.contactSolverTolerance);
		m_LCPSettings.Relaxation = std::max(0.01f, std::min(1.99f, param.contactSolverRelaxation));
		// Resolve strict backend requests before allocating world-owned raw resources.
		std::unique_ptr<ConstraintSolver> ContactSolver;
		std::unique_ptr<PBDSolver> PositionSolver;
		CreateResolutionSolvers(param.resolutionSolver, param.solverBackend, param.allowBackendFallback,
			m_PBDSettings, m_XPBDSettings, m_LCPSettings, ContactSolver, PositionSolver, param.contactPersistence, param.frictionPatches, param.sequentialImpulseSettings);
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
		m_CollisionFilter = param.collisionFilter;
		m_CollisionFilteringEnabled = param.collisionFilteringEnabled;
		m_BPhase->SetCollisionFilter(GetCollisionFilter());
		m_GeometryQuery->SetDefaultCollisionFilter(GetCollisionFilter());
		m_ContactGeneration = param.contactGeneration;
		m_ContactPersistence = param.contactPersistence;
		m_FrictionPatches = param.frictionPatches;
		m_JointDrives = param.jointDrives;
		m_JointBreaking = param.jointBreaking;
		SetCCDContinuationSettings(param.ccdContinuation);
		SetTemporalSubstepSettings(param.temporalSubsteps);
		m_SupportStabilization = param.supportStabilization;
		m_IslandSleeping.Configure(param.islandSleeping);
		m_SequentialImpulseSettings = param.sequentialImpulseSettings;
		m_MeshContactSettings = param.meshContactSettings;
		m_NarrowPhaseSolver = param.narrowphase;
		m_NPhase = param.narrowphase == NarrowPhaseSolver::PCM
			? NarrowPhase::Create_PCM(m_ContactPersistence, m_MeshContactSettings) : NarrowPhase::Create_GJKEPA(m_MeshContactSettings);
		m_Solver = ContactSolver.get();
		m_SequentialImpulseWarmStart = param.sequentialImpulseWarmStart;
		if (m_Solver) m_Solver->SetWarmStartingEnabled(m_SequentialImpulseWarmStart);
		m_PositionSolver = PositionSolver.get();
		m_ResolutionSolver = param.resolutionSolver;
		m_RequestedSolverBackend = param.solverBackend;
		m_AllowBackendFallback = param.allowBackendFallback;
		m_IntegrateMethod = param.integrateMethod;
		m_OnContact = param.onContact ? param.onContact : OnContactCallback(DefaultOnContact);
		m_OnJointBreak = param.onJointBreak;
		m_OnTrigger = param.onTrigger ? param.onTrigger : OnTriggerCallback(DefaultOnTrigger);
		m_Fields.push_back(ForceField::CreateGrivityField(param.gravityAcc));
		m_GravityAcceleration = param.gravityAcc;
		m_PhysicsFields.reset(new PhysicsFieldSystem(*this));
		m_Jobsystem = new JobSystem;
		if (param.workerThreads != 0)
		{
			m_Jobsystem->CreateWorkers(param.workerThreads);
		}
		m_SceneResource = nullptr;
		m_LastCCDHitCount = 0;
		m_ConstraintIterations = std::max(1, param.constraintIterations);
		m_UnifiedConstraints = param.unifiedConstraints;
		ContactSolver.release();
		PositionSolver.release();
	}

	PhysicsWorld::~PhysicsWorld()
	{
		RemoveAerodynamics();
		for (auto* Strands : m_Strands) delete Strands;
		m_Strands.clear();
		while (!m_DestructionPlaybacks.empty()) m_DestructionPlaybacks.back()->Detach();
		while (!m_PhysicalAnimations.empty()) RemovePhysicalAnimation(m_PhysicalAnimations.back());
		while (!m_ReducedRagdolls.empty()) RemoveRagdoll(m_ReducedRagdolls.back());
		while (!m_Vehicles.empty()) RemoveVehicle(m_Vehicles.back());
		while (!m_RobotControllers.empty()) RemoveRobotController(m_RobotControllers.back());
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
		m_DeformableContactBodies.clear();

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
		for (auto* Body : m_KinematicBodies) { Body->ReleaseGeometries(); delete Body; }
		m_KinematicBodies.clear();

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
		if (m_SimulationInProgress) throw std::logic_error("Physics world stepping is not reentrant");
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		if (!m_FrameCollisions) m_FrameCollisions.reset(new FrameCollisionData());
		m_FrameCollisions->Built = false;
		m_FrameCollisions->ForcesReady = false;
		m_FrameCollisions->Dt = Dt;
		bool CollisionPolicyChanged = false;
		for (GeometryWorldRecord& Record : m_GeometryWorldRecords)
		{
			if (!Record.Active || !Record.State.Geom) continue;
			const uint64_t FilterRevision = Record.State.Geom->GetFilterRevision();
			const uint64_t OffsetRevision = Record.State.Geom->GetContactOffsetRevision();
			if (FilterRevision != Record.FilterRevision || OffsetRevision != Record.ContactOffsetRevision)
			{
				Record.FilterRevision = FilterRevision; Record.ContactOffsetRevision = OffsetRevision;
				CollisionPolicyChanged = true;
			}
		}
		if (CollisionPolicyChanged) RefreshCollisionFiltering();
		const int Count = std::max(AerodynamicsSubsteps(Dt), std::max(m_TemporalSubsteps.Enabled ? m_TemporalSubsteps.Substeps : 1, DeformableCouplingSubsteps()));
		const float StepDt = Dt / Count;
		if (!(StepDt > 0)) return;
		++m_Clock.tick;
		m_StepInfo = { StepDt, Dt, 0, Count };
		m_SupportStabilizationStats = SupportStabilizationStats();
		m_IslandSleeping.BeginFrame();
		m_PhysicsFields->BeginFrame();
		for (auto* Strands : m_Strands) Strands->BeginWorldFrame(Dt);
		m_SimulationInProgress = true;
		m_DeformableCouplingActive = !m_DeformableRigidAttachments.empty() || HasDeformableRigidContacts() || HasClothRigidCoupling();
		if (Count > 1) for (auto* Joint : m_Constraints) if (Joint) Joint->BeginFrame(Dt);
		auto RestoreDeferredLoads = [&]()
		{
			for (auto* Body : m_DynamicBodies)
			{
				Body->ExtForce += Body->m_DeferredForce; Body->ExtTorque += Body->m_DeferredTorque;
				Body->m_DeferredForce.SetZero(); Body->m_DeferredTorque.SetZero();
			}
		};
		int CCDHits = 0;
		CCDContinuationStats CCDStats;
		try
		{
			for (int Step = 0; Step < Count; ++Step)
			{
				m_StepInfo.SubstepIndex = Step;
				// Playback edits topology before any substep borrows bodies or contacts.
				m_UpdatingPlayback = true;
				for (auto* Playback : m_DestructionPlaybacks) Playback->Step(StepDt);
				m_UpdatingPlayback = false;
				m_UpdatingFields = true;
				m_PhysicsFields->ApplyBoundary();
				m_UpdatingFields = false;
				++m_SolverFrame;
				m_BPhase->SetContactGeneration(m_ContactGeneration, StepDt);
				m_NPhase->SetContactGeneration(m_ContactGeneration, StepDt);
				m_NPhase->BeginFrame(m_SolverFrame);
				SimulateST(StepDt);
				for (RobotController* Character : m_RobotControllers) Character->Simulate(StepDt);
				FinishIslandSleeping(StepDt);
				m_PhysicsFields->FinishSubstep(StepDt);
				for (auto* Strands : m_Strands) Strands->SimulateWorldStep(StepDt);
				CCDHits += m_LastCCDHitCount;
				CCDStats.ImpactCount += m_CCDContinuationStats.ImpactCount;
				CCDStats.ResweepCount += m_CCDContinuationStats.ResweepCount;
				CCDStats.ClampedBodyCount += m_CCDContinuationStats.ClampedBodyCount;
				CCDStats.MaximumDroppedTime += m_CCDContinuationStats.MaximumDroppedTime;
				if (Count > 1) for (auto* Body : m_DynamicBodies)
				{
					Body->m_DeferredForce += Body->ExtForce; Body->m_DeferredTorque += Body->ExtTorque;
					Body->ExtForce.SetZero(); Body->ExtTorque.SetZero();
				}
			}
		}
		catch (...)
		{
			m_UpdatingPlayback = false;
			m_UpdatingFields = false;
			m_PhysicsFields->EndFrame();
			for (auto* Strands : m_Strands) Strands->EndWorldFrame(false);
			// A failed substep is not a rollback of physical state. Preserve queued
			// input and close frame bookkeeping so a caller can inspect or recover.
			RestoreDeferredLoads();
			if (Count > 1) for (auto* Joint : m_Constraints) if (Joint) Joint->EndFrame(false);
			m_SimulationInProgress = false;
			m_DeformableCouplingActive = false;
			throw;
		}
		m_LastCCDHitCount = CCDHits; m_CCDContinuationStats = CCDStats;
		m_PhysicsFields->EndFrame();
		for (auto* Strands : m_Strands) Strands->EndWorldFrame(true);
		m_SimulationInProgress = false;
		m_DeformableCouplingActive = false;
		if (Count > 1)
		{
			RestoreDeferredLoads();
			FinalizeConstraints(Dt, true);
		}
	}

	/* Temporal refinement and iteration refinement are independent choices.
	   Smaller physical steps update contacts, inertia, forces and joint geometry
	   more frequently; more iterations reduce the residual of a fixed step. See
	   Macklin et al., "Small Steps in Physics Simulation" (SCA 2019).
	   Broad phase runs once per public frame. Substeps and position iterations
	   refresh narrow-phase contacts using that shared candidate set.

	   Simulate(Dt) advances one public clock tick. Solver caches use a separate,
	   monotonically increasing substep id so warm starting never replays a load
	   from an earlier solve in the same substep. Fields, kinematics, CCD, ropes,
	   cloth and deformables all advance by Dt / N. Existing iteration settings
	   remain per-substep; equal-work comparisons must divide their budgets by N.
	   Temporal refinement does not remove poor mass conditioning. The low-budget
	   100:1 top-load fixture still collapses with substeps alone. The independent
	   SupportStabilization stage applies explicit nonphysical support corrections;
	   it leaves physical drive budgets and reaction reporting in this pipeline.

	   Queued legacy force/torque inputs are consumed once, as before. Loads added
	   by collision callbacks are held until the next outer frame. Persistent
	   fields instead integrate every substep. Contact/trigger callbacks report
	   each physical substep and include timing metadata; CCD fractions are in
	   outer-frame time. Joint reports integrate physical impulse across substeps,
	   and break callbacks run once at the outer boundary. Sleep decisions are also
	   deferred to that boundary. CCD counters sum substep events; dropped time is
	   the sum of each substep's maximum discarded interval.
	*/
	void PhysicsWorld::SetIslandSleepingSettings(const IslandSleepingSettings& Settings)
	{
		m_IslandSleeping.Configure(Settings);
		for (auto* Body : m_DynamicBodies) { Body->IslandSleepManaged = Settings.Enabled; Body->Wakeup(); }
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
	}
	void PhysicsWorld::TrackSleepContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts)
	{
		if (m_IslandSleeping.GetSettings().Enabled) m_IslandSleeping.AddContacts(Geometries, Contacts, m_DynamicBodies, m_Constraints);
	}
	void PhysicsWorld::FinishIslandSleeping(float Dt)
	{
		if (!m_IslandSleeping.GetSettings().Enabled) return;
		for (auto* Body : m_DynamicBodies)
			if (Body->ExtForce != Vector3::Zero() || Body->ExtTorque != Vector3::Zero()
				|| Body->m_DeferredForce != Vector3::Zero() || Body->m_DeferredTorque != Vector3::Zero()) m_IslandSleeping.Activate(Body);
		for (auto* Owner : m_ReducedRagdolls) if (m_JointDrives && Owner->HasActiveDrive())
			for (auto* Body : Owner->GetBodies()) m_IslandSleeping.Activate(Body);
		for (const auto& Link : m_DeformableRigidAttachments) m_IslandSleeping.Activate(Link.Rigid);
		for (auto* Body : m_DeformableContactBodies) m_IslandSleeping.Activate(Body);
		const bool SoftContacts = m_ResolutionSolver == ResolutionPhaseSolver::XPBD && m_XPBDSettings.ContactCompliance > 0;
		m_IslandSleeping.Finish(m_DynamicBodies, m_Constraints, Dt, m_StepInfo.SubstepIndex + 1 == m_StepInfo.SubstepCount, SoftContacts);
	}
	void PhysicsWorld::SetSupportStabilizationSettings(const SupportStabilizationSettings& Settings)
	{
		if (m_SimulationInProgress && m_DeformableCouplingActive) throw std::logic_error("Edit coupled world support settings between steps");
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid support stabilization settings");
		if ((!m_DeformableRigidAttachments.empty() || HasDeformableRigidContacts() || HasClothRigidCoupling()) && Settings.Enabled) throw std::invalid_argument("Support projection is incompatible with coupled deformables");
		if (!m_ReducedRagdolls.empty() && Settings.Enabled) throw std::invalid_argument("Support projection is not available for reduced-coordinate ragdolls");
		m_SupportStabilization = Settings;
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		for (auto* Body : m_DynamicBodies) Body->Wakeup();
	}
	void PhysicsWorld::StabilizeConstraintSupports(float Dt)
	{
		if (!m_SupportStabilization.Enabled) return;
		UpdateGeometryWorldStates();
		std::vector<const GeometryWorldState*> States;
		std::vector<Geometry*> Geometries;
		GetActiveGeometryWorldStates(&States, &Geometries);
		const GeometryWorldStateSpan Span(States.data(), States.size());
		const auto& Overlaps = GetRigidCollisionPairs(Span);
		std::vector<ContactManifold*> Contacts;
		if (!Overlaps.empty()) m_NPhase->CollisionDetection(Span, Overlaps, &Contacts);
		TrackSleepContacts(Geometries, Contacts);
		SupportGraph Graph; Graph.Build(Geometries, Contacts, m_Constraints, m_SupportStabilization.ContactTolerance);
		StabilizeSupports(Graph, m_SupportStabilization, Dt, m_SupportStabilizationStats);
	}

	void PhysicsWorld::SetTemporalSubstepSettings(const TemporalSubstepSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid temporal substep settings");
		m_TemporalSubsteps = Settings;
	}

	void PhysicsWorld::SetContactGenerationSettings(const ContactGenerationSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid contact generation settings");
		m_ContactGeneration = Settings;
		RefreshCollisionFiltering();
	}

	void PhysicsWorld::SetCollisionFilter(std::shared_ptr<CollisionFilter> Filter)
	{
		m_CollisionFilter = std::move(Filter);
		RefreshCollisionFiltering();
	}

	void PhysicsWorld::SetCollisionFilteringEnabled(bool Enabled)
	{
		if (m_CollisionFilteringEnabled == Enabled) return;
		m_CollisionFilteringEnabled = Enabled;
		RefreshCollisionFiltering();
	}

	void PhysicsWorld::RefreshCollisionFiltering()
	{
		// Policy changes occur between steps. Pair decisions are evaluated afresh,
		// but cached impulses and sleeping support relationships must also expire.
		// Call this after changing state captured by a custom filter callback.
		m_BPhase->SetCollisionFilter(GetCollisionFilter());
		m_GeometryQuery->SetDefaultCollisionFilter(GetCollisionFilter());
		m_NPhase->ResetState();
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
	}

	bool PhysicsWorld::SetNarrowPhaseSolver(NarrowPhaseSolver Solver)
	{
		if (Solver != NarrowPhaseSolver::GJKEPA && Solver != NarrowPhaseSolver::PCM) return false;
		if (Solver == m_NarrowPhaseSolver) return true;
		std::unique_ptr<NarrowPhase> Replacement(Solver == NarrowPhaseSolver::PCM
			? NarrowPhase::Create_PCM(m_ContactPersistence, m_MeshContactSettings) : NarrowPhase::Create_GJKEPA(m_MeshContactSettings));
		delete m_NPhase;
		m_NPhase = Replacement.release();
		m_NarrowPhaseSolver = Solver;
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
		return true;
	}

	void PhysicsWorld::SetTriangleContactsEnabled(bool Enabled)
	{
		if (m_MeshContactSettings.Enabled == Enabled) return;
		m_MeshContactSettings.Enabled = Enabled;
		m_NPhase->SetTriangleContactsEnabled(Enabled);
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
	}

	void PhysicsWorld::SetSequentialImpulseWarmStarting(bool Enabled)
	{
		m_SequentialImpulseWarmStart = Enabled;
		if (m_Solver) m_Solver->SetWarmStartingEnabled(Enabled);
	}

	size_t PhysicsWorld::GetCachedContactPairCount() const { return m_NPhase->GetCachedPairCount(); }

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
		if (m_SimulationInProgress && m_DeformableCouplingActive)
		{
			if (ErrorMessage) *ErrorMessage = "Change coupled world solvers between steps";
			return false;
		}
		if ((!m_DeformableRigidAttachments.empty() || HasDeformableRigidContacts() || HasClothRigidCoupling()) && Solver != ResolutionPhaseSolver::PBD && Solver != ResolutionPhaseSolver::XPBD)
		{
			if (ErrorMessage) *ErrorMessage = "Rigid deformable coupling currently requires PBD/XPBD";
			return false;
		}
		if (!m_ReducedRagdolls.empty() && (Solver != ResolutionPhaseSolver::SequentialImpulse || Backend != SolverBackend::CPU))
		{
			if (ErrorMessage) *ErrorMessage = "ReducedRagdolls currently require the CPU sequential impulse solver";
			return false;
		}
		if (Solver == m_ResolutionSolver && Backend == m_RequestedSolverBackend
			&& AllowFallback == m_AllowBackendFallback && Backend == GetSolverBackend()
			&& (!m_PositionSolver || m_PositionSolver->GetBackendStatistics().Message.empty())) return true;

		std::unique_ptr<ConstraintSolver> ContactSolver;
		std::unique_ptr<PBDSolver> PositionSolver;
		try
		{
			CreateResolutionSolvers(Solver, Backend, AllowFallback, m_PBDSettings, m_XPBDSettings,
				m_LCPSettings, ContactSolver, PositionSolver, m_ContactPersistence, m_FrictionPatches, m_SequentialImpulseSettings);
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
		if (m_Solver) m_Solver->SetWarmStartingEnabled(m_SequentialImpulseWarmStart);
		m_PositionSolver = PositionSolver.release();
		m_ResolutionSolver = Solver;
		m_RequestedSolverBackend = Backend;
		m_AllowBackendFallback = AllowFallback;
		for (PositionConstraint* Constraint : m_Constraints)
		{
			if (Constraint) Constraint->BeginStep(m_Clock.deltatime, Solver == ResolutionPhaseSolver::XPBD, m_JointDrives, m_PositionSolver != nullptr);
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
				point.RestOffset = contact.RestOffset;
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
			contactInfo.Step = m_StepInfo;
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
			triggerInfo.Fraction = (float)m_StepInfo.SubstepIndex / m_StepInfo.SubstepCount;
			triggerInfo.Step = m_StepInfo;
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
			triggerInfo.Fraction = (m_StepInfo.SubstepIndex + hit.Fraction) / m_StepInfo.SubstepCount;
			triggerInfo.Step = m_StepInfo;
			triggerInfo.IsCCD = true;
			m_OnTrigger(triggerInfo);
		}
	}

	void		PhysicsWorld::Reset()
	{
		if (m_SimulationInProgress && m_PhysicsFields->GetCount()) throw std::logic_error("Reset field worlds between steps");
		if (m_SimulationInProgress && !m_Strands.empty()) throw std::logic_error("Reset strand worlds between steps");
		if (m_SimulationInProgress && !m_DestructionPlaybacks.empty()) throw std::logic_error("Reset playback worlds between steps");
		while (!m_DestructionPlaybacks.empty()) m_DestructionPlaybacks.back()->Detach();
		if (m_SimulationInProgress && m_DeformableCouplingActive) throw std::logic_error("Reset coupled worlds between steps");
		m_PhysicsFields->Reset();
		for (auto* Strands : m_Strands) delete Strands;
		m_Strands.clear();
		for (const auto& Link : m_DeformableRigidAttachments) --Link.Soft->m_RigidAttachmentCount;
		m_DeformableRigidAttachments.clear();
		while (!m_PhysicalAnimations.empty()) RemovePhysicalAnimation(m_PhysicalAnimations.back());
		while (!m_ReducedRagdolls.empty()) RemoveRagdoll(m_ReducedRagdolls.back());
		while (!m_Vehicles.empty()) RemoveVehicle(m_Vehicles.back());
		while (!m_RobotControllers.empty()) RemoveRobotController(m_RobotControllers.back());
		m_IslandSleeping.Reset();
		m_NPhase->ResetState();
		if (m_Solver)
		{
			m_Solver->ResetState();
		}
		if (m_PositionSolver) m_PositionSolver->ResetState();

		RemoveAerodynamics();
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
		m_DeformableContactBodies.clear();

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
		for (auto* Body : m_KinematicBodies) { Body->ReleaseGeometries(); delete Body; }
		m_KinematicBodies.clear();

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
		m_CCDContinuationStats = CCDContinuationStats();
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
		record.QueryVersion = 0;
		record.State = GeometryWorldState();
		record.State.Geom = Object;
		record.FilterRevision = Object->GetFilterRevision();
		record.ContactOffsetRevision = Object->GetContactOffsetRevision();
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
		State.NonRigid = record->State.NonRigid;
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
			State.MovingRigid = State.NonRigid;
		}

		if (m_IslandSleeping.GetSettings().Enabled && m_IslandSleeping.IsActive(State.Body)) State.MovingRigid = true;
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
		if (m_SimulationInProgress && m_FrameCollisions && m_FrameCollisions->Built)
		{
			auto& Frame = *m_FrameCollisions;
			Frame.Registered.resize(m_GeometryWorldRecords.size());
			if (Frame.Registered[State.Handle.Index] != State.Handle)
			{
				// Playback/fracture can register geometry after the spatial pass. A new
				// registration conservatively joins the existing frame candidate set;
				// generation checks discard removed shapes without another spatial search.
				const int Next = (int)Frame.States.size();
				const size_t Begin = Frame.Pairs.size();
				for (int I = 0; I < Next; ++I)
				{
					const auto* Other = GetGeometryWorldRecord(Frame.States[I].Handle);
					if (Other && BroadPhase::AllowsPair(State, Other->State, GetCollisionFilter())) Frame.Pairs.emplace_back(I, Next);
				}
				Frame.States.push_back(State); Frame.Registered[State.Handle.Index] = State.Handle;
				Frame.DispatchPairs(Begin);
			}
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

		if (m_FrameCollisions) for (auto& Cached : m_FrameCollisions->States)
			if (Cached.Handle == Handle) { Cached.Geom = nullptr; Cached.Body = nullptr; break; }
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

	void PhysicsWorld::PrepareFrameCollisionPairs()
	{
		auto& Frame = *m_FrameCollisions;
		if (Frame.Built) return;
		Frame.Clear(); Frame.Pointers.clear();
		Frame.Contacts.clear(); Frame.Triggers.clear();
		UpdateGeometryWorldStates();
		const float Dt = Frame.Dt;
		// Temporal substeps can transfer momentum to an initially stationary body.
		// Reserve travel from the frame's kinetic energy as well as its own motion.
		double TwiceEnergy = 0;
		if (m_StepInfo.SubstepCount > 1) for (const auto* B : m_DynamicBodies) if (B->InvMass > 0)
		{
			const float Acceleration = Frame.ForcesReady ? B->ExtForce.Length() * B->InvMass : (B->DisableGravity ? 0 : m_GravityAcceleration.Length());
			const float Speed = B->V.Length() + Acceleration * Dt;
			TwiceEnergy += Speed * Speed / B->InvMass;
			const Matrix3 InverseInertia = B->GetInverseInertia_WorldSpace();
			if (InverseInertia.Invertible()) TwiceEnergy += std::max(0.0f, B->W.Dot(InverseInertia.Inverse() * B->W));
		}
		Frame.ProjectionTravel.assign(m_GeometryWorldRecords.size(), 0);
		for (const auto* J : m_Constraints) if (J && J->IsEnabled())
		{
			auto* A = J->GetBodyA(); auto* B = J->GetBodyB();
			if (!A || !B) continue;
			float Travel = (A->X - B->X).Length();
			switch (J->GetType())
			{
			case PositionConstraintType::Fixed:
				Travel += (A->Q * static_cast<const FixedConstraint*>(J)->GetRelativePosition() - (B->X - A->X)).Length(); break;
			case PositionConstraintType::Point: case PositionConstraintType::Hinge: case PositionConstraintType::Cone:
			{
				const auto* P = static_cast<const PointConstraint*>(J);
				Travel += (P->GetWorldAnchorA() - P->GetWorldAnchorB()).Length(); break;
			}
			case PositionConstraintType::Distance:
			{
				const auto* D = static_cast<const DistanceConstraint*>(J); const float Current = D->GetCurrentDistance();
				Travel += std::max(0.0f, std::max(D->GetMinDistance() - Current, Current - D->GetMaxDistance())); break;
			}
			case PositionConstraintType::Slider:
			{
				const auto* S = static_cast<const SliderConstraint*>(J); const float Current = S->GetCurrentPosition();
				Travel += std::max(0.0f, std::max(S->GetMinDistance() - Current, Current - S->GetMaxDistance()));
				if (S->HasActiveDrive()) Travel += std::abs(S->GetDrive().TargetPosition - Current) + std::abs(S->GetDrive().TargetVelocity) * Dt;
				break;
			}
			case PositionConstraintType::SixDOF:
				if (J->HasActiveDrive()) for (const auto& D : static_cast<const SixDOFConstraint*>(J)->GetDrives().Linear)
					Travel += std::abs(D.TargetPosition) + std::abs(D.TargetVelocity) * Dt;
				break;
			case PositionConstraintType::PoseDrive:
				if (J->HasActiveDrive())
				{
					const auto& D = static_cast<const PoseDriveConstraint*>(J)->GetDrive();
					Travel += (A->X + A->Q * D.Target.pos - B->X).Length() + D.LinearVelocityTarget.Length() * Dt;
				}
				break;
			}
			for (auto* Body : { A, B }) for (size_t I = 0; I < Body->mGeometryHandles.size(); ++I)
			{
				const auto H = Body->mGeometryHandles[I]; if (!IsValidGeometryHandle(H)) continue;
				const auto& S = GetGeometryWorldRecord(H)->State;
				const float Radius = (S.ShapeToWorld.pos - Body->X).Length() + ContactGeneration::ShapeScale(S.Geom, m_ContactGeneration);
				Frame.ProjectionTravel[H.Index] = std::max(Frame.ProjectionTravel[H.Index], Travel + 2 * Radius);
			}
		}
		const bool CanWakeRigid = m_DeformableCouplingActive || !m_KinematicBodies.empty()
			|| std::any_of(m_DynamicBodies.begin(), m_DynamicBodies.end(), [](const RigidBodyDynamic* B) { return !B->Sleeping; });
		for (const auto* S : m_GeometryWorldStates)
		{
			if (!S->Body || (!S->Geom->IsSimulationEnabled() && !S->Geom->IsTriggerEnabled())) continue;
			Frame.States.push_back(*S);
			auto& State = Frame.States.back();
			auto* B = State.Body;
			// Sleeping shapes may be woken by another solver later in this frame.
			State.MovingRigid = State.MovingRigid || (CanWakeRigid && B->mRigidType == RigidType::Dynamic);
			if (!State.MovingRigid) continue;
			const float Radius = (State.ShapeToWorld.pos - B->X).Length() + ContactGeneration::ShapeScale(State.Geom, m_ContactGeneration);
			State.FrameMotion = B->V.Length() * Dt + std::min(2.0f, B->W.Length() * Dt) * Radius;
			if (TwiceEnergy > 0 && B->InvMass > 0) State.FrameMotion = std::max(State.FrameMotion, (float)std::sqrt(TwiceEnergy * B->InvMass) * Dt);
			if (m_PositionSolver) State.FrameMotion += m_PositionSolver->GetSettings().MaxPositionCorrection;
			if (const auto* D = B->CastDynamic())
			{
				State.FrameMotion += D->ExtForce.Length() * D->InvMass * (Frame.ForcesReady ? Dt * Dt : Dt * 0.5f);
				if (Frame.ForcesReady) State.FrameMotion += (B->GetInverseInertia_WorldSpace() * D->ExtTorque).Length() * Dt * Dt * Radius;
				else if (!D->DisableGravity) State.FrameMotion += m_GravityAcceleration.Length() * Dt * Dt;
				if (m_AerodynamicsCoupling)
				{
					const auto& S = m_AerodynamicsCoupling->GetSettings();
					State.FrameMotion += (S.MaxLinearAcceleration + S.MaxAngularAcceleration * Radius) * Dt * Dt;
				}
			}
			State.FrameMotion += Frame.ProjectionTravel[State.Handle.Index];
		}
		auto AddProxy = [&](Geometry* G, float Motion, BroadPhaseObjectType Type, void* Owner)
		{
			if (!G) return;
			const auto Handle = EnsureGeometryHandle(G);
			GetGeometryWorldRecord(Handle)->State.NonRigid = true;
			UpdateGeometryWorldState(Handle, m_Clock.tick);
			Frame.States.push_back(GetGeometryWorldRecord(Handle)->State);
			Frame.States.back().FrameMotion = Motion;
			Frame.SetObject((int)Frame.States.size() - 1, Type, Owner);
		};
		// Each world-owned surface has exactly one registration. No geometry hash
		// table, per-solver proxy population, or per-iteration candidate copy is needed.
		for (auto* C : m_Cloths)
		{
			const auto& S = C->GetSettings();
			if (!S.Collision.RigidCoupling && !S.Collision.InterCollision
				&& !(m_Aerodynamics && m_AerodynamicsClothCoupling && S.AerodynamicsEnabled)) continue;
			C->UpdateRenderMesh();
			auto* G = C->GetRenderGeometry();
			G->SetFilterData(S.Collision.FilterData);
			G->SetContactOffsets(S.Thickness + S.Collision.ContactTolerance, 0);
			float Motion = 0;
			for (int I = 0; I < C->GetParticleCount(); ++I)
			{
				const auto& P = C->GetParticle(I);
				Motion = std::max(Motion, P.Pinned ? (P.PinPosition - P.Position).Length() : C->GetParticleVelocity(I).Length() * Dt + S.Gravity.Length() * Dt * Dt);
			}
			for (const auto& A : C->m_RigidAttachments)
			{
				Vector3 Point = Vector3::Zero();
				for (int I = 0; I < A.Point.Count; ++I) Point += C->GetParticle(A.Point.Indices[I]).Position * (float)A.Point.Weights[I];
				Motion = std::max(Motion, (A.Body->BodyLocalToWorld(A.LocalAnchor) - Point).Length() + A.Body->V.Length() * Dt);
			}
			if (S.Wind.NormalDrag > 0 || S.Wind.TangentialDrag > 0) Motion += S.Wind.Velocity.Length() * Dt;
			if (S.AnimationDrive.Enabled) for (size_t I = 0; I < C->m_AnimationEnd.size(); ++I)
				Motion = std::max(Motion, (C->m_AnimationEnd[I] - C->m_Particles[I].Position).Length());
			AddProxy(G, Motion, BroadPhaseObjectType::Cloth, C);
		}
		for (auto* B : m_DeformableBodies)
		{
			const auto& S = B->GetSettings(); B->UpdateRenderGeometry(); auto* G = B->GetRenderGeometry();
			if (!G || (!S.Collision.RigidCollision && !S.Collision.InterCollision)) continue;
			G->SetFilterData(S.Collision.FilterData);
			G->SetContactOffsets(S.ParticleRadius + S.Collision.ContactTolerance, 0);
			float Motion = 0;
			for (const auto& P : B->GetParticles())
				Motion = std::max(Motion, P.Pinned ? (P.PinPosition - P.Position).Length()
					: P.Velocity.Length() * Dt + (S.Gravity + P.Force * P.InverseMass).Length() * Dt * Dt);
			AddProxy(G, Motion, BroadPhaseObjectType::Deformable, B);
		}
		AddProxy(m_AerodynamicsDomain.get(), 0, BroadPhaseObjectType::Aerodynamics, m_Aerodynamics.get());
		Frame.Registered.assign(m_GeometryWorldRecords.size(), GeometryHandle());
		for (size_t I = 0; I < Frame.States.size(); ++I)
		{
			Frame.Pointers.push_back(&Frame.States[I]);
			Frame.Registered[Frame.States[I].Handle.Index] = Frame.States[I].Handle;
		}
		if (m_BPhase)
		{
			m_BPhase->SetContactGeneration(m_ContactGeneration, Dt);
			++m_BroadPhasePassCount;
			m_BPhase->ProduceOverlaps(GeometryWorldStateSpan(Frame.Pointers.data(), Frame.Pointers.size()), &Frame.Pairs);
		}
		Frame.DispatchPairs();
		Frame.Built = true;
	}
	const std::vector<OverlapPair>& PhysicsWorld::GetRigidCollisionPairs(GeometryWorldStateSpan States)
	{
		PrepareFrameCollisionPairs();
		auto& Frame = *m_FrameCollisions;
		Frame.Indices.assign(m_GeometryWorldRecords.size(), -1);
		Frame.ContactBounds.resize(States.size());
		for (size_t I = 0; I < States.size(); ++I)
		{
			Frame.Indices[States[I].Handle.Index] = (int)I;
			Frame.ContactBounds[I] = ContactGeneration::Bounds(States[I], m_ContactGeneration, m_StepInfo.DeltaTime);
		}
		Frame.Contacts.clear(); Frame.Triggers.clear();
		auto Collect = [&](const auto& Candidates)
		{
			for (const auto& P : Candidates)
			{
				const auto HA = Frame.States[P.index1].Handle, HB = Frame.States[P.index2].Handle;
				if (!IsValidGeometryHandle(HA) || !IsValidGeometryHandle(HB)) continue;
				const int A = Frame.Indices[HA.Index], B = Frame.Indices[HB.Index];
				if (A < 0 || B < 0 || !BroadPhase::AllowsPair(States[A], States[B], GetCollisionFilter())) continue;
				// Refresh contact eligibility without another spatial search.
				if (!Frame.ContactBounds[A].Intersect(Frame.ContactBounds[B])) continue;
				auto& Result = States[A].Geom->IsTriggerEnabled() || States[B].Geom->IsTriggerEnabled() ? Frame.Triggers : Frame.Contacts;
				Result.emplace_back(A, B);
			}
		};
		const auto Candidates = Frame.GetPairs(BroadPhasePairType::Rigid);
		if (Candidates.Indices) Collect(Candidates);
		else Collect(Frame.Pairs);
		return Frame.Contacts;
	}
	const std::vector<OverlapPair>& PhysicsWorld::GetTriggerCollisionPairs() const
	{
		// GetRigidCollisionPairs refreshes both views at the same poses.
		return m_FrameCollisions->Triggers;
	}
	const BroadPhaseResult& PhysicsWorld::GetCollisionCandidates()
	{
		// A world-owned material can also be advanced through its standalone API.
		// Only an active PhysicsWorld frame owns a reusable candidate snapshot.
		if (!m_FrameCollisions) m_FrameCollisions.reset(new FrameCollisionData());
		if (!m_SimulationInProgress) { m_FrameCollisions->Built = false; m_FrameCollisions->ForcesReady = false; m_FrameCollisions->Dt = 0; }
		PrepareFrameCollisionPairs();
		return *m_FrameCollisions;
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

	void PhysicsWorld::SetUnifiedConstraintSolving(bool Enabled)
	{
		if (m_UnifiedConstraints == Enabled) return;
		m_UnifiedConstraints = Enabled;
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
	}

	void PhysicsWorld::SetFrictionPatchSettings(const FrictionPatchSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid friction patch settings");
		if (m_Solver) { m_Solver->SetFrictionPatchSettings(Settings); m_Solver->ResetState(); }
		if (m_PositionSolver) { m_PositionSolver->SetFrictionPatchSettings(Settings); m_PositionSolver->ResetState(); }
		m_FrictionPatches = Settings;
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
	}
	void PhysicsWorld::SetJointDrivesEnabled(bool Enabled)
	{
		if (m_JointDrives == Enabled) return;
		m_JointDrives = Enabled;
		for (PositionConstraint* Constraint : m_Constraints)
			if (Constraint) Constraint->BeginStep(m_Clock.deltatime, m_ResolutionSolver == ResolutionPhaseSolver::XPBD, Enabled, m_PositionSolver != nullptr);
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
	}
	void PhysicsWorld::SetCCDContinuationSettings(const CCDContinuationSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid CCD continuation settings");
		m_CCDContinuation = Settings;
	}

	void PhysicsWorld::ResolveContinuousMotion(const std::vector<RigidBody*>& Bodies, float Dt)
	{
		std::vector<ContinuousCollisionHit> Hits;
		m_CCDContinuationStats = CCDContinuationStats();
		if (m_CCDContinuation.Enabled && m_ReducedRagdolls.empty())
		{
			m_CCDContinuationStats = ContinuousCollisionDetection::ResolveRemainingTime(m_DynamicBodies,
				Bodies, Dt, m_CCDContinuation, m_Constraints, &Hits, GetCollisionFilter());
			m_LastCCDHitCount = m_CCDContinuationStats.ImpactCount;
		}
		else
		{
			ContinuousCollisionDetection::FindTriggerHits(Bodies, Dt, &Hits, GetCollisionFilter());
			DispatchTriggerCCDCallbacks(Hits);
			m_LastCCDHitCount = ContinuousCollisionDetection::Resolve(m_DynamicBodies, Bodies, Dt, GetCollisionFilter());
			if (m_IslandSleeping.GetSettings().Enabled) m_IslandSleeping.Propagate(m_DynamicBodies, m_Constraints);
			return;
		}
		DispatchTriggerCCDCallbacks(Hits);
		if (m_IslandSleeping.GetSettings().Enabled) m_IslandSleeping.Propagate(m_DynamicBodies, m_Constraints);
	}

	void PhysicsWorld::SetJointBreakingEnabled(bool Enabled)
	{
		if (m_JointBreaking == Enabled) return;
		m_JointBreaking = Enabled;
		for (RigidBodyDynamic* Body : m_DynamicBodies) Body->Wakeup();
	}
	void PhysicsWorld::FinalizeConstraints(float Dt, bool EndFrame)
	{
		// Finalize every constraint before dispatch. Callbacks may remove joints or
		// bodies; a world-local monotonic id also rejects allocator address reuse.
		std::vector<PhysicsJointBreakInfo> Events;
		for (PositionConstraint* Joint : m_Constraints)
			if (Joint && (EndFrame ? Joint->EndFrame(m_JointBreaking) : Joint->EndStep(m_JointBreaking)))
				Events.push_back({ Joint, Joint->GetBodyA(), Joint->GetBodyB(), Joint->GetId(), m_Clock.tick, Dt, Joint->GetReaction() });
		if (!m_OnJointBreak) return;
		for (const auto& Event : Events)
		{
			const auto Found = std::find(m_Constraints.begin(), m_Constraints.end(), Event.Constraint);
			if (Found != m_Constraints.end() && (*Found)->GetId() == Event.ConstraintId) m_OnJointBreak(Event);
		}
	}
	void PhysicsWorld::SolveUnifiedVelocities(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts, float Dt, bool JointIslandsOnly)
	{
		// Each connected component alternates joint blocks with persistent contact
		// rows. Static anchors do not merge components. After pose correction, only
		// joint-bearing islands need another velocity solve at the new lever arms.
		// The selectable legacy route retains its independent contact/joint passes.
		ConstraintGraph Graph; Graph.Build(Geometries, Contacts, m_Constraints, m_DynamicBodies);
		m_Solver->PreResolve(Geometries, m_Constraints);
		for (auto& Island : Graph.GetIslands()) if (!JointIslandsOnly || !Island.Joints.empty())
			m_Solver->ResolveConstraints(Geometries, Island.Contacts, Island.Joints, Dt, m_ConstraintIterations);
		m_Solver->PostResolve(Geometries);
	}

	void		PhysicsWorld::SimulateST(float dt)
	{
		m_DeformableContactBodies.clear();
		std::vector<DeformableRigidCollider> SoftColliders;
		if (HasDeformableRigidContacts() || HasClothRigidCoupling())
		{
			UpdateGeometryWorldStates();
			for (const auto& Record : m_GeometryWorldRecords) if (Record.Active && Record.State.Body && Record.State.Geom->IsSimulationEnabled() && !Record.State.Geom->IsTriggerEnabled())
			{
				auto* B = Record.State.Body;
				SoftColliders.push_back({ Record.State.Geom, B, Transform(B->X - B->Q * B->CM, B->Q) });
			}
		}
		for (const auto& Link : m_DeformableRigidAttachments) if (auto* Body = Link.Rigid->CastDynamic()) Body->Wakeup();
		for (auto* C : m_Cloths) for (const auto& A : C->m_RigidAttachments) if (auto* B = A.Body->CastDynamic()) B->Wakeup();
		if (!m_ReducedRagdolls.empty()) for (auto* Body : m_DynamicBodies)
			if (Body->mMotionType == MorionType::Continuous) throw std::invalid_argument("Ragdoll worlds currently require discrete motion");
		for (auto* Owner : m_ReducedRagdolls) if (m_JointDrives && Owner->HasActiveDrive()) Owner->Wakeup();
		if (m_Solver)
		{
			m_Solver->BeginFrame(m_SolverFrame);
		}

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			m_Kinematics[i]->Simulate(dt);
		}
		for (auto* Animation : m_PhysicalAnimations) Animation->BeginStep(dt);
		for (PositionConstraint* Constraint : m_Constraints)
		{
			if (Constraint) Constraint->BeginStep(dt, m_ResolutionSolver == ResolutionPhaseSolver::XPBD, m_JointDrives, m_PositionSolver != nullptr);
		}
		if (m_IslandSleeping.GetSettings().Enabled)
		{
			for (auto& Record : m_GeometryWorldRecords) if (Record.Active && Record.State.Body) m_IslandSleeping.Observe(Record.State.Body);
			for (auto* Body : m_DynamicBodies) Body->IslandSleepManaged = true;
			m_IslandSleeping.Prepare(m_DynamicBodies, m_Constraints, m_SolverFrame, m_JointDrives, m_ResolutionSolver == ResolutionPhaseSolver::XPBD);
		}
		// Project joint drift before collision generation so joint correction cannot
		// move a body back into an already solved contact later in the frame.
		if (!m_PositionSolver) SolveConstraintPositions(dt);
		if (!m_Vehicles.empty())
		{
			SynchronizeSceneQueries();
			for (auto* Vehicle : m_Vehicles) Vehicle->Prepare(dt);
		}
		SimulateAerodynamics(dt);
		PreIntegrate(dt);
		m_FrameCollisions->ForcesReady = true;
		PrepareFrameCollisionPairs();
		for (auto* Owner : m_ReducedRagdolls) Owner->BeginStep(dt, m_JointDrives);
		MotionIntegration::IntegrateVelocities(m_DynamicBodies, dt);
		for (auto* Vehicle : m_Vehicles) Vehicle->Simulate(dt);
		if (m_PositionSolver)
		{
			SimulatePositionBasedRigidBodies(dt, SoftColliders);
			StabilizeConstraintSupports(dt);
			SimulateCloths(dt);
			if (!m_DeformableCouplingActive) DeformableBody::SimulateGroup(m_DeformableBodies, dt,
				m_StepInfo.FrameDeltaTime, m_StepInfo.SubstepIndex == 0, m_StepInfo.SubstepIndex + 1 == m_StepInfo.SubstepCount);
			PostIntegrate(dt);
			FinalizeConstraints(dt);
			UpdateDestructionSets(dt);
			return;
		}

		std::vector<const GeometryWorldState*> simulationStates;
		std::vector<Geometry*> geoms;
		UpdateGeometryWorldStates();
		GetActiveGeometryWorldStates(&simulationStates, &geoms);

		GeometryWorldStateSpan stateSpan(simulationStates.data(), simulationStates.size());
		const auto& contactOverlaps = GetRigidCollisionPairs(stateSpan);
		const auto& triggerOverlaps = GetTriggerCollisionPairs();

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
			TrackSleepContacts(geoms, manifolds);
		}

		if (m_UnifiedConstraints && m_Solver)
		{
			SolveUnifiedVelocities(geoms, manifolds, dt);
			DispatchContactCallbacks(geoms, manifolds);
		}
		else if (!manifolds.empty())
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
		allBodies.insert(allBodies.end(), m_KinematicBodies.begin(), m_KinematicBodies.end());
		if (!m_UnifiedConstraints || !m_Solver) SolveConstraintVelocities(dt);
		// Rope attachments generate body velocity before CCD so fast mouse-driven
		// pulls are swept instead of teleporting bodies after collision handling.
		SimulateRopes(dt);
		if (!m_CCDContinuation.Enabled) ResolveContinuousMotion(allBodies, dt);

		// Contact impulses and joints affect the same articulated bodies. Finish the
		// velocity solve with contacts so a joint cannot reintroduce inward velocity
		// after the collision island has been solved.
		if (!m_UnifiedConstraints && m_Solver && !m_Constraints.empty() && !manifolds.empty())
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

		// Continuation must see every velocity correction before caching the final pose.
		if (m_CCDContinuation.Enabled) ResolveContinuousMotion(allBodies, dt);
		MotionIntegration::IntegratePoses(m_DynamicBodies, dt, (uint8_t)m_IntegrateMethod);
		for (auto* Owner : m_ReducedRagdolls) Owner->IntegratePoses(dt);
		SolvePostIntegrationPositions(dt);
		StabilizeConstraintSupports(dt);
		SimulateCloths(dt);
		DeformableBody::SimulateGroup(m_DeformableBodies, dt,
			m_StepInfo.FrameDeltaTime, m_StepInfo.SubstepIndex == 0, m_StepInfo.SubstepIndex + 1 == m_StepInfo.SubstepCount);

		PostIntegrate(dt);
		FinalizeConstraints(dt);
		UpdateDestructionSets(dt);

		return;
	}

	void PhysicsWorld::SimulatePositionBasedRigidBodies(float Dt, const std::vector<DeformableRigidCollider>& Colliders)
	{
		m_PositionSolver->BeginStep(m_DynamicBodies, Dt);

		// Rope attachments and joint velocity limits must act before CCD, while
		// poses still describe the start of the interval being swept.
		if (m_UnifiedConstraints)
		{
			for (int Iteration = 0; Iteration < m_ConstraintIterations; ++Iteration)
				m_PositionSolver->SolveJointVelocities(m_Constraints, Dt);
		}
		else SolveConstraintVelocities(Dt);
		SimulateRopes(Dt);
		if (m_ContactGeneration.SpeculativeContacts || m_FrictionPatches.Enabled)
		{
			UpdateGeometryWorldStates();
			std::vector<const GeometryWorldState*> States;
			std::vector<Geometry*> Geometries;
			GetActiveGeometryWorldStates(&States, &Geometries);
			const GeometryWorldStateSpan Span(States.data(), States.size());
			const auto& Overlaps = GetRigidCollisionPairs(Span);
			std::vector<ContactManifold*> Manifolds;
			m_NPhase->CollisionDetection(Span, Overlaps, &Manifolds);
			TrackSleepContacts(Geometries, Manifolds);
			if (!m_ContactGeneration.SpeculativeContacts)
			{
				// Static friction must stop force-induced slip before PBD predicts a
				// pose. Restrict this pre-pass to touching contacts; separated shapes
				// still use the selected discrete collision route.
				for (ContactManifold* Manifold : Manifolds)
				{
					int Touching = 0;
					for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
						if (Manifold->ContactPoints[Index].PenetrationDepth >= -m_FrictionPatches.ContactTolerance)
							Manifold->ContactPoints[Touching++] = Manifold->ContactPoints[Index];
					Manifold->NumContactPointCount = Touching;
				}
			}
			if (m_UnifiedConstraints)
				m_PositionSolver->SolveSpeculativeContacts(Geometries, Manifolds, Dt, m_Constraints, m_ConstraintIterations, !m_ContactGeneration.SpeculativeContacts);
			else m_PositionSolver->SolveSpeculativeContacts(Geometries, Manifolds, Dt, {}, 0, !m_ContactGeneration.SpeculativeContacts);
		}
		std::vector<RigidBody*> AllBodies;
		AllBodies.reserve(m_StaticBodies.size() + m_DynamicBodies.size());
		AllBodies.insert(AllBodies.end(), m_StaticBodies.begin(), m_StaticBodies.end());
		AllBodies.insert(AllBodies.end(), m_DynamicBodies.begin(), m_DynamicBodies.end());
		AllBodies.insert(AllBodies.end(), m_KinematicBodies.begin(), m_KinematicBodies.end());
		ResolveContinuousMotion(AllBodies, Dt);
		m_PositionSolver->PredictPoses(Dt);

		std::unique_ptr<ClothStep> ClothStage;
		if (HasClothRigidCoupling()) ClothStage.reset(new ClothStep(m_Cloths, Colliders, GetCollisionFilter(), Dt));
		std::unique_ptr<DeformableStep> SoftStep;
		if (m_DeformableCouplingActive && !m_DeformableBodies.empty()) SoftStep.reset(new DeformableStep(m_DeformableBodies, m_DeformableRigidAttachments,
			Dt, m_StepInfo.FrameDeltaTime, m_StepInfo.SubstepIndex == 0, m_StepInfo.SubstepIndex + 1 == m_StepInfo.SubstepCount, Colliders, GetCollisionFilter()));
		const PBDSolverSettings& Settings = m_PositionSolver->GetSettings();
		const int JointIterations = m_UnifiedConstraints && !m_Constraints.empty() ? m_ConstraintIterations : 0;
		int PositionIterations = Settings.PositionIterations;
		if (SoftStep) for (auto* Body : m_DeformableBodies) PositionIterations = std::max(PositionIterations, Body->GetSettings().SolverIterations);
		if (ClothStage) for (auto* C : m_Cloths) if (C->HasRigidCoupling() || C->GetSettings().Collision.InterCollision) PositionIterations = std::max(PositionIterations, C->GetSettings().SolverIterations);
		for (int Iteration = 0; Iteration < PositionIterations; ++Iteration)
		{
			if (SoftStep) SoftStep->SolveConstraints();
			if (ClothStage) ClothStage->Solve(Iteration);
			if (m_UnifiedConstraints) m_PositionSolver->SolveJointPositions(m_Constraints, Dt);
			else for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled()) Constraint->SolvePosition(Dt);
			}
			// Refresh narrow-phase contacts after joint projection. Keep owned rows in
			// the solver because NarrowPhase reuses its manifold pool on each call.
			UpdateGeometryWorldStates();
			std::vector<const GeometryWorldState*> States;
			std::vector<Geometry*> Geometries;
			GetActiveGeometryWorldStates(&States, &Geometries);
			const GeometryWorldStateSpan Span(States.data(), States.size());
			const auto& ContactOverlaps = GetRigidCollisionPairs(Span);
			const auto& TriggerOverlaps = GetTriggerCollisionPairs();
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
				TrackSleepContacts(Geometries, Manifolds);
				m_PositionSolver->AddContacts(Geometries, Manifolds);
			}
			if (SoftStep) SoftStep->SolveContacts(GetCollisionCandidates());
			if (ClothStage) ClothStage->SolveContacts(GetCollisionCandidates());
			m_PositionSolver->SolvePositions(Dt);
		}
		if (SoftStep) { SoftStep->Finish(); m_DeformableContactBodies = SoftStep->GetContactBodies(); SoftStep.reset(); }
		if (ClothStage)
		{
			ClothStage->Finish();
			for (auto* B : ClothStage->GetContactBodies()) m_DeformableContactBodies.push_back(B);
			ClothStage.reset();
		}
		m_PositionSolver->ReconstructVelocities(Dt);
		for (int Iteration = 0; Iteration < std::max(Settings.VelocityIterations, JointIterations); ++Iteration)
		{
			if (m_UnifiedConstraints) m_PositionSolver->SolveJointVelocities(m_Constraints, Dt);
			else for (PositionConstraint* Constraint : m_Constraints)
			{
				if (Constraint && Constraint->IsEnabled()) Constraint->SolveVelocity(Dt);
			}
			m_PositionSolver->SolveVelocities(Dt, Iteration >= Settings.VelocityIterations);
		}
		m_PositionSolver->EndStep(Dt);
		DispatchContactCallbacks(m_PositionSolver->GetContactGeometries(), m_PositionSolver->GetContactManifolds());
	}

	void PhysicsWorld::UpdateDestructionSets(float dt)
	{
		// Partitioning replaces bodies after the rigid solve has released its
		// borrowed state. Strand root removal notifications are safe in this phase.
		m_UpdatingDestruction=true;
		try
		{
			for (DestructionSet* DestructSet : m_DestructionSets) DestructSet->Update(dt);
		}
		catch (...) { m_UpdatingDestruction=false; throw; }
		m_UpdatingDestruction=false;
	}

	void		PhysicsWorld::PreIntegrate(float dt)
	{
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			RigidBodyDynamic* Body = m_DynamicBodies[i];
			Body->ExtForce /= (dt * 2.0f);	 // Force Unit  1 Newton =  1 kg m / s^2
			// Composable fields bypass this historical input conversion below.
			Body->ExtTorque /= (dt * 2.0f);
			if (!Body->Sleeping)
				continue;
			if (!m_IslandSleeping.GetSettings().Enabled && Body->GetKinematicsEnergy() > Body->SleepThreshold)
			{
				Body->Wakeup();
			}
		}

		std::vector<RigidBodyDynamic*> SleepingBeforeFields;
		if (m_IslandSleeping.GetSettings().Enabled) for (auto* Body : m_DynamicBodies) if (Body->Sleeping) SleepingBeforeFields.push_back(Body);
		for (size_t j = 0; j < m_Fields.size(); ++j)
		{
			m_Fields[j]->Update(dt);

			for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
			{
				auto* Body = m_DynamicBodies[i];
				const Vector3 Force = Body->ExtForce, Torque = Body->ExtTorque;
				m_Fields[j]->ApplyForce(Body);
				if (m_IslandSleeping.GetSettings().Enabled && j != 0 && (Force != Body->ExtForce || Torque != Body->ExtTorque)) m_IslandSleeping.Activate(Body);
			}
		}
		m_PhysicsFields->ApplyForces();
		if (m_IslandSleeping.GetSettings().Enabled)
		{
			m_IslandSleeping.Propagate(m_DynamicBodies, m_Constraints);
			for (auto* Body : SleepingBeforeFields) if (!Body->Sleeping && !m_Fields.empty()) m_Fields[0]->ApplyForce(Body);
		}
	}

	void		PhysicsWorld::PostIntegrate(float dt)
	{
		(void)dt;
		// Gather active supports once instead of scanning every joint for every body.
		std::unordered_set<const RigidBody*> ActiveSupports;
		for (const PositionConstraint* Constraint : m_Constraints)
		{
			if (Constraint && Constraint->IsEnabled()
				&& (Constraint->HasActiveDrive() || (m_ResolutionSolver == ResolutionPhaseSolver::XPBD
					&& (Constraint->GetLinearCompliance() > 0.0f || Constraint->GetAngularCompliance() > 0.0f))))
			{
				ActiveSupports.insert(Constraint->GetBodyA());
				ActiveSupports.insert(Constraint->GetBodyB());
			}
		}
		for (auto* Vehicle : m_Vehicles) if (Vehicle->RequiresAwake()) ActiveSupports.insert(Vehicle->GetChassis());
		for (const auto& Link : m_DeformableRigidAttachments) ActiveSupports.insert(Link.Rigid);
		for (auto* Body : m_DeformableContactBodies) ActiveSupports.insert(Body);
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			RigidBodyDynamic* Body = m_DynamicBodies[i];
			if (Body->Sleeping)
				continue;
			// Loads applied by callbacks after the velocity phase are queued for the
			// next frame. Keep the body awake so that integration can consume them.
			const bool HasPendingLoad = Body->ExtForce != Vector3::Zero()
				|| Body->ExtTorque != Vector3::Zero() || Body->m_DeferredForce != Vector3::Zero() || Body->m_DeferredTorque != Vector3::Zero();
			const bool HasActiveJoint = ActiveSupports.find(Body) != ActiveSupports.end();
			if (!Body->ReducedRagdoll && !m_IslandSleeping.GetSettings().Enabled && m_StepInfo.SubstepIndex + 1 == m_StepInfo.SubstepCount && !HasPendingLoad && !HasActiveJoint
				&& (!m_PositionSolver || m_PositionSolver->CanSleep(Body)))
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
				GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
				if (record)
				{
					const GeometryWorldState& queryState = record->State;
					if (m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld))
						record->QueryVersion = queryState.Version;
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
			GeometryWorldRecord* record = GetGeometryWorldRecord(State.Handle);
			if (record)
			{
				const GeometryWorldState& queryState = record->State;
				if (m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld))
					record->QueryVersion = queryState.Version;
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
		GeometryWorldRecord* record = GetGeometryWorldRecord(EnsureGeometryHandle(Object));
		if (record)
		{
			const GeometryWorldState& queryState = record->State;
			if (m_GeometryQuery->AddGeometry(queryState.Handle, queryState.Geom, &queryState.WorldBounds, &queryState.ShapeToWorld))
				record->QueryVersion = queryState.Version;
		}
	}

	void PhysicsWorld::RemoveGeometryFromSceneQuery(Geometry* Object)
	{
		m_NPhase->Invalidate(Object);
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
		if (!State.Geom) return;
		GeometryWorldRecord* Record = GetGeometryWorldRecord(State.Handle);
		if (Record)
		{
			// Collision generation may already have observed a pose change. Track
			// the version committed to the query tree separately from State.Moved.
			const GeometryWorldState& QueryState = Record->State;
			if (Record->QueryVersion == QueryState.Version) return;
			if (m_GeometryQuery->UpdateGeometry(QueryState.Handle, QueryState.Geom,
				&QueryState.WorldBounds, &QueryState.ShapeToWorld, QueryState.Displacement))
				Record->QueryVersion = QueryState.Version;
		}
		else if (State.Moved)
			m_GeometryQuery->UpdateGeometry(State.Handle, State.Geom, State.WorldBounds, State.ShapeToWorld, State.Body, State.Displacement);
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
		if (m_SimulationInProgress && m_DeformableCouplingActive && !m_UpdatingPlayback && !m_UpdatingFields) throw std::logic_error("Create coupled world bodies between steps");
		RigidBody* body = RigidBody::CreateRigidBody(param, init_pose);
		if (!body) throw std::invalid_argument("Invalid rigid body type");
		if (body->mRigidType == RigidType::Static)
		{
			m_StaticBodies.push_back(body->CastStatic());
		}
		else if (body->mRigidType == RigidType::Dynamic)
		{
			m_DynamicBodies.push_back(body->CastDynamic());
		}

		if (body->CastKinematic()) m_KinematicBodies.push_back(body->CastKinematic());
		return body;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
	{
		if (!Geom) throw std::invalid_argument("Rigid body requires geometry");
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
		if (!Body) return false;
		if (m_SimulationInProgress && !m_Strands.empty() && !m_UpdatingPlayback && !m_UpdatingFields && !m_UpdatingDestruction) throw std::logic_error("Remove strand world bodies between steps");
		if (m_SimulationInProgress && !m_UpdatingPlayback)
			for (auto* Playback : m_DestructionPlaybacks) if (Body->Parent == Playback)
				throw std::logic_error("Remove playback bodies between world steps");
		if (m_SimulationInProgress && m_DeformableCouplingActive && !m_UpdatingPlayback && !m_UpdatingFields) throw std::logic_error("Remove coupled world bodies between steps");
		if (std::find(m_StaticBodies.begin(), m_StaticBodies.end(), Body) == m_StaticBodies.end()
			&& std::find(m_DynamicBodies.begin(), m_DynamicBodies.end(), Body) == m_DynamicBodies.end()
			&& std::find(m_KinematicBodies.begin(), m_KinematicBodies.end(), Body) == m_KinematicBodies.end()) return false;
		for (auto* Playback : m_DestructionPlaybacks) Playback->OnBodyRemoved(Body);
		for (auto* Strands : m_Strands) Strands->OnBodyRemoved(Body);
		m_DeformableContactBodies.erase(std::remove(m_DeformableContactBodies.begin(), m_DeformableContactBodies.end(), Body), m_DeformableContactBodies.end());
		m_DeformableRigidAttachments.erase(std::remove_if(m_DeformableRigidAttachments.begin(), m_DeformableRigidAttachments.end(),
			[Body](const RigidDeformableAttachment& Link) { if (Link.Rigid != Body) return false; --Link.Soft->m_RigidAttachmentCount; return true; }), m_DeformableRigidAttachments.end());
		for (size_t I = m_PhysicalAnimations.size(); I > 0; --I)
			if (m_PhysicalAnimations[I - 1]->UsesBody(Body)) RemovePhysicalAnimation(m_PhysicalAnimations[I - 1]);
		if (Body->ReducedRagdoll) RemoveRagdoll(Body->ReducedRagdoll);
		for (auto* Instance : m_Ragdolls) Instance->OnBodyRemoved(Body);
		for (size_t I = m_Vehicles.size(); I > 0; --I)
			if (m_Vehicles[I - 1]->GetChassis() == Body) RemoveVehicle(m_Vehicles[I - 1]);
		if (m_IslandSleeping.GetSettings().Enabled) m_IslandSleeping.Forget(Body);
		if (auto* Dynamic = Body->CastDynamic()) Dynamic->IslandSleepManaged = false;
		for (ClothSystem* Cloth : m_Cloths)
		{
			if (Cloth) { Cloth->RemoveColliders(Body); Cloth->DetachRigidBody(Body); }
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
				if (m_IslandSleeping.GetSettings().Enabled) for (auto* Endpoint : { Constraint->GetBodyA(), Constraint->GetBodyB() })
					if (Endpoint != Body && Endpoint->CastDynamic()) Endpoint->CastDynamic()->Wakeup();
				RemoveConstraint(Constraint);
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
		if (Body->CastKinematic())
		{
			if (m_Solver) m_Solver->ResetState();
			if (m_PositionSolver) m_PositionSolver->ResetState();
			m_KinematicBodies.erase(std::remove(m_KinematicBodies.begin(), m_KinematicBodies.end(), Body), m_KinematicBodies.end());
			for (auto* Geom : Body->Geometries()) RemoveGeometryFromSceneQuery(Geom);
			return true;
		}
		return false;
	}

	PositionConstraint* PhysicsWorld::AddConstraint(PositionConstraint* Constraint)
	{
		if (Constraint == nullptr || Constraint->GetBodyA() == nullptr || Constraint->GetBodyB() == nullptr || Constraint->GetBodyA() == Constraint->GetBodyB()
			|| Constraint->GetBodyA()->ReducedRagdoll || Constraint->GetBodyB()->ReducedRagdoll)
		{
			delete Constraint;
			return nullptr;
		}
		if (m_IslandSleeping.GetSettings().Enabled) for (auto* Endpoint : { Constraint->GetBodyA(), Constraint->GetBodyB() })
			if (Endpoint->CastDynamic()) Endpoint->CastDynamic()->Wakeup();
		m_Constraints.push_back(Constraint);
		Constraint->m_Id = m_NextConstraintId++;
		Constraint->Setup();
		if (m_SimulationInProgress && m_StepInfo.SubstepCount > 1) Constraint->BeginFrame(m_StepInfo.FrameDeltaTime);
		Constraint->BeginStep(m_SimulationInProgress ? m_StepInfo.DeltaTime : m_Clock.deltatime,
			m_ResolutionSolver == ResolutionPhaseSolver::XPBD, m_JointDrives, m_PositionSolver != nullptr);
		return Constraint;
	}

	uint64_t PhysicsWorld::AddPhysicsField(const PhysicsFieldCommand& Command) { return m_PhysicsFields->Add(Command); }
	bool PhysicsWorld::RemovePhysicsField(uint64_t Id) { return m_PhysicsFields->Remove(Id); }
	void PhysicsWorld::ClearPhysicsFields() { m_PhysicsFields->Clear(); }
	size_t PhysicsWorld::GetPhysicsFieldCount() const { return m_PhysicsFields->GetCount(); }
	const PhysicsFieldStats& PhysicsWorld::GetPhysicsFieldStats() const { return m_PhysicsFields->GetStats(); }

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
		if (Field == m_AerodynamicsCoupling && m_SimulationInProgress) throw std::logic_error("Edit aerodynamics between world steps");
		const auto It = std::find(m_Fields.begin(), m_Fields.end(), Field);
		if (It == m_Fields.end())
		{
			return false;
		}
		if (Field == m_AerodynamicsCoupling) m_AerodynamicsCoupling = nullptr;
		delete *It;
		m_Fields.erase(It);
		return true;
	}

	bool PhysicsWorld::RemoveConstraint(PositionConstraint* Constraint)
	{
		for (auto* Animation : m_PhysicalAnimations) if (Animation->OwnsConstraint(Constraint))
		{
			RemovePhysicalAnimation(Animation); return true;
		}
		const auto it = std::find(m_Constraints.begin(), m_Constraints.end(), Constraint);
		if (it == m_Constraints.end())
		{
			return false;
		}
		if (m_IslandSleeping.GetSettings().Enabled) for (auto* Endpoint : { Constraint->GetBodyA(), Constraint->GetBodyB() })
			if (Endpoint->CastDynamic()) Endpoint->CastDynamic()->Wakeup();
		for (auto* Instance : m_Ragdolls) Instance->OnConstraintRemoved(Constraint);
		delete *it;
		m_Constraints.erase(it);
		return true;
	}

	FixedConstraint* PhysicsWorld::CreateFixedConstraint(RigidBody* BodyA, RigidBody* BodyB)
	{
		if (!BodyA || !BodyB) return nullptr;
		return static_cast<FixedConstraint*>(AddConstraint(new FixedConstraint(BodyA, BodyB)));
	}
	PoseDriveConstraint* PhysicsWorld::CreatePoseDriveConstraint(RigidBody* Reference, RigidBody* Body)
	{
		if (!Reference || !Body) return nullptr;
		return static_cast<PoseDriveConstraint*>(AddConstraint(new PoseDriveConstraint(Reference, Body)));
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
		if (m_UnifiedConstraints)
		{
			ConstraintGraph Graph;
			Graph.Build(Geometries, Manifolds, m_Constraints);
			for (const ConstraintIsland& Island : Graph.GetIslands())
				if (!Island.Joints.empty()) Bodies->insert(Bodies->end(), Island.Bodies.begin(), Island.Bodies.end());
			return;
		}
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
		// Interleave each joint sweep with fresh contacts. Several joint-only sweeps
		// followed by a large contact translation fight at mesh corners and steps.
		const int positionIterations = m_ConstraintIterations;
		const float penetrationSlop = 0.005f;
		for (int iteration = 0; iteration < positionIterations; ++iteration)
		{
			for (PositionConstraint* constraint : m_Constraints)
			{
				if (constraint && constraint->IsEnabled()) constraint->SolvePosition(dt);
			}
			// A reverse sweep propagates distal corrections back through a chain,
			// rather than leaving the first joint invalidated by its children.
			for (auto It = m_Constraints.rbegin(); It != m_Constraints.rend(); ++It)
			{
				PositionConstraint* constraint = *It;
				if (constraint && constraint->IsEnabled()) constraint->SolvePosition(dt);
			}

			std::vector<GeometryWorldState> stateStorage;
			std::vector<const GeometryWorldState*> states;
			std::vector<Geometry*> geoms;
			GetCurrentGeometryWorldStates(&stateStorage, &states, &geoms);
			if (m_BPhase == nullptr || m_NPhase == nullptr || states.empty())
			{
				continue;
			}

			const auto& contactOverlaps = GetRigidCollisionPairs(GeometryWorldStateSpan(states.data(), states.size()));
			if (contactOverlaps.empty())
			{
				continue;
			}
			std::vector<ContactManifold*> manifolds;
			m_NPhase->CollisionDetection(GeometryWorldStateSpan(states.data(), states.size()), contactOverlaps, &manifolds);
			TrackSleepContacts(geoms, manifolds);
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
				if ((bodyA && bodyA->ReducedRagdoll) || (bodyB && bodyB->ReducedRagdoll)) continue;
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
				const Contact* deepestContact = nullptr;
				float deepestDepth = 0.0f;
				for (int contactIndex = 0; contactIndex < manifold->NumContactPointCount; ++contactIndex)
				{
					const Contact& contact = manifold->ContactPoints[contactIndex];
					// Earlier patches may already have translated these bodies. Reproject
					// their anchors so adjacent triangles do not apply the same correction twice.
					const Vector3 pointA = bodyA ? bodyA->X + contact.PositionLocalA : contact.PositionWorldA;
					const Vector3 pointB = bodyB ? bodyB->X + contact.PositionLocalB : contact.PositionWorldB;
					const float currentDepth = (pointA - pointB).Dot(contact.Normal) + contact.RestOffset;
					if (currentDepth > deepestDepth)
					{
						deepestContact = &contact;
						deepestDepth = currentDepth;
					}
				}
				if (!deepestContact) continue;
				const float depth = std::min(0.2f, deepestDepth - (deepestContact->SolverSlop >= 0.0f ? deepestContact->SolverSlop : penetrationSlop));
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
		if (!m_UnifiedConstraints) SolveConstraintVelocities(dt);
		else if (m_Solver)
		{
			// Position projection changes lever arms and joint axes. Rebuild one
			// coordinated velocity solve at those poses instead of finishing with
			// a joint-only correction that can reintroduce inward contact velocity.
			std::vector<GeometryWorldState> Storage;
			std::vector<const GeometryWorldState*> States;
			std::vector<Geometry*> Geometries;
			GetCurrentGeometryWorldStates(&Storage, &States, &Geometries);
			const GeometryWorldStateSpan Span(States.data(), States.size());
			const auto& Overlaps = GetRigidCollisionPairs(Span);
			std::vector<ContactManifold*> Contacts;
			if (m_NPhase && !Overlaps.empty()) m_NPhase->CollisionDetection(Span, Overlaps, &Contacts);
			TrackSleepContacts(Geometries, Contacts);
			SolveUnifiedVelocities(Geometries, Contacts, dt, true);
		}
	}

	AerodynamicsSimulation* PhysicsWorld::CreateAerodynamics(const AerodynamicsSimulationSettings& Settings)
	{
		return CreateAerodynamics(Settings, AerodynamicsRigidCouplingSettings());
	}
	AerodynamicsSimulation* PhysicsWorld::CreateAerodynamics(const AerodynamicsSimulationSettings& Settings,
		const AerodynamicsRigidCouplingSettings& Coupling, bool CoupleRigids, bool CoupleCloths)
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit aerodynamics between world steps");
		std::unique_ptr<AerodynamicsSimulation> Next(new AerodynamicsSimulation(Settings));
		if (!Next->IsInitialized()) return nullptr;
		std::unique_ptr<AerodynamicsRigidCoupling> Field;
		if (CoupleRigids)
		{
			Field.reset(new AerodynamicsRigidCoupling(Next.get(), Coupling));
			if (!Field->IsValid()) return nullptr;
		}
		RemoveAerodynamics();
		m_Aerodynamics = std::move(Next); m_AerodynamicsClothCoupling = CoupleCloths;
		m_AerodynamicsDomain.reset(GeometryFactory::CreateOBB((Settings.BoundsMin + Settings.BoundsMax) * 0.5f, (Settings.BoundsMax - Settings.BoundsMin) * 0.5f));
		m_AerodynamicsDomain->SetSimulationEnabled(false); m_AerodynamicsDomain->SetQueryEnabled(false);
		if (Field) { AddForceField(Field.get()); m_AerodynamicsCoupling = Field.release(); }
		return m_Aerodynamics.get();
	}
	bool PhysicsWorld::RemoveAerodynamics()
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit aerodynamics between world steps");
		if (!m_Aerodynamics) return false;
		if (m_AerodynamicsCoupling) RemoveForceField(m_AerodynamicsCoupling);
		m_AerodynamicsCoupling = nullptr;
		RemoveGeometryWorldState(m_AerodynamicsDomain.get()); m_AerodynamicsDomain.reset();
		m_Aerodynamics.reset(); return true;
	}
	int PhysicsWorld::AerodynamicsSubsteps(float Dt) const
	{
		if (!m_Aerodynamics || !m_Aerodynamics->SupportsMovingObstacles()) return 1;
		if (!m_Aerodynamics->IsInitialized()) throw std::runtime_error("Aerodynamics backend is unavailable");
		float Step = m_Aerodynamics->GetStableTimeStep();
		const auto H = m_Aerodynamics->GetCellSize(); float Rate = 0;
		for (auto* B : m_DynamicBodies) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, std::fabs(B->V[A]) / H[A]);
		for (auto* C : m_Cloths) if (m_AerodynamicsClothCoupling && C->GetSettings().AerodynamicsEnabled)
			for (int I = 0; I < C->GetParticleCount(); ++I) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, std::fabs(C->GetParticleVelocity(I)[A]) / H[A]);
		Step = std::min(Step, 0.9f / std::max(Rate, 1e-5f));
		const double Count = std::ceil((double)Dt / Step);
		if (!std::isfinite(Count) || Count > 4096) throw std::runtime_error("Aerodynamics world substep budget exceeded");
		return std::max(1, (int)Count);
	}
	void PhysicsWorld::SimulateAerodynamics(float Dt)
	{
		if (!m_Aerodynamics) return;
		const bool MovingBoundaries = m_Aerodynamics->SupportsMovingObstacles();
		const BroadPhaseResult& Pairs = GetCollisionCandidates();
		std::vector<Geometry*> Shapes;
		std::vector<ClothSystem*> Cloths;
		for (const auto& P : Pairs.GetPairs(BroadPhasePairType::Aerodynamics))
		{
			auto* A = Pairs.GetGeometry(P.index1); auto* B = Pairs.GetGeometry(P.index2);
			if (!A || !B) continue;
			auto* G = A == m_AerodynamicsDomain.get() ? B : B == m_AerodynamicsDomain.get() ? A : nullptr;
			if (!G) continue;
			if (m_AerodynamicsCoupling && G->GetParent<RigidBody>() && G->IsSimulationEnabled() && !G->IsTriggerEnabled()) Shapes.push_back(G);
			const int Object = A == m_AerodynamicsDomain.get() ? P.index2 : P.index1;
			if (MovingBoundaries && m_AerodynamicsClothCoupling && Pairs.GetObjectType(Object) == BroadPhaseObjectType::Cloth)
			{
				auto* C = static_cast<ClothSystem*>(Pairs.GetObjectOwner(Object));
				if (C && C->GetSettings().AerodynamicsEnabled) Cloths.push_back(C);
			}
		}
		if (!MovingBoundaries)
		{
			if (m_AerodynamicsCoupling && !m_AerodynamicsCoupling->Synchronize(Shapes))
				throw std::runtime_error("Aerodynamics boundary synchronization failed");
			m_Aerodynamics->Step(Dt); return;
		}
		std::vector<AerodynamicsSurfaceMesh> Surfaces;
		for (auto* C : Cloths)
		{
			AerodynamicsSurfaceMesh S; S.Thickness = C->GetSettings().Thickness;
			for (int I = 0; I < C->GetParticleCount(); ++I) { S.Positions.push_back(C->GetParticle(I).Position); S.Velocities.push_back(C->GetParticleVelocity(I)); S.InverseMasses.push_back(C->GetParticle(I).Pinned ? 0 : C->GetParticle(I).InverseMass); }
			for (const auto& T : C->GetTopology().GetTriangles()) S.Triangles.push_back(T.Particles);
			Surfaces.push_back(std::move(S));
		}
		const bool Ready = m_AerodynamicsCoupling ? m_AerodynamicsCoupling->Synchronize(Shapes, Surfaces) : m_Aerodynamics->SetMovingBoundaries({}, Surfaces);
		if (!Ready || !m_Aerodynamics->StepSynchronized(Dt)) throw std::runtime_error("Aerodynamics failed to consume the world substep");
		std::vector<std::vector<Vector3>> Forces;
		if (!m_Aerodynamics->ComputeSurfaceLoads(Forces) || Forces.size() != Cloths.size()) throw std::runtime_error("Aerodynamics surface loads are unavailable");
		for (size_t I = 0; I < Cloths.size(); ++I) for (size_t J = 0; J < Forces[I].size(); ++J)
			Cloths[I]->ApplyParticleImpulse((int)J, Forces[I][J] * Dt);
	}

	DeformableBody* PhysicsWorld::CreateDeformableBody(const DeformableBodySettings& Settings)
	{
		if (m_SimulationInProgress && m_DeformableCouplingActive) throw std::logic_error("Create coupled deformables between steps");
		if (Settings.Collision.RigidCollision && (!m_PositionSolver || m_SupportStabilization.Enabled)) throw std::invalid_argument("Rigid deformable collision requires PBD/XPBD without support projection");
		DeformableBody* body = new DeformableBody(Settings);
		m_DeformableBodies.push_back(body);
		body->m_CollisionQuery = [this]() -> const BroadPhaseResult& { return GetCollisionCandidates(); };
		return body;
	}
	DeformableBody* PhysicsWorld::CreateDeformableBody(const TetrahedralAsset& Asset, const DeformableBodySettings& Settings, const Transform& Pose)
	{
		if (m_SimulationInProgress && m_DeformableCouplingActive) throw std::logic_error("Create coupled deformables between steps");
		if (Settings.Collision.RigidCollision && (!m_PositionSolver || m_SupportStabilization.Enabled)) throw std::invalid_argument("Rigid deformable collision requires PBD/XPBD without support projection");
		std::unique_ptr<DeformableBody> Body(new DeformableBody(Settings)); Body->InitializeAsset(Asset, Pose);
		m_DeformableBodies.push_back(Body.get());
		Body->m_CollisionQuery = [this]() -> const BroadPhaseResult& { return GetCollisionCandidates(); }; return Body.release();
	}

	bool PhysicsWorld::RemoveDeformableBody(DeformableBody* Body)
	{
		if (m_SimulationInProgress && m_DeformableCouplingActive) throw std::logic_error("Remove coupled deformables between steps");
		const auto it = std::find(m_DeformableBodies.begin(), m_DeformableBodies.end(), Body);
		if (it == m_DeformableBodies.end()) return false;
		m_DeformableRigidAttachments.erase(std::remove_if(m_DeformableRigidAttachments.begin(), m_DeformableRigidAttachments.end(),
			[Body](const RigidDeformableAttachment& Link) { if (Link.Soft != Body) return false; --Link.Soft->m_RigidAttachmentCount; return true; }), m_DeformableRigidAttachments.end());
		RemoveGeometryWorldState(Body->GetRenderGeometry());
		delete *it;
		m_DeformableBodies.erase(it);
		return true;
	}

	StrandSystem* PhysicsWorld::CreateStrands(const std::vector<StrandGuide>& Guides, const StrandSettings& Settings)
	{
		if (m_SimulationInProgress) throw std::logic_error("Create strands between world steps");
		std::unique_ptr<StrandSystem> Strands(new StrandSystem(Guides,Settings)); m_Strands.push_back(Strands.get()); Strands->SetWorld(this); return Strands.release();
	}
	bool PhysicsWorld::RemoveStrands(StrandSystem* Strands)
	{
		if (m_SimulationInProgress) throw std::logic_error("Remove strands between world steps");
		const auto It=std::find(m_Strands.begin(),m_Strands.end(),Strands); if (It==m_Strands.end()) return false;
		m_Strands.erase(It); delete Strands; return true;
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

	/* The first world adapter supports CPU sequential impulses and discrete motion.
	   Maximal-coordinate joints, CCD trajectory continuation and support projection
	   need generalized responses before they may operate on these link proxies.
	   Reject unsupported configurations instead of silently integrating links twice. */
	Ragdoll* PhysicsWorld::CreateRagdoll(const RagdollSettings& Settings, const std::vector<RigidBodyDynamic*>& Bodies)
	{
		if (Settings.Algorithm != RagdollAlgorithm::ReducedCoordinates)
			throw std::invalid_argument("External ragdoll bodies require the reduced-coordinate algorithm");
		if (m_ResolutionSolver != ResolutionPhaseSolver::SequentialImpulse || GetSolverBackend() != SolverBackend::CPU)
			throw std::invalid_argument("ReducedRagdolls currently require the CPU sequential impulse solver");
		if (m_SupportStabilization.Enabled) throw std::invalid_argument("ReducedRagdolls do not support independent link position projection");
		for (auto* Body : m_DynamicBodies) if (Body->mMotionType == MorionType::Continuous)
			throw std::invalid_argument("Ragdoll worlds currently require discrete motion");
		for (auto* Body : Bodies)
		{
			if (!Body || std::find(m_DynamicBodies.begin(), m_DynamicBodies.end(), Body) == m_DynamicBodies.end())
				throw std::invalid_argument("Ragdoll links must belong to this world");
			for (auto* Joint : m_Constraints) if (Joint->GetBodyA() == Body || Joint->GetBodyB() == Body)
				throw std::invalid_argument("Ragdoll links cannot be endpoints of maximal-coordinate joints");
			for (auto* Vehicle : m_Vehicles) if (Vehicle->GetChassis() == Body)
				throw std::invalid_argument("Vehicle chassis cannot be reduced ragdoll links");
			for (auto* Rope : m_Ropes) if (Rope->HasRigidBodyAttachment(Body))
				throw std::invalid_argument("Rope attachments currently require independent rigid bodies");
		}
		std::unique_ptr<Ragdoll> Owner(new Ragdoll(*this, Settings, Bodies));
		m_Ragdolls.push_back(Owner.get());
		try { m_ReducedRagdolls.push_back(Owner.get()); }
		catch (...) { m_Ragdolls.pop_back(); throw; }
		m_NPhase->ResetState(); m_Solver->ResetState();
		for (auto* Body : Bodies) UpdateSceneQuery(Body);
		return Owner.release();
	}
	VehicleDynamics* PhysicsWorld::CreateVehicle(RigidBodyDynamic* Chassis, const VehicleDynamicsSettings& Settings)
	{
		if (!Chassis || Chassis->ReducedRagdoll || std::find(m_DynamicBodies.begin(), m_DynamicBodies.end(), Chassis) == m_DynamicBodies.end())
			throw std::invalid_argument("Vehicle chassis must belong to this world");
		for (auto* Vehicle : m_Vehicles) if (Vehicle->GetChassis() == Chassis) throw std::invalid_argument("Chassis already has a vehicle");
		std::unique_ptr<VehicleDynamics> Vehicle(new VehicleDynamics(*this, Chassis, Settings));
		m_Vehicles.push_back(Vehicle.get()); return Vehicle.release();
	}
	bool PhysicsWorld::RemoveVehicle(VehicleDynamics* Vehicle)
	{
		const auto Found = std::find(m_Vehicles.begin(), m_Vehicles.end(), Vehicle);
		if (Found == m_Vehicles.end()) return false;
		m_Vehicles.erase(Found); delete Vehicle; return true;
	}

	ArticulatedRobotController* PhysicsWorld::CreateRobotController(const ArticulatedRobotSettings& Settings)
	{
		std::unique_ptr<ArticulatedRobotController> Robot(new ArticulatedRobotController(*this, Settings));
		m_RobotControllers.push_back(Robot.get());
		return Robot.release();
	}

	SweepRobotController* PhysicsWorld::CreateRobotController(const RobotControllerSettings& Settings, const Vector3& Position)
	{
		std::unique_ptr<SweepRobotController> Character(new SweepRobotController(*this, Settings, Position));
		AddGeometryToSceneQuery(Character->GetGeometry());
		m_RobotControllers.push_back(Character.get());
		return Character.release();
	}

	bool PhysicsWorld::RemoveRobotController(RobotController* Character)
	{
		const auto Found = std::find(m_RobotControllers.begin(), m_RobotControllers.end(), Character);
		if (Found == m_RobotControllers.end())
			return false;
		if (Character->GetGeometry())
			RemoveGeometryFromSceneQuery(Character->GetGeometry());
		m_RobotControllers.erase(Found); delete Character; return true;
	}

	void PhysicsWorld::SynchronizeSceneQueries()
	{
		for (const auto* State : m_GeometryWorldStates)
		{
			if (!State) continue;
			UpdateSceneQuery(UpdateGeometryWorldState(State->Handle, m_Clock.tick));
		}
	}

	ClothSystem* PhysicsWorld::CreateCloth(const ClothSettings& Settings, const Vector3& Center, const Vector3& HorizontalAxis, const Vector3& VerticalAxis)
	{
		if (m_SimulationInProgress) throw std::logic_error("Create cloth between world steps");
		ClothSystem* Cloth = new ClothSystem(Settings, Center, HorizontalAxis, VerticalAxis);
		m_Cloths.push_back(Cloth);
		Cloth->m_CollisionQuery = [this]() -> const BroadPhaseResult& { return GetCollisionCandidates(); };
		return Cloth;
	}

	bool PhysicsWorld::RemoveCloth(ClothSystem* Cloth)
	{
		if (m_SimulationInProgress) throw std::logic_error("Remove cloth between world steps");
		const auto it = std::find(m_Cloths.begin(), m_Cloths.end(), Cloth);
		if (it == m_Cloths.end())
		{
			return false;
		}
		RemoveGeometryWorldState(Cloth->GetRenderGeometry());
		delete *it;
		m_Cloths.erase(it);
		return true;
	}

	ClothSystem* PhysicsWorld::CreateCloth(const ClothSettings& Settings, const ClothMesh& Mesh)
	{
		if (m_SimulationInProgress) throw std::logic_error("Create cloth between world steps");
		std::unique_ptr<ClothSystem> Cloth(new ClothSystem(Settings, Mesh)); m_Cloths.push_back(Cloth.get());
		Cloth->m_CollisionQuery = [this]() -> const BroadPhaseResult& { return GetCollisionCandidates(); }; return Cloth.release();
	}

	void PhysicsWorld::SimulateCloths(float dt)
	{
		std::vector<ClothSystem*> Independent;
		const bool Coupled = HasClothRigidCoupling();
		for (auto* C : m_Cloths) if (!C->HasRigidCoupling() && !(Coupled && C->GetSettings().Collision.InterCollision)) Independent.push_back(C);
		ClothSystem::SimulateGroup(Independent, dt);
	}

	Ragdoll* PhysicsWorld::CreateRagdoll(const RagdollSettings& Settings, const Transform& RootTransform)
	{
		if (Settings.Algorithm == RagdollAlgorithm::ReducedCoordinates)
		{
			if (m_ResolutionSolver != ResolutionPhaseSolver::SequentialImpulse || GetSolverBackend() != SolverBackend::CPU
				|| m_SupportStabilization.Enabled || Settings.EnableCCD)
				throw std::invalid_argument("Reduced-coordinate ragdolls require discrete CPU sequential impulses without support projection");
			for (auto* Body : m_DynamicBodies) if (Body->mMotionType == MorionType::Continuous)
				throw std::invalid_argument("Reduced-coordinate ragdoll worlds require discrete motion");
		}
		Ragdoll* RagdollInstance = new Ragdoll(this, Settings, RootTransform);
		if (!RagdollInstance->IsValid())
		{
			RagdollInstance->Destroy();
			delete RagdollInstance;
			return nullptr;
		}
		m_Ragdolls.push_back(RagdollInstance);
		if (RagdollInstance->IsReduced())
		{
			try { m_ReducedRagdolls.push_back(RagdollInstance); }
			catch (...) { m_Ragdolls.pop_back(); RagdollInstance->Destroy(); delete RagdollInstance; throw; }
			m_NPhase->ResetState(); if (m_Solver) m_Solver->ResetState();
			for (auto* Body : RagdollInstance->GetBodies()) UpdateSceneQuery(Body);
		}
		return RagdollInstance;
	}
	PhysicalAnimation* PhysicsWorld::CreatePhysicalAnimation(const std::vector<PhysicalAnimationBone>& Bones)
	{
		if (Bones.empty()) throw std::invalid_argument("Physical animation requires bone bindings");
		std::unordered_set<RigidBody*> Unique;
		for (size_t I = 0; I < Bones.size(); ++I)
		{
			const auto& Bone = Bones[I];
			if (!Bone.Body || Bone.Body->ReducedRagdoll || Bone.Body->Parent || Bone.Parent < -1 || Bone.Parent >= (int)I
				|| std::find(m_DynamicBodies.begin(), m_DynamicBodies.end(), Bone.Body) == m_DynamicBodies.end() || !Unique.insert(Bone.Body).second)
				throw std::invalid_argument("Physical animation requires distinct world-owned bodies in parent-first order");
			for (auto* Instance : m_PhysicalAnimations) if (Instance->UsesBody(Bone.Body))
				throw std::invalid_argument("A body is already bound to physical animation");
		}
		std::unique_ptr<PhysicalAnimation> Instance(new PhysicalAnimation(*this, Bones));
		m_PhysicalAnimations.push_back(Instance.get()); return Instance.release();
	}
	PhysicalAnimation* PhysicsWorld::CreatePhysicalAnimation(Ragdoll* Instance)
	{
		if (std::find(m_Ragdolls.begin(), m_Ragdolls.end(), Instance) == m_Ragdolls.end() || !Instance->IsValid())
			throw std::invalid_argument("Physical animation requires a valid ragdoll in this world");
		std::vector<PhysicalAnimationBone> Bones;
		for (auto* Body : Instance->GetBodies()) Bones.push_back({ Body, -1 });
		for (auto* Joint : Instance->GetConstraints())
		{
			const auto& Bodies = Instance->GetBodies();
			const int Parent = (int)(std::find(Bodies.begin(), Bodies.end(), Joint->GetBodyA()) - Bodies.begin());
			const size_t Child = std::find(Bodies.begin(), Bodies.end(), Joint->GetBodyB()) - Bodies.begin();
			if (Child >= Bones.size() || Bones[Child].Parent >= 0) throw std::invalid_argument("Ragdoll joints do not form a bone tree");
			Bones[Child].Parent = Parent;
		}
		return CreatePhysicalAnimation(Bones);
	}
	bool PhysicsWorld::RemovePhysicalAnimation(PhysicalAnimation* Instance)
	{
		const auto Found = std::find(m_PhysicalAnimations.begin(), m_PhysicalAnimations.end(), Instance);
		if (Found == m_PhysicalAnimations.end()) return false;
		m_PhysicalAnimations.erase(Found);
		m_IslandSleeping.Forget(Instance->m_Reference.get());
		delete Instance;
		if (m_Solver) m_Solver->ResetState();
		if (m_PositionSolver) m_PositionSolver->ResetState();
		return true;
	}

	bool PhysicsWorld::RemoveRagdoll(Ragdoll* RagdollInstance)
	{
		const auto it = std::find(m_Ragdolls.begin(), m_Ragdolls.end(), RagdollInstance);
		if (it == m_Ragdolls.end())
		{
			return false;
		}
		if (RagdollInstance->IsReduced())
		{
			const bool OwnsBodies = RagdollInstance->m_OwnsBodies;
			const auto Bodies = OwnsBodies ? RagdollInstance->m_Bodies : std::vector<RigidBodyDynamic*>();
			const auto Reduced = std::find(m_ReducedRagdolls.begin(), m_ReducedRagdolls.end(), RagdollInstance);
			if (Reduced != m_ReducedRagdolls.end()) m_ReducedRagdolls.erase(Reduced);
			m_Ragdolls.erase(it);
			delete RagdollInstance;
			for (auto* Body : Bodies)
			{
				if (RemoveRigidBody(Body)) { Body->ReleaseGeometries(); delete Body; }
			}
			m_NPhase->ResetState();
			if (m_Solver) m_Solver->ResetState();
			return true;
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
		for (auto* Body : m_KinematicBodies) Body->GetGeometries(geoms);
		for (RobotController* Robot : m_RobotControllers)
			if (Robot->GetGeometry())
				geoms->push_back(Robot->GetGeometry());
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

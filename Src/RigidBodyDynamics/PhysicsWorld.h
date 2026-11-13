#pragma once

#include <stdint.h>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "RigidBody.h"
#include "Constraints.h"
#include "SupportStabilization.h"
#include "Island.h"
#include "ConstraintSolver.h"
#include "../Solver/LCPSolver.h"
#include "../Solver/PBDSolver.h"
#include "../Solver/XPBDSolver.h"
#include "CollidingContact.h"
#include "MeshContactGenerator.h"
#include "ContactGeneration.h"
#include "ContinuousCollisionDetection.h"

namespace Riemann
{
	class Geometry;
	class GeometryQuery;
	class CollisionFilter;
	struct DeformableRigidCollider;
	class RigidBody;
	class RigidBodyStatic;
	class RigidBodyDynamic;
	class BroadPhase;
	class NarrowPhase;
	class ContactManifold;
	class ConstraintSolver;
	class DestructionSet;
	class DestructionPlayback;
	class ForceField;
	class PhysicsFieldSystem;
	struct PhysicsFieldCommand;
	struct PhysicsFieldStats;
	class KinematicsDriver;
	class JobSystem;
	class IBinaryData;
	class RopeSystem;
	struct RopeSettings;
	class ClothSystem;
	struct ClothSettings;
	struct ClothMesh;
	class DeformableBody;
	class StrandSystem;
	struct StrandGuide;
	struct StrandSettings;
	class TetrahedralAsset;
	struct BarycentricPoint;
	struct RigidDeformableAttachment;
	struct DeformableBodySettings;
	class Ragdoll;
	class CharacterController;
	class VehicleDynamics;
	class PhysicalAnimation;
	struct PhysicalAnimationBone;
	class PoseDriveConstraint;
	struct RagdollSettings;
	struct VehicleDynamicsSettings;
	struct CharacterControllerSettings;

	enum class ThreadMode : uint8_t
	{
		SingleThread,
		PhysicsThread,
		JobSystem,
	};

	enum class BroadPhaseSolver : uint8_t
	{
		SAP,
		ABP,
		MBP,
		AllPairs,
		Bruteforce,
		DynamicAABB,
	};

	enum class SceneQueryType : uint8_t
	{
		StaticAABB,
		DynamicAABB,
	};

	enum class NarrowPhaseSolver : uint8_t
	{
		GJKEPA,
		PCM,
	};

	enum class ResolutionPhaseSolver : uint8_t
	{
		SequentialImpulse,
		LCPGlobal,
		XPBD,
		PBD,
	};

	enum class IntegrateMethod : uint8_t
	{
		ExplicitEuler = 0x00,
		SymplecticEuler = 0x01,
	};

	struct WorldClock
	{
		WorldClock()
		{
			tick = 0;
			simhz = 60;
			deltatime = 1.0f / simhz;
		}

		uint64_t	tick;
		int			simhz;		// Update per second, default 60
		float		deltatime;
	};

	struct TemporalSubstepSettings
	{
		static constexpr int MaximumSubsteps = 64;
		bool Enabled = false;
		int Substeps = 4;
		bool IsValid() const { return Substeps > 0 && Substeps <= MaximumSubsteps; }
	};

	struct PhysicsStepInfo
	{
		float DeltaTime = 0, FrameDeltaTime = 0;
		int SubstepIndex = 0, SubstepCount = 1;
	};

	struct PhysicsContactPoint
	{
		Vector3	Position;
		Vector3	PositionWorldA;
		Vector3	PositionWorldB;
		Vector3	PositionLocalA;
		Vector3	PositionLocalB;
		Vector3	Normal;
		Vector3	Tangent;
		Vector3	Binormal;
		Vector3	Impulse;
		float	PenetrationDepth; // Physical penetration plus RestOffset; negative values describe an open solver gap.
		float RestOffset = 0.0f;
		float	NormalImpulse;
		float	TangentImpulse;
		float	BinormalImpulse;
	};

	struct PhysicsContactInfo
	{
		RigidBody*					BodyA;
		RigidBody*					BodyB;
		Geometry*					GeomA;
		Geometry*					GeomB;
		const PhysicsContactPoint*	ContactPoints;
		int							NumContactPoints;
		PhysicsStepInfo Step;
	};

	struct PhysicsTriggerInfo
	{
		RigidBody*	BodyA;
		RigidBody*	BodyB;
		Geometry*	GeomA;
		Geometry*	GeomB;
		Vector3		Position;
		Vector3		Normal;
		float		Fraction;
		bool		IsCCD;
		PhysicsStepInfo Step;
	};

	using OnContactCallback = std::function<void(const PhysicsContactInfo&)>;
	using OnTriggerCallback = std::function<void(const PhysicsTriggerInfo&)>;
	struct PhysicsJointBreakInfo
	{
		PositionConstraint* Constraint;
		RigidBody* BodyA;
		RigidBody* BodyB;
		uint64_t ConstraintId;
		uint64_t Tick;
		float DeltaTime;
		JointReaction Reaction;
	};
	using OnJointBreakCallback = std::function<void(const PhysicsJointBreakInfo&)>;

	void DefaultOnContact(const PhysicsContactInfo& ContactInfo);
	void DefaultOnTrigger(const PhysicsTriggerInfo& TriggerInfo);

	struct PhysicsWorldParam
	{
		PhysicsWorldParam()
		{
			gravityAcc = Vector3::Zero();
			threadMode = ThreadMode::SingleThread;
			broadphase = BroadPhaseSolver::SAP;
			sceneQueryType = SceneQueryType::DynamicAABB;
			narrowphase = NarrowPhaseSolver::GJKEPA;
			resolutionSolver = ResolutionPhaseSolver::SequentialImpulse;
			integrateMethod = IntegrateMethod::ExplicitEuler;
			workerThreads = 0;
			constraintIterations = 12;
			contactSolverIterations = 32;
			contactSolverTolerance = 1.0e-5f;
			contactSolverRelaxation = 1.0f;
			solverBackend = SolverBackend::CPU;
			allowBackendFallback = true;
			onContact = DefaultOnContact;
			onTrigger = DefaultOnTrigger;
		}
		Vector3 				gravityAcc;		// gravity acc
		ThreadMode				threadMode;
		BroadPhaseSolver		broadphase;
		SceneQueryType			sceneQueryType;
		NarrowPhaseSolver		narrowphase;
		ResolutionPhaseSolver	resolutionSolver;
		IntegrateMethod 		integrateMethod;
		int						workerThreads;
		int						constraintIterations;
		int							contactSolverIterations;
		float						contactSolverTolerance;
		float						contactSolverRelaxation;
		SolverBackend				solverBackend;
		bool					allowBackendFallback;
		PBDSolverSettings		pbdSettings;
		XPBDSolverSettings		xpbdSettings;
		OnContactCallback		onContact;
		OnJointBreakCallback onJointBreak;
		OnTriggerCallback		onTrigger;
		ContactPersistenceSettings contactPersistence;
		FrictionPatchSettings frictionPatches;
		MeshContactSettings meshContactSettings;
		ContactGenerationSettings contactGeneration;
		std::shared_ptr<CollisionFilter> collisionFilter;
		bool collisionFilteringEnabled = true;
		bool sequentialImpulseWarmStart = true;
		bool unifiedConstraints = true;
		bool jointDrives = true;
		bool jointBreaking = true;
		CCDContinuationSettings ccdContinuation;
		TemporalSubstepSettings temporalSubsteps;
		SupportStabilizationSettings supportStabilization;
		IslandSleepingSettings islandSleeping;
		SequentialImpulseSettings sequentialImpulseSettings;
	};

	struct GeometryWorldRecord
	{
		GeometryWorldRecord()
			: Generation(1)
			, Active(false)
			, HasState(false)
		{
		}

		GeometryWorldState	State;
		uint32_t			Generation;
		bool				Active;
		bool				HasState;
		uint64_t QueryVersion = 0;
		uint64_t FilterRevision = 0;
		uint64_t ContactOffsetRevision = 0;
	};

	class PhysicsWorld
	{
	public:
		PhysicsWorld(const PhysicsWorldParam& param);
		~PhysicsWorld();

	public:
		void				Simulate();
		// A one-step override; the no-argument step retains the configured clock interval.
		void				Simulate(float Dt);
		bool IsSimulating() const { return m_SimulationInProgress; }
		void				Reset();

		bool				LoadScene(const char* name, bool shared_mem);

		RigidBody*			CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose);
		RigidBody*			CreateRigidBody(Geometry* Geom, const RigidBodyParam& param);
		RigidBody*			CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform, bool RegisterDisabledGeometries = true);
		bool				RemoveRigidBody(RigidBody* Body);
		size_t GetKinematicBodyCount() const { return m_KinematicBodies.size(); }
		// The world owns registered fields and deletes them on removal, reset, or destruction.
		ForceField*		AddForceField(ForceField* Field);
		bool				RemoveForceField(ForceField* Field);
		size_t				GetForceFieldCount() const { return m_Fields.size(); }
		uint64_t AddPhysicsField(const PhysicsFieldCommand& Command);
		bool RemovePhysicsField(uint64_t Id);
		void ClearPhysicsFields();
		size_t GetPhysicsFieldCount() const;
		const PhysicsFieldStats& GetPhysicsFieldStats() const;
		PositionConstraint* AddConstraint(PositionConstraint* Constraint);
		bool				RemoveConstraint(PositionConstraint* Constraint);
		FixedConstraint*	CreateFixedConstraint(RigidBody* BodyA, RigidBody* BodyB);
		PointConstraint*	CreatePointConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor);
		DistanceConstraint* CreateDistanceConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchorA, const Vector3& WorldAnchorB, float MinDistance, float MaxDistance);
		HingeConstraint*	CreateHingeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinAngle, float MaxAngle);
		SliderConstraint*	CreateSliderConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinDistance, float MaxDistance);
		ConeConstraint*		CreateConeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxisA, const Vector3& WorldAxisB, float MaxAngle);
		SixDOFConstraint*	CreateSixDOFConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const SixDOFConstraintSettings& Settings);
		PoseDriveConstraint* CreatePoseDriveConstraint(RigidBody* Reference, RigidBody* Body);
		size_t				GetConstraintCount() const { return m_Constraints.size(); }
		RopeSystem*			CreateRope(const RopeSettings& Settings, const Vector3& Start, const Vector3& End);
		bool				RemoveRope(RopeSystem* Rope);
		size_t				GetRopeCount() const { return m_Ropes.size(); }
		StrandSystem* CreateStrands(const std::vector<StrandGuide>& Guides, const StrandSettings& Settings);
		bool RemoveStrands(StrandSystem* Strands);
		size_t GetStrandSystemCount() const { return m_Strands.size(); }
		StrandSystem* GetStrandSystem(size_t Index) const { return Index<m_Strands.size() ? m_Strands[Index] : nullptr; }
		Ragdoll* CreateRagdoll(const RagdollSettings& Settings, const std::vector<RigidBodyDynamic*>& Bodies);
		size_t GetReducedRagdollCount() const { return m_ReducedRagdolls.size(); }
		Ragdoll* GetReducedRagdoll(size_t Index) const { return m_ReducedRagdolls.at(Index); }
		PhysicalAnimation* CreatePhysicalAnimation(const std::vector<PhysicalAnimationBone>& Bones);
		PhysicalAnimation* CreatePhysicalAnimation(Ragdoll* Instance);
		bool RemovePhysicalAnimation(PhysicalAnimation* Instance);
		size_t GetPhysicalAnimationCount() const { return m_PhysicalAnimations.size(); }
		PhysicalAnimation* GetPhysicalAnimation(size_t Index) const { return m_PhysicalAnimations.at(Index); }
		VehicleDynamics* CreateVehicle(RigidBodyDynamic* Chassis, const VehicleDynamicsSettings& Settings);
		bool RemoveVehicle(VehicleDynamics* Vehicle);
		size_t GetVehicleCount() const { return m_Vehicles.size(); }
		CharacterController* CreateCharacterController(const CharacterControllerSettings& Settings, const Vector3& Position);
		bool RemoveCharacterController(CharacterController* Controller);
		size_t GetCharacterControllerCount() const { return m_Characters.size(); }
		ClothSystem*		CreateCloth(const ClothSettings& Settings, const Vector3& Center, const Vector3& HorizontalAxis, const Vector3& VerticalAxis);
		ClothSystem* CreateCloth(const ClothSettings& Settings, const ClothMesh& Mesh);
		bool				RemoveCloth(ClothSystem* Cloth);
		size_t				GetClothCount() const { return m_Cloths.size(); }
		ClothSystem* GetCloth(size_t Index) const { return Index < m_Cloths.size() ? m_Cloths[Index] : nullptr; }
		DeformableBody*		CreateDeformableBody(const DeformableBodySettings& Settings);
		DeformableBody* CreateDeformableBody(const TetrahedralAsset& Asset, const DeformableBodySettings& Settings, const Transform& Pose = Transform::Identity());
		bool				RemoveDeformableBody(DeformableBody* Body);
		size_t				GetDeformableBodyCount() const { return m_DeformableBodies.size(); }
		const std::vector<DeformableBody*>& GetDeformableBodies() const { return m_DeformableBodies; }
		uint64_t AttachDeformableToRigid(DeformableBody* Soft, const BarycentricPoint& Point, RigidBody* Rigid,
			const Vector3& WorldAnchor, double Compliance = 0);
		bool RemoveDeformableRigidAttachment(uint64_t Id);
		size_t GetDeformableRigidAttachmentCount() const;
		Maths::TVector3<double> GetDeformableRigidAttachmentForce(uint64_t Id) const;
		Maths::TVector3<double> GetDeformableRigidAttachmentTorque(uint64_t Id) const;
		Ragdoll*			CreateRagdoll(const RagdollSettings& Settings, const Transform& RootTransform);
		bool				RemoveRagdoll(Ragdoll* RagdollInstance);
		size_t				GetRagdollCount() const { return m_Ragdolls.size(); }
		Ragdoll* GetRagdoll(size_t Index) const { return Index < m_Ragdolls.size() ? m_Ragdolls[Index] : nullptr; }
		bool				SetHeightFieldHeight(Geometry* HeightFieldGeometry, int X, int Z, float Height);
		int					SetHeightFieldHeights(Geometry* HeightFieldGeometry, int X, int Z, int Width, int Height, const std::vector<float>& Heights);
		// Brush centers are world positions; height values and deltas use terrain-local Y.
		int					ModifyHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float HeightDelta, bool LinearFalloff = true);
		int					FillHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float TargetHeight, float Strength = 1.0f, bool LinearFalloff = true);
		int					SmoothHeightField(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float Strength = 1.0f, int Iterations = 1, bool LinearFalloff = true);
		int					DigHeightFieldHole(Geometry* HeightFieldGeometry, const Vector3& WorldCenter, float Radius, float Depth, bool LinearFalloff = true);
		bool				SetHeightFieldTriangleHole(Geometry* HeightFieldGeometry, int CellX, int CellZ, int TriangleIndex, bool Hole);
		void				AddDestructionSet(DestructionSet* DestructSet);
		void				RemoveDestructionSet(DestructionSet* DestructSet);

		bool				LoadAnimation(const std::string& resname, const std::string& filepath, float play_rate, bool begin_play);
		KinematicsDriver*	FindKinematics(const std::string& resname);

		void				GetAllGeometries(std::vector<Geometry*>* AllObjects);
		GeometryQuery*		GetGeometryQuery() { return m_GeometryQuery; }
		const GeometryQuery* GetGeometryQuery() const { return m_GeometryQuery; }

		float				GetSystemTotalEnergy() const;
		float				GetSystemTotalLinearKinematicsEnergy() const;
		float				GetSystemTotalAngularKinematicsEnergy() const;
		Vector3				GetSystemTotalLinearMomentum() const;
		Vector3				GetSystemTotalAngularMomentum() const;
		int					GetLastCCDHitCount() const { return m_LastCCDHitCount; }
		ResolutionPhaseSolver GetResolutionSolver() const { return m_ResolutionSolver; }
		SolverBackend GetSolverBackend() const;
		SolverBackend GetRequestedSolverBackend() const { return m_RequestedSolverBackend; }
		bool GetAllowBackendFallback() const { return m_AllowBackendFallback; }
		static bool IsSolverBackendAvailable(ResolutionPhaseSolver Solver, SolverBackend Backend);
		// Call between simulation steps. Failure preserves the previous solver and world state.
		bool SetSolverConfiguration(ResolutionPhaseSolver Solver, SolverBackend Backend,
			bool AllowFallback, std::string* ErrorMessage = nullptr);
		const PBDSolver* GetPositionSolver() const { return m_PositionSolver; }
		bool SetNarrowPhaseSolver(NarrowPhaseSolver Solver);
		void SetTriangleContactsEnabled(bool Enabled);
		bool GetTriangleContactsEnabled() const { return m_MeshContactSettings.Enabled; }
		void SetCollisionFilter(std::shared_ptr<CollisionFilter> Filter);
		CollisionFilter* GetCollisionFilter() const { return m_CollisionFilteringEnabled ? m_CollisionFilter.get() : nullptr; }
		void SetCollisionFilteringEnabled(bool Enabled);
		bool GetCollisionFilteringEnabled() const { return m_CollisionFilteringEnabled; }
		void RefreshCollisionFiltering();
		void SetContactGenerationSettings(const ContactGenerationSettings& Settings);
		const ContactGenerationSettings& GetContactGenerationSettings() const { return m_ContactGeneration; }
		NarrowPhaseSolver GetNarrowPhaseSolver() const { return m_NarrowPhaseSolver; }
		void SetUnifiedConstraintSolving(bool Enabled);
		bool GetUnifiedConstraintSolving() const { return m_UnifiedConstraints; }
		void SetJointDrivesEnabled(bool Enabled);
		bool GetJointDrivesEnabled() const { return m_JointDrives; }
		const SequentialImpulseSettings& GetSequentialImpulseSettings() const { return m_SequentialImpulseSettings; }
		void SetTemporalSubstepSettings(const TemporalSubstepSettings& Settings);
		const TemporalSubstepSettings& GetTemporalSubstepSettings() const { return m_TemporalSubsteps; }
		const PhysicsStepInfo& GetStepInfo() const { return m_StepInfo; }
		uint64_t GetTick() const { return m_Clock.tick; }
		void SetIslandSleepingSettings(const IslandSleepingSettings& Settings);
		const IslandSleepingSettings& GetIslandSleepingSettings() const { return m_IslandSleeping.GetSettings(); }
		const IslandSleepingStats& GetLastIslandSleepingStats() const { return m_IslandSleeping.GetStats(); }
		float GetIslandQuietTime(const RigidBodyDynamic* Body) const { return m_IslandSleeping.GetQuietTime(Body); }
		void SetSupportStabilizationSettings(const SupportStabilizationSettings& Settings);
		const SupportStabilizationSettings& GetSupportStabilizationSettings() const { return m_SupportStabilization; }
		const SupportStabilizationStats& GetLastSupportStabilizationStats() const { return m_SupportStabilizationStats; }
		void SetCCDContinuationSettings(const CCDContinuationSettings& Settings);
		const CCDContinuationSettings& GetCCDContinuationSettings() const { return m_CCDContinuation; }
		const CCDContinuationStats& GetLastCCDContinuationStats() const { return m_CCDContinuationStats; }
		void SetJointBreakingEnabled(bool Enabled);
		bool GetJointBreakingEnabled() const { return m_JointBreaking; }
		void SetFrictionPatchSettings(const FrictionPatchSettings& Settings);
		const FrictionPatchSettings& GetFrictionPatchSettings() const { return m_FrictionPatches; }
		void SetSequentialImpulseWarmStarting(bool Enabled);
		bool GetSequentialImpulseWarmStarting() const { return m_SequentialImpulseWarmStart; }
		int GetWarmStartContactCount() const { return m_Solver ? m_Solver->GetWarmStartContactCount() : 0; }
		size_t GetCachedContactPairCount() const;

	private:
		friend class CharacterController;
		friend class VehicleDynamics;
		friend class Ragdoll;
		std::vector<VehicleDynamics*> m_Vehicles;
		std::vector<Ragdoll*> m_ReducedRagdolls;
		std::vector<PhysicalAnimation*> m_PhysicalAnimations;
		void SynchronizeSceneQueries();
		std::vector<CharacterController*> m_Characters;
		void				SimulateST(float dt);
		void SolveUnifiedVelocities(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts, float Dt, bool JointIslandsOnly = false);
		void				SimulatePositionBasedRigidBodies(float dt, const std::vector<DeformableRigidCollider>& Colliders);

		GeometryHandle		EnsureGeometryHandle(Geometry* Object);
		bool				IsValidGeometryHandle(GeometryHandle Handle) const;
		GeometryWorldRecord* GetGeometryWorldRecord(GeometryHandle Handle);
		const GeometryWorldRecord* GetGeometryWorldRecord(GeometryHandle Handle) const;
		GeometryWorldState	UpdateGeometryWorldState(GeometryHandle Handle, uint64_t FrameId);
		GeometryWorldState	UpdateGeometryWorldState(Geometry* Object, uint64_t FrameId);
		void				AddGeometryWorldState(Geometry* Object);
		void				UpdateGeometryWorldStates(const std::vector<Geometry*>& Objects);
		void				UpdateGeometryWorldStates();
		void				RemoveGeometryWorldState(GeometryHandle Handle);
		void				RemoveGeometryWorldState(Geometry* Object);
		void				GetActiveGeometryWorldStates(std::vector<const GeometryWorldState*>* States, std::vector<Geometry*>* Objects) const;
		void				GetCurrentGeometryWorldStates(std::vector<GeometryWorldState>* StateStorage,
			std::vector<const GeometryWorldState*>* States, std::vector<Geometry*>* Objects) const;
		void				PreIntegrate(float dt);
		void				PostIntegrate(float dt);
		void StabilizeConstraintSupports(float Dt);
		void FinalizeConstraints(float Dt, bool EndFrame = false);
		void				DispatchContactCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);
		void				DispatchTriggerCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);
		void ResolveContinuousMotion(const std::vector<RigidBody*>& Bodies, float Dt);
		void				DispatchTriggerCCDCallbacks(const std::vector<struct ContinuousCollisionHit>& hits);
		void				UpdateDestructionSets(float dt);
		void				SolveConstraintPositions(float dt);
		void				SolveConstraintVelocities(float dt);
		void				SolvePostIntegrationPositions(float dt);
		void				BuildConstrainedContactBodies(const std::vector<Geometry*>& Geometries,
			const std::vector<ContactManifold*>& Manifolds, std::vector<RigidBody*>* Bodies) const;
		void				SimulateRopes(float dt);
		void				SimulateCloths(float dt);
		void				RefreshHeightFieldGeometry(Geometry* HeightFieldGeometry);
		void				BuildSceneQuery(const std::vector<Geometry*>& Objects);
		void				BuildSceneQuery(GeometryWorldStateSpan States);
		void				AddGeometryToSceneQuery(Geometry* Object);
		void				RemoveGeometryFromSceneQuery(Geometry* Object);
		void				UpdateSceneQuery(const GeometryWorldState& State);
		void				UpdateSceneQuery(RigidBodyDynamic* Body);

	private:
		std::vector<RigidBodyStatic*>	m_StaticBodies;
		std::vector<RigidBodyDynamic*>	m_DynamicBodies;
		std::vector<RigidBodyKinematics*> m_KinematicBodies;
		std::vector<KinematicsDriver*>	m_Kinematics;
		friend class DestructionPlayback;
		friend class PhysicsFieldSystem;
		friend class StrandSystem;
		std::unique_ptr<PhysicsFieldSystem> m_PhysicsFields;
		bool m_UpdatingFields = false;
		bool m_UpdatingDestruction = false;
		std::vector<DestructionPlayback*> m_DestructionPlaybacks;
		bool m_UpdatingPlayback = false;

		JobSystem*						m_Jobsystem;
		GeometryQuery*					m_GeometryQuery;
		BroadPhase*						m_BPhase;
		NarrowPhase*					m_NPhase;
		ConstraintSolver*				m_Solver;
		PBDSolver*						m_PositionSolver;
		SceneQueryType					m_SceneQuery;
		IntegrateMethod					m_IntegrateMethod;
		ResolutionPhaseSolver			m_ResolutionSolver;
		SolverBackend					m_RequestedSolverBackend;
		bool							m_AllowBackendFallback;
		PBDSolverSettings				m_PBDSettings;
		XPBDSolverSettings				m_XPBDSettings;
		LCPSolverSettings				m_LCPSettings;
		OnContactCallback				m_OnContact;
		OnJointBreakCallback m_OnJointBreak;
		OnTriggerCallback				m_OnTrigger;
		std::vector<DestructionSet*>	m_DestructionSets;
		std::vector<ForceField*>		m_Fields;
		std::vector<PositionConstraint*> m_Constraints;
		uint64_t m_NextConstraintId = 1;
		std::vector<RopeSystem*>		m_Ropes;
		std::vector<ClothSystem*>		m_Cloths;
		std::vector<StrandSystem*> m_Strands;
		std::vector<DeformableBody*>	m_DeformableBodies;
		std::vector<RigidDeformableAttachment> m_DeformableRigidAttachments;
		uint64_t m_NextDeformableRigidAttachmentId = 1;
		int DeformableCouplingSubsteps() const;
		bool HasDeformableRigidContacts() const;
		bool m_DeformableCouplingActive = false;
		std::vector<RigidBody*> m_DeformableContactBodies;
		std::vector<Ragdoll*>			m_Ragdolls;
		int									m_ConstraintIterations;
		std::vector<const GeometryWorldState*> m_GeometryWorldStates;
		std::deque<GeometryWorldRecord> m_GeometryWorldRecords;
		std::vector<uint32_t>			m_FreeGeometryWorldRecords;
		WorldClock						m_Clock;
		IBinaryData*							m_SceneResource;
		int									m_LastCCDHitCount;
		ContactPersistenceSettings m_ContactPersistence;
		FrictionPatchSettings m_FrictionPatches;
		MeshContactSettings m_MeshContactSettings;
		std::shared_ptr<CollisionFilter> m_CollisionFilter;
		ContactGenerationSettings m_ContactGeneration;
		bool m_CollisionFilteringEnabled = true;
		NarrowPhaseSolver m_NarrowPhaseSolver = NarrowPhaseSolver::GJKEPA;
		bool m_UnifiedConstraints = true;
		bool m_JointDrives = true;
		bool m_JointBreaking = true;
		CCDContinuationSettings m_CCDContinuation;
		CCDContinuationStats m_CCDContinuationStats;
		TemporalSubstepSettings m_TemporalSubsteps;
		IslandSleeping m_IslandSleeping;
		void TrackSleepContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts);
		void FinishIslandSleeping(float Dt);
		SupportStabilizationSettings m_SupportStabilization;
		SupportStabilizationStats m_SupportStabilizationStats;
		SequentialImpulseSettings m_SequentialImpulseSettings;
		PhysicsStepInfo m_StepInfo;
		uint64_t m_SolverFrame = 0;
		bool m_SimulationInProgress = false;
		bool m_SequentialImpulseWarmStart = true;
	};
}

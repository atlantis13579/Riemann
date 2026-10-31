#pragma once

#include <stdint.h>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "RigidBody.h"
#include "Constraints.h"
#include "ConstraintSolver.h"
#include "LCPSolver.h"
#include "../Deformation/PBDSolver.h"
#include "../Deformation/XPBDSolver.h"

namespace Riemann
{
	class Geometry;
	class GeometryQuery;
	class RigidBody;
	class RigidBodyStatic;
	class RigidBodyDynamic;
	class BroadPhase;
	class NarrowPhase;
	class ContactManifold;
	class ConstraintSolver;
	class DestructionSet;
	class ForceField;
	class KinematicsDriver;
	class JobSystem;
	class IBinaryData;
	class RopeSystem;
	struct RopeSettings;
	class ClothSystem;
	struct ClothSettings;
	class DeformableBody;
	struct DeformableBodySettings;
	class Ragdoll;
	struct RagdollSettings;

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
		float	PenetrationDepth;
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
	};

	using OnContactCallback = std::function<void(const PhysicsContactInfo&)>;
	using OnTriggerCallback = std::function<void(const PhysicsTriggerInfo&)>;

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
		OnTriggerCallback		onTrigger;
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
		void				Reset();

		bool				LoadScene(const char* name, bool shared_mem);

		RigidBody*			CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose);
		RigidBody*			CreateRigidBody(Geometry* Geom, const RigidBodyParam& param);
		RigidBody*			CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform, bool RegisterDisabledGeometries = true);
		bool				RemoveRigidBody(RigidBody* Body);
		// The world owns registered fields and deletes them on removal, reset, or destruction.
		ForceField*		AddForceField(ForceField* Field);
		bool				RemoveForceField(ForceField* Field);
		size_t				GetForceFieldCount() const { return m_Fields.size(); }
		PositionConstraint* AddConstraint(PositionConstraint* Constraint);
		bool				RemoveConstraint(PositionConstraint* Constraint);
		FixedConstraint*	CreateFixedConstraint(RigidBody* BodyA, RigidBody* BodyB);
		PointConstraint*	CreatePointConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor);
		DistanceConstraint* CreateDistanceConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchorA, const Vector3& WorldAnchorB, float MinDistance, float MaxDistance);
		HingeConstraint*	CreateHingeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinAngle, float MaxAngle);
		SliderConstraint*	CreateSliderConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxis, float MinDistance, float MaxDistance);
		ConeConstraint*		CreateConeConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const Vector3& WorldAxisA, const Vector3& WorldAxisB, float MaxAngle);
		SixDOFConstraint*	CreateSixDOFConstraint(RigidBody* BodyA, RigidBody* BodyB, const Vector3& WorldAnchor, const SixDOFConstraintSettings& Settings);
		size_t				GetConstraintCount() const { return m_Constraints.size(); }
		RopeSystem*			CreateRope(const RopeSettings& Settings, const Vector3& Start, const Vector3& End);
		bool				RemoveRope(RopeSystem* Rope);
		size_t				GetRopeCount() const { return m_Ropes.size(); }
		ClothSystem*		CreateCloth(const ClothSettings& Settings, const Vector3& Center, const Vector3& HorizontalAxis, const Vector3& VerticalAxis);
		bool				RemoveCloth(ClothSystem* Cloth);
		size_t				GetClothCount() const { return m_Cloths.size(); }
		DeformableBody*		CreateDeformableBody(const DeformableBodySettings& Settings);
		bool				RemoveDeformableBody(DeformableBody* Body);
		size_t				GetDeformableBodyCount() const { return m_DeformableBodies.size(); }
		Ragdoll*			CreateRagdoll(const RagdollSettings& Settings, const Transform& RootTransform);
		bool				RemoveRagdoll(Ragdoll* RagdollInstance);
		size_t				GetRagdollCount() const { return m_Ragdolls.size(); }
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

	private:
		void				SimulateST(float dt);
		void				SimulatePositionBasedRigidBodies(float dt);

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
		void				DispatchContactCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);
		void				DispatchTriggerCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);
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
		std::vector<KinematicsDriver*>	m_Kinematics;

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
		OnTriggerCallback				m_OnTrigger;
		std::vector<DestructionSet*>	m_DestructionSets;
		std::vector<ForceField*>		m_Fields;
		std::vector<PositionConstraint*> m_Constraints;
		std::vector<RopeSystem*>		m_Ropes;
		std::vector<ClothSystem*>		m_Cloths;
		std::vector<DeformableBody*>	m_DeformableBodies;
		std::vector<Ragdoll*>			m_Ragdolls;
		int									m_ConstraintIterations;
		std::vector<const GeometryWorldState*> m_GeometryWorldStates;
		std::deque<GeometryWorldRecord> m_GeometryWorldRecords;
		std::vector<uint32_t>			m_FreeGeometryWorldRecords;
		WorldClock						m_Clock;
		IBinaryData*							m_SceneResource;
		int									m_LastCCDHitCount;
	};
}

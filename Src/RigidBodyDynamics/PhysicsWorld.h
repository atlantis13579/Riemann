#pragma once

#include <stdint.h>
#include <functional>
#include <string>
#include <vector>

#include "RigidBody.h"

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

	using OnContactCallback = std::function<void(const PhysicsContactInfo&)>;

	void DefaultOnContact(const PhysicsContactInfo& ContactInfo);

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
			onContact = DefaultOnContact;
		}
		Vector3 				gravityAcc;		// gravity acc
		ThreadMode				threadMode;
		BroadPhaseSolver		broadphase;
		SceneQueryType			sceneQueryType;
		NarrowPhaseSolver		narrowphase;
		ResolutionPhaseSolver	resolutionSolver;
		IntegrateMethod 		integrateMethod;
		int						workerThreads;
		OnContactCallback		onContact;
	};

	class PhysicsWorld
	{
	public:
		PhysicsWorld(const PhysicsWorldParam& param);
		~PhysicsWorld();

	public:
		void				Simulate();
		void				Reset();

		bool				LoadScene(const char* name, bool shared_mem);

		RigidBody*			CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose);
		RigidBody*			CreateRigidBody(Geometry* Geom, const RigidBodyParam& param);
		RigidBody*			CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform);
		bool				RemoveRigidBody(RigidBody* Body);
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

	private:
		void				SimulateST(float dt);

		void				PreIntegrate(float dt);
		void				PostIntegrate(float dt);
		void				DispatchContactCallbacks(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);
		void				UpdateDestructionSets(float dt);
		void				BuildSceneQuery(const std::vector<Geometry*>& Objects);
		void				AddGeometryToSceneQuery(Geometry* Object);
		void				RemoveGeometryFromSceneQuery(Geometry* Object);
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
		SceneQueryType					m_SceneQuery;
		IntegrateMethod					m_IntegrateMethod;
		OnContactCallback				m_OnContact;
		std::vector<DestructionSet*>	m_DestructionSets;
		std::vector<ForceField*>		m_Fields;
		WorldClock						m_Clock;
		IBinaryData*							m_SceneResource;
	};
}

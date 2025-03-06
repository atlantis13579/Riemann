#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include "RigidBody.h"

namespace Riemann
{
	class GeometryBase;
	class GeometryQuery;
	class RigidBody;
	class RigidBodyStatic;
	class RigidBodyDynamic;
	class BroadPhase;
	class NarrowPhase;
	class ConstraintSolver;
	class ForceField;
	class KinematicsDriver;
	class JobSystem;

	enum class ThreadMode : uint8_t
	{
		SingleThread,
		PhysicsThread,
		JobSystem,
	};

	enum class BroadPhaseSolver : uint8_t
	{
		SAP,
		AllPairs,
		Bruteforce,
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

	struct RigidBodySimulationParam
	{
		RigidBodySimulationParam()
		{
			gravityAcc = Vector3::Zero();
			threadMode = ThreadMode::SingleThread;
			broadphase = BroadPhaseSolver::SAP;
			narrowphase = NarrowPhaseSolver::GJKEPA;
			resolutionSolver = ResolutionPhaseSolver::SequentialImpulse;
			integrateMethod = IntegrateMethod::ExplicitEuler;
			workerThreads = 0;
		}
		Vector3 				gravityAcc;		// gravity acc
		ThreadMode				threadMode;
		BroadPhaseSolver		broadphase;
		NarrowPhaseSolver		narrowphase;
		ResolutionPhaseSolver	resolutionSolver;
		IntegrateMethod 		integrateMethod;
		int						workerThreads;
	};

	class RigidBodySimulation
	{
	public:
		RigidBodySimulation(const RigidBodySimulationParam& param);
		~RigidBodySimulation();

	public:
		void				Simulate();
		void				Reset();

		bool				LoadScene(const char* name, bool shared_mem);

		RigidBody*			CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose);
		RigidBody*			CreateRigidBody(GeometryBase* Geom, const RigidBodyParam& param);
		bool				RemoveRigidBody(RigidBody* Body);

		bool				LoadAnimation(const std::string& resname, const std::string& filepath, float play_rate, bool begin_play);
		KinematicsDriver*	FindKinematics(const std::string& resname);

		void				GetAllGeometries(std::vector<GeometryBase*>* AllObjects);
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

	private:
		std::vector<RigidBodyStatic*>	m_StaticBodies;
		std::vector<RigidBodyDynamic*>	m_DynamicBodies;
		std::vector<KinematicsDriver*>	m_Kinematics;

		JobSystem*						m_Jobsystem;
		GeometryQuery*					m_GeometryQuery;
		BroadPhase*						m_BPhase;
		NarrowPhase*					m_NPhase;
		ConstraintSolver*				m_Solver;
		IntegrateMethod					m_IntegrateMethod;
		std::vector<ForceField*>		m_Fields;
		WorldClock						m_Clock;
		void*							m_SharedMem;
		size_t							m_SharedMemSize;
	};
}
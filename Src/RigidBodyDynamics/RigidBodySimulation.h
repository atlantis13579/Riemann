#pragma once

#include <string>
#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;
class ResolutionPhase;
class ForceField;
class KinematicsDriver;

enum class BroadPhaseSolver : uint8_t
{
	SAP,
	AllPairs,
	DynamicAABB,
};

enum class NarrowPhaseSolver : uint8_t
{
	GJKEPA,
	PCM,
};

enum class ConstraintSolver : uint8_t
{
	SequentialImpulse,
	LCPGlobal,
	PBD,
	XPBD,
};

enum class IntegrateMethod : uint8_t
{
	ExplicitEuler,
	MidpointEuler,
	SymplecticEuler,
	ImplicitEuler
};

struct RigidBodySimulationParam
{
	RigidBodySimulationParam()
	{
		gravity = Vector3::Zero();
		broadphase = BroadPhaseSolver::SAP;
		narrowphase = NarrowPhaseSolver::GJKEPA;
		constraintSolver = ConstraintSolver::SequentialImpulse;
		integrateMethod = IntegrateMethod::ExplicitEuler;
	}
	Vector3 			gravity;		// gravity acc
	BroadPhaseSolver	broadphase;
	NarrowPhaseSolver	narrowphase;
	ConstraintSolver	constraintSolver;
	IntegrateMethod 	integrateMethod;
};

class RigidBodySimulation
{
public:
	RigidBodySimulation(const RigidBodySimulationParam&param);
	~RigidBodySimulation();

public:
	void				Simulate(float dt);
	void				ApplyForceFields();
    
    bool				LoadPhysxScene(const char *name, bool shared_mem);

	RigidBody*			CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);

	bool				LoadAnimation(const std::string& resname, const std::string &filepath, float play_rate, bool begin_play);
	KinematicsDriver*	FindKinematics(const std::string& resname);

    GeometryQuery*          GetGeometryQuery() { return m_GeometryQuery; }
    const GeometryQuery*    GetGeometryQuery() const { return m_GeometryQuery; }
    
	float				GetSystemTotalEnergy() const;
	float				GetSystemTotalLinearKinematicsEnergy() const;
	float				GetSystemTotalAngularKinematicsEnergy() const;
	Vector3				GetSystemTotalLinearMomentum() const;
	Vector3				GetSystemTotalAngularMomentum() const;

private:
	void				SimulateST(float dt);

private:
	std::vector<RigidBodyStatic*>	m_StaticBodies;
	std::vector<RigidBodyDynamic*>	m_DynamicBodies;
	std::vector<KinematicsDriver*>	m_Kinematics;

	GeometryQuery*					m_GeometryQuery;
	BroadPhase*						m_BPhase;
	NarrowPhase*					m_NPhase;
	ResolutionPhase*				m_RPhase;
	IntegrateMethod					m_IntegrateMethod;
	std::vector<ForceField*>		m_Fields;
	void*							m_SharedMem;
	size_t							m_SharedMemSize;
};

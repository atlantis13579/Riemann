#pragma once

#include <string>
#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;
class ForceField;
class KinematicsTree;

struct RigidBodySimulationParam
{
	Vector3d Gravity;
};

class RigidBodySimulation
{
public:
	RigidBodySimulation(const RigidBodySimulationParam&param);
	~RigidBodySimulation();

public:
	void			Simulate(float dt);
	void			ApplyForceFields();
    
    bool            LoadPhysxScene(const char *name, bool shared_mem);

	RigidBody*		CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);
	bool			LoadAnimation(const std::string& resname, const std::string &filepath, float play_rate, bool begin_play);

	bool			BindKinematicsNode(const std::string& anim, const std::string& node, RigidBodyStatic* body);
	KinematicsTree*	FindKinematics(const std::string& resname);

    GeometryQuery*          GetGeometryQuery() { return m_GeometryQuery; }
    const GeometryQuery*    GetGeometryQuery() const { return m_GeometryQuery; }
    
private:
	void			SimulateSingleThread(float dt);

private:
	std::vector<RigidBodyStatic*>	m_RigidStatics;
	std::vector<RigidBodyDynamic*>	m_RigidDynamics;
	std::vector<KinematicsTree*>	m_Kinematics;

	GeometryQuery*					m_GeometryQuery;
	BroadPhase*						m_BPhase;
	NarrowPhase*					m_NPhase;
	std::vector<ForceField*>		m_Fields;
	void*							m_SharedMem;
	size_t							m_SharedMemSize;
};

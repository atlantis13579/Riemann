#pragma once

#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;
class ForceField;

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
	void		Simulate(float dt);
	void		ApplyGravity();
	void		ApplyWind();

	RigidBodyDynamic*	CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);

private:
	std::vector<RigidBodyDynamic*> m_Entities;

	GeometryQuery*	m_GeometryQuery;
	BroadPhase*		m_BPhase;
	NarrowPhase*	m_NPhase;
	ForceField*		m_GravityField;
	ForceField*		m_WindField;
};
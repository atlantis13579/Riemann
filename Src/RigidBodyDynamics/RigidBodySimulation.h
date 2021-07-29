#pragma once

#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;

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
	void Simulate(float dt);

	void CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);

private:
	std::vector<Geometry*> m_Entities;

	GeometryQuery*	m_GeometryQuery;
	BroadPhase*		m_BPhase;
	NarrowPhase*	m_NPhase;

	// Global Physics parameters
	Vector3d		m_Gravity;
};
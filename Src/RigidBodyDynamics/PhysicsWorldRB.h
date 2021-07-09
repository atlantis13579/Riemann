#pragma once

#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;

struct PhysicsWorldRBParam
{
	Vector3d Gravity;
};

class PhysicsWorldRB
{
public:
	PhysicsWorldRB(const PhysicsWorldRBParam &param);
	~PhysicsWorldRB();

public:
	void Simulate(float dt);

	void CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);

private:
	std::vector<Geometry*> m_Entities;

	GeometryQuery*	m_GeometryQuery;
	BroadPhase*		m_BPhase;
	NarrowPhase*	m_NPhase;
};
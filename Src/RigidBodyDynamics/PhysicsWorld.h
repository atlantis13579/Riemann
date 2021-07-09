#pragma once

#include <vector>

class Geometry;
class BroadPhase;
class NarrowPhase;

struct PhysicsWorldParam
{

};

class PhysicsWorld
{
public:
	PhysicsWorld(const PhysicsWorldParam &param);
	~PhysicsWorld();

public:
	void Simulate(float dt);

	void MotionIntegration(float dt);

	void AddGeometry(Geometry *Geom);

private:
	std::vector<Geometry*> m_Entities;

	BroadPhase* m_BPhase;
	NarrowPhase* m_NPhase;
};
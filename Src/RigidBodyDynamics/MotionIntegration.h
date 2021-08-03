#pragma once

#include <vector>

class RigidBody;

class MotionIntegration
{
public:
	static void Integrate(std::vector<RigidBody*> Entities, float dt);
};
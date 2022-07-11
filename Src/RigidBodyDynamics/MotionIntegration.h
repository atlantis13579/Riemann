#pragma once

#include <vector>

class RigidBodyDynamic;

class MotionIntegration
{
public:
	static void Integrate(std::vector<RigidBodyDynamic*> Entities, float dt);
};

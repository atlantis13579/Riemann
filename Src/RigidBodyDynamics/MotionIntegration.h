#pragma once

#include <vector>

class RigidBodyDynamic;

#define USE_SOLVER_VW	0x02

class MotionIntegration
{
public:
	static void Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, uint8_t method);
};

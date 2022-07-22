#pragma once

#include <vector>

class RigidBodyDynamic;

class MotionIntegration
{
public:
	enum IntegrateMethod
	{
		ExplicitEuler,
		MidpointEuler,
		SymplecticEuler,
		ImplicitEuler
	};

	static void Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, IntegrateMethod method);
};

#pragma once

#include <vector>

namespace Riemann
{
	class RigidBodyDynamic;

	class MotionIntegration
	{
	public:
		static void Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, uint8_t method);
	};
}

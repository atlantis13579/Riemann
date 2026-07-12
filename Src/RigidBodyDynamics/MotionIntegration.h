#pragma once

#include <vector>

namespace Riemann
{
	class RigidBodyDynamic;

	class MotionIntegration
	{
	public:
		static void Integrate(std::vector<RigidBodyDynamic*> Bodies, float Dt, uint8_t Method);
		static void IntegrateVelocities(const std::vector<RigidBodyDynamic*>& Bodies, float Dt);
		static void IntegratePoses(const std::vector<RigidBodyDynamic*>& Bodies, float Dt, uint8_t Method);
	};
}

#pragma once

#include "SnowSimulation.h"

namespace Riemann
{
	class SnowGPUBackend
	{
	public:
		virtual ~SnowGPUBackend() {}
		virtual bool Step(const SnowSettings& Settings, const SnowCapsule& Capsule, float Dt,
			std::vector<SnowParticle>& Particles, std::string& Error) = 0;
	};
	bool IsSnowCUDAAvailable();
	bool IsSnowComputeAvailable(bool MetalOnly);
	std::unique_ptr<SnowGPUBackend> CreateSnowCUDA();
	std::unique_ptr<SnowGPUBackend> CreateSnowCompute();
}

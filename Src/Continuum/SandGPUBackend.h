#pragma once

#include "SandSimulation.h"

namespace Riemann
{
	class SandGPUBackend
	{
	public:
		virtual ~SandGPUBackend() = default;
		virtual bool Relax(std::vector<float>& Heights, const std::vector<float>& Limits,
			int Resolution, float Slope, float Rate, int Iterations, std::string& Error) = 0;
	};
	std::unique_ptr<SandGPUBackend> CreateSandCUDA();
	std::unique_ptr<SandGPUBackend> CreateSandCompute();
}

#include "SnowGPUBackend.h"
#include "SandGPUBackend.h"
#include "../Compute/BackendAvailability.h"
#include "../Fluid/FluidBackend.h"

namespace Riemann
{
#if !defined(RIEMANN_SNOW_CUDA)
	bool IsSnowCUDAAvailable()
	{
		return false;
	}
	std::unique_ptr<SnowGPUBackend> CreateSnowCUDA()
	{
		return {};
	}
#endif
#if !defined(_WIN32) || !defined(RIEMANN_ENABLE_COMPUTE_SHADER)
	std::unique_ptr<SnowGPUBackend> CreateSnowDX11()
	{
		return {};
	}
	std::unique_ptr<SandGPUBackend> CreateSandDX11()
	{
		return {};
	}
#endif
#if !defined(RIEMANN_ENABLE_METAL)
	std::unique_ptr<SnowGPUBackend> CreateSnowMetal()
	{
		return {};
	}
	std::unique_ptr<SandGPUBackend> CreateSandMetal()
	{
		return {};
	}
#endif
	bool IsSnowGPUBackendAvailable(SnowBackend Backend)
	{
		switch (Backend)
		{
		case SnowBackend::CUDA: return IsSnowCUDAAvailable();
		case SnowBackend::ComputeShader: return IsDirectComputeAvailable();
		case SnowBackend::Metal: return IsMetalComputeAvailable();
		default: return false;
		}
	}
	bool IsSandGPUBackendAvailable(SnowBackend Backend)
	{
		switch (Backend)
		{
#if defined(RIEMANN_SNOW_CUDA)
		case SnowBackend::CUDA: return IsFluidBackendAvailable(FluidBackend::CUDA);
#endif
		case SnowBackend::ComputeShader: return IsDirectComputeAvailable();
		case SnowBackend::Metal: return IsMetalComputeAvailable();
		default: return false;
		}
	}
} // namespace Riemann

#include "SnowGPUBackend.h"

namespace Riemann
{
#if !defined(RIEMANN_SNOW_CUDA)
	bool IsSnowCUDAAvailable() { return false; }
	std::unique_ptr<SnowGPUBackend> CreateSnowCUDA() { return {}; }
#endif
#if !defined(RIEMANN_SNOW_COMPUTE)
	bool IsSnowComputeAvailable(bool) { return false; }
	std::unique_ptr<SnowGPUBackend> CreateSnowCompute() { return {}; }
#endif
}

#pragma once

#include <stdint.h>

namespace Riemann
{
	enum class SolverBackend : uint8_t
	{
		CPU,
		SIMD,
		ComputeShader,
		CUDA,
		Metal,
	};
}

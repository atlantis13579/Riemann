#pragma once

#include <cstddef>
#include <cstdint>

#include "../Maths/Vector3.h"

namespace Riemann
{
	enum class FluidBackend : uint8_t
	{
		// CUDA and ComputeShader currently accelerate velocity addition only.
		// Grid advection, pressure projection, and SPH remain CPU-side.
		CPU,
		SIMD,
		CUDA,
		ComputeShader,
	};

	struct FluidBackendResult
	{
		FluidBackend RequestedBackend = FluidBackend::CPU;
		FluidBackend Backend = FluidBackend::CPU;
		bool UsedFallback = false;
	};

	bool IsFluidBackendAvailable(FluidBackend Backend);
	bool ApplyFluidVelocityDelta(const Vector3* Velocity, const Vector3* Delta,
		size_t Count, Vector3* Output, FluidBackend Backend, bool AllowFallback,
		FluidBackendResult* Result = nullptr);
}

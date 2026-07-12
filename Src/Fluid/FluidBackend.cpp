#include "FluidBackend.h"

#include <algorithm>

#include "../Maths/SIMD.h"

namespace Riemann
{
#if defined(RIEMANN_ENABLE_CUDA)
	bool IsFluidCUDABackendAvailable();
	bool ApplyFluidVelocityDeltaCUDA(const Vector3* Velocity, const Vector3* Delta,
		size_t Count, Vector3* Output);
#endif

#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
	bool IsFluidComputeShaderBackendAvailable();
	bool ApplyFluidVelocityDeltaComputeShader(const Vector3* Velocity,
		const Vector3* Delta, size_t Count, Vector3* Output);
#endif

	namespace
	{
		void ApplyCPU(const Vector3* Velocity, const Vector3* Delta,
			size_t Count, Vector3* Output)
		{
			for (size_t Index = 0; Index < Count; ++Index)
				Output[Index] = Velocity[Index] + Delta[Index];
		}

		void ApplySIMD(const Vector3* Velocity, const Vector3* Delta,
			size_t Count, Vector3* Output)
		{
			static_assert(sizeof(Vector3) == sizeof(float) * 3,
				"Fluid SIMD backend requires tightly packed Vector3 values");
			const float* Source = reinterpret_cast<const float*>(Velocity);
			const float* Increment = reinterpret_cast<const float*>(Delta);
			float* Destination = reinterpret_cast<float*>(Output);
			const size_t FloatCount = Count * 3;
			size_t Index = 0;
			for (; Index + 4 <= FloatCount; Index += 4)
				F128_StoreU(F128_Add(F128_LoadU(Source + Index),
					F128_LoadU(Increment + Index)), Destination + Index);
			for (; Index < FloatCount; ++Index)
				Destination[Index] = Source[Index] + Increment[Index];
		}

		bool Execute(FluidBackend Backend, const Vector3* Velocity,
			const Vector3* Delta, size_t Count, Vector3* Output)
		{
			switch (Backend)
			{
			case FluidBackend::CPU:
				ApplyCPU(Velocity, Delta, Count, Output);
				return true;
			case FluidBackend::SIMD:
				ApplySIMD(Velocity, Delta, Count, Output);
				return true;
			case FluidBackend::CUDA:
#if defined(RIEMANN_ENABLE_CUDA)
				return ApplyFluidVelocityDeltaCUDA(Velocity, Delta, Count, Output);
#else
				return false;
#endif
			case FluidBackend::ComputeShader:
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
				return ApplyFluidVelocityDeltaComputeShader(Velocity, Delta, Count, Output);
#else
				return false;
#endif
			}
			return false;
		}
	}

	bool IsFluidBackendAvailable(FluidBackend Backend)
	{
		switch (Backend)
		{
		case FluidBackend::CPU:
		case FluidBackend::SIMD:
			return true;
		case FluidBackend::CUDA:
#if defined(RIEMANN_ENABLE_CUDA)
			return IsFluidCUDABackendAvailable();
#else
			return false;
#endif
		case FluidBackend::ComputeShader:
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			return IsFluidComputeShaderBackendAvailable();
#else
			return false;
#endif
		}
		return false;
	}

	bool ApplyFluidVelocityDelta(const Vector3* Velocity, const Vector3* Delta,
		size_t Count, Vector3* Output, FluidBackend Backend, bool AllowFallback,
		FluidBackendResult* Result)
	{
		FluidBackendResult LocalResult;
		LocalResult.RequestedBackend = Backend;
		LocalResult.Backend = Backend;
		if (Count == 0)
		{
			if (Result) *Result = LocalResult;
			return true;
		}
		if (Velocity == nullptr || Delta == nullptr || Output == nullptr) return false;
		if (IsFluidBackendAvailable(Backend)
			&& Execute(Backend, Velocity, Delta, Count, Output))
		{
			if (Result) *Result = LocalResult;
			return true;
		}
		if (!AllowFallback || Backend == FluidBackend::CPU) return false;
		ApplyCPU(Velocity, Delta, Count, Output);
		LocalResult.Backend = FluidBackend::CPU;
		LocalResult.UsedFallback = true;
		if (Result) *Result = LocalResult;
		return true;
	}
}

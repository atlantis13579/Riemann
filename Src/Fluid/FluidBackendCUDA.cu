#if defined(RIEMANN_ENABLE_CUDA)

#include "FluidBackend.h"

#include <cuda_runtime.h>

namespace Riemann
{
	namespace
	{
		__global__ void ApplyVelocityDeltaKernel(const Vector3* Velocity,
			const Vector3* Delta, size_t Count, Vector3* Output)
		{
			const size_t Index = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
			if (Index < Count)
			{
				Output[Index].x = Velocity[Index].x + Delta[Index].x;
				Output[Index].y = Velocity[Index].y + Delta[Index].y;
				Output[Index].z = Velocity[Index].z + Delta[Index].z;
			}
		}
	}

	bool IsFluidCUDABackendAvailable()
	{
		int DeviceCount = 0;
		return cudaGetDeviceCount(&DeviceCount) == cudaSuccess && DeviceCount > 0;
	}

	bool ApplyFluidVelocityDeltaCUDA(const Vector3* Velocity, const Vector3* Delta,
		size_t Count, Vector3* Output)
	{
		if (Count == 0) return true;
		const size_t Bytes = Count * sizeof(Vector3);
		Vector3* DeviceVelocity = nullptr;
		Vector3* DeviceDelta = nullptr;
		Vector3* DeviceOutput = nullptr;
		bool Success = cudaMalloc(&DeviceVelocity, Bytes) == cudaSuccess
			&& cudaMalloc(&DeviceDelta, Bytes) == cudaSuccess
			&& cudaMalloc(&DeviceOutput, Bytes) == cudaSuccess;
		if (Success) Success = cudaMemcpy(DeviceVelocity, Velocity, Bytes,
			cudaMemcpyHostToDevice) == cudaSuccess
			&& cudaMemcpy(DeviceDelta, Delta, Bytes, cudaMemcpyHostToDevice) == cudaSuccess;
		if (Success)
		{
			ApplyVelocityDeltaKernel<<<(unsigned int)((Count + 255) / 256), 256>>>(
				DeviceVelocity, DeviceDelta, Count, DeviceOutput);
			Success = cudaGetLastError() == cudaSuccess
				&& cudaMemcpy(Output, DeviceOutput, Bytes, cudaMemcpyDeviceToHost) == cudaSuccess;
		}
		if (DeviceOutput) cudaFree(DeviceOutput);
		if (DeviceDelta) cudaFree(DeviceDelta);
		if (DeviceVelocity) cudaFree(DeviceVelocity);
		return Success;
	}
}

#endif

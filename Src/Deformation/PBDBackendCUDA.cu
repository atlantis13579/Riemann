#if defined(RIEMANN_ENABLE_CUDA)

#include "PBDBackend.h"
#include "PBDBackendMath.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <new>

namespace Riemann
{
	namespace
	{
		struct PBDCUDAContext
		{
			int Device = -1;
			uint32_t MaxGridSize = 0;
			cudaStream_t Stream = nullptr;
			bool Failed = false;
			PBDBackendBody* DeviceBodies = nullptr;
			PBDBackendContact* DeviceContacts = nullptr;
			uint32_t* DeviceRows = nullptr;
			std::unique_ptr<PBDBackendBody[]> HostBodies;
			std::unique_ptr<PBDBackendContact[]> HostContacts;
			size_t BodyCapacity = 0;
			size_t ContactCapacity = 0;
			size_t RowCapacity = 0;
		};

		// A solver retains allocations on its original device, even if other CUDA
		// clients change the calling thread's current device between simulation steps.
		class ScopedCUDADevice
		{
		public:
			explicit ScopedCUDADevice(int Device)
			{
				if (cudaGetDevice(&m_Previous) != cudaSuccess) return;
				m_Changed = m_Previous != Device;
				m_Valid = !m_Changed || cudaSetDevice(Device) == cudaSuccess;
			}

			~ScopedCUDADevice()
			{
				if (m_Valid && m_Changed) (void)cudaSetDevice(m_Previous);
			}

			bool IsValid() const { return m_Valid; }

		private:
			int m_Previous = -1;
			bool m_Changed = false;
			bool m_Valid = false;
		};

		template<typename T>
		bool GrowthCapacity(size_t Required, size_t& Capacity)
		{
			const size_t Maximum = std::numeric_limits<size_t>::max() / sizeof(T);
			if (Required > Maximum) return false;
			Capacity = 64;
			while (Capacity < Required)
			{
				if (Capacity > Maximum / 2) { Capacity = Required; break; }
				Capacity *= 2;
			}
			return true;
		}

		template<typename T>
		bool GrowTransferBuffer(T*& Device, std::unique_ptr<T[]>& Host, size_t& Capacity, size_t Required)
		{
			if (Required <= Capacity) return true;
			size_t NewCapacity = 0;
			if (!GrowthCapacity<T>(Required, NewCapacity)) return false;
			std::unique_ptr<T[]> NewHost(new (std::nothrow) T[NewCapacity]);
			if (!NewHost) return false;
			void* NewDevice = nullptr;
			if (cudaMalloc(&NewDevice, NewCapacity * sizeof(T)) != cudaSuccess) return false;
			if (Device && cudaFree(Device) != cudaSuccess)
			{
				(void)cudaFree(NewDevice);
				return false;
			}
			Device = static_cast<T*>(NewDevice);
			Host.swap(NewHost);
			Capacity = NewCapacity;
			return true;
		}

		bool GrowRowBuffer(PBDCUDAContext& Context, size_t Required)
		{
			if (Required <= Context.RowCapacity) return true;
			size_t NewCapacity = 0;
			if (!GrowthCapacity<uint32_t>(Required, NewCapacity)) return false;
			void* NewDevice = nullptr;
			if (cudaMalloc(&NewDevice, NewCapacity * sizeof(uint32_t)) != cudaSuccess) return false;
			if (Context.DeviceRows && cudaFree(Context.DeviceRows) != cudaSuccess)
			{
				(void)cudaFree(NewDevice);
				return false;
			}
			Context.DeviceRows = static_cast<uint32_t*>(NewDevice);
			Context.RowCapacity = NewCapacity;
			return true;
		}

		bool AbortExecution(PBDCUDAContext& Context)
		{
			// Finish any already queued transfers before host staging is reused or
			// destroyed. A failed stream is not reused by a later fallback attempt.
			(void)cudaStreamSynchronize(Context.Stream);
			Context.Failed = true;
			(void)cudaGetLastError();
			return false;
		}

		__global__ void SolveContactLayerKernel(PBDBackendBody* Bodies,
			PBDBackendContact* Contacts, const uint32_t* RowIndices, uint32_t Begin,
			uint32_t End, PBDBackendParameters Parameters, PBDBackendStage Stage)
		{
			const size_t Stride = size_t(blockDim.x) * gridDim.x;
			for (size_t Index = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
				Index < size_t(End - Begin); Index += Stride)
			{
				const uint32_t Row = RowIndices[size_t(Begin) + Index];
				// Each layer owns disjoint writable bodies. Shared static bodies remain
				// read-only inside SolveRow; do not copy whole body records back here.
				PBDBackendMath::SolveRow<PBDBackendMath::Scalar>(Bodies, Contacts[Row], Parameters, Stage);
			}
		}
	}

	bool IsPBDCUDAAvailable()
	{
		int Count = 0;
		if (cudaGetDeviceCount(&Count) != cudaSuccess)
		{
			(void)cudaGetLastError();
			return false;
		}
		return Count > 0;
	}

	void* CreatePBDCUDAContext()
	{
		if (!IsPBDCUDAAvailable()) return nullptr;
		std::unique_ptr<PBDCUDAContext> Context(new (std::nothrow) PBDCUDAContext());
		if (!Context || cudaGetDevice(&Context->Device) != cudaSuccess) return nullptr;
		cudaDeviceProp Properties;
		if (cudaGetDeviceProperties(&Properties, Context->Device) != cudaSuccess
			|| Properties.maxGridSize[0] <= 0 || Properties.maxThreadsPerBlock < 128) return nullptr;
		Context->MaxGridSize = static_cast<uint32_t>(Properties.maxGridSize[0]);
		if (cudaStreamCreateWithFlags(&Context->Stream, cudaStreamNonBlocking) != cudaSuccess) return nullptr;
		return Context.release();
	}

	void DestroyPBDCUDAContext(void* OpaqueContext)
	{
		PBDCUDAContext* Context = static_cast<PBDCUDAContext*>(OpaqueContext);
		if (!Context) return;
		{
			ScopedCUDADevice Device(Context->Device);
			if (Device.IsValid())
			{
				if (Context->Stream) (void)cudaStreamSynchronize(Context->Stream);
				if (Context->DeviceRows) (void)cudaFree(Context->DeviceRows);
				if (Context->DeviceContacts) (void)cudaFree(Context->DeviceContacts);
				if (Context->DeviceBodies) (void)cudaFree(Context->DeviceBodies);
				if (Context->Stream) (void)cudaStreamDestroy(Context->Stream);
			}
		}
		delete Context;
	}

	bool ExecutePBDCUDA(void* OpaqueContext, PBDBackendBody* Bodies, uint32_t BodyCount,
		PBDBackendContact* Contacts, uint32_t ContactCount, const uint32_t* RowIndices,
		const uint32_t* LayerOffsets, uint32_t LayerCount,
		const PBDBackendParameters& Parameters, PBDBackendStage Stage)
	{
		if (ContactCount == 0) return true;
		PBDCUDAContext* Context = static_cast<PBDCUDAContext*>(OpaqueContext);
		if (!Context || Context->Failed || !Bodies || BodyCount == 0 || !Contacts
			|| !RowIndices || !LayerOffsets || LayerCount == 0 || LayerCount > ContactCount
			|| LayerOffsets[0] != 0 || LayerOffsets[LayerCount] != ContactCount
			|| (Stage != PBDBackendStage::Position && Stage != PBDBackendStage::Reconstruct
				&& Stage != PBDBackendStage::Velocity)) return false;
		for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
		{
			if (LayerOffsets[Layer] >= LayerOffsets[Layer + 1]
				|| LayerOffsets[Layer + 1] > ContactCount) return false;
		}
		for (uint32_t Row = 0; Row < ContactCount; ++Row)
		{
			if (RowIndices[Row] >= ContactCount || Contacts[Row].Bodies.x >= BodyCount
				|| Contacts[Row].Bodies.y >= BodyCount) return false;
		}
		ScopedCUDADevice Device(Context->Device);
		if (!Device.IsValid()) return false;
		if (!GrowTransferBuffer(Context->DeviceBodies, Context->HostBodies, Context->BodyCapacity, BodyCount)
			|| !GrowTransferBuffer(Context->DeviceContacts, Context->HostContacts, Context->ContactCapacity, ContactCount)
			|| !GrowRowBuffer(*Context, ContactCount)) return AbortExecution(*Context);
		const size_t BodyBytes = size_t(BodyCount) * sizeof(PBDBackendBody);
		const size_t ContactBytes = size_t(ContactCount) * sizeof(PBDBackendContact);
		const size_t RowBytes = size_t(ContactCount) * sizeof(uint32_t);
		if (cudaMemcpyAsync(Context->DeviceBodies, Bodies, BodyBytes, cudaMemcpyHostToDevice, Context->Stream) != cudaSuccess
			|| cudaMemcpyAsync(Context->DeviceContacts, Contacts, ContactBytes, cudaMemcpyHostToDevice, Context->Stream) != cudaSuccess
			|| cudaMemcpyAsync(Context->DeviceRows, RowIndices, RowBytes, cudaMemcpyHostToDevice, Context->Stream) != cudaSuccess)
			return AbortExecution(*Context);
		for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
		{
			const uint32_t Begin = LayerOffsets[Layer], End = LayerOffsets[Layer + 1];
			const size_t RequiredBlocks = (size_t(End - Begin) + 127) / 128;
			const uint32_t Blocks = static_cast<uint32_t>(std::min(RequiredBlocks, size_t(Context->MaxGridSize)));
			// A stream orders complete layers without synchronizing the CPU between
			// kernels, preserving the original Gauss-Seidel body dependencies.
			SolveContactLayerKernel<<<Blocks, 128, 0, Context->Stream>>>(Context->DeviceBodies,
				Context->DeviceContacts, Context->DeviceRows, Begin, End, Parameters, Stage);
			if (cudaGetLastError() != cudaSuccess) return AbortExecution(*Context);
		}
		if (cudaMemcpyAsync(Context->HostBodies.get(), Context->DeviceBodies, BodyBytes, cudaMemcpyDeviceToHost, Context->Stream) != cudaSuccess
			|| cudaMemcpyAsync(Context->HostContacts.get(), Context->DeviceContacts, ContactBytes, cudaMemcpyDeviceToHost, Context->Stream) != cudaSuccess
			|| cudaStreamSynchronize(Context->Stream) != cudaSuccess) return AbortExecution(*Context);
		if (!ValidatePBDBackendOutput(Context->HostBodies.get(), BodyCount, Context->HostContacts.get(), ContactCount))
			return false;
		// The two caller arrays are untouched until every kernel, readback and
		// numerical check has succeeded, so CPU fallback can start from the input.
		std::memcpy(Bodies, Context->HostBodies.get(), BodyBytes);
		std::memcpy(Contacts, Context->HostContacts.get(), ContactBytes);
		return true;
	}
}

#endif

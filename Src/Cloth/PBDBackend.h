#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../RigidBodyDynamics/ConstraintSolver.h"

namespace Riemann
{
	struct PBDBackendVector
	{
		float x, y, z, w;
	};

	struct PBDBackendIndices
	{
		uint32_t x, y, z, w;
	};

	// Fixed-width records are shared with CUDA and HLSL structured buffers.
	struct PBDBackendBody
	{
		PBDBackendVector Position; // xyz: center of mass, w: effective inverse mass.
		PBDBackendVector Rotation; // Unit quaternion, xyzw.
		PBDBackendVector LinearVelocity; // xyz: velocity, w: maximum contact impulse.
		PBDBackendVector AngularVelocity;
		PBDBackendVector InverseInertia0, InverseInertia1, InverseInertia2; // Local rows.
		PBDBackendIndices Flags; // x: writable dynamic body, y: point velocity enabled, z: changed.
	};

	struct PBDBackendContact
	{
		PBDBackendIndices Bodies; // x/y: body indices. Body zero may represent the static world.
		PBDBackendVector AnchorA, AnchorB; // Body-local offsets relative to center of mass.
		PBDBackendVector Normal, Tangent, Binormal;
		PBDBackendVector Lambda; // Position, normal velocity, tangent, binormal multipliers.
		PBDBackendVector Material; // Restitution velocity, dynamic friction, static friction.
	};

	enum class PBDBackendStage : uint32_t
	{
		Position = 0,
		Reconstruct = 1,
		Velocity = 2,
	};

	struct PBDBackendParameters
	{
		float DeltaTime = 1.0f / 60.0f;
		float Alpha = 0.0f; // Contact compliance / dt^2; zero selects hard PBD.
		float PenetrationSlop = 0.001f;
		float MaxPositionCorrection = 0.2f;
		float MaxAngularCorrection = 0.2f;
	};

	struct PBDBackendStatistics
	{
		SolverBackend RequestedBackend = SolverBackend::CPU;
		SolverBackend Backend = SolverBackend::CPU;
		bool UsedFallback = false;
		uint64_t PositionDispatches = 0;
		uint64_t ReconstructionDispatches = 0;
		uint64_t VelocityDispatches = 0;
		uint64_t ContactRows = 0;
		uint32_t MaxBatchSize = 0;
		std::string Message;
	};

	bool IsPBDBackendAvailable(SolverBackend Backend);
	// Used by device implementations before committing any readback to the caller.
	bool ValidatePBDBackendOutput(const PBDBackendBody* Bodies, uint32_t BodyCount,
		const PBDBackendContact* Contacts, uint32_t ContactCount);

	class PBDBackend
	{
	public:
		PBDBackend() = default;
		~PBDBackend();
		PBDBackend(const PBDBackend&) = delete;
		PBDBackend& operator=(const PBDBackend&) = delete;
		bool Configure(SolverBackend Backend, bool AllowFallback);
		// Shared writable bodies retain original row order. Independent rows run together.
		// Failed device execution never partially commits host records; fallback uses the input.
		bool Execute(std::vector<PBDBackendBody>& Bodies, std::vector<PBDBackendContact>& Contacts,
			const PBDBackendParameters& Parameters, PBDBackendStage Stage);
		const PBDBackendStatistics& GetStatistics() const { return m_Statistics; }

	private:
		void ReleaseContext();
		bool m_AllowFallback = true;
		bool m_Ready = true;
		void* m_Context = nullptr;
		PBDBackendStatistics m_Statistics;
		std::vector<uint32_t> m_RowIndices, m_LayerOffsets;
		std::vector<PBDBackendBody> m_WorkBodies;
		std::vector<PBDBackendContact> m_WorkContacts;
	};

	// Internal device entry points. The common dispatcher validates records and creates
	// a complete row permutation and conflict-free dependency layers before dispatch.
	bool IsPBDComputeShaderBackendAvailable();
	void* CreatePBDComputeShaderContext();
	void DestroyPBDComputeShaderContext(void* Context);
	bool ExecutePBDComputeShader(void* Context, PBDBackendBody* Bodies, uint32_t BodyCount,
		PBDBackendContact* Contacts, uint32_t ContactCount, const uint32_t* RowIndices,
		const uint32_t* LayerOffsets, uint32_t LayerCount,
		const PBDBackendParameters& Parameters, PBDBackendStage Stage);
#if defined(RIEMANN_ENABLE_CUDA)
	bool IsPBDCUDAAvailable();
	void* CreatePBDCUDAContext();
	void DestroyPBDCUDAContext(void* Context);
	bool ExecutePBDCUDA(void* Context, PBDBackendBody* Bodies, uint32_t BodyCount,
		PBDBackendContact* Contacts, uint32_t ContactCount, const uint32_t* RowIndices,
		const uint32_t* LayerOffsets, uint32_t LayerCount,
		const PBDBackendParameters& Parameters, PBDBackendStage Stage);
#endif

	static_assert(sizeof(PBDBackendVector) == 16, "PBD vector layout must match device shaders");
	static_assert(sizeof(PBDBackendBody) == 128, "PBD body layout must match device shaders");
	static_assert(sizeof(PBDBackendContact) == 128, "PBD contact layout must match device shaders");
}

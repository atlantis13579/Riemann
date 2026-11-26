#pragma once

#include <memory>
#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct AerodynamicsSimulationSettings;
	struct AerodynamicsGridRegion { Vector3i Min, Max; };
	// K maps pressure impulses to mass-scaled surface velocity. The pressure
	// operator is the fluid Laplacian plus K^T K. CellIndices compact the nonempty
	// rows of K^T; inactive grid cells do not need host storage or transfer.
	struct AerodynamicsPressureEntry { int Index; float Weight; };
	struct AerodynamicsPressureFace
	{
		int Cell, Axis, Nodes[3];
		float Weights[3];
	};
	struct AerodynamicsPressureSystem
	{
		std::vector<int> RowOffsets, CellOffsets, CellIndices;
		std::vector<AerodynamicsPressureEntry> RowEntries, CellEntries;
		std::vector<AerodynamicsPressureFace> Faces;
	};
	// Device kernels for the incompressible algorithm. Barotropic gas uses FluidBackend.
	class AerodynamicsGPUBackend
	{
	public:
		virtual ~AerodynamicsGPUBackend() {}
		static std::unique_ptr<AerodynamicsGPUBackend> CreateComputeShader(const AerodynamicsSimulationSettings& Settings,
			const std::vector<unsigned char>& Solid);
		static std::unique_ptr<AerodynamicsGPUBackend> CreateMetal(const AerodynamicsSimulationSettings& Settings,
			const std::vector<unsigned char>& Solid);
		static std::unique_ptr<AerodynamicsGPUBackend> CreateCUDA(const AerodynamicsSimulationSettings& Settings,
			const std::vector<unsigned char>& Solid);
		virtual void Reset() = 0;
		virtual bool UpdateObstacles(const std::vector<unsigned char>& Solid, const std::vector<Vector3>& Velocity, const AerodynamicsGridRegion* Region = nullptr) = 0;
		virtual void Step(float Dt, float Time, bool Transport = true, bool ProjectPressure = true) = 0;
		virtual void TransportScalars() = 0;
		virtual void ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Change) = 0;
		virtual bool Readback(std::vector<float>& Density, std::vector<float>& Temperature) = 0;
		virtual bool ReadVelocity(std::vector<Vector3>& Velocity) const = 0;
		virtual bool ReadPressure(std::vector<float>& Pressure) const = 0;
		virtual bool ProjectSurfaceMass(const AerodynamicsPressureSystem& System, std::vector<float>& Response) = 0;
		virtual float GetMaxRate() const = 0;
	};
}

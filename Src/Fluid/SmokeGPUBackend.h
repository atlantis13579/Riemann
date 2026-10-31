#pragma once

#include <memory>
#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct SmokeSimulationSettings;
	class SmokeGPUBackend
	{
	public:
		virtual ~SmokeGPUBackend() {}
		static std::unique_ptr<SmokeGPUBackend> CreateComputeShader(const SmokeSimulationSettings& Settings,
			const std::vector<unsigned char>& Solid);
		static std::unique_ptr<SmokeGPUBackend> CreateCUDA(const SmokeSimulationSettings& Settings,
			const std::vector<unsigned char>& Solid);
		virtual void Reset() = 0;
		virtual bool UpdateObstacles(const std::vector<unsigned char>& Solid, const std::vector<Vector3>& Velocity) = 0;
		virtual void Step(float Dt, float Time) = 0;
		virtual void ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Change) = 0;
		virtual bool Readback(std::vector<float>& Density, std::vector<float>& Temperature) = 0;
		virtual bool ReadVelocity(std::vector<Vector3>& Velocity) const = 0;
		virtual bool ReadPressure(std::vector<float>& Pressure) const = 0;
		virtual float GetMaxRate() const = 0;
	};
}

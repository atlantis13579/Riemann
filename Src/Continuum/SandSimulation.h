#pragma once

#include "SnowSimulation.h"

namespace Riemann
{
	struct SandSettings
	{
		float ReposeAngle = 32, Viscosity = 0;
	};
	class SandGPUBackend;
	// Depth-integrated dry sand: conservative transport with a static friction threshold.
	// The 8 m square tray holds a 1.5 m column, without overhangs, airborne grains or elastic strain.
	class SandSimulation
	{
	public:
		SandSimulation();
		~SandSimulation();
		void Reset(SnowQuality Quality = SnowQuality::Realtime);
		void Step(float DeltaTime);
		bool SetBackend(SnowBackend Backend);
		static bool IsBackendAvailable(SnowBackend Backend);
		SnowBackend GetBackend() const { return m_Backend; }
		const std::string& GetBackendError() const { return m_BackendError; }
		SandSettings& GetSettings() { return m_Settings; }
		const SandSettings& GetSettings() const { return m_Settings; }
		SnowCapsule& GetCapsule() { return m_Capsule; }
		const SnowCapsule& GetCapsule() const { return m_Capsule; }
		void SetCapsuleTarget(float X, float Z);
		const std::vector<float>& GetHeights() const { return m_Heights; }
		int GetResolution() const { return m_Resolution; }
		float GetCellSize() const { return 8.0f / m_Resolution; }
		double GetVolume() const;
	private:
		void Displace();
		void RelaxCPU(int Iterations, float Rate, bool Simd);
		SandSettings m_Settings;
		SnowCapsule m_Capsule;
		MPM::Vector m_Target;
		std::vector<float> m_Heights, m_Limits, m_Flux, m_Next, m_Weights;
		int m_Resolution = 64;
		SnowBackend m_Backend = SnowBackend::CPU;
		std::unique_ptr<SandGPUBackend> m_GPU;
		std::string m_BackendError;
	};
}

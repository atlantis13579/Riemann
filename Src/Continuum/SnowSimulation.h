#pragma once

#include <memory>
#include <string>
#include <vector>
#include "MPMMath.h"

namespace Riemann
{
	enum class SnowBackend { CPU, SIMD, CUDA, ComputeShader, Metal };
	enum class MPMTransfer { MPM, APIC, MLS };
	enum class SnowQuality { Realtime, High };
	struct SnowParticle
	{
		MPM::Vector Position, Velocity;
		MPM::Matrix Elastic = MPM::Matrix::Identity(), Affine;
		float PlasticVolume = 1, Volume = 0, Mass = 0, LogVolumeCorrection = 0;
	};
	struct SnowCapsule
	{
		MPM::Vector Center = {0,0.7f,0}, Velocity;
		float HalfLength = 0.5f, Radius = 0.45f, Friction = 0.55f;
		bool Enabled = true;
	};
	struct SnowSettings
	{
		MPM::Vector BoundsMin = {-4.5f,-0.9f,-4.5f}, BoundsMax = {4.5f,4.5f,4.5f};
		MPM::Vector Gravity = {0,-9.81f,0};
		float CellSize = 0.6f, Density = 200, FlipRatio = 0.95f;
		float GroundFriction = 0;
		float FrameBudgetMilliseconds = 0;
		int MaximumSubsteps = 32;
		MPMTransfer Transfer = MPMTransfer::MLS;
		MPM::Material Material;
	};
	struct SnowStatistics
	{
		double TotalMass = 0;
		MPM::Vector Momentum;
		float MaximumSpeed = 0, PendingTime = 0, SimulatedTime = 0;
		int Substeps = 0;
		bool TimeLimited = false;
	};
	class SnowGPUBackend;
	class SnowSimulation
	{
	public:
		explicit SnowSimulation(const SnowSettings& Settings = {});
		~SnowSimulation();
		void Reset(SnowQuality Quality = SnowQuality::Realtime);
		void Clear();
		void AddBox(MPM::Vector Minimum, MPM::Vector Maximum, float Spacing, MPM::Vector Velocity = {});
		void Step(float DeltaTime);
		bool SetBackend(SnowBackend Backend);
		static bool IsBackendAvailable(SnowBackend Backend);
		void SetCapsuleTarget(float X, float Z) { m_CapsuleTarget = {X,0,Z}; }
		SnowSettings& GetSettings() { return m_Settings; }
		const SnowSettings& GetSettings() const { return m_Settings; }
		SnowCapsule& GetCapsule() { return m_Capsule; }
		const SnowCapsule& GetCapsule() const { return m_Capsule; }
		const std::vector<SnowParticle>& GetParticles() const { return m_Particles; }
		const SnowStatistics& GetStatistics() const { return m_Statistics; }
		SnowBackend GetBackend() const { return m_Backend; }
		const std::string& GetBackendError() const { return m_BackendError; }
	private:
		void PrepareGrid();
		void GatherStatistics();
		void StepCPU(float Dt, bool Simd);
		struct GridNode { MPM::Vector Momentum, Force, OldVelocity; float Mass = 0; };
		SnowSettings m_Settings;
		SnowCapsule m_Capsule;
		MPM::Vector m_CapsuleTarget;
		SnowStatistics m_Statistics;
		std::vector<SnowParticle> m_Particles;
		std::vector<GridNode> m_Grid;
		int m_Width = 0, m_Height = 0, m_Depth = 0;
		float m_PendingTime = 0;
		SnowBackend m_Backend = SnowBackend::CPU;
		std::unique_ptr<SnowGPUBackend> m_GPU;
		std::string m_BackendError;
	};
}

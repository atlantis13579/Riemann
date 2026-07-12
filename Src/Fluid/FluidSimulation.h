#pragma once

#include <cstdint>
#include <vector>

#include "../Maths/Vector3.h"
#include "FluidBackend.h"

namespace Riemann
{
	enum class FluidSimulationMethod : uint8_t
	{
		Eulerian = 0,
		Lagrangian = 1,
	};

	struct FluidSimulationSettings
	{
		FluidSimulationSettings();

		FluidSimulationMethod Method;
		FluidBackend Backend;
		bool AllowBackendFallback;
		Vector3 BoundsMin;
		Vector3 BoundsMax;
		Vector3i Resolution;
		Vector3 Gravity;
		float RestDensity;
		float DynamicViscosity;
		float PressureStiffness;
		float ParticleSpacing;
		float SmoothingRadius;
		float BoundaryRestitution;
		float BoundaryFriction;
		float MaxTimeStep;
		int MaxSubsteps;
		int PressureIterations;
	};

	struct FluidParticle
	{
		FluidParticle();
		FluidParticle(const Vector3& InPosition, const Vector3& InVelocity, float InMass);

		Vector3 Position;
		Vector3 Velocity;
		Vector3 Acceleration;
		float Mass;
		float Density;
		float Pressure;
	};

	// Incompressible grid flow and weakly-compressible SPH share this facade so
	// callers can change representations without changing coupling code.
	class FluidSimulation
	{
	public:
		FluidSimulation();
		explicit FluidSimulation(const FluidSimulationSettings& Settings);

		bool Initialize(const FluidSimulationSettings& Settings);
		void Reset();
		bool SetMethod(FluidSimulationMethod Method, bool PreserveState = true);
		void Step(float DeltaTime);

		void AddFluidBox(const Vector3& Min, const Vector3& Max,
			const Vector3& InitialVelocity = Vector3::Zero());
		void AddFluidSphere(const Vector3& Center, float Radius,
			const Vector3& InitialVelocity = Vector3::Zero());
		bool ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Impulse);

		float SampleDensity(const Vector3& Position) const;
		Vector3 SampleVelocity(const Vector3& Position) const;
		float GetTotalMass() const;
		float GetMaxDivergence() const;
		bool BuildDensityField(std::vector<float>* DensityField) const;

		bool IsInitialized() const { return m_Initialized; }
		FluidSimulationMethod GetMethod() const { return m_Settings.Method; }
		FluidBackend GetBackend() const { return m_BackendResult.Backend; }
		FluidBackend GetRequestedBackend() const { return m_BackendResult.RequestedBackend; }
		bool UsedBackendFallback() const { return m_BackendResult.UsedFallback; }
		const FluidSimulationSettings& GetSettings() const { return m_Settings; }
		const Vector3& GetCellSize() const { return m_CellSize; }
		const std::vector<float>& GetGridDensity() const { return m_GridDensity; }
		const std::vector<Vector3>& GetGridVelocity() const { return m_GridVelocity; }
		const std::vector<FluidParticle>& GetParticles() const { return m_Particles; }

	private:
		int GridIndex(int X, int Y, int Z) const;
		bool IsGridCoordinateValid(int X, int Y, int Z) const;
		Vector3 GridCellCenter(int X, int Y, int Z) const;
		Vector3 GridFaceCenter(int X, int Y, int Z, int Component) const;
		float SampleGridScalar(const std::vector<float>& Field, const Vector3& Position) const;
		Vector3 SampleGridVector(const std::vector<Vector3>& Field, const Vector3& Position) const;
		float SampleGridVelocityComponent(const std::vector<Vector3>& Field,
			const Vector3& Position, int Component) const;
		bool IsFaceFluid(int X, int Y, int Z, int Component, float ActiveThreshold) const;

		void StepEulerian(float DeltaTime);
		void AdvectEulerian(float DeltaTime);
		void ApplyEulerianForces(float DeltaTime);
		void ProjectEulerian(float DeltaTime);
		void EnforceEulerianBoundaries();
		void AdvectEulerianDensityConservative(float DeltaTime,
			const std::vector<float>& OldDensity, const std::vector<Vector3>& OldVelocity);

		void StepLagrangian(float DeltaTime);
		void ComputeParticleDensities();
		void ComputeParticleAccelerations();
		void EnforceParticleBoundary(FluidParticle& Particle) const;

		void ConvertEulerianToLagrangian();
		void ConvertLagrangianToEulerian();
		void RebuildParticleBuckets();
		uint64_t ParticleBucketKey(int X, int Y, int Z) const;

		FluidSimulationSettings m_Settings;
		Vector3 m_CellSize;
		bool m_Initialized;
		float m_TimeRemainder;
		FluidBackendResult m_BackendResult;
		std::vector<float> m_GridDensity;
		std::vector<Vector3> m_GridVelocity;
		std::vector<float> m_Pressure;
		std::vector<float> m_Divergence;
		std::vector<FluidParticle> m_Particles;

		struct ParticleBucket
		{
			uint64_t Key;
			std::vector<int> Indices;
		};
		std::vector<ParticleBucket> m_ParticleBuckets;
	};
}

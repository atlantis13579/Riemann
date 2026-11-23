#pragma once

#include "AerodynamicsTypes.h"
#include "ParticleBucketGrid.h"

namespace Riemann
{
	class AerodynamicsBarotropicSolver
	{
	public:
		AerodynamicsBarotropicSolver();
		explicit AerodynamicsBarotropicSolver(const AerodynamicsSimulationSettings& Settings);

		bool Initialize(const AerodynamicsSimulationSettings& Settings);
		void Reset();
		bool SetMethod(FluidSimulationMethod Method, bool PreserveState = true);
		void Step(float DeltaTime);

		void SetDensityBox(const Vector3& Min, const Vector3& Max, float Density);
		void SetDensitySphere(const Vector3& Center, float Radius, float Density);
		void SetVelocityBox(const Vector3& Min, const Vector3& Max, const Vector3& Velocity);
		bool ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Impulse);

		AerodynamicSample Sample(const Vector3& Position) const;
		float GetTotalMass() const;
		Vector3 GetTotalMomentum() const;
		bool BuildDensityField(std::vector<float>* DensityField) const;

		bool IsInitialized() const { return m_Initialized; }
		FluidSimulationMethod GetMethod() const { return m_Settings.Method; }
		FluidBackend GetBackend() const { return m_BackendResult.Backend; }
		FluidBackend GetRequestedBackend() const { return m_BackendResult.RequestedBackend; }
		bool UsedBackendFallback() const { return m_BackendResult.UsedFallback; }
		const AerodynamicsSimulationSettings& GetSettings() const { return m_Settings; }
		const std::vector<float>& GetGridDensity() const { return m_GridDensity; }
		const std::vector<float>& GetGridPressure() const { return m_GridPressure; }
		const std::vector<Vector3>& GetGridVelocity() const { return m_GridVelocity; }
		const std::vector<AirParticle>& GetParticles() const { return m_Particles; }
		const Vector3& GetCellSize() const { return m_CellSize; }
		double GetTime() const { return m_Time; }
		double GetPendingTime() const { return m_TimeRemainder; }

	private:
		int GridIndex(int X, int Y, int Z) const;
		bool IsGridCoordinateValid(int X, int Y, int Z) const;
		Vector3 GridCellCenter(int X, int Y, int Z) const;
		Vector3 GridFaceCenter(int X, int Y, int Z, int Component) const;
		float SampleGridScalar(const std::vector<float>& Field, const Vector3& Position) const;
		float SampleGridVelocityComponent(const std::vector<Vector3>& Field,
			const Vector3& Position, int Component) const;
		Vector3 SampleGridVelocity(const std::vector<Vector3>& Field, const Vector3& Position) const;

		void StepEulerian(float DeltaTime);
		void AdvectEulerianDensity(float DeltaTime, const std::vector<float>& OldDensity,
			const std::vector<Vector3>& OldVelocity);
		void AdvectEulerianVelocity(float DeltaTime, const std::vector<Vector3>& OldVelocity);
		void ApplyEulerianPressureAndForces(float DeltaTime);
		void UpdateEulerianPressure();
		void EnforceEulerianBoundaries();
		float ComputeStableTimeStep() const;

		void SeedAmbientParticles();
		void InitializeParticleNormalizations();
		void RebuildReferenceParticleBuckets();
		float ComputeReferenceDensityAt(const Vector3& Position) const;
		void StepLagrangian(float DeltaTime);
		void RebuildParticleBuckets();
		void ComputeParticleDensities();
		void ComputeParticleAccelerations();
		void EnforceParticleBoundary(AirParticle& Particle) const;
		uint64_t ParticleBucketKey(int X, int Y, int Z) const;

		bool ConvertEulerianToLagrangian();
		bool ConvertLagrangianToEulerian();
		float SampleLagrangianDensity(const Vector3& Position) const;
		Vector3 SampleLagrangianVelocity(const Vector3& Position) const;
		float SamplePressureOnly(const Vector3& Position) const;
		float PressureFromDensity(float Density) const;

		AerodynamicsSimulationSettings m_Settings;
		Vector3 m_CellSize;
		bool m_Initialized;
		double m_TimeRemainder;
		double m_Time = 0;
		FluidBackendResult m_BackendResult;
		std::vector<float> m_GridDensity;
		std::vector<float> m_GridPressure;
		std::vector<Vector3> m_GridVelocity;
		std::vector<float> m_DensityScratch;
		std::vector<Vector3> m_VelocityScratch;
		std::vector<Vector3> m_VelocityOutput;
		std::vector<Vector3> m_VelocityDelta;
		std::vector<AirParticle> m_Particles;

		ParticleBucketGrid m_ParticleBuckets;
		ParticleBucketGrid m_ReferenceParticleBuckets;

	};
}

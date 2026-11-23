#pragma once

#include <memory>
#include "AerodynamicsTypes.h"

namespace Riemann
{
	class AerodynamicsBarotropicSolver;
	class AerodynamicsIncompressibleSolver;

	// One entry point for barotropic gas and incompressible tracer transport.
	// Select an algorithm at initialization; switching algorithms resets state.
	class AerodynamicsSimulation
	{
	public:
		AerodynamicsSimulation();
		explicit AerodynamicsSimulation(const AerodynamicsSimulationSettings& Settings);
		~AerodynamicsSimulation();
		AerodynamicsSimulation(const AerodynamicsSimulation&) = delete;
		AerodynamicsSimulation& operator=(const AerodynamicsSimulation&) = delete;

		bool Initialize(const AerodynamicsSimulationSettings& Settings);
		void Reset();
		void Step(float DeltaTime);
		// World coupling consumes the complete interval and averages all pressure impulses.
		bool StepSynchronized(float DeltaTime);
		float GetStableTimeStep() const;
		// Incompressible transport supports Eulerian discretization only.
		bool SetMethod(FluidSimulationMethod Method, bool PreserveState = true);
		// Physical momentum input (N s), distributed over the affected air.
		bool ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Impulse);
		// Smooth velocity brush (m/s) for incompressible transport; returns false otherwise.
		bool ApplyVelocityImpulse(const Vector3& Position, float Radius, const Vector3& VelocityChange);

		// Barotropic initial-condition edits; unsupported algorithms throw logic_error.
		void SetDensityBox(const Vector3& Min, const Vector3& Max, float Density);
		void SetDensitySphere(const Vector3& Center, float Radius, float Density);
		void SetVelocityBox(const Vector3& Min, const Vector3& Max, const Vector3& Velocity);
		AerodynamicSample Sample(const Vector3& Position) const;
		float GetTotalMass() const;
		Vector3 GetTotalMomentum() const;
		// Always air mass density, never tracer concentration.
		bool BuildDensityField(std::vector<float>* DensityField) const;
		const std::vector<float>& GetGridDensity() const;
		const std::vector<float>& GetGridPressure() const;
		const std::vector<Vector3>& GetGridVelocity() const;
		const std::vector<AirParticle>& GetParticles() const;
		const std::vector<float>& GetTracerDensity() const;
		const std::vector<float>& GetTemperature() const;

		bool SupportsMovingObstacles() const { return GetAlgorithm() == AerodynamicsAlgorithm::Incompressible; }
		bool HasTracerField() const { return GetAlgorithm() == AerodynamicsAlgorithm::Incompressible; }
		bool SetRigidObstacles(const std::vector<AerodynamicsObstacle>& Obstacles);
		bool SetMovingBoundaries(const std::vector<AerodynamicsObstacle>& Obstacles, const std::vector<AerodynamicsSurfaceMesh>& Surfaces);
		bool ComputeSurfaceLoads(std::vector<std::vector<Vector3>>& Forces) const;
		bool ComputeRigidLoads(float AirDensity, std::vector<AerodynamicsRigidLoad>& Loads) const;
		const std::vector<unsigned char>& GetSolidCells() const;
		const std::vector<Vector3>& GetBoundaryVelocity() const;
		float GetDivergenceRMS() const;
		AerodynamicForceResult ComputeSurfaceForce(const Vector3& CenterOfMass,
			const std::vector<AerodynamicSurfaceSample>& SurfaceSamples, float DragCoefficient = 1.0f) const;
		AerodynamicForceResult ComputeSphereForce(const Vector3& Center, float Radius,
			const Vector3& LinearVelocity, float DragCoefficient = 0.47f) const;

		bool IsInitialized() const;
		AerodynamicsAlgorithm GetAlgorithm() const { return m_Settings.Algorithm; }
		FluidSimulationMethod GetMethod() const { return m_Settings.Method; }
		FluidBackend GetBackend() const;
		FluidBackend GetRequestedBackend() const { return m_Settings.Backend; }
		bool UsedBackendFallback() const { return GetBackend() != GetRequestedBackend(); }
		const AerodynamicsSimulationSettings& GetSettings() const { return m_Settings; }
		const Vector3& GetCellSize() const { return m_CellSize; }
		double GetTime() const;
		double GetPendingTime() const;

	private:
		AerodynamicsSimulationSettings m_Settings;
		Vector3 m_CellSize = Vector3::Zero();
		std::unique_ptr<AerodynamicsBarotropicSolver> m_Barotropic;
		std::unique_ptr<AerodynamicsIncompressibleSolver> m_Incompressible;
	};
}

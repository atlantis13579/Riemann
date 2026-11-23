#pragma once

#include <array>
#include <memory>
#include "AerodynamicsTypes.h"

namespace Riemann
{
	class AerodynamicsGPUBackend;
	// Incompressible 3D smoke: MAC velocities, red-black SOR projection,
	// RK2 characteristics, limited MacCormack scalar transport and confinement.
	// Smoke concentration and temperature are independent of the air density.
	// Conceptual reference: https://github.com/SebLague/Smoke-Simulation
	class AerodynamicsIncompressibleSolver
	{
	public:
		AerodynamicsIncompressibleSolver();
		~AerodynamicsIncompressibleSolver();
		bool Initialize(const AerodynamicsSimulationSettings& Settings);
		void Reset();
		void Step(float DeltaTime);
		bool StepSynchronized(float DeltaTime);
		float GetStableTimeStep() const;
		bool ApplyVelocityImpulse(const Vector3& Position, float Radius, const Vector3& VelocityChange);
		// Replaces moving solids while retaining the scene's configured obstacles.
		bool SetRigidObstacles(const std::vector<AerodynamicsObstacle>& Obstacles);
		bool SetMovingBoundaries(const std::vector<AerodynamicsObstacle>& Obstacles, const std::vector<AerodynamicsSurfaceMesh>& Surfaces);
		bool ComputeSurfaceLoads(std::vector<std::vector<Vector3>>& Forces) const;
		// Pressure traction on voxel faces, in SetRigidObstacles order; torque is about each obstacle center.
		bool ComputeRigidLoads(float AirDensity, std::vector<AerodynamicsRigidLoad>& Loads) const;
		const std::vector<Vector3>& GetBoundaryVelocity() const { return m_BoundaryVelocity; }
		bool IsInitialized() const { return m_Initialized; }
		const AerodynamicsSimulationSettings& GetSettings() const { return m_Settings; }
		const std::vector<float>& GetTracerDensity() const { return m_Density; }
		const std::vector<float>& GetTemperature() const { return m_Temperature; }
		const std::vector<Vector3>& GetVelocity() const;
		FluidBackend GetBackend() const { return m_Backend; }
		const std::vector<unsigned char>& GetSolidCells() const { return m_Solid; }
		float GetDivergenceRMS() const;
		double GetTime() const { return m_Time; }
		double GetPendingTime() const { return m_PendingTime; }

		AerodynamicSample Sample(const Vector3& Position) const;
		const std::vector<float>& GetGridDensity() const;
		const std::vector<float>& GetGridPressure() const;
		const Vector3& GetCellSize() const { return m_CellSize; }
		float GetImpulseMass(const Vector3& Position, float Radius) const;
	private:
		int Index(int X, int Y, int Z) const;
		Vector3 GridPosition(int I) const;
		float Scalar(const std::vector<float>& Field, const Vector3& P, float* Min = nullptr, float* Max = nullptr) const;
		float Face(const std::vector<Vector3>& Field, const Vector3& P, int Axis) const;
		Vector3 Velocity(const std::vector<Vector3>& Field, const Vector3& P) const;
		Vector3 Trace(const Vector3& P, float Dt) const;
		bool Solid(const Vector3& P) const;
		void Boundaries();
		void Emit(float Dt);
		void AddForces(float Dt);
		void Project();
		void AdvectScalars(float Dt);
		float Divergence(int I) const;
		void RebuildObstacles(const std::vector<AerodynamicsObstacle>& Obstacles);
		bool Advance(float DeltaTime, bool Synchronized);
		bool ReadLoads(std::vector<AerodynamicsRigidLoad>& Rigid, std::vector<std::vector<Vector3>>& Surface) const;
		bool ProjectSurfaceMass();

		AerodynamicsSimulationSettings m_Settings;
		Vector3 m_CellSize;
		bool m_Initialized = false;
		double m_Time = 0;
		double m_PendingTime = 0;
		std::vector<float> m_Density, m_Temperature, m_Pressure, m_Divergence;
		std::vector<float> m_DensityForward, m_TemperatureForward, m_DensityNext, m_TemperatureNext;
		mutable std::vector<Vector3> m_Velocity;
		std::vector<Vector3> m_VelocityNext, m_Curl, m_Force;
		std::vector<unsigned char> m_Solid;
		std::vector<std::array<int, 6>> m_Neighbors;
		std::vector<float> m_Diagonal;
		std::vector<Vector3> m_BoundaryVelocity;
		std::vector<int> m_CellOwners, m_CellTriangles, m_OccupiedCells;
		std::vector<float> m_SurfaceDistances;
		Vector3i m_UpdateMin, m_UpdateMax;
		std::vector<AerodynamicsObstacle> m_Obstacles;
		std::vector<AerodynamicsSurfaceMesh> m_Surfaces;
		struct BoundaryFace { int Cell, Owner, Axis; float Sign; Vector3 Position; int Triangle = -1; Vector3 Weights = Vector3::Zero(); };
		std::vector<BoundaryFace> m_BoundaryFaces;
		mutable std::vector<float> m_PressureReadback;
		mutable bool m_PressureDirty = false;
		float m_LastStepDt = 0;
		bool m_HasAveragedLoads = false;
		std::vector<AerodynamicsRigidLoad> m_AveragedRigidLoads;
		std::vector<std::vector<Vector3>> m_AveragedSurfaceLoads;
		std::unique_ptr<AerodynamicsGPUBackend> m_GPU;
		FluidBackend m_Backend = FluidBackend::CPU;
		mutable std::vector<float> m_AirDensity, m_AbsolutePressure;
		mutable bool m_VelocityDirty = false;
	};
}

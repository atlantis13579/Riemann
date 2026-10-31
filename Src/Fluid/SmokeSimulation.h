#pragma once

#include <array>
#include <memory>
#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Quaternion.h"

namespace Riemann
{
	class SmokeGPUBackend;
	enum class SmokeBackend { CPU, CUDA, ComputeShader };
	struct SmokeEmitter
	{
		Vector3 Position = Vector3(0, 0.55f, 0);
		Vector3 Velocity = Vector3(0, 1.8f, 0);
		float Radius = 0.32f;
		float Density = 1.4f;
		float Temperature = 1.0f;
	};

	struct SmokeObstacle
	{
		enum class Shape { Sphere, Box };
		Shape Type = Shape::Sphere;
		Vector3 Center = Vector3::Zero();
		float Radius = 0.5f;
		Vector3 HalfExtent = Vector3(0.5f);
		Quaternion Rotation = Quaternion::One();
		Vector3 LinearVelocity = Vector3::Zero();
		Vector3 AngularVelocity = Vector3::Zero();
	};

	struct SmokeSimulationSettings
	{
		SmokeBackend Backend = SmokeBackend::CPU;
		bool AllowBackendFallback = false;
		Vector3 BoundsMin = Vector3(-2.4f, 0, -2.4f);
		Vector3 BoundsMax = Vector3(2.4f, 8, 2.4f);
		Vector3i Resolution = Vector3i(48, 80, 48);
		int PressureIterations = 40;
		float Buoyancy = 3.0f;
		float SmokeWeight = 0.15f;
		float Vorticity = 2.5f;
		float DensityDissipation = 0.10f;
		float Cooling = 0.45f;
		float MaxTimeStep = 1.0f / 60.0f;
		int MaxSubsteps = 8;
		std::vector<SmokeEmitter> Emitters;
		std::vector<SmokeObstacle> Obstacles;
	};

	struct SmokeRigidLoad
	{
		Vector3 Force = Vector3::Zero();
		Vector3 Torque = Vector3::Zero();
	};

	// Incompressible 3D smoke: MAC velocities, red-black SOR projection,
	// RK2 characteristics, limited MacCormack scalar transport and confinement.
	// Smoke concentration and temperature are independent of the air density.
	// Conceptual reference: https://github.com/SebLague/Smoke-Simulation
	class SmokeSimulation
	{
	public:
		SmokeSimulation();
		~SmokeSimulation();
		bool Initialize(const SmokeSimulationSettings& Settings);
		void Reset();
		void Step(float DeltaTime);
		bool ApplyImpulse(const Vector3& Position, float Radius, const Vector3& VelocityChange);
		// Replaces moving solids while retaining the scene's configured obstacles.
		bool SetRigidObstacles(const std::vector<SmokeObstacle>& Obstacles);
		// Pressure traction on voxel faces, in SetRigidObstacles order; torque is about each obstacle center.
		bool ComputeRigidLoads(float AirDensity, std::vector<SmokeRigidLoad>& Loads) const;
		const std::vector<Vector3>& GetBoundaryVelocity() const { return m_BoundaryVelocity; }
		bool IsInitialized() const { return m_Initialized; }
		const SmokeSimulationSettings& GetSettings() const { return m_Settings; }
		const std::vector<float>& GetDensity() const { return m_Density; }
		const std::vector<float>& GetTemperature() const { return m_Temperature; }
		const std::vector<Vector3>& GetVelocity() const;
		SmokeBackend GetBackend() const { return m_Backend; }
		const std::vector<unsigned char>& GetSolidCells() const { return m_Solid; }
		float GetDivergenceRMS() const;
		double GetTime() const { return m_Time; }
		double GetPendingTime() const { return m_PendingTime; }

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
		void RebuildObstacles(const std::vector<SmokeObstacle>& Obstacles);

		SmokeSimulationSettings m_Settings;
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
		std::vector<SmokeObstacle> m_Obstacles;
		struct BoundaryFace { int Cell, Owner, Axis; float Sign; Vector3 Position; };
		std::vector<BoundaryFace> m_BoundaryFaces;
		mutable std::vector<float> m_PressureReadback;
		mutable bool m_PressureDirty = false;
		float m_LastStepDt = 0;
		std::unique_ptr<SmokeGPUBackend> m_GPU;
		SmokeBackend m_Backend = SmokeBackend::CPU;
		mutable bool m_VelocityDirty = false;
	};
}

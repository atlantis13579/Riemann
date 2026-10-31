#include "SmokeSimulation.h"
#include "FluidParallelFor.h"
#include "SmokeGPUBackend.h"
#include "MACGridSampling.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	SmokeSimulation::SmokeSimulation() = default;
	SmokeSimulation::~SmokeSimulation() = default;
#if !defined(RIEMANN_ENABLE_CUDA)
	std::unique_ptr<SmokeGPUBackend> SmokeGPUBackend::CreateCUDA(const SmokeSimulationSettings&,
		const std::vector<unsigned char>&) { return nullptr; }
#endif
	namespace
	{
		float Clamp(float V, float A, float B) { return std::max(A, std::min(V, B)); }
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		Vector3 Divide(const Vector3& A, const Vector3& B) { return Vector3(A.x / B.x, A.y / B.y, A.z / B.z); }
		bool ValidObstacle(const SmokeObstacle& O)
		{
			return Finite(O.Center) && Finite(O.LinearVelocity) && Finite(O.AngularVelocity)
				&& Finite(O.HalfExtent) && O.HalfExtent.x > 0 && O.HalfExtent.y > 0 && O.HalfExtent.z > 0
				&& std::isfinite(O.Radius) && O.Radius > 0 && std::isfinite(O.Rotation.SquareLength())
				&& std::fabs(O.Rotation.SquareLength() - 1.0f) < 1e-3f
				&& (O.Type == SmokeObstacle::Shape::Sphere || O.Type == SmokeObstacle::Shape::Box);
		}
	}

	bool SmokeSimulation::Initialize(const SmokeSimulationSettings& Settings)
	{
		m_Initialized = false;
		m_GPU.reset(); m_Backend = SmokeBackend::CPU; m_VelocityDirty = false;
		const Vector3 Size = Settings.BoundsMax - Settings.BoundsMin;
		if (!Finite(Settings.BoundsMin) || !Finite(Settings.BoundsMax) || !Finite(Size)) return false;
		size_t Count = 1;
		for (int A = 0; A < 3; ++A)
		{
			if (Size[A] <= 0 || Settings.Resolution[A] < 4 || Settings.Resolution[A] > 256) return false;
			Count *= Settings.Resolution[A];
		}
		if (Count > 4 * 1024 * 1024 || Settings.PressureIterations < 1 || Settings.PressureIterations > 500
			|| !std::isfinite(Settings.MaxTimeStep) || Settings.MaxTimeStep <= 0 || Settings.MaxTimeStep > 0.1f
			|| Settings.MaxSubsteps < 1 || Settings.MaxSubsteps > 128) return false;
		const float Values[] = { Settings.Buoyancy, Settings.SmokeWeight, Settings.Vorticity,
			Settings.DensityDissipation, Settings.Cooling };
		for (float V : Values) if (!std::isfinite(V) || V < 0) return false;
		for (const SmokeEmitter& E : Settings.Emitters)
			if (!Finite(E.Position) || !Finite(E.Velocity) || !std::isfinite(E.Radius) || E.Radius <= 0
				|| !std::isfinite(E.Density) || E.Density < 0 || !std::isfinite(E.Temperature) || E.Temperature < 0) return false;
		for (const SmokeObstacle& O : Settings.Obstacles)
			if (!ValidObstacle(O)) return false;
		m_Settings = Settings;
		m_CellSize = Vector3(Size.x / Settings.Resolution.x, Size.y / Settings.Resolution.y, Size.z / Settings.Resolution.z);
		m_Density.resize(Count); m_Temperature.resize(Count); m_Pressure.resize(Count); m_Divergence.resize(Count);
		m_DensityForward.resize(Count); m_TemperatureForward.resize(Count);
		m_DensityNext.resize(Count); m_TemperatureNext.resize(Count);
		m_Velocity.resize(Count); m_VelocityNext.resize(Count); m_Curl.resize(Count); m_Force.resize(Count);
		m_Solid.assign(Count, 0); m_Neighbors.resize(Count); m_Diagonal.resize(Count);
		RebuildObstacles(Settings.Obstacles);
		if (Settings.Backend == SmokeBackend::ComputeShader) m_GPU = SmokeGPUBackend::CreateComputeShader(Settings, m_Solid);
		if (Settings.Backend == SmokeBackend::CUDA) m_GPU = SmokeGPUBackend::CreateCUDA(Settings, m_Solid);
		if (Settings.Backend != SmokeBackend::CPU && !m_GPU && !Settings.AllowBackendFallback) return false;
		if (m_GPU) m_Backend = Settings.Backend;
		m_Initialized = true;
		Reset();
		return m_Initialized;
	}

	void SmokeSimulation::RebuildObstacles(const std::vector<SmokeObstacle>& Obstacles)
	{
		m_Obstacles = Obstacles;
		m_BoundaryFaces.clear();
		std::fill(m_Solid.begin(), m_Solid.end(), 0);
		std::vector<int> Owners(m_Solid.size(), -1);
		std::vector<int> Occupied;
		m_BoundaryVelocity.assign(m_Solid.size(), Vector3::Zero());
		for (size_t O = 0; O < Obstacles.size(); ++O)
		{
			const SmokeObstacle& Obstacle = Obstacles[O];
			Vector3 Extent(Obstacle.Radius);
			if (Obstacle.Type == SmokeObstacle::Shape::Box)
			{
				const Vector3 X = Obstacle.Rotation * Vector3(Obstacle.HalfExtent.x, 0, 0);
				const Vector3 Y = Obstacle.Rotation * Vector3(0, Obstacle.HalfExtent.y, 0);
				const Vector3 Z = Obstacle.Rotation * Vector3(0, 0, Obstacle.HalfExtent.z);
				Extent = Vector3(std::fabs(X.x) + std::fabs(Y.x) + std::fabs(Z.x),
					std::fabs(X.y) + std::fabs(Y.y) + std::fabs(Z.y), std::fabs(X.z) + std::fabs(Y.z) + std::fabs(Z.z));
			}
			int Lo[3], Hi[3];
			bool Outside = false;
			for (int A = 0; A < 3; ++A)
			{
				if (Obstacle.Center[A] + Extent[A] < m_Settings.BoundsMin[A]
					|| Obstacle.Center[A] - Extent[A] > m_Settings.BoundsMax[A]) Outside = true;
				Lo[A] = (int)Clamp(std::floor((Obstacle.Center[A] - Extent[A] - m_Settings.BoundsMin[A]) / m_CellSize[A]), 0, (float)m_Settings.Resolution[A] - 1);
				Hi[A] = (int)Clamp(std::ceil((Obstacle.Center[A] + Extent[A] - m_Settings.BoundsMin[A]) / m_CellSize[A]), 0, (float)m_Settings.Resolution[A] - 1);
			}
			if (Outside) continue;
			for (int Z = Lo[2]; Z <= Hi[2]; ++Z)
			for (int Y = Lo[1]; Y <= Hi[1]; ++Y)
			for (int X = Lo[0]; X <= Hi[0]; ++X)
			{
				const Vector3 P = m_Settings.BoundsMin + Vector3((X + 0.5f) * m_CellSize.x, (Y + 0.5f) * m_CellSize.y, (Z + 0.5f) * m_CellSize.z);
				const Vector3 Local = Obstacle.Rotation.Conjugate() * (P - Obstacle.Center);
				const bool Inside = Obstacle.Type == SmokeObstacle::Shape::Sphere
					? Local.SquareLength() < Obstacle.Radius * Obstacle.Radius
					: std::fabs(Local.x) <= Obstacle.HalfExtent.x && std::fabs(Local.y) <= Obstacle.HalfExtent.y && std::fabs(Local.z) <= Obstacle.HalfExtent.z;
				if (Inside)
				{
					const int I = Index(X, Y, Z);
					if (!m_Solid[I]) Occupied.push_back(I);
					m_Solid[I] = 1; Owners[I] = (int)O;
				}
			}
		}
		const int NX = m_Settings.Resolution.x, NY = m_Settings.Resolution.y, NZ = m_Settings.Resolution.z;
		// Only the CPU pressure solver needs a host-side neighbor stencil.
		if (!m_GPU) FluidParallelFor((int)m_Solid.size(), [&](int I)
		{
			const int X = I % NX, Y = I / NX % NY, Z = I / (NX * NY);
			auto& N = m_Neighbors[I];
			N = {{ X > 0 ? I - 1 : -1, X + 1 < NX ? I + 1 : -1,
				Y > 0 ? I - NX : -1, Y + 1 < NY ? I + NX : -2,
				Z > 0 ? I - NX * NY : -1, Z + 1 < NZ ? I + NX * NY : -1 }};
			m_Diagonal[I] = 0;
			for (int D = 0; D < 6; ++D)
			{
				if (N[D] >= 0 && m_Solid[N[D]]) N[D] = -1;
				// -1 is a closed face; -2 is the zero-pressure open ceiling.
				if (N[D] != -1) m_Diagonal[I] += 1.0f / (m_CellSize[D / 2] * m_CellSize[D / 2]);
			}
		});

		// Visit only occupied voxels when building moving faces and pressure loads.
		for (int I : Occupied)
		{
			m_Density[I] = m_Temperature[I] = 0;
			const int G[] = { I % NX, I / NX % NY, I / (NX * NY) }, Stride[] = { 1, NX, NX * NY };
			for (int A = 0; A < 3; ++A)
			for (int Sign : { -1, 1 })
			{
				if (G[A] + Sign < 0 || G[A] + Sign >= m_Settings.Resolution[A]) continue;
				const int J = I + Sign * Stride[A];
				Vector3 P = m_Settings.BoundsMin + Vector3((G[0] + 0.5f) * m_CellSize.x, (G[1] + 0.5f) * m_CellSize.y, (G[2] + 0.5f) * m_CellSize.z);
				P[A] += Sign * 0.5f * m_CellSize[A];
				if (!m_Solid[J] && Owners[I] >= (int)m_Settings.Obstacles.size())
					m_BoundaryFaces.push_back({ J, Owners[I], A, (float)Sign, P });
				const int Face = Sign > 0 ? I : J;
				const SmokeObstacle& O = Obstacles[Owners[Face] >= 0 ? Owners[Face] : Owners[I]];
				m_BoundaryVelocity[Face][A] = (O.LinearVelocity + O.AngularVelocity.Cross(P - O.Center))[A];
			}
		}
	}

	bool SmokeSimulation::SetRigidObstacles(const std::vector<SmokeObstacle>& Obstacles)
	{
		if (!m_Initialized) return false;
		for (const SmokeObstacle& O : Obstacles) if (!ValidObstacle(O)) return false;
		std::vector<SmokeObstacle> All = m_Settings.Obstacles;
		All.insert(All.end(), Obstacles.begin(), Obstacles.end());
		bool Changed = All.size() != m_Obstacles.size();
		for (size_t I = 0; I < All.size() && !Changed; ++I)
		{
			const SmokeObstacle& A = All[I]; const SmokeObstacle& B = m_Obstacles[I];
			Changed = A.Type != B.Type || A.Center != B.Center || A.Radius != B.Radius || A.HalfExtent != B.HalfExtent
				|| A.Rotation != B.Rotation || A.LinearVelocity != B.LinearVelocity || A.AngularVelocity != B.AngularVelocity;
		}
		if (!Changed) return true;
		RebuildObstacles(All);
		m_LastStepDt = 0;
		if (!m_GPU) Boundaries();
		if (m_GPU)
		{
			m_Initialized = m_GPU->UpdateObstacles(m_Solid, m_BoundaryVelocity);
			m_VelocityDirty = true;
		}
		return m_Initialized;
	}

	void SmokeSimulation::Reset()
	{
		m_Time = m_PendingTime = 0;
		m_LastStepDt = 0; m_PressureDirty = false;
		std::fill(m_Density.begin(), m_Density.end(), 0.0f);
		std::fill(m_Temperature.begin(), m_Temperature.end(), 0.0f);
		std::fill(m_Pressure.begin(), m_Pressure.end(), 0.0f);
		std::fill(m_Velocity.begin(), m_Velocity.end(), Vector3::Zero());
		Boundaries();
		if (m_GPU)
		{
			m_GPU->Reset();
			m_Initialized = m_GPU->UpdateObstacles(m_Solid, m_BoundaryVelocity);
		}
		m_VelocityDirty = m_GPU != nullptr;
	}

	bool SmokeSimulation::ComputeRigidLoads(float AirDensity, std::vector<SmokeRigidLoad>& Loads) const
	{
		Loads.clear();
		if (!m_Initialized || !std::isfinite(AirDensity) || AirDensity <= 0) return false;
		Loads.resize(m_Obstacles.size() - m_Settings.Obstacles.size());
		if (m_LastStepDt <= 0 || m_BoundaryFaces.empty()) return true;
		if (m_GPU && m_PressureDirty)
		{
			m_PressureReadback.resize(m_Pressure.size());
			if (!m_GPU->ReadPressure(m_PressureReadback)) return false;
			m_PressureDirty = false;
		}
		const auto& Pressure = m_GPU ? m_PressureReadback : m_Pressure;
		for (const BoundaryFace& Face : m_BoundaryFaces)
		{
			const float Area = m_CellSize[(Face.Axis + 1) % 3] * m_CellSize[(Face.Axis + 2) % 3];
			Vector3 Force = Vector3::Zero();
			Force[Face.Axis] = -Face.Sign * Pressure[Face.Cell] * AirDensity * Area / m_LastStepDt;
			if (!Finite(Force)) return false;
			SmokeRigidLoad& Load = Loads[Face.Owner - m_Settings.Obstacles.size()];
			Load.Force += Force;
			Load.Torque += (Face.Position - m_Obstacles[Face.Owner].Center).Cross(Force);
		}
		return true;
	}

	int SmokeSimulation::Index(int X, int Y, int Z) const
	{
		return X + m_Settings.Resolution.x * (Y + m_Settings.Resolution.y * Z);
	}

	Vector3 SmokeSimulation::GridPosition(int I) const
	{
		return Vector3((float)(I % m_Settings.Resolution.x), (float)(I / m_Settings.Resolution.x % m_Settings.Resolution.y),
			(float)(I / (m_Settings.Resolution.x * m_Settings.Resolution.y)));
	}

	float SmokeSimulation::Scalar(const std::vector<float>& Field, const Vector3& P, float* Min, float* Max) const
	{
		int B[3]; float T[3];
		for (int A = 0; A < 3; ++A)
		{
			float G = Clamp(P[A], 0, (float)m_Settings.Resolution[A] - 1);
			B[A] = std::min((int)G, m_Settings.Resolution[A] - 2); T[A] = G - B[A];
		}
		float Result = 0, Lo = 1e30f, Hi = -1e30f;
		for (int Z = 0; Z < 2; ++Z)
		for (int Y = 0; Y < 2; ++Y)
		for (int X = 0; X < 2; ++X)
		{
			float V = Field[Index(B[0] + X, B[1] + Y, B[2] + Z)];
			Result += V * (X ? T[0] : 1 - T[0]) * (Y ? T[1] : 1 - T[1]) * (Z ? T[2] : 1 - T[2]);
			Lo = std::min(Lo, V); Hi = std::max(Hi, V);
		}
		if (Min) *Min = Lo;
		if (Max) *Max = Hi;
		return Result;
	}

	float SmokeSimulation::Face(const std::vector<Vector3>& Field, const Vector3& P, int Axis) const
	{
		if (Axis == 0) return SampleMACComponent<0>(Field.data(), m_Settings.Resolution, P);
		if (Axis == 2) return SampleMACComponent<2>(Field.data(), m_Settings.Resolution, P);
		Vector3 OpenTop = P;
		OpenTop.y = std::min(OpenTop.y, m_Settings.Resolution.y - 0.5f);
		return SampleMACComponent<1>(Field.data(), m_Settings.Resolution, OpenTop);
	}

	Vector3 SmokeSimulation::Velocity(const std::vector<Vector3>& Field, const Vector3& P) const
	{
		return Vector3(Face(Field, P, 0), Face(Field, P, 1), Face(Field, P, 2));
	}

	bool SmokeSimulation::Solid(const Vector3& P) const
	{
		int C[3];
		for (int A = 0; A < 3; ++A) C[A] = (int)Clamp(P[A] + 0.5f, 0, (float)m_Settings.Resolution[A] - 1);
		return m_Solid[Index(C[0], C[1], C[2])] != 0;
	}

	Vector3 SmokeSimulation::Trace(const Vector3& P, float Dt) const
	{
		const Vector3 Mid = P - Divide(Velocity(m_Velocity, P), m_CellSize) * (0.5f * Dt);
		Vector3 End = P - Divide(Velocity(m_Velocity, Mid), m_CellSize) * Dt;
		// CFL-limited traces cannot cross a full solid cell unnoticed.
		if (Solid(End))
		{
			Vector3 Start = P;
			for (int Iteration = 0; Iteration < 5; ++Iteration)
			{
				const Vector3 Test = (Start + End) * 0.5f;
				if (Solid(Test)) End = Test; else Start = Test;
			}
			End = Start;
		}
		return End;
	}

	void SmokeSimulation::Boundaries()
	{
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) { m_Velocity[I] = m_BoundaryVelocity[I]; m_Density[I] = m_Temperature[I] = 0; return; }
			for (int A = 0; A < 3; ++A)
				if (m_Neighbors[I][A * 2 + 1] == -1) m_Velocity[I][A] = m_BoundaryVelocity[I][A];
		});
	}

	void SmokeSimulation::Emit(float Dt)
	{
		const float Time = (float)m_Time;
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) return;
			const Vector3 G = GridPosition(I) + Vector3(0.5f);
			const Vector3 P = m_Settings.BoundsMin + Vector3(G.x * m_CellSize.x, G.y * m_CellSize.y, G.z * m_CellSize.z);
			for (const SmokeEmitter& E : m_Settings.Emitters)
			{
				Vector3 Offset = P - E.Position;
				const float R = Offset.Length() / E.Radius;
				if (R >= 1) continue;
				const float Weight = 1 - R * R * (3 - 2 * R);
				const float Pulse = 0.92f + 0.08f * std::sin(Time * 3.1f);
				m_Density[I] = std::max(m_Density[I], E.Density * Weight * Pulse);
				m_Temperature[I] = std::max(m_Temperature[I], E.Temperature * Weight);
				// A small, deterministic nozzle perturbation breaks axial symmetry.
				const Vector3 Jet = E.Velocity + Vector3(0.16f * std::sin(Time * 2.7f + P.z * 13),
					0.08f * std::sin(Time * 4.1f + P.x * 11), 0.16f * std::cos(Time * 3.3f + P.x * 12));
				m_Velocity[I] += (Jet - m_Velocity[I]) * (1 - std::exp(-18 * Weight * Dt));
			}
		});
	}

	void SmokeSimulation::AddForces(float Dt)
	{
		const int NX = m_Settings.Resolution.x, NY = m_Settings.Resolution.y, NZ = m_Settings.Resolution.z;
		// Store cell-centered velocities before differentiating the staggered field.
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			const int X = I % NX, Y = I / NX % NY, Z = I / (NX * NY);
			m_Force[I] = Vector3((m_Velocity[I].x + (X > 0 ? m_Velocity[I - 1].x : 0)) * 0.5f,
				(m_Velocity[I].y + (Y > 0 ? m_Velocity[I - NX].y : 0)) * 0.5f,
				(m_Velocity[I].z + (Z > 0 ? m_Velocity[I - NX * NY].z : 0)) * 0.5f);
		});
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			const int X = I % NX, Y = I / NX % NY, Z = I / (NX * NY);
			const Vector3 DX = (m_Force[I + (X + 1 < NX ? 1 : 0)] - m_Force[I - (X > 0 ? 1 : 0)]) / (2 * m_CellSize.x);
			const Vector3 DY = (m_Force[I + (Y + 1 < NY ? NX : 0)] - m_Force[I - (Y > 0 ? NX : 0)]) / (2 * m_CellSize.y);
			const Vector3 DZ = (m_Force[I + (Z + 1 < NZ ? NX * NY : 0)] - m_Force[I - (Z > 0 ? NX * NY : 0)]) / (2 * m_CellSize.z);
			m_Curl[I] = Vector3(DY.z - DZ.y, DZ.x - DX.z, DX.y - DY.x);
		});
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			const int X = I % NX, Y = I / NX % NY, Z = I / (NX * NY);
			const Vector3 Gradient(
				(m_Curl[I + (X + 1 < NX ? 1 : 0)].Length() - m_Curl[I - (X > 0 ? 1 : 0)].Length()) / m_CellSize.x,
				(m_Curl[I + (Y + 1 < NY ? NX : 0)].Length() - m_Curl[I - (Y > 0 ? NX : 0)].Length()) / m_CellSize.y,
				(m_Curl[I + (Z + 1 < NZ ? NX * NY : 0)].Length() - m_Curl[I - (Z > 0 ? NX * NY : 0)].Length()) / m_CellSize.z);
			const float H = std::min(m_CellSize.x, std::min(m_CellSize.y, m_CellSize.z));
			m_Force[I] = Gradient / std::max(Gradient.Length(), 1e-5f);
			m_Force[I] = m_Force[I].Cross(m_Curl[I]) * (m_Settings.Vorticity * H);
			m_Force[I].y += m_Settings.Buoyancy * m_Temperature[I] - m_Settings.SmokeWeight * m_Density[I];
			if (m_Solid[I]) m_Force[I] = Vector3::Zero();
		});
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			for (int A = 0; A < 3; ++A)
			{
				const int N = m_Neighbors[I][A * 2 + 1];
				m_Velocity[I][A] += (m_Force[I][A] + (N >= 0 ? m_Force[N][A] : m_Force[I][A])) * (0.5f * Dt);
			}
		});
	}

	float SmokeSimulation::Divergence(int I) const
	{
		if (m_Solid[I]) return 0;
		float D = 0;
		for (int A = 0; A < 3; ++A)
		{
			const int Stride[] = { 1, m_Settings.Resolution.x, m_Settings.Resolution.x * m_Settings.Resolution.y };
			int N = GridPosition(I)[A] > 0 ? I - Stride[A] : -1;
			D += (m_Velocity[I][A] - (N >= 0 ? m_Velocity[N][A] : 0)) / m_CellSize[A];
		}
		return D;
	}

	void SmokeSimulation::Project()
	{
		Boundaries();
		FluidParallelFor((int)m_Density.size(), [&](int I) { m_Divergence[I] = Divergence(I); m_Pressure[I] = 0; });
		const Vector3 W(1 / (m_CellSize.x * m_CellSize.x), 1 / (m_CellSize.y * m_CellSize.y), 1 / (m_CellSize.z * m_CellSize.z));
		const int NX = m_Settings.Resolution.x, NY = m_Settings.Resolution.y;
		const int ShortAxis = std::min(NX, std::min(NY, m_Settings.Resolution.z));
		const float Relaxation = Clamp(2.0f / (1.0f + std::sin(3.14159265f / ShortAxis)), 1.75f, 1.93f);
		for (int Iteration = 0; Iteration < m_Settings.PressureIterations; ++Iteration)
		for (int Parity = 0; Parity < 2; ++Parity)
		{
			FluidParallelFor((int)m_Density.size(), [&](int I)
			{
				if (((I % NX + I / NX % NY + I / (NX * NY)) & 1) != Parity || m_Solid[I] || m_Diagonal[I] == 0) return;
				const auto& N = m_Neighbors[I];
				float Sum = 0;
				for (int D = 0; D < 6; ++D) if (N[D] >= 0) Sum += W[D / 2] * m_Pressure[N[D]];
				const float P = (Sum - m_Divergence[I]) / m_Diagonal[I];
				m_Pressure[I] += Relaxation * (P - m_Pressure[I]);
			});
		}
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) return;
			for (int A = 0; A < 3; ++A)
			{
				const int N = m_Neighbors[I][A * 2 + 1];
				if (N != -1) m_Velocity[I][A] -= ((N >= 0 ? m_Pressure[N] : 0) - m_Pressure[I]) / m_CellSize[A];
			}
		});
	}

	void SmokeSimulation::AdvectScalars(float Dt)
	{
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			const Vector3 P = Trace(GridPosition(I), Dt);
			m_DensityForward[I] = m_Solid[I] ? 0 : Scalar(m_Density, P);
			m_TemperatureForward[I] = m_Solid[I] ? 0 : Scalar(m_Temperature, P);
		});
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) { m_DensityNext[I] = m_TemperatureNext[I] = 0; return; }
			const Vector3 G = GridPosition(I), Back = Trace(G, Dt), Forward = Trace(G, -Dt);
			float Min, Max;
			Scalar(m_Density, Back, &Min, &Max);
			float D = Clamp(m_DensityForward[I] + 0.5f * (m_Density[I] - Scalar(m_DensityForward, Forward)), Min, Max);
			Scalar(m_Temperature, Back, &Min, &Max);
			float T = Clamp(m_TemperatureForward[I] + 0.5f * (m_Temperature[I] - Scalar(m_TemperatureForward, Forward)), Min, Max);
			// Fade gently into the open ceiling rather than showing a clipped lid.
			const float Edge = Clamp((G.y - (m_Settings.Resolution.y - 7)) / 6, 0, 1);
			m_DensityNext[I] = D * std::exp(-Dt * (m_Settings.DensityDissipation + Edge * 4));
			m_TemperatureNext[I] = T * std::exp(-Dt * (m_Settings.Cooling + Edge * 4));
		});
		m_Density.swap(m_DensityNext); m_Temperature.swap(m_TemperatureNext);
	}

	void SmokeSimulation::Step(float DeltaTime)
	{
		if (!m_Initialized || !std::isfinite(DeltaTime) || DeltaTime <= 0) return;
		m_PendingTime += DeltaTime;
		for (int Substep = 0; Substep < m_Settings.MaxSubsteps && m_PendingTime > 1e-7; ++Substep)
		{
			float Rate = m_GPU ? m_GPU->GetMaxRate() : 0;
			if (!m_GPU)
				for (const Vector3& V : m_Velocity)
					Rate = std::max(Rate, std::max(std::fabs(V.x) / m_CellSize.x, std::max(std::fabs(V.y) / m_CellSize.y, std::fabs(V.z) / m_CellSize.z)));
			const float Dt = std::min((float)m_PendingTime, std::min(m_Settings.MaxTimeStep, 0.9f / std::max(Rate, 1e-5f)));
			m_LastStepDt = Dt; m_PressureDirty = true;
			if (m_GPU)
			{
				m_GPU->Step(Dt, (float)m_Time);
				m_Time += Dt; m_PendingTime -= Dt; m_VelocityDirty = true;
				if (!m_GPU->Readback(m_Density, m_Temperature)) { m_Initialized = false; return; }
				continue;
			}
			FluidParallelFor((int)m_Velocity.size(), [&](int I)
			{
				const Vector3 G = GridPosition(I);
				for (int A = 0; A < 3; ++A)
				{
					Vector3 P = G; P[A] += 0.5f;
					m_VelocityNext[I][A] = Face(m_Velocity, Trace(P, Dt), A);
				}
			});
			m_Velocity.swap(m_VelocityNext);
			Emit(Dt); AddForces(Dt); Project(); AdvectScalars(Dt); Boundaries();
			m_Time += Dt; m_PendingTime -= Dt;
		}
	}

	bool SmokeSimulation::ApplyImpulse(const Vector3& Position, float Radius, const Vector3& VelocityChange)
	{
		if (!m_Initialized || !Finite(Position) || !Finite(VelocityChange) || !std::isfinite(Radius) || Radius <= 0) return false;
		const Vector3 Change = VelocityChange * std::min(1.0f, 12.0f / std::max(1e-5f, VelocityChange.Length()));
		if (m_GPU)
		{
			m_GPU->ApplyImpulse(Position, Radius, Change); m_VelocityDirty = true;
			return true;
		}
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) return;
			const Vector3 G = GridPosition(I) + Vector3(0.5f);
			const Vector3 P = m_Settings.BoundsMin + Vector3(G.x * m_CellSize.x, G.y * m_CellSize.y, G.z * m_CellSize.z);
			const float Weight = std::max(0.0f, 1 - (P - Position).SquareLength() / (Radius * Radius));
			m_Velocity[I] += Change * (Weight * Weight);
		});
		Boundaries();
		return true;
	}

	float SmokeSimulation::GetDivergenceRMS() const
	{
		if (!m_Initialized) return 0;
		GetVelocity();
		double Sum = 0; size_t Count = 0;
		for (int I = 0; I < (int)m_Density.size(); ++I)
			if (!m_Solid[I]) { const float D = Divergence(I); Sum += D * D; ++Count; }
		return Count ? (float)std::sqrt(Sum / Count) : 0;
	}

	const std::vector<Vector3>& SmokeSimulation::GetVelocity() const
	{
		if (m_GPU && m_VelocityDirty && m_GPU->ReadVelocity(m_Velocity)) m_VelocityDirty = false;
		return m_Velocity;
	}
}

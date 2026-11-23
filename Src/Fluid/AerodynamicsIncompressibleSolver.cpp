#include "AerodynamicsIncompressibleSolver.h"
#include "FluidParallelFor.h"
#include "AerodynamicsGPUBackend.h"
#include "MACGridSampling.h"
#include "../CollisionPrimitive/Triangle3.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Riemann
{
	AerodynamicsIncompressibleSolver::AerodynamicsIncompressibleSolver() = default;
	AerodynamicsIncompressibleSolver::~AerodynamicsIncompressibleSolver() = default;
#if !defined(RIEMANN_ENABLE_CUDA)
	std::unique_ptr<AerodynamicsGPUBackend> AerodynamicsGPUBackend::CreateCUDA(const AerodynamicsSimulationSettings&,
		const std::vector<unsigned char>&) { return nullptr; }
#endif
	namespace
	{
		float Clamp(float V, float A, float B) { return std::max(A, std::min(V, B)); }
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		Vector3 Divide(const Vector3& A, const Vector3& B) { return Vector3(A.x / B.x, A.y / B.y, A.z / B.z); }
		bool ValidObstacle(const AerodynamicsObstacle& O)
		{
			return Finite(O.Center) && Finite(O.LinearVelocity) && Finite(O.AngularVelocity)
				&& Finite(O.HalfExtent) && O.HalfExtent.x > 0 && O.HalfExtent.y > 0 && O.HalfExtent.z > 0
				&& std::isfinite(O.Radius) && O.Radius > 0 && std::isfinite(O.Rotation.SquareLength())
				&& std::fabs(O.Rotation.SquareLength() - 1.0f) < 1e-3f
				&& (O.Type == AerodynamicsObstacle::Shape::Sphere || O.Type == AerodynamicsObstacle::Shape::Box);
		}
	}

	bool AerodynamicsIncompressibleSolver::Initialize(const AerodynamicsSimulationSettings& Settings)
	{
		m_Initialized = false;
		m_Surfaces.clear(); m_HasAveragedLoads = false;
		m_GPU.reset(); m_Backend = FluidBackend::CPU; m_VelocityDirty = false;
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
			|| Settings.MaxSubsteps < 1 || Settings.MaxSubsteps > 4096) return false;
		const float Values[] = { Settings.Buoyancy, Settings.TracerWeight, Settings.Vorticity,
			Settings.DensityDissipation, Settings.Cooling };
		for (float V : Values) if (!std::isfinite(V) || V < 0) return false;
		for (const AerodynamicsEmitter& E : Settings.Emitters)
			if (!Finite(E.Position) || !Finite(E.Velocity) || !std::isfinite(E.Radius) || E.Radius <= 0
				|| !std::isfinite(E.Density) || E.Density < 0 || !std::isfinite(E.Temperature) || E.Temperature < 0) return false;
		for (const AerodynamicsObstacle& O : Settings.Obstacles)
			if (!ValidObstacle(O)) return false;
		m_Settings = Settings;
		m_CellSize = Vector3(Size.x / Settings.Resolution.x, Size.y / Settings.Resolution.y, Size.z / Settings.Resolution.z);
		m_Density.resize(Count); m_Temperature.resize(Count); m_Pressure.resize(Count); m_Divergence.resize(Count);
		m_DensityForward.resize(Count); m_TemperatureForward.resize(Count);
		m_DensityNext.resize(Count); m_TemperatureNext.resize(Count);
		m_Velocity.resize(Count); m_VelocityNext.resize(Count); m_Curl.resize(Count); m_Force.resize(Count);
		m_Solid.assign(Count, 0); m_Neighbors.resize(Count); m_Diagonal.resize(Count);
		m_CellOwners.assign(Count, -1); m_CellTriangles.assign(Count, -1); m_OccupiedCells.clear();
		m_SurfaceDistances.assign(Count, std::numeric_limits<float>::max()); m_BoundaryVelocity.assign(Count, Vector3::Zero());
		RebuildObstacles(Settings.Obstacles);
		if (Settings.Backend == FluidBackend::ComputeShader) m_GPU = AerodynamicsGPUBackend::CreateComputeShader(Settings, m_Solid);
		if (Settings.Backend == FluidBackend::CUDA) m_GPU = AerodynamicsGPUBackend::CreateCUDA(Settings, m_Solid);
		if (Settings.Backend != FluidBackend::CPU && !m_GPU && !Settings.AllowBackendFallback) return false;
		if (m_GPU) m_Backend = Settings.Backend;
		m_Initialized = true;
		Reset();
		return m_Initialized;
	}

	void AerodynamicsIncompressibleSolver::RebuildObstacles(const std::vector<AerodynamicsObstacle>& Obstacles)
	{
		m_Obstacles = Obstacles;
		m_BoundaryFaces.clear();
		auto& Owners = m_CellOwners; auto& Triangles = m_CellTriangles; auto& Distances = m_SurfaceDistances; auto& Occupied = m_OccupiedCells;
		m_UpdateMin = m_Settings.Resolution; m_UpdateMax = Vector3i(-1);
		auto Include = [&](int I)
		{
			const int P[] = { I % m_Settings.Resolution.x, I / m_Settings.Resolution.x % m_Settings.Resolution.y, I / (m_Settings.Resolution.x * m_Settings.Resolution.y) };
			for (int A = 0; A < 3; ++A) { m_UpdateMin[A] = std::min(m_UpdateMin[A], std::max(0, P[A] - 1)); m_UpdateMax[A] = std::max(m_UpdateMax[A], P[A] + 1); }
		};
		for (int I : Occupied)
		{
			Include(I); m_Solid[I] = 0; Owners[I] = Triangles[I] = -1; Distances[I] = std::numeric_limits<float>::max();
			m_BoundaryVelocity[I] = Vector3::Zero();
			if (I % m_Settings.Resolution.x > 0) m_BoundaryVelocity[I - 1].x = 0;
			if (I / m_Settings.Resolution.x % m_Settings.Resolution.y > 0) m_BoundaryVelocity[I - m_Settings.Resolution.x].y = 0;
			if (I / (m_Settings.Resolution.x * m_Settings.Resolution.y) > 0) m_BoundaryVelocity[I - m_Settings.Resolution.x * m_Settings.Resolution.y].z = 0;
		}
		Occupied.clear();
		for (size_t O = 0; O < Obstacles.size(); ++O)
		{
			const AerodynamicsObstacle& Obstacle = Obstacles[O];
			Vector3 Extent(Obstacle.Radius);
			if (Obstacle.Type == AerodynamicsObstacle::Shape::Box)
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
				const bool Inside = Obstacle.Type == AerodynamicsObstacle::Shape::Sphere
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
		// Conservative thick-shell rasterization closes sub-cell cracks between triangles.
		// Rigid solids take precedence; overlapping sheets choose the nearest surface.
		for (size_t S = 0; S < m_Surfaces.size(); ++S)
		{
			const auto& Surface = m_Surfaces[S];
			const float Width = std::max(Surface.Thickness, 0.55f * m_CellSize.Length());
			for (size_t T = 0; T < Surface.Triangles.size(); ++T)
			{
				const auto& Indices = Surface.Triangles[T];
				const Vector3 P[] = { Surface.Positions[Indices.a], Surface.Positions[Indices.b], Surface.Positions[Indices.c] };
				if ((P[1] - P[0]).Cross(P[2] - P[0]).SquareLength() <= 1e-16f) continue;
				int Lo[3], Hi[3]; bool Outside = false;
				for (int A = 0; A < 3; ++A)
				{
					const float Min = std::min(P[0][A], std::min(P[1][A], P[2][A])) - Width;
					const float Max = std::max(P[0][A], std::max(P[1][A], P[2][A])) + Width;
					Outside |= Max < m_Settings.BoundsMin[A] || Min > m_Settings.BoundsMax[A];
					Lo[A] = (int)Clamp(std::floor((Min - m_Settings.BoundsMin[A]) / m_CellSize[A]), 0, (float)m_Settings.Resolution[A] - 1);
					Hi[A] = (int)Clamp(std::floor((Max - m_Settings.BoundsMin[A]) / m_CellSize[A]), 0, (float)m_Settings.Resolution[A] - 1);
				}
				if (Outside) continue;
				for (int Z = Lo[2]; Z <= Hi[2]; ++Z) for (int Y = Lo[1]; Y <= Hi[1]; ++Y) for (int X = Lo[0]; X <= Hi[0]; ++X)
				{
					const int I = Index(X, Y, Z); if (Owners[I] >= 0 && Owners[I] < (int)Obstacles.size()) continue;
					const Vector3 Center = m_Settings.BoundsMin + Vector3((X + 0.5f) * m_CellSize.x, (Y + 0.5f) * m_CellSize.y, (Z + 0.5f) * m_CellSize.z);
					const float D = Triangle3::SqrDistancePointToTriangle(Center, P[0], P[1], P[2]);
					if (D > Width * Width || D >= Distances[I]) continue;
					if (!m_Solid[I]) Occupied.push_back(I);
					m_Solid[I] = 1; Owners[I] = (int)(Obstacles.size() + S); Triangles[I] = (int)T; Distances[I] = D;
				}
			}
		}
		auto SurfaceWeights = [&](int Owner, int Triangle, const Vector3& Position)
		{
			const auto& S = m_Surfaces[Owner - Obstacles.size()]; const auto& T = S.Triangles[Triangle];
			const Vector3 P = Triangle3::ClosestPointOnTriangleToPoint(Position, S.Positions[T.a], S.Positions[T.b], S.Positions[T.c]);
			Vector3 W = Triangle3::BarycentricCoods(P, S.Positions[T.a], S.Positions[T.b], S.Positions[T.c]);
			for (int A = 0; A < 3; ++A) W[A] = Clamp(W[A], 0, 1);
			return W / std::max(W.x + W.y + W.z, 1e-8f);
		};
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
			Include(I);
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
				{
					BoundaryFace B{ J, Owners[I], A, (float)Sign, P };
					if (B.Owner >= (int)Obstacles.size()) { B.Triangle = Triangles[I]; B.Weights = SurfaceWeights(B.Owner, B.Triangle, P); }
					m_BoundaryFaces.push_back(B);
				}
				const int Face = Sign > 0 ? I : J, OwnerCell = Owners[Face] >= 0 ? Face : I, Owner = Owners[OwnerCell];
				if (Owner < (int)Obstacles.size())
				{
					const auto& O = Obstacles[Owner];
					m_BoundaryVelocity[Face][A] = (O.LinearVelocity + O.AngularVelocity.Cross(P - O.Center))[A];
				}
				else
				{
					const auto& S = m_Surfaces[Owner - Obstacles.size()]; const auto& T = S.Triangles[Triangles[OwnerCell]];
					const Vector3 W = SurfaceWeights(Owner, Triangles[OwnerCell], P);
					m_BoundaryVelocity[Face][A] = (S.Velocities[T.a] * W.x + S.Velocities[T.b] * W.y + S.Velocities[T.c] * W.z)[A];
				}
			}
		}
	}

	bool AerodynamicsIncompressibleSolver::SetRigidObstacles(const std::vector<AerodynamicsObstacle>& Obstacles)
	{
		return SetMovingBoundaries(Obstacles, m_Surfaces);
	}
	bool AerodynamicsIncompressibleSolver::SetMovingBoundaries(const std::vector<AerodynamicsObstacle>& Obstacles, const std::vector<AerodynamicsSurfaceMesh>& Surfaces)
	{
		if (!m_Initialized) return false;
		for (const auto& S : Surfaces)
		{
			if ((!S.InverseMasses.empty() && S.InverseMasses.size() != S.Positions.size()) || S.Positions.empty() || S.Velocities.size() != S.Positions.size() || !std::isfinite(S.Thickness) || S.Thickness < 0) return false;
			for (size_t I = 0; I < S.Positions.size(); ++I) if (!Finite(S.Positions[I]) || !Finite(S.Velocities[I])) return false;
			for (float M : S.InverseMasses) if (!std::isfinite(M) || M < 0) return false;
			for (const auto& T : S.Triangles) for (int A = 0; A < 3; ++A) if (T[A] < 0 || (size_t)T[A] >= S.Positions.size()) return false;
		}
		for (const AerodynamicsObstacle& O : Obstacles) if (!ValidObstacle(O)) return false;
		std::vector<AerodynamicsObstacle> All = m_Settings.Obstacles;
		All.insert(All.end(), Obstacles.begin(), Obstacles.end());
		bool Changed = All.size() != m_Obstacles.size() || !Surfaces.empty() || !m_Surfaces.empty();
		for (size_t I = 0; I < All.size() && !Changed; ++I)
		{
			const AerodynamicsObstacle& A = All[I]; const AerodynamicsObstacle& B = m_Obstacles[I];
			Changed = A.Type != B.Type || A.Center != B.Center || A.Radius != B.Radius || A.HalfExtent != B.HalfExtent
				|| A.Rotation != B.Rotation || A.LinearVelocity != B.LinearVelocity || A.AngularVelocity != B.AngularVelocity;
		}
		if (!Changed) return true;
		m_Surfaces = Surfaces;
		RebuildObstacles(All);
		m_LastStepDt = 0; m_HasAveragedLoads = false;
		if (!m_GPU) Boundaries();
		if (m_GPU)
		{
			const AerodynamicsGridRegion Region{ m_UpdateMin, m_UpdateMax };
			m_Initialized = m_GPU->UpdateObstacles(m_Solid, m_BoundaryVelocity, &Region);
			m_VelocityDirty = true;
		}
		return m_Initialized;
	}

	void AerodynamicsIncompressibleSolver::Reset()
	{
		m_Time = m_PendingTime = 0;
		m_LastStepDt = 0; m_PressureDirty = false; m_HasAveragedLoads = false;
		std::fill(m_Density.begin(), m_Density.end(), 0.0f);
		std::fill(m_Temperature.begin(), m_Temperature.end(), 0.0f);
		std::fill(m_Pressure.begin(), m_Pressure.end(), 0.0f);
		std::fill(m_Velocity.begin(), m_Velocity.end(), Vector3::Zero());
		if (!m_GPU) Boundaries();
		if (m_GPU)
		{
			m_GPU->Reset();
			m_Initialized = m_GPU->UpdateObstacles(m_Solid, m_BoundaryVelocity);
		}
		m_VelocityDirty = m_GPU != nullptr;
	}

	bool AerodynamicsIncompressibleSolver::ReadLoads(std::vector<AerodynamicsRigidLoad>& Loads, std::vector<std::vector<Vector3>>& Forces) const
	{
		Loads.assign(m_Obstacles.size() - m_Settings.Obstacles.size(), AerodynamicsRigidLoad());
		Forces.resize(m_Surfaces.size());
		for (size_t I = 0; I < Forces.size(); ++I) Forces[I].assign(m_Surfaces[I].Positions.size(), Vector3::Zero());
		if (!m_Initialized) return false;
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
			Force[Face.Axis] = -Face.Sign * Pressure[Face.Cell] * m_Settings.AmbientDensity * Area / m_LastStepDt;
			if (!Finite(Force)) return false;
			if (Face.Triangle < 0)
			{
				auto& Load = Loads[Face.Owner - m_Settings.Obstacles.size()];
				Load.Force += Force; Load.Torque += (Face.Position - m_Obstacles[Face.Owner].Center).Cross(Force);
			}
			else
			{
				const size_t S = Face.Owner - m_Obstacles.size(); const auto& T = m_Surfaces[S].Triangles[Face.Triangle];
				for (int A = 0; A < 3; ++A) Forces[S][T[A]] += Force * Face.Weights[A];
			}
		}
		return true;
	}
	bool AerodynamicsIncompressibleSolver::ComputeRigidLoads(float AirDensity, std::vector<AerodynamicsRigidLoad>& Loads) const
	{
		Loads.clear(); if (!m_Initialized || !std::isfinite(AirDensity) || AirDensity <= 0) return false;
		if (m_HasAveragedLoads) Loads = m_AveragedRigidLoads;
		else { std::vector<std::vector<Vector3>> Surface; if (!ReadLoads(Loads, Surface)) return false; }
		const float Scale = AirDensity / m_Settings.AmbientDensity;
		for (auto& L : Loads) { L.Force *= Scale; L.Torque *= Scale; }
		return true;
	}
	bool AerodynamicsIncompressibleSolver::ComputeSurfaceLoads(std::vector<std::vector<Vector3>>& Forces) const
	{
		if (m_HasAveragedLoads) { Forces = m_AveragedSurfaceLoads; return m_Initialized; }
		std::vector<AerodynamicsRigidLoad> Rigid; return ReadLoads(Rigid, Forces);
	}

	int AerodynamicsIncompressibleSolver::Index(int X, int Y, int Z) const
	{
		return X + m_Settings.Resolution.x * (Y + m_Settings.Resolution.y * Z);
	}

	Vector3 AerodynamicsIncompressibleSolver::GridPosition(int I) const
	{
		return Vector3((float)(I % m_Settings.Resolution.x), (float)(I / m_Settings.Resolution.x % m_Settings.Resolution.y),
			(float)(I / (m_Settings.Resolution.x * m_Settings.Resolution.y)));
	}

	float AerodynamicsIncompressibleSolver::Scalar(const std::vector<float>& Field, const Vector3& P, float* Min, float* Max) const
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

	float AerodynamicsIncompressibleSolver::Face(const std::vector<Vector3>& Field, const Vector3& P, int Axis) const
	{
		if (Axis == 0) return SampleMACComponent<0>(Field.data(), m_Settings.Resolution, P);
		if (Axis == 2) return SampleMACComponent<2>(Field.data(), m_Settings.Resolution, P);
		Vector3 OpenTop = P;
		OpenTop.y = std::min(OpenTop.y, m_Settings.Resolution.y - 0.5f);
		return SampleMACComponent<1>(Field.data(), m_Settings.Resolution, OpenTop);
	}

	Vector3 AerodynamicsIncompressibleSolver::Velocity(const std::vector<Vector3>& Field, const Vector3& P) const
	{
		return Vector3(Face(Field, P, 0), Face(Field, P, 1), Face(Field, P, 2));
	}

	bool AerodynamicsIncompressibleSolver::Solid(const Vector3& P) const
	{
		int C[3];
		for (int A = 0; A < 3; ++A) C[A] = (int)Clamp(P[A] + 0.5f, 0, (float)m_Settings.Resolution[A] - 1);
		return m_Solid[Index(C[0], C[1], C[2])] != 0;
	}

	Vector3 AerodynamicsIncompressibleSolver::Trace(const Vector3& P, float Dt) const
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

	void AerodynamicsIncompressibleSolver::Boundaries()
	{
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) { m_Velocity[I] = m_BoundaryVelocity[I]; m_Density[I] = m_Temperature[I] = 0; return; }
			for (int A = 0; A < 3; ++A)
				if (m_Neighbors[I][A * 2 + 1] == -1) m_Velocity[I][A] = m_BoundaryVelocity[I][A];
		});
	}

	void AerodynamicsIncompressibleSolver::Emit(float Dt)
	{
		const float Time = (float)m_Time;
		FluidParallelFor((int)m_Density.size(), [&](int I)
		{
			if (m_Solid[I]) return;
			const Vector3 G = GridPosition(I) + Vector3(0.5f);
			const Vector3 P = m_Settings.BoundsMin + Vector3(G.x * m_CellSize.x, G.y * m_CellSize.y, G.z * m_CellSize.z);
			for (const AerodynamicsEmitter& E : m_Settings.Emitters)
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

	void AerodynamicsIncompressibleSolver::AddForces(float Dt)
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
			m_Force[I].y += m_Settings.Buoyancy * m_Temperature[I] - m_Settings.TracerWeight * m_Density[I];
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

	float AerodynamicsIncompressibleSolver::Divergence(int I) const
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

	void AerodynamicsIncompressibleSolver::Project()
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

	void AerodynamicsIncompressibleSolver::AdvectScalars(float Dt)
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

	bool AerodynamicsIncompressibleSolver::ProjectSurfaceMass()
	{
		bool Dynamic = false;
		for (const auto& S : m_Surfaces) for (float M : S.InverseMasses) Dynamic |= M > 0;
		if (!Dynamic || m_BoundaryFaces.empty()) return true;
		if (m_GPU)
		{
			const float Volume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
			std::vector<int> Starts(m_Surfaces.size() + 1, 0);
			std::vector<float> MassScale;
			for (size_t I = 0; I < m_Surfaces.size(); ++I)
			{
				const auto& S = m_Surfaces[I]; Starts[I + 1] = Starts[I] + (int)S.Positions.size() * 3;
				for (size_t J = 0; J < S.Positions.size(); ++J) for (int A = 0; A < 3; ++A)
					MassScale.push_back(S.InverseMasses.empty() ? 0 : std::sqrt(m_Settings.AmbientDensity * Volume * S.InverseMasses[J]));
			}
			std::vector<std::vector<AerodynamicsPressureEntry>> Rows(MassScale.size());
			AerodynamicsPressureSystem System;
			const int Stride[] = { 1, m_Settings.Resolution.x, m_Settings.Resolution.x * m_Settings.Resolution.y };
			for (const auto& F : m_BoundaryFaces) if (F.Triangle >= 0)
			{
				const int S = F.Owner - (int)m_Obstacles.size(); const auto& T = m_Surfaces[S].Triangles[F.Triangle];
				AerodynamicsPressureFace Face; Face.Cell = F.Sign > 0 ? F.Cell - Stride[F.Axis] : F.Cell; Face.Axis = F.Axis;
				for (int A = 0; A < 3; ++A)
				{
					const int Row = Starts[S] + T[A] * 3 + F.Axis; Face.Nodes[A] = Row; Face.Weights[A] = -F.Weights[A] * MassScale[Row];
					const float Weight = F.Sign * F.Weights[A] * MassScale[Row] / m_CellSize[F.Axis];
					if (Weight != 0) Rows[Row].push_back({ F.Cell, Weight });
				}
				System.Faces.push_back(Face);
			}
			System.RowOffsets.push_back(0);
			for (auto& Row : Rows)
			{
				std::sort(Row.begin(), Row.end(), [](const AerodynamicsPressureEntry& A, const AerodynamicsPressureEntry& B) { return A.Index < B.Index; });
				for (size_t I = 0; I < Row.size();)
				{
					auto E = Row[I++]; while (I < Row.size() && Row[I].Index == E.Index) E.Weight += Row[I++].Weight;
					System.RowEntries.push_back(E);
				}
				System.RowOffsets.push_back((int)System.RowEntries.size());
			}
			struct TransposedEntry { int Cell, Row; float Weight; };
			std::vector<TransposedEntry> Transposed; Transposed.reserve(System.RowEntries.size());
			for (int R = 0; R < (int)Rows.size(); ++R) for (int J = System.RowOffsets[R]; J < System.RowOffsets[R + 1]; ++J)
			{
				const auto& E = System.RowEntries[J]; Transposed.push_back({ E.Index, R, E.Weight });
			}
			std::sort(Transposed.begin(), Transposed.end(), [](const TransposedEntry& A, const TransposedEntry& B) { return A.Cell < B.Cell || (A.Cell == B.Cell && A.Row < B.Row); });
			for (const auto& E : Transposed)
			{
				if (System.CellIndices.empty() || System.CellIndices.back() != E.Cell)
				{
					System.CellIndices.push_back(E.Cell); System.CellOffsets.push_back((int)System.CellEntries.size());
				}
				System.CellEntries.push_back({ E.Row, E.Weight });
			}
			System.CellOffsets.push_back((int)System.CellEntries.size());
			std::vector<float> Response;
			if (!m_GPU->ProjectSurfaceMass(System, Response) || Response.size() != Rows.size()) return false;
			for (const auto& F : System.Faces) for (int A = 0; A < 3; ++A) m_BoundaryVelocity[F.Cell][F.Axis] += F.Weights[A] * Response[F.Nodes[A]];
			for (size_t I = 0; I < m_Surfaces.size(); ++I) for (size_t J = 0; J < m_Surfaces[I].Positions.size(); ++J) for (int A = 0; A < 3; ++A)
			{
				const int R = Starts[I] + (int)J * 3 + A; m_Surfaces[I].Velocities[J][A] -= MassScale[R] * Response[R];
			}
			m_VelocityDirty = m_PressureDirty = true; return true;
		}
		// Eliminate surface velocities from the incompressibility constraint. The
		// added operator B M^-1 B^T is positive semidefinite and includes every face
		// sharing a cloth vertex. This resolves light-sheet added mass implicitly.
		// Use one pressure solution for fluid velocity, moving wall velocity and
		// cloth impulses, without an additional fixed-wall pressure projection.
		const int Count = (int)m_Solid.size();
		const float Volume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		const Vector3 Weight(1 / (m_CellSize.x * m_CellSize.x), 1 / (m_CellSize.y * m_CellSize.y), 1 / (m_CellSize.z * m_CellSize.z));
		std::vector<std::vector<Vector3>> Response(m_Surfaces.size());
		for (size_t I = 0; I < Response.size(); ++I) Response[I].resize(m_Surfaces[I].Positions.size());
		auto ResponseFor = [&](const std::vector<float>& Pressure)
		{
			for (auto& R : Response) std::fill(R.begin(), R.end(), Vector3::Zero());
			for (const auto& F : m_BoundaryFaces) if (F.Triangle >= 0)
			{
				const size_t I = F.Owner - m_Obstacles.size(); const auto& S = m_Surfaces[I]; if (S.InverseMasses.empty()) continue;
				const auto& T = S.Triangles[F.Triangle];
				const float Impulse = -F.Sign * (Volume / m_CellSize[F.Axis]) * m_Settings.AmbientDensity * Pressure[F.Cell];
				for (int A = 0; A < 3; ++A) Response[I][T[A]][F.Axis] += Impulse * F.Weights[A] * S.InverseMasses[T[A]];
			}
		};
		auto SurfaceProduct = [&](const std::vector<float>& X, std::vector<float>& Product)
		{
			ResponseFor(X);
			for (const auto& F : m_BoundaryFaces) if (F.Triangle >= 0)
			{
				const size_t I = F.Owner - m_Obstacles.size(); const auto& T = m_Surfaces[I].Triangles[F.Triangle];
				float Velocity = 0; for (int A = 0; A < 3; ++A) Velocity += Response[I][T[A]][F.Axis] * F.Weights[A];
				Product[F.Cell] -= F.Sign * Velocity / m_CellSize[F.Axis];
			}
		};
		std::vector<float> Correction(Count, 0), Residual(Count, 0), Search(Count), Product(Count), Diagonal = m_Diagonal;
		for (const auto& F : m_BoundaryFaces) if (F.Triangle >= 0)
		{
			const auto& S = m_Surfaces[F.Owner - m_Obstacles.size()]; if (S.InverseMasses.empty()) continue;
			const auto& T = S.Triangles[F.Triangle]; float InverseMass = 0;
			for (int A = 0; A < 3; ++A) InverseMass += S.InverseMasses[T[A]] * F.Weights[A] * F.Weights[A];
			Diagonal[F.Cell] += m_Settings.AmbientDensity * Volume * Weight[F.Axis] * InverseMass;
		}
		SurfaceProduct(m_Pressure, Residual);
		double Sigma = 0; float Maximum = 0;
		for (int I = 0; I < Count; ++I)
		{
			if (m_Solid[I] || Diagonal[I] <= 0) { Residual[I] = Search[I] = 0; continue; }
			Residual[I] = -Residual[I] - Divergence(I); Search[I] = Residual[I] / Diagonal[I];
			Sigma += (double)Residual[I] * Search[I]; Maximum = std::max(Maximum, std::fabs(Residual[I]));
		}
		const float Tolerance = std::max(1e-5f, Maximum * 1e-4f);
		for (int Iteration = 0; Iteration < std::max(80, m_Settings.PressureIterations * 2) && Maximum > Tolerance; ++Iteration)
		{
			FluidParallelFor(Count, [&](int I) {
				if (m_Solid[I]) { Product[I] = 0; return; }
				float V = m_Diagonal[I] * Search[I];
				for (int D = 0; D < 6; ++D) if (m_Neighbors[I][D] >= 0) V -= Weight[D / 2] * Search[m_Neighbors[I][D]];
				Product[I] = V;
			});
			SurfaceProduct(Search, Product);
			double Denominator = 0; for (int I = 0; I < Count; ++I) Denominator += (double)Search[I] * Product[I];
			if (!(Denominator > 0) || !(Sigma > 0)) break;
			const float Alpha = (float)(Sigma / Denominator); double NextSigma = 0; Maximum = 0;
			for (int I = 0; I < Count; ++I) if (!m_Solid[I] && Diagonal[I] > 0)
			{
				Correction[I] += Alpha * Search[I]; Residual[I] -= Alpha * Product[I];
				NextSigma += (double)Residual[I] * Residual[I] / Diagonal[I]; Maximum = std::max(Maximum, std::fabs(Residual[I]));
			}
			const float Beta = (float)(NextSigma / Sigma);
			for (int I = 0; I < Count; ++I) if (!m_Solid[I] && Diagonal[I] > 0) Search[I] = Residual[I] / Diagonal[I] + Beta * Search[I];
			Sigma = NextSigma;
		}
		if (!std::isfinite(Maximum)) return false;
		FluidParallelFor(Count, [&](int I) {
			m_Pressure[I] += Correction[I];
			if (m_Solid[I]) return;
			for (int A = 0; A < 3; ++A)
			{
				const int N = m_Neighbors[I][2 * A + 1];
				if (N != -1) m_Velocity[I][A] -= ((N >= 0 ? Correction[N] : 0) - Correction[I]) / m_CellSize[A];
			}
		});
		ResponseFor(m_Pressure);
		for (const auto& F : m_BoundaryFaces) if (F.Triangle >= 0)
		{
			const size_t I = F.Owner - m_Obstacles.size(); const auto& T = m_Surfaces[I].Triangles[F.Triangle];
			float Velocity = 0; for (int A = 0; A < 3; ++A) Velocity += Response[I][T[A]][F.Axis] * F.Weights[A];
			const int Stride[] = { 1, m_Settings.Resolution.x, m_Settings.Resolution.x * m_Settings.Resolution.y };
			const int Face = F.Sign > 0 ? F.Cell - Stride[F.Axis] : F.Cell;
			m_BoundaryVelocity[Face][F.Axis] += Velocity;
		}
		for (size_t I = 0; I < Response.size(); ++I) for (size_t J = 0; J < Response[I].size(); ++J) m_Surfaces[I].Velocities[J] += Response[I][J];
		Boundaries();
		return true;
	}

	void AerodynamicsIncompressibleSolver::Step(float DeltaTime) { Advance(DeltaTime, false); }
	bool AerodynamicsIncompressibleSolver::StepSynchronized(float DeltaTime)
	{
		if (m_PendingTime > 1e-7) return false;
		return Advance(DeltaTime, true);
	}
	float AerodynamicsIncompressibleSolver::GetStableTimeStep() const
	{
		float Rate = m_GPU ? m_GPU->GetMaxRate() : 0;
		if (!m_GPU) for (const auto& V : m_Velocity) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, std::fabs(V[A]) / m_CellSize[A]);
		for (const auto& E : m_Settings.Emitters) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, (std::fabs(E.Velocity[A]) + 0.2f) / m_CellSize[A]);
		// Limited semi-Lagrangian transport accepts characteristics spanning more than one cell.
		// World boundary motion keeps its separate conservative displacement limit.
		return std::min(m_Settings.MaxTimeStep, 1.5f / std::max(Rate, 1e-5f));
	}
	bool AerodynamicsIncompressibleSolver::Advance(float DeltaTime, bool Synchronized)
	{
		if (!m_Initialized || !std::isfinite(DeltaTime) || DeltaTime <= 0) return false;
		m_HasAveragedLoads = false;
		const double StartTime = m_Time;
		bool DynamicSurface = false;
		for (const auto& S : m_Surfaces) for (float M : S.InverseMasses) DynamicSurface |= M > 0;
		DynamicSurface &= std::any_of(m_BoundaryFaces.begin(), m_BoundaryFaces.end(), [](const BoundaryFace& F) { return F.Triangle >= 0; });
		m_PendingTime += DeltaTime;
		m_AveragedRigidLoads.assign(m_Obstacles.size() - m_Settings.Obstacles.size(), AerodynamicsRigidLoad());
		m_AveragedSurfaceLoads.resize(m_Surfaces.size());
		for (size_t I = 0; I < m_Surfaces.size(); ++I) m_AveragedSurfaceLoads[I].assign(m_Surfaces[I].Positions.size(), Vector3::Zero());
		auto Accumulate = [&](float Dt)
		{
			if (!Synchronized) return true;
			std::vector<AerodynamicsRigidLoad> R; std::vector<std::vector<Vector3>> C;
			if (!ReadLoads(R, C)) return false;
			for (size_t I = 0; I < R.size(); ++I) { m_AveragedRigidLoads[I].Force += R[I].Force * Dt; m_AveragedRigidLoads[I].Torque += R[I].Torque * Dt; }
			for (size_t I = 0; I < C.size(); ++I) for (size_t J = 0; J < C[I].size(); ++J) m_AveragedSurfaceLoads[I][J] += C[I][J] * Dt;
			return true;
		};
		for (int Substep = 0; Substep < (Synchronized ? 4096 : m_Settings.MaxSubsteps) && m_PendingTime > 1e-7; ++Substep)
		{
			const float Dt = std::min((float)m_PendingTime, GetStableTimeStep());
			m_LastStepDt = Dt; m_PressureDirty = true;
			if (m_GPU)
			{
				m_GPU->Step(Dt, (float)m_Time, m_Surfaces.empty(), !DynamicSurface);
				if (DynamicSurface)
				{
					if (!ProjectSurfaceMass()) return false;
				}
				if (!m_Surfaces.empty()) m_GPU->TransportScalars();
				m_Time += Dt; m_PendingTime -= Dt; m_VelocityDirty = true;
				if (!m_GPU->Readback(m_Density, m_Temperature)) { m_Initialized = false; return false; }
				if (!Accumulate(Dt)) return false;
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
			Emit(Dt); AddForces(Dt);
			if (DynamicSurface) { Boundaries(); std::fill(m_Pressure.begin(), m_Pressure.end(), 0.0f); } else Project();
			if (DynamicSurface && !ProjectSurfaceMass()) return false;
			AdvectScalars(Dt); Boundaries();
			m_Time += Dt; m_PendingTime -= Dt;
			if (!Accumulate(Dt)) return false;
		}
		if (Synchronized)
		{
			const float Time = (float)(m_Time - StartTime);
			if (m_PendingTime > 1e-7 || !(Time > 0)) return false;
			for (auto& R : m_AveragedRigidLoads) { R.Force /= Time; R.Torque /= Time; }
			for (auto& C : m_AveragedSurfaceLoads) for (auto& F : C) F /= Time;
			m_HasAveragedLoads = true;
		}
		return true;
	}

	bool AerodynamicsIncompressibleSolver::ApplyVelocityImpulse(const Vector3& Position, float Radius, const Vector3& VelocityChange)
	{
		if (!m_Initialized || !Finite(Position) || !Finite(VelocityChange) || !std::isfinite(Radius) || Radius <= 0) return false;
		const Vector3 Change = VelocityChange;
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

	float AerodynamicsIncompressibleSolver::GetDivergenceRMS() const
	{
		if (!m_Initialized) return 0;
		GetVelocity();
		double Sum = 0; size_t Count = 0;
		for (int I = 0; I < (int)m_Density.size(); ++I)
			if (!m_Solid[I]) { const float D = Divergence(I); Sum += D * D; ++Count; }
		return Count ? (float)std::sqrt(Sum / Count) : 0;
	}

	const std::vector<Vector3>& AerodynamicsIncompressibleSolver::GetVelocity() const
	{
		if (m_GPU && m_VelocityDirty && m_GPU->ReadVelocity(m_Velocity)) m_VelocityDirty = false;
		return m_Velocity;
	}

	const std::vector<float>& AerodynamicsIncompressibleSolver::GetGridDensity() const
	{
		m_AirDensity.resize(m_Solid.size());
		for (size_t I = 0; I < m_Solid.size(); ++I) m_AirDensity[I] = m_Solid[I] ? 0 : m_Settings.AmbientDensity;
		return m_AirDensity;
	}
	const std::vector<float>& AerodynamicsIncompressibleSolver::GetGridPressure() const
	{
		if (m_GPU && m_PressureDirty)
		{
			m_PressureReadback.resize(m_Pressure.size());
			if (!m_GPU->ReadPressure(m_PressureReadback)) return m_AbsolutePressure;
			m_PressureDirty = false;
		}
		const auto& Pressure = m_GPU ? m_PressureReadback : m_Pressure;
		m_AbsolutePressure.assign(m_Pressure.size(), m_Settings.AmbientPressure);
		if (m_LastStepDt > 0) for (size_t I = 0; I < Pressure.size(); ++I)
			m_AbsolutePressure[I] += Pressure[I] * (m_Settings.AmbientDensity / m_LastStepDt);
		return m_AbsolutePressure;
	}
	AerodynamicSample AerodynamicsIncompressibleSolver::Sample(const Vector3& Position) const
	{
		AerodynamicSample Result; Result.Density = m_Settings.AmbientDensity; Result.Pressure = m_Settings.AmbientPressure;
		if (!m_Initialized || !Finite(Position)) return Result;
		for (int A = 0; A < 3; ++A) if (Position[A] < m_Settings.BoundsMin[A] || Position[A] > m_Settings.BoundsMax[A]) return Result;
		const Vector3 P = Divide(Position - m_Settings.BoundsMin, m_CellSize) - Vector3(0.5f);
		Result.Velocity = Velocity(GetVelocity(), P); Result.TracerDensity = Scalar(m_Density, P); Result.Temperature = Scalar(m_Temperature, P);
		if (m_LastStepDt <= 0) return Result;
		if (m_GPU && m_PressureDirty)
		{
			m_PressureReadback.resize(m_Pressure.size());
			if (!m_GPU->ReadPressure(m_PressureReadback)) return Result;
			m_PressureDirty = false;
		}
		const auto& Pressure = m_GPU ? m_PressureReadback : m_Pressure;
		const float Scale = m_Settings.AmbientDensity / m_LastStepDt;
		Result.Pressure += Scalar(Pressure, P) * Scale;
		for (int A = 0; A < 3; ++A)
		{
			Vector3 Lo = P, Hi = P; Lo[A] = std::max(0.0f, P[A] - 1); Hi[A] = std::min((float)m_Settings.Resolution[A] - 1, P[A] + 1);
			const float Width = (Hi[A] - Lo[A]) * m_CellSize[A];
			Result.PressureGradient[A] = Width > 0 ? (Scalar(Pressure, Hi) - Scalar(Pressure, Lo)) * Scale / Width : 0;
		}
		return Result;
	}
	float AerodynamicsIncompressibleSolver::GetImpulseMass(const Vector3& Position, float Radius) const
	{
		double WeightSum = 0;
		for (int I = 0; I < (int)m_Solid.size(); ++I)
		{
			if (m_Solid[I]) continue;
			const auto G = GridPosition(I) + Vector3(0.5f);
			const auto P = m_Settings.BoundsMin + Vector3(G.x * m_CellSize.x, G.y * m_CellSize.y, G.z * m_CellSize.z);
			const float W = std::max(0.0f, 1 - (P - Position).SquareLength() / (Radius * Radius));
			WeightSum += W * W;
		}
		return (float)(WeightSum * m_Settings.AmbientDensity * m_CellSize.x * m_CellSize.y * m_CellSize.z);
	}
}

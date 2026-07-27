#include "FluidSimulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Riemann
{
	namespace
	{
		const float Pi = 3.14159265358979323846f;
		const float DensityEpsilon = 1.0e-5f;

		float ClampFloat(float Value, float Min, float Max)
		{
			return std::max(Min, std::min(Max, Value));
		}

		int ClampInt(int Value, int Min, int Max)
		{
			return std::max(Min, std::min(Max, Value));
		}

		bool IsFiniteVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		bool IsFiniteFloat(float Value)
		{
			return std::isfinite(Value);
		}

		double SmoothCompactSupportWeight(const Vector3& Point, const Vector3& Center, float Radius)
		{
			const double DX = (double)Point.x - (double)Center.x;
			const double DY = (double)Point.y - (double)Center.y;
			const double DZ = (double)Point.z - (double)Center.z;
			const double DistanceSquared = DX * DX + DY * DY + DZ * DZ;
			const double RadiusSquared = (double)Radius * (double)Radius;
			if (!(DistanceSquared < RadiusSquared)) return 0.0;
			const double Falloff = 1.0 - DistanceSquared / RadiusSquared;
			return Falloff * Falloff * Falloff;
		}

		uint64_t MixBucketCoordinate(uint64_t Value)
		{
			Value ^= Value >> 30;
			Value *= 0xbf58476d1ce4e5b9ULL;
			Value ^= Value >> 27;
			Value *= 0x94d049bb133111ebULL;
			return Value ^ (Value >> 31);
		}

		bool CalculateLatticeCounts(const Vector3& Start, const Vector3& End, float Spacing,
			size_t ExistingCount, size_t* CountX, size_t* CountY, size_t* CountZ)
		{
			*CountX = *CountY = *CountZ = 0;
			if (Start.x > End.x || Start.y > End.y || Start.z > End.z)
			{
				return true;
			}
			auto CountAxis = [Spacing](float AxisStart, float AxisEnd)
			{
				double Ratio = ((double)AxisEnd - (double)AxisStart) / (double)Spacing;
				const double Nearest = std::round(Ratio);
				const double Tolerance = 8.0 * (double)std::numeric_limits<float>::epsilon()
					* std::max(1.0, std::fabs(Ratio));
				if (std::fabs(Ratio - Nearest) <= Tolerance) Ratio = Nearest;
				return std::floor(Ratio) + 1.0;
			};
			const double Counts[3] = { CountAxis(Start.x, End.x), CountAxis(Start.y, End.y),
				CountAxis(Start.z, End.z) };
			const size_t MaxParticleCount = (size_t)4 * 1024 * 1024;
			if (Counts[0] < 0.0 || Counts[1] < 0.0 || Counts[2] < 0.0
				|| Counts[0] > (double)MaxParticleCount || Counts[1] > (double)MaxParticleCount
				|| Counts[2] > (double)MaxParticleCount)
			{
				return false;
			}
			*CountX = (size_t)Counts[0];
			*CountY = (size_t)Counts[1];
			*CountZ = (size_t)Counts[2];
			if (*CountX != 0 && *CountY > MaxParticleCount / *CountX)
			{
				return false;
			}
			const size_t CountXY = *CountX * *CountY;
			if (CountXY != 0 && *CountZ > MaxParticleCount / CountXY)
			{
				return false;
			}
			const size_t NewCount = CountXY * *CountZ;
			return ExistingCount <= MaxParticleCount && NewCount <= MaxParticleCount - ExistingCount;
		}
	}

	FluidSimulationSettings::FluidSimulationSettings()
		: Method(FluidSimulationMethod::Eulerian)
		, Backend(FluidBackend::CPU)
		, AllowBackendFallback(true)
		, BoundsMin(-1.0f, -1.0f, -1.0f)
		, BoundsMax(1.0f, 1.0f, 1.0f)
		, Resolution(24, 24, 24)
		, Gravity(0.0f, -9.81f, 0.0f)
		, RestDensity(1000.0f)
		, DynamicViscosity(0.001f)
		, PressureStiffness(2500.0f)
		, ParticleSpacing(0.08f)
		, SmoothingRadius(0.16f)
		, BoundaryRestitution(0.05f)
		, BoundaryFriction(0.02f)
		, OpenTopBoundary(false)
		, MaxTimeStep(1.0f / 120.0f)
		, MaxSubsteps(256)
		, PressureIterations(32)
	{
	}

	FluidParticle::FluidParticle()
		: Position(Vector3::Zero())
		, Velocity(Vector3::Zero())
		, Acceleration(Vector3::Zero())
		, Mass(0.0f)
		, Density(0.0f)
		, Pressure(0.0f)
	{
	}

	FluidParticle::FluidParticle(const Vector3& InPosition, const Vector3& InVelocity, float InMass)
		: Position(InPosition)
		, Velocity(InVelocity)
		, Acceleration(Vector3::Zero())
		, Mass(InMass)
		, Density(0.0f)
		, Pressure(0.0f)
	{
	}

	FluidSimulation::FluidSimulation()
		: m_CellSize(Vector3::Zero())
		, m_ContainerTransform(Transform::Identity())
		, m_Initialized(false)
		, m_TimeRemainder(0.0f)
	{
	}

	FluidSimulation::FluidSimulation(const FluidSimulationSettings& Settings)
		: m_CellSize(Vector3::Zero())
		, m_ContainerTransform(Transform::Identity())
		, m_Initialized(false)
		, m_TimeRemainder(0.0f)
	{
		Initialize(Settings);
	}

	bool FluidSimulation::Initialize(const FluidSimulationSettings& Settings)
	{
		const Vector3 Extent = Settings.BoundsMax - Settings.BoundsMin;
		const bool ValidMethod = Settings.Method == FluidSimulationMethod::Eulerian
			|| Settings.Method == FluidSimulationMethod::Lagrangian;
		if (!ValidMethod || !IsFiniteVector(Settings.BoundsMin) || !IsFiniteVector(Settings.BoundsMax)
			|| !IsFiniteVector(Extent)
			|| !IsFiniteVector(Settings.Gravity) || !IsFiniteFloat(Settings.RestDensity)
			|| !IsFiniteFloat(Settings.DynamicViscosity) || !IsFiniteFloat(Settings.PressureStiffness)
			|| !IsFiniteFloat(Settings.ParticleSpacing) || !IsFiniteFloat(Settings.SmoothingRadius)
			|| !IsFiniteFloat(Settings.BoundaryRestitution) || !IsFiniteFloat(Settings.BoundaryFriction)
			|| !IsFiniteFloat(Settings.MaxTimeStep)
			|| !(Extent.x > 0.0f && Extent.y > 0.0f && Extent.z > 0.0f)
			|| Settings.Resolution.x < 3 || Settings.Resolution.y < 3 || Settings.Resolution.z < 3
			|| Settings.Resolution.x > 2048 || Settings.Resolution.y > 2048 || Settings.Resolution.z > 2048
			|| Settings.RestDensity <= 0.0f || Settings.RestDensity > 1.0e9f
			|| Settings.DynamicViscosity < 0.0f || Settings.DynamicViscosity > 1.0e9f
			|| Settings.PressureStiffness <= 0.0f || Settings.PressureStiffness > 1.0e12f
			|| Settings.ParticleSpacing <= 0.0f
			|| Settings.SmoothingRadius < Settings.ParticleSpacing
			|| Settings.ParticleSpacing > Extent.x || Settings.ParticleSpacing > Extent.y
			|| Settings.ParticleSpacing > Extent.z
			|| Settings.BoundaryRestitution < 0.0f || Settings.BoundaryRestitution > 1.0f
			|| Settings.BoundaryFriction < 0.0f || Settings.BoundaryFriction > 1.0f
			|| Settings.MaxTimeStep < 1.0e-6f || Settings.MaxSubsteps < 1
			|| Settings.MaxSubsteps > 4096 || Settings.PressureIterations < 1
			|| Settings.PressureIterations > 4096)
		{
			return false;
		}

		const size_t ResolutionX = (size_t)Settings.Resolution.x;
		const size_t ResolutionY = (size_t)Settings.Resolution.y;
		const size_t ResolutionZ = (size_t)Settings.Resolution.z;
		const size_t MaxCellCount = (size_t)16 * 1024 * 1024;
		if (ResolutionX > MaxCellCount / ResolutionY
			|| ResolutionX * ResolutionY > MaxCellCount / ResolutionZ)
		{
			return false;
		}
		const size_t CellCount = ResolutionX * ResolutionY * ResolutionZ;
		const double ParticleCountX = std::ceil((double)Extent.x / (double)Settings.ParticleSpacing);
		const double ParticleCountY = std::ceil((double)Extent.y / (double)Settings.ParticleSpacing);
		const double ParticleCountZ = std::ceil((double)Extent.z / (double)Settings.ParticleSpacing);
		if (ParticleCountX > 1024.0 || ParticleCountY > 1024.0 || ParticleCountZ > 1024.0
			|| ParticleCountX * ParticleCountY * ParticleCountZ > 4.0 * 1024.0 * 1024.0)
		{
			return false;
		}

		m_Settings = Settings;
		m_BackendResult.RequestedBackend = Settings.Backend;
		m_BackendResult.Backend = IsFluidBackendAvailable(Settings.Backend)
			? Settings.Backend : FluidBackend::CPU;
		m_BackendResult.UsedFallback = m_BackendResult.Backend != Settings.Backend;
		if (m_BackendResult.UsedFallback && !Settings.AllowBackendFallback)
		{
			m_Initialized = false;
			return false;
		}
		m_CellSize = Vector3(Extent.x / (float)Settings.Resolution.x,
			Extent.y / (float)Settings.Resolution.y, Extent.z / (float)Settings.Resolution.z);
		m_ContainerTransform = Transform::Identity();
		m_GridDensity.assign(CellCount, 0.0f);
		m_GridVelocity.assign(CellCount, Vector3::Zero());
		m_Pressure.assign(CellCount, 0.0f);
		m_Divergence.assign(CellCount, 0.0f);
		m_Particles.clear();
		m_ParticleBuckets.clear();
		m_TimeRemainder = 0.0f;
		m_Initialized = true;
		return true;
	}

	void FluidSimulation::Reset()
	{
		std::fill(m_GridDensity.begin(), m_GridDensity.end(), 0.0f);
		std::fill(m_GridVelocity.begin(), m_GridVelocity.end(), Vector3::Zero());
		std::fill(m_Pressure.begin(), m_Pressure.end(), 0.0f);
		std::fill(m_Divergence.begin(), m_Divergence.end(), 0.0f);
		m_Particles.clear();
		m_ParticleBuckets.clear();
		m_TimeRemainder = 0.0f;
	}

	bool FluidSimulation::SetMethod(FluidSimulationMethod Method, bool PreserveState)
	{
		if (!m_Initialized || (Method != FluidSimulationMethod::Eulerian
			&& Method != FluidSimulationMethod::Lagrangian))
		{
			return false;
		}
		if (Method == m_Settings.Method)
		{
			return true;
		}

		if (PreserveState)
		{
			if (Method == FluidSimulationMethod::Lagrangian)
			{
				ConvertEulerianToLagrangian();
			}
			else
			{
				ConvertLagrangianToEulerian();
			}
		}
		else
		{
			Reset();
		}
		m_Settings.Method = Method;
		return true;
	}

	int FluidSimulation::GridIndex(int X, int Y, int Z) const
	{
		return X + m_Settings.Resolution.x * (Y + m_Settings.Resolution.y * Z);
	}

	bool FluidSimulation::IsGridCoordinateValid(int X, int Y, int Z) const
	{
		return X >= 0 && X < m_Settings.Resolution.x
			&& Y >= 0 && Y < m_Settings.Resolution.y
			&& Z >= 0 && Z < m_Settings.Resolution.z;
	}

	Vector3 FluidSimulation::GridCellCenter(int X, int Y, int Z) const
	{
		return m_Settings.BoundsMin + Vector3(
			((float)X + 0.5f) * m_CellSize.x,
			((float)Y + 0.5f) * m_CellSize.y,
			((float)Z + 0.5f) * m_CellSize.z);
	}

	Vector3 FluidSimulation::GridFaceCenter(int X, int Y, int Z, int Component) const
	{
		Vector3 Position = GridCellCenter(X, Y, Z);
		if (Component == 0) Position.x += 0.5f * m_CellSize.x;
		if (Component == 1) Position.y += 0.5f * m_CellSize.y;
		if (Component == 2) Position.z += 0.5f * m_CellSize.z;
		return Position;
	}

	void FluidSimulation::AddFluidBox(const Vector3& Min, const Vector3& Max,
		const Vector3& InitialVelocity)
	{
		if (!m_Initialized || !IsFiniteVector(Min) || !IsFiniteVector(Max)
			|| !IsFiniteVector(InitialVelocity))
		{
			return;
		}

		const Vector3 ClippedMin = Min.Max(m_Settings.BoundsMin);
		const Vector3 ClippedMax = Max.Min(m_Settings.BoundsMax);
		if (!(ClippedMin.x < ClippedMax.x && ClippedMin.y < ClippedMax.y && ClippedMin.z < ClippedMax.z))
		{
			return;
		}

		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			std::vector<uint8_t> Added(m_GridDensity.size(), 0);
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			{
				for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
				{
					for (int X = 0; X < m_Settings.Resolution.x; ++X)
					{
						const Vector3 Center = GridCellCenter(X, Y, Z);
						if (Center.x >= ClippedMin.x && Center.x <= ClippedMax.x
							&& Center.y >= ClippedMin.y && Center.y <= ClippedMax.y
							&& Center.z >= ClippedMin.z && Center.z <= ClippedMax.z)
						{
							const int Index = GridIndex(X, Y, Z);
							m_GridDensity[(size_t)Index] = m_Settings.RestDensity;
							Added[(size_t)Index] = 1;
						}
					}
				}
			}
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			{
				for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
				{
					for (int X = 0; X < m_Settings.Resolution.x; ++X)
					{
						Vector3& FaceVelocity = m_GridVelocity[(size_t)GridIndex(X, Y, Z)];
						const bool AddedHere = Added[(size_t)GridIndex(X, Y, Z)] != 0;
						const bool AddedX = X + 1 < m_Settings.Resolution.x
							&& Added[(size_t)GridIndex(X + 1, Y, Z)] != 0;
						const bool AddedY = Y + 1 < m_Settings.Resolution.y
							&& Added[(size_t)GridIndex(X, Y + 1, Z)] != 0;
						const bool AddedZ = Z + 1 < m_Settings.Resolution.z
							&& Added[(size_t)GridIndex(X, Y, Z + 1)] != 0;
						if (AddedHere || AddedX) FaceVelocity.x = InitialVelocity.x;
						if (AddedHere || AddedY) FaceVelocity.y = InitialVelocity.y;
						if (AddedHere || AddedZ) FaceVelocity.z = InitialVelocity.z;
					}
				}
			}
			EnforceEulerianBoundaries();
			return;
		}

		const float Spacing = m_Settings.ParticleSpacing;
		const float ParticleMass = m_Settings.RestDensity * Spacing * Spacing * Spacing;
		const Vector3 Start = ClippedMin + Vector3(0.5f * Spacing);
		const Vector3 End = ClippedMax - Vector3(0.5f * Spacing);
		size_t CountX = 0, CountY = 0, CountZ = 0;
		if (!CalculateLatticeCounts(Start, End, Spacing, m_Particles.size(), &CountX, &CountY, &CountZ))
		{
			return;
		}
		m_Particles.reserve(m_Particles.size() + CountX * CountY * CountZ);
		for (size_t IZ = 0; IZ < CountZ; ++IZ)
		{
			const float Z = std::min(End.z, (float)((double)Start.z + (double)IZ * (double)Spacing));
			for (size_t IY = 0; IY < CountY; ++IY)
			{
				const float Y = std::min(End.y, (float)((double)Start.y + (double)IY * (double)Spacing));
				for (size_t IX = 0; IX < CountX; ++IX)
				{
					const float X = std::min(End.x, (float)((double)Start.x + (double)IX * (double)Spacing));
					m_Particles.push_back(FluidParticle(Vector3(X, Y, Z), InitialVelocity, ParticleMass));
				}
			}
		}
		ComputeParticleDensities();
	}

	void FluidSimulation::AddFluidSphere(const Vector3& Center, float Radius,
		const Vector3& InitialVelocity)
	{
		if (!m_Initialized || !IsFiniteVector(Center) || !IsFiniteFloat(Radius)
			|| !IsFiniteVector(InitialVelocity) || Radius <= 0.0f)
		{
			return;
		}

		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			const float RadiusSquared = Radius * Radius;
			std::vector<uint8_t> Added(m_GridDensity.size(), 0);
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			{
				for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
				{
					for (int X = 0; X < m_Settings.Resolution.x; ++X)
					{
						if ((GridCellCenter(X, Y, Z) - Center).SquareLength() <= RadiusSquared)
						{
							const int Index = GridIndex(X, Y, Z);
							m_GridDensity[(size_t)Index] = m_Settings.RestDensity;
							Added[(size_t)Index] = 1;
						}
					}
				}
			}
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			{
				for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
				{
					for (int X = 0; X < m_Settings.Resolution.x; ++X)
					{
						Vector3& FaceVelocity = m_GridVelocity[(size_t)GridIndex(X, Y, Z)];
						const bool AddedHere = Added[(size_t)GridIndex(X, Y, Z)] != 0;
						const bool AddedX = X + 1 < m_Settings.Resolution.x
							&& Added[(size_t)GridIndex(X + 1, Y, Z)] != 0;
						const bool AddedY = Y + 1 < m_Settings.Resolution.y
							&& Added[(size_t)GridIndex(X, Y + 1, Z)] != 0;
						const bool AddedZ = Z + 1 < m_Settings.Resolution.z
							&& Added[(size_t)GridIndex(X, Y, Z + 1)] != 0;
						if (AddedHere || AddedX) FaceVelocity.x = InitialVelocity.x;
						if (AddedHere || AddedY) FaceVelocity.y = InitialVelocity.y;
						if (AddedHere || AddedZ) FaceVelocity.z = InitialVelocity.z;
					}
				}
			}
			EnforceEulerianBoundaries();
			return;
		}

		const float Spacing = m_Settings.ParticleSpacing;
		const float RadiusSquared = Radius * Radius;
		const float ParticleMass = m_Settings.RestDensity * Spacing * Spacing * Spacing;
		const Vector3 Start = (Center - Vector3(Radius)).Max(m_Settings.BoundsMin) + Vector3(0.5f * Spacing);
		const Vector3 End = (Center + Vector3(Radius)).Min(m_Settings.BoundsMax) - Vector3(0.5f * Spacing);
		size_t CountX = 0, CountY = 0, CountZ = 0;
		if (!CalculateLatticeCounts(Start, End, Spacing, m_Particles.size(), &CountX, &CountY, &CountZ))
		{
			return;
		}
		m_Particles.reserve(m_Particles.size() + CountX * CountY * CountZ);
		for (size_t IZ = 0; IZ < CountZ; ++IZ)
		{
			const float Z = std::min(End.z, (float)((double)Start.z + (double)IZ * (double)Spacing));
			for (size_t IY = 0; IY < CountY; ++IY)
			{
				const float Y = std::min(End.y, (float)((double)Start.y + (double)IY * (double)Spacing));
				for (size_t IX = 0; IX < CountX; ++IX)
				{
					const float X = std::min(End.x, (float)((double)Start.x + (double)IX * (double)Spacing));
					const Vector3 Position(X, Y, Z);
					if ((Position - Center).SquareLength() <= RadiusSquared)
					{
						m_Particles.push_back(FluidParticle(Position, InitialVelocity, ParticleMass));
					}
				}
			}
		}
		ComputeParticleDensities();
	}

	bool FluidSimulation::SetContainerTransform(const Transform& ContainerTransform)
	{
		if (!IsFiniteVector(ContainerTransform.pos)
			|| !std::isfinite(ContainerTransform.quat.x)
			|| !std::isfinite(ContainerTransform.quat.y)
			|| !std::isfinite(ContainerTransform.quat.z)
			|| !std::isfinite(ContainerTransform.quat.w)
			|| ContainerTransform.quat.SquareLength() <= 1.0e-12f)
		{
			return false;
		}
		m_ContainerTransform = Transform(ContainerTransform.pos, ContainerTransform.quat.Unit());
		return true;
	}

	bool FluidSimulation::ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Impulse)
	{
		return ApplyLocalImpulse(m_ContainerTransform.WorldToLocal(Position), Radius,
			m_ContainerTransform.WorldToLocalDirection(Impulse));
	}

	bool FluidSimulation::ApplyLocalImpulse(const Vector3& Position, float Radius,
		const Vector3& Impulse)
	{
		if (!m_Initialized || !IsFiniteVector(Position) || !IsFiniteFloat(Radius)
			|| !IsFiniteVector(Impulse) || !(Radius > 0.0f))
		{
			return false;
		}

		const double MaximumFloat = (double)std::numeric_limits<float>::max();
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			const double CellVolume = (double)m_CellSize.x * (double)m_CellSize.y
				* (double)m_CellSize.z;
			if (!(CellVolume > 0.0) || !std::isfinite(CellVolume)) return false;
			const bool Requested[3] = { Impulse.x != 0.0f, Impulse.y != 0.0f,
				Impulse.z != 0.0f };
			if (!Requested[0] && !Requested[1] && !Requested[2]) return true;

			int Begin[3] = { 0, 0, 0 };
			int End[3] = { -1, -1, -1 };
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				const double MinimumIndex = std::ceil(((double)Position[Axis] - (double)Radius
					- (double)m_Settings.BoundsMin[Axis]) / (double)m_CellSize[Axis] - 0.5);
				const double MaximumIndex = std::floor(((double)Position[Axis] + (double)Radius
					- (double)m_Settings.BoundsMin[Axis]) / (double)m_CellSize[Axis] - 0.5);
				const int MaximumGridIndex = m_Settings.Resolution[Axis] - 1;
				if (MaximumIndex < 0.0 || MinimumIndex > (double)MaximumGridIndex) return false;
				Begin[Axis] = (int)std::max(0.0, MinimumIndex);
				End[Axis] = (int)std::min((double)MaximumGridIndex, MaximumIndex);
			}

			const float ActiveThreshold = m_Settings.RestDensity * 0.001f;
			double WeightedMass[3] = { 0.0, 0.0, 0.0 };
			for (int Z = Begin[2]; Z <= End[2]; ++Z)
			for (int Y = Begin[1]; Y <= End[1]; ++Y)
			for (int X = Begin[0]; X <= End[0]; ++X)
			{
				const int Index = GridIndex(X, Y, Z);
				const float Density = m_GridDensity[(size_t)Index];
				if (!std::isfinite(Density) || Density <= ActiveThreshold) continue;
				const double Weight = SmoothCompactSupportWeight(GridCellCenter(X, Y, Z), Position, Radius);
				if (!(Weight > 0.0)) continue;
				const int Coordinate[3] = { X, Y, Z };
				for (int Component = 0; Component < 3; ++Component)
				{
					if (!Requested[Component]
						|| Coordinate[Component] + 1 == m_Settings.Resolution[Component]
						|| !IsFaceFluid(X, Y, Z, Component, ActiveThreshold))
					{
						continue;
					}
					WeightedMass[Component] += (double)Density * CellVolume * Weight;
				}
			}
			for (int Component = 0; Component < 3; ++Component)
			{
				if (Requested[Component] && (!(WeightedMass[Component] > 0.0)
					|| !std::isfinite(WeightedMass[Component]))) return false;
			}

			for (int Z = Begin[2]; Z <= End[2]; ++Z)
			for (int Y = Begin[1]; Y <= End[1]; ++Y)
			for (int X = Begin[0]; X <= End[0]; ++X)
			{
				const int Index = GridIndex(X, Y, Z);
				const float Density = m_GridDensity[(size_t)Index];
				if (!std::isfinite(Density) || Density <= ActiveThreshold) continue;
				const double Weight = SmoothCompactSupportWeight(GridCellCenter(X, Y, Z), Position, Radius);
				if (!(Weight > 0.0)) continue;
				const Vector3& Velocity = m_GridVelocity[(size_t)Index];
				const int Coordinate[3] = { X, Y, Z };
				for (int Component = 0; Component < 3; ++Component)
				{
					if (!Requested[Component]
						|| Coordinate[Component] + 1 == m_Settings.Resolution[Component]
						|| !IsFaceFluid(X, Y, Z, Component, ActiveThreshold))
					{
						continue;
					}
					const double NewValue = (double)Velocity[Component]
						+ (double)Impulse[Component] * Weight / WeightedMass[Component];
					if (!std::isfinite(NewValue) || std::fabs(NewValue) > MaximumFloat) return false;
				}
			}

			for (int Z = Begin[2]; Z <= End[2]; ++Z)
			for (int Y = Begin[1]; Y <= End[1]; ++Y)
			for (int X = Begin[0]; X <= End[0]; ++X)
			{
				const int Index = GridIndex(X, Y, Z);
				const float Density = m_GridDensity[(size_t)Index];
				if (!std::isfinite(Density) || Density <= ActiveThreshold) continue;
				const double Weight = SmoothCompactSupportWeight(GridCellCenter(X, Y, Z), Position, Radius);
				if (!(Weight > 0.0)) continue;
				Vector3& Velocity = m_GridVelocity[(size_t)Index];
				const int Coordinate[3] = { X, Y, Z };
				for (int Component = 0; Component < 3; ++Component)
				{
					if (!Requested[Component]
						|| Coordinate[Component] + 1 == m_Settings.Resolution[Component]
						|| !IsFaceFluid(X, Y, Z, Component, ActiveThreshold))
					{
						continue;
					}
					Velocity[Component] = (float)((double)Velocity[Component]
						+ (double)Impulse[Component] * Weight / WeightedMass[Component]);
				}
			}
			EnforceEulerianBoundaries();
			return true;
		}

		if (m_Particles.empty()) return false;
		if (m_ParticleBuckets.empty()) RebuildParticleBuckets();
		const float BucketSize = m_Settings.SmoothingRadius;
		int MinimumBucket[3];
		int MaximumBucket[3];
		double QueryBucketCount = 1.0;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const int DomainMaximum = std::max(0, (int)std::floor(
				((double)m_Settings.BoundsMax[Axis] - (double)m_Settings.BoundsMin[Axis])
				/ (double)BucketSize));
			const double QueryMinimum = std::floor(((double)Position[Axis] - (double)Radius
				- (double)m_Settings.BoundsMin[Axis]) / (double)BucketSize);
			const double QueryMaximum = std::floor(((double)Position[Axis] + (double)Radius
				- (double)m_Settings.BoundsMin[Axis]) / (double)BucketSize);
			if (QueryMaximum < 0.0 || QueryMinimum > (double)DomainMaximum) return false;
			MinimumBucket[Axis] = (int)std::max(0.0, QueryMinimum);
			MaximumBucket[Axis] = (int)std::min((double)DomainMaximum, QueryMaximum);
			QueryBucketCount *= (double)(MaximumBucket[Axis] - MinimumBucket[Axis] + 1);
		}

		std::vector<int> CandidateIndices;
		if (QueryBucketCount > std::max(64.0, 2.0 * (double)m_ParticleBuckets.size()))
		{
			CandidateIndices.resize(m_Particles.size());
			for (size_t ParticleIndex = 0; ParticleIndex < m_Particles.size(); ++ParticleIndex)
				CandidateIndices[ParticleIndex] = (int)ParticleIndex;
		}
		else
		{
			for (int Z = MinimumBucket[2]; Z <= MaximumBucket[2]; ++Z)
			for (int Y = MinimumBucket[1]; Y <= MaximumBucket[1]; ++Y)
			for (int X = MinimumBucket[0]; X <= MaximumBucket[0]; ++X)
			{
				const uint64_t Key = ParticleBucketKey(X, Y, Z);
				auto It = std::lower_bound(m_ParticleBuckets.begin(), m_ParticleBuckets.end(), Key,
					[](const ParticleBucket& Bucket, uint64_t Value) { return Bucket.Key < Value; });
				if (It != m_ParticleBuckets.end() && It->Key == Key)
					CandidateIndices.insert(CandidateIndices.end(), It->Indices.begin(), It->Indices.end());
			}
			std::sort(CandidateIndices.begin(), CandidateIndices.end());
			CandidateIndices.erase(std::unique(CandidateIndices.begin(), CandidateIndices.end()),
				CandidateIndices.end());
		}

		double WeightedMass = 0.0;
		for (int ParticleIndex : CandidateIndices)
		{
			const FluidParticle& Particle = m_Particles[(size_t)ParticleIndex];
			if (!IsFiniteVector(Particle.Position) || !std::isfinite(Particle.Mass)
				|| !(Particle.Mass > 0.0f))
			{
				continue;
			}
			const double Weight = SmoothCompactSupportWeight(Particle.Position, Position, Radius);
			if (Weight > 0.0) WeightedMass += (double)Particle.Mass * Weight;
		}
		if (!(WeightedMass > 0.0) || !std::isfinite(WeightedMass)) return false;

		for (int ParticleIndex : CandidateIndices)
		{
			const FluidParticle& Particle = m_Particles[(size_t)ParticleIndex];
			if (!IsFiniteVector(Particle.Position) || !std::isfinite(Particle.Mass)
				|| !(Particle.Mass > 0.0f))
			{
				continue;
			}
			const double Weight = SmoothCompactSupportWeight(Particle.Position, Position, Radius);
			if (!(Weight > 0.0)) continue;
			const double NewX = (double)Particle.Velocity.x + (double)Impulse.x * Weight / WeightedMass;
			const double NewY = (double)Particle.Velocity.y + (double)Impulse.y * Weight / WeightedMass;
			const double NewZ = (double)Particle.Velocity.z + (double)Impulse.z * Weight / WeightedMass;
			if (!std::isfinite(NewX) || !std::isfinite(NewY) || !std::isfinite(NewZ)
				|| std::fabs(NewX) > MaximumFloat || std::fabs(NewY) > MaximumFloat
				|| std::fabs(NewZ) > MaximumFloat)
			{
				return false;
			}
		}

		for (int ParticleIndex : CandidateIndices)
		{
			FluidParticle& Particle = m_Particles[(size_t)ParticleIndex];
			if (!IsFiniteVector(Particle.Position) || !std::isfinite(Particle.Mass)
				|| !(Particle.Mass > 0.0f))
			{
				continue;
			}
			const double Weight = SmoothCompactSupportWeight(Particle.Position, Position, Radius);
			if (!(Weight > 0.0)) continue;
			Particle.Velocity.x = (float)((double)Particle.Velocity.x
				+ (double)Impulse.x * Weight / WeightedMass);
			Particle.Velocity.y = (float)((double)Particle.Velocity.y
				+ (double)Impulse.y * Weight / WeightedMass);
			Particle.Velocity.z = (float)((double)Particle.Velocity.z
				+ (double)Impulse.z * Weight / WeightedMass);
		}
		return true;
	}

	void FluidSimulation::Step(float DeltaTime)
	{
		if (!m_Initialized || !(DeltaTime > 0.0f) || !std::isfinite(DeltaTime))
		{
			return;
		}

		m_TimeRemainder = DeltaTime > std::numeric_limits<float>::max() - m_TimeRemainder
			? std::numeric_limits<float>::max() : m_TimeRemainder + DeltaTime;
		const float MaximumDuration = m_Settings.MaxTimeStep * (float)m_Settings.MaxSubsteps;
		const float SimulatedDuration = std::min(m_TimeRemainder, MaximumDuration);
		const int SubstepCount = std::max(1, std::min(m_Settings.MaxSubsteps,
			(int)std::ceil(SimulatedDuration / m_Settings.MaxTimeStep)));
		const float TimeStep = SimulatedDuration / (float)SubstepCount;
		for (int Substep = 0; Substep < SubstepCount; ++Substep)
		{
			if (m_Settings.Method == FluidSimulationMethod::Eulerian)
			{
				StepEulerian(TimeStep);
			}
			else
			{
				StepLagrangian(TimeStep);
			}
			if (!m_Initialized) break;
		}
		m_TimeRemainder = std::max(0.0f, m_TimeRemainder - SimulatedDuration);
	}

	float FluidSimulation::SampleGridScalar(const float* Field, const Vector3& Position) const
	{
		if (Field == nullptr)
		{
			return 0.0f;
		}

		const Vector3 GridPosition(
			(Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
			(Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
			(Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
		const int X0 = ClampInt((int)std::floor(GridPosition.x), 0, m_Settings.Resolution.x - 1);
		const int Y0 = ClampInt((int)std::floor(GridPosition.y), 0, m_Settings.Resolution.y - 1);
		const int Z0 = ClampInt((int)std::floor(GridPosition.z), 0, m_Settings.Resolution.z - 1);
		const int X1 = std::min(X0 + 1, m_Settings.Resolution.x - 1);
		const int Y1 = std::min(Y0 + 1, m_Settings.Resolution.y - 1);
		const int Z1 = std::min(Z0 + 1, m_Settings.Resolution.z - 1);
		const float TX = ClampFloat(GridPosition.x - (float)X0, 0.0f, 1.0f);
		const float TY = ClampFloat(GridPosition.y - (float)Y0, 0.0f, 1.0f);
		const float TZ = ClampFloat(GridPosition.z - (float)Z0, 0.0f, 1.0f);

		const int RowStride = m_Settings.Resolution.x;
		const int SliceStride = RowStride * m_Settings.Resolution.y;
		const int Index000 = X0 + RowStride * Y0 + SliceStride * Z0;
		const int Index010 = X0 + RowStride * Y1 + SliceStride * Z0;
		const int Index001 = X0 + RowStride * Y0 + SliceStride * Z1;
		const int Index011 = X0 + RowStride * Y1 + SliceStride * Z1;
		const int DeltaX = X1 - X0;
		const float C00 = Field[Index000] * (1.0f - TX) + Field[Index000 + DeltaX] * TX;
		const float C10 = Field[Index010] * (1.0f - TX) + Field[Index010 + DeltaX] * TX;
		const float C01 = Field[Index001] * (1.0f - TX) + Field[Index001 + DeltaX] * TX;
		const float C11 = Field[Index011] * (1.0f - TX) + Field[Index011 + DeltaX] * TX;
		const float C0 = C00 * (1.0f - TY) + C10 * TY;
		const float C1 = C01 * (1.0f - TY) + C11 * TY;
		return C0 * (1.0f - TZ) + C1 * TZ;
	}

	Vector3 FluidSimulation::SampleGridVector(const Vector3* Field, const Vector3& Position) const
	{
		if (Field == nullptr)
		{
			return Vector3::Zero();
		}
		return Vector3(SampleGridVelocityComponent(Field, Position, 0),
			SampleGridVelocityComponent(Field, Position, 1),
			SampleGridVelocityComponent(Field, Position, 2));
	}

	float FluidSimulation::SampleGridVelocityComponent(const Vector3* Field,
		const Vector3& Position, int Component) const
	{
		Vector3 GridPosition(
			(Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
			(Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
			(Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
		if (Component == 0) GridPosition.x -= 0.5f;
		if (Component == 1) GridPosition.y -= 0.5f;
		if (Component == 2) GridPosition.z -= 0.5f;
		const int X0 = (int)std::floor(GridPosition.x);
		const int Y0 = (int)std::floor(GridPosition.y);
		const int Z0 = (int)std::floor(GridPosition.z);
		const float TX = ClampFloat(GridPosition.x - (float)X0, 0.0f, 1.0f);
		const float TY = ClampFloat(GridPosition.y - (float)Y0, 0.0f, 1.0f);
		const float TZ = ClampFloat(GridPosition.z - (float)Z0, 0.0f, 1.0f);
		const int RowStride = m_Settings.Resolution.x;
		const int SliceStride = RowStride * m_Settings.Resolution.y;
		float Values[2][2][2];
		for (int DZ = 0; DZ <= 1; ++DZ)
		{
			for (int DY = 0; DY <= 1; ++DY)
			{
				for (int DX = 0; DX <= 1; ++DX)
				{
					int X = X0 + DX;
					int Y = Y0 + DY;
					int Z = Z0 + DZ;
					const int NormalCoordinate = Component == 0 ? X : (Component == 1 ? Y : Z);
					const int NormalResolution = Component == 0 ? m_Settings.Resolution.x
						: (Component == 1 ? m_Settings.Resolution.y : m_Settings.Resolution.z);
					if (NormalCoordinate < 0 || NormalCoordinate >= NormalResolution)
					{
						Values[DZ][DY][DX] = 0.0f;
						continue;
					}
					X = ClampInt(X, 0, m_Settings.Resolution.x - 1);
					Y = ClampInt(Y, 0, m_Settings.Resolution.y - 1);
					Z = ClampInt(Z, 0, m_Settings.Resolution.z - 1);
					const Vector3& Value = Field[X + RowStride * Y + SliceStride * Z];
					Values[DZ][DY][DX] = Component == 0 ? Value.x : (Component == 1 ? Value.y : Value.z);
				}
			}
		}
		const float C00 = Values[0][0][0] * (1.0f - TX) + Values[0][0][1] * TX;
		const float C10 = Values[0][1][0] * (1.0f - TX) + Values[0][1][1] * TX;
		const float C01 = Values[1][0][0] * (1.0f - TX) + Values[1][0][1] * TX;
		const float C11 = Values[1][1][0] * (1.0f - TX) + Values[1][1][1] * TX;
		return (C00 * (1.0f - TY) + C10 * TY) * (1.0f - TZ)
			+ (C01 * (1.0f - TY) + C11 * TY) * TZ;
	}

	bool FluidSimulation::IsFaceFluid(int X, int Y, int Z, int Component,
		float ActiveThreshold) const
	{
		int NeighborX = X;
		int NeighborY = Y;
		int NeighborZ = Z;
		if (Component == 0) ++NeighborX;
		if (Component == 1) ++NeighborY;
		if (Component == 2) ++NeighborZ;
		const float* Density = m_GridDensity.data();
		const bool LeftActive = IsGridCoordinateValid(X, Y, Z)
			&& Density[GridIndex(X, Y, Z)] > ActiveThreshold;
		const bool RightActive = IsGridCoordinateValid(NeighborX, NeighborY, NeighborZ)
			&& Density[GridIndex(NeighborX, NeighborY, NeighborZ)] > ActiveThreshold;
		return LeftActive || RightActive;
	}

	float FluidSimulation::SampleDensity(const Vector3& Position) const
	{
		return SampleLocalDensity(m_ContainerTransform.WorldToLocal(Position));
	}

	float FluidSimulation::SampleLocalDensity(const Vector3& Position) const
	{
		if (!m_Initialized || !IsFiniteVector(Position)
			|| Position.x < m_Settings.BoundsMin.x || Position.x > m_Settings.BoundsMax.x
			|| Position.y < m_Settings.BoundsMin.y || Position.y > m_Settings.BoundsMax.y
			|| Position.z < m_Settings.BoundsMin.z || Position.z > m_Settings.BoundsMax.z)
		{
			return 0.0f;
		}

		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			return SampleGridScalar(m_GridDensity.data(), Position);
		}

		const float H = m_Settings.SmoothingRadius;
		const float H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		float Density = 0.0f;
		for (const FluidParticle& Particle : m_Particles)
		{
			const float R2 = (Position - Particle.Position).SquareLength();
			if (R2 < H2)
			{
				const float Term = H2 - R2;
				Density += Particle.Mass * Poly6 * Term * Term * Term;
			}
		}
		return Density;
	}

	Vector3 FluidSimulation::SampleVelocity(const Vector3& Position) const
	{
		return m_ContainerTransform.LocalToWorldDirection(SampleLocalVelocity(
			m_ContainerTransform.WorldToLocal(Position)));
	}

	Vector3 FluidSimulation::SampleLocalVelocity(const Vector3& Position) const
	{
		if (!m_Initialized || !IsFiniteVector(Position)
			|| Position.x < m_Settings.BoundsMin.x || Position.x > m_Settings.BoundsMax.x
			|| Position.y < m_Settings.BoundsMin.y || Position.y > m_Settings.BoundsMax.y
			|| Position.z < m_Settings.BoundsMin.z || Position.z > m_Settings.BoundsMax.z)
		{
			return Vector3::Zero();
		}

		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			return SampleGridVector(m_GridVelocity.data(), Position);
		}

		const float H = m_Settings.SmoothingRadius;
		const float H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		Vector3 WeightedVelocity = Vector3::Zero();
		float Weight = 0.0f;
		for (const FluidParticle& Particle : m_Particles)
		{
			const float R2 = (Position - Particle.Position).SquareLength();
			if (R2 < H2)
			{
				const float Term = H2 - R2;
				const float W = Particle.Mass * Poly6 * Term * Term * Term;
				WeightedVelocity += Particle.Velocity * W;
				Weight += W;
			}
		}
		return Weight > DensityEpsilon ? WeightedVelocity / Weight : Vector3::Zero();
	}

	float FluidSimulation::GetTotalMass() const
	{
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian)
		{
			float Mass = 0.0f;
			for (const FluidParticle& Particle : m_Particles)
			{
				Mass += Particle.Mass;
			}
			return Mass;
		}

		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		float Mass = 0.0f;
		for (float Density : m_GridDensity)
		{
			Mass += Density * CellVolume;
		}
		return Mass;
	}

	bool FluidSimulation::BuildDensityField(std::vector<float>* DensityField) const
	{
		if (!m_Initialized || DensityField == nullptr) return false;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			*DensityField = m_GridDensity;
			return true;
		}

		DensityField->assign(m_GridDensity.size(), 0.0f);
		const float H = m_Settings.SmoothingRadius;
		const float H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const Vector3 Position = GridCellCenter(X, Y, Z);
			const int BaseX = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / H);
			const int BaseY = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / H);
			const int BaseZ = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / H);
			float Density = 0.0f;
			for (int DZ = -1; DZ <= 1; ++DZ)
			for (int DY = -1; DY <= 1; ++DY)
			for (int DX = -1; DX <= 1; ++DX)
			{
				const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
				auto It = std::lower_bound(m_ParticleBuckets.begin(), m_ParticleBuckets.end(), Key,
					[](const ParticleBucket& Bucket, uint64_t Value) { return Bucket.Key < Value; });
				if (It == m_ParticleBuckets.end() || It->Key != Key) continue;
				for (int ParticleIndex : It->Indices)
				{
					const FluidParticle& Particle = m_Particles[(size_t)ParticleIndex];
					const float R2 = (Position - Particle.Position).SquareLength();
					if (R2 < H2)
					{
						const float Term = H2 - R2;
						Density += Particle.Mass * Poly6 * Term * Term * Term;
					}
				}
			}
			(*DensityField)[(size_t)GridIndex(X, Y, Z)] = Density;
		}
		return true;
	}

	float FluidSimulation::GetMaxDivergence() const
	{
		if (!m_Initialized || m_Settings.Method != FluidSimulationMethod::Eulerian)
		{
			return 0.0f;
		}
		const float ActiveThreshold = m_Settings.RestDensity * 0.001f;
		float MaxDivergence = 0.0f;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = GridIndex(X, Y, Z);
					if (m_GridDensity[(size_t)Index] <= ActiveThreshold) continue;
					const float RightU = m_GridVelocity[(size_t)Index].x;
					const float LeftU = X > 0 ? m_GridVelocity[(size_t)GridIndex(X - 1, Y, Z)].x : 0.0f;
					const float TopV = m_GridVelocity[(size_t)Index].y;
					const float BottomV = Y > 0 ? m_GridVelocity[(size_t)GridIndex(X, Y - 1, Z)].y : 0.0f;
					const float FrontW = m_GridVelocity[(size_t)Index].z;
					const float BackW = Z > 0 ? m_GridVelocity[(size_t)GridIndex(X, Y, Z - 1)].z : 0.0f;
					const float Divergence = (RightU - LeftU) / m_CellSize.x
						+ (TopV - BottomV) / m_CellSize.y + (FrontW - BackW) / m_CellSize.z;
					MaxDivergence = std::max(MaxDivergence, std::fabs(Divergence));
				}
			}
		}
		return MaxDivergence;
	}

	void FluidSimulation::StepEulerian(float DeltaTime)
	{
		AdvectEulerian(DeltaTime);
		ApplyEulerianForces(DeltaTime);
		if (!m_Initialized) return;
		ProjectEulerian(DeltaTime);
		EnforceEulerianBoundaries();
		DrainEulerianOverflow(DeltaTime);
	}

	void FluidSimulation::AdvectEulerian(float DeltaTime)
	{
		const std::vector<float> OldDensity = m_GridDensity;
		const std::vector<Vector3> OldVelocity = m_GridVelocity;
		AdvectEulerianDensityConservative(DeltaTime, OldDensity, OldVelocity);

		std::vector<Vector3> AdvectedVelocity(m_GridVelocity.size(), Vector3::Zero());
		const Vector3* OldVelocityData = OldVelocity.data();
		Vector3* AdvectedVelocityData = AdvectedVelocity.data();
		const int ResolutionX = m_Settings.Resolution.x;
		const int ResolutionY = m_Settings.Resolution.y;
		const int SliceStride = ResolutionX * ResolutionY;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					Vector3& Value = AdvectedVelocityData[RowOffset + X];
					for (int Component = 0; Component < 3; ++Component)
					{
						const Vector3 FaceCenter = GridFaceCenter(X, Y, Z, Component);
						const Vector3 FaceVelocity = SampleGridVector(OldVelocityData, FaceCenter);
						const Vector3 MidpointVelocity = SampleGridVector(OldVelocityData,
							FaceCenter - FaceVelocity * (0.5f * DeltaTime));
						Vector3 Source = FaceCenter - MidpointVelocity * DeltaTime;
						Source.x = ClampFloat(Source.x, m_Settings.BoundsMin.x, m_Settings.BoundsMax.x);
						Source.y = ClampFloat(Source.y, m_Settings.BoundsMin.y, m_Settings.BoundsMax.y);
						Source.z = ClampFloat(Source.z, m_Settings.BoundsMin.z, m_Settings.BoundsMax.z);
						const float ComponentValue = SampleGridVelocityComponent(
							OldVelocityData, Source, Component);
						if (Component == 0) Value.x = ComponentValue;
						if (Component == 1) Value.y = ComponentValue;
						if (Component == 2) Value.z = ComponentValue;
					}
				}
			}
		}
		m_GridVelocity.swap(AdvectedVelocity);
	}

	void FluidSimulation::AdvectEulerianDensityConservative(float DeltaTime,
		const std::vector<float>& OldDensity, const std::vector<Vector3>& OldVelocity)
	{
		m_GridDensity = OldDensity;
		const float* OldDensityData = OldDensity.data();
		const Vector3* OldVelocityData = OldVelocity.data();
		float* DensityData = m_GridDensity.data();
		const int ResolutionX = m_Settings.Resolution.x;
		const int ResolutionY = m_Settings.Resolution.y;
		const int SliceStride = ResolutionX * ResolutionY;
		auto TransferAcrossFace = [&](int LeftIndex, int RightIndex, float Velocity, float Spacing)
		{
			if (Velocity == 0.0f) return;
			const int SourceIndex = Velocity > 0.0f ? LeftIndex : RightIndex;
			const int DestinationIndex = Velocity > 0.0f ? RightIndex : LeftIndex;
			const double Desired = std::fabs((double)Velocity) * (double)DeltaTime
				* (double)OldDensityData[SourceIndex] / (double)Spacing;
			const float Available = std::max(0.0f, DensityData[SourceIndex]);
			const float Capacity = std::max(0.0f,
				m_Settings.RestDensity - DensityData[DestinationIndex]);
			const float Amount = (float)std::min(Desired, (double)std::min(Available, Capacity));
			DensityData[SourceIndex] -= Amount;
			DensityData[DestinationIndex] += Amount;
		};

		// Downstream cells release capacity before receiving from their upstream neighbor.
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = m_Settings.Resolution.x - 2; X >= 0; --X)
				{
					const int Index = RowOffset + X;
					const float Velocity = OldVelocityData[Index].x;
					if (Velocity > 0.0f) TransferAcrossFace(Index, Index + 1, Velocity, m_CellSize.x);
				}
				for (int X = 0; X + 1 < m_Settings.Resolution.x; ++X)
				{
					const int Index = RowOffset + X;
					const float Velocity = OldVelocityData[Index].x;
					if (Velocity < 0.0f) TransferAcrossFace(Index, Index + 1, Velocity, m_CellSize.x);
				}
			}
		}
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				for (int Y = m_Settings.Resolution.y - 2; Y >= 0; --Y)
				{
					const int Index = X + ResolutionX * Y + SliceStride * Z;
					const float Velocity = OldVelocityData[Index].y;
					if (Velocity > 0.0f) TransferAcrossFace(Index, Index + ResolutionX, Velocity, m_CellSize.y);
				}
				for (int Y = 0; Y + 1 < m_Settings.Resolution.y; ++Y)
				{
					const int Index = X + ResolutionX * Y + SliceStride * Z;
					const float Velocity = OldVelocityData[Index].y;
					if (Velocity < 0.0f) TransferAcrossFace(Index, Index + ResolutionX, Velocity, m_CellSize.y);
				}
			}
		}
		if (m_Settings.OpenTopBoundary)
		{
			const int TopY = ResolutionY - 1;
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			{
				for (int X = 0; X < ResolutionX; ++X)
				{
					const int Index = X + ResolutionX * TopY + SliceStride * Z;
					const float Velocity = OldVelocityData[Index].y;
					if (Velocity <= 0.0f) continue;
					const double Desired = (double)Velocity * (double)DeltaTime
						* (double)OldDensityData[Index] / (double)m_CellSize.y;
					const float Available = std::max(0.0f, DensityData[Index]);
					DensityData[Index] -= (float)std::min(Desired, (double)Available);
				}
			}
		}
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		{
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				for (int Z = m_Settings.Resolution.z - 2; Z >= 0; --Z)
				{
					const int Index = X + ResolutionX * Y + SliceStride * Z;
					const float Velocity = OldVelocityData[Index].z;
					if (Velocity > 0.0f) TransferAcrossFace(Index, Index + SliceStride, Velocity, m_CellSize.z);
				}
				for (int Z = 0; Z + 1 < m_Settings.Resolution.z; ++Z)
				{
					const int Index = X + ResolutionX * Y + SliceStride * Z;
					const float Velocity = OldVelocityData[Index].z;
					if (Velocity < 0.0f) TransferAcrossFace(Index, Index + SliceStride, Velocity, m_CellSize.z);
				}
			}
		}
	}

	void FluidSimulation::ApplyEulerianForces(float DeltaTime)
	{
		const float ActiveThreshold = m_Settings.RestDensity * 0.001f;
		const Vector3 LocalGravity = m_ContainerTransform.WorldToLocalDirection(m_Settings.Gravity);
		const float KinematicViscosity = m_Settings.DynamicViscosity / m_Settings.RestDensity;
		const std::vector<Vector3> OldVelocity = m_GridVelocity;
		std::vector<Vector3> NewVelocity = OldVelocity;
		const Vector3* OldVelocityData = OldVelocity.data();
		Vector3* NewVelocityData = NewVelocity.data();
		const float* DensityData = m_GridDensity.data();
		const int ResolutionX = m_Settings.Resolution.x;
		const int ResolutionY = m_Settings.Resolution.y;
		const int SliceStride = ResolutionX * ResolutionY;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = RowOffset + X;
					for (int Component = 0; Component < 3; ++Component)
					{
						const bool PositiveBoundary = (Component == 0 && X + 1 == m_Settings.Resolution.x)
							|| (Component == 1 && Y + 1 == m_Settings.Resolution.y)
							|| (Component == 2 && Z + 1 == m_Settings.Resolution.z);
						const bool OpenTopFace = m_Settings.OpenTopBoundary && Component == 1
							&& Y + 1 == m_Settings.Resolution.y;
						const int NeighborIndex = Index + (Component == 0 ? 1
							: (Component == 1 ? ResolutionX : SliceStride));
						float* NewComponent = Component == 0 ? &NewVelocityData[Index].x
							: (Component == 1 ? &NewVelocityData[Index].y : &NewVelocityData[Index].z);
						const bool FaceActive = DensityData[Index] > ActiveThreshold
							|| (!PositiveBoundary && DensityData[NeighborIndex] > ActiveThreshold);
						if ((PositiveBoundary && !OpenTopFace) || !FaceActive)
						{
							*NewComponent = 0.0f;
							continue;
						}

						const Vector3& OldValue = OldVelocityData[Index];
						float Velocity = (Component == 0 ? OldValue.x : (Component == 1 ? OldValue.y : OldValue.z))
							+ (Component == 0 ? LocalGravity.x
								: (Component == 1 ? LocalGravity.y : LocalGravity.z)) * DeltaTime;
						if (KinematicViscosity > 0.0f)
						{
							float Laplacian = 0.0f;
						float Weight = 0.0f;
						const int Offset[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 },
							{ 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };
						for (int N = 0; N < 6; ++N)
						{
							const int NX = X + Offset[N][0];
							const int NY = Y + Offset[N][1];
							const int NZ = Z + Offset[N][2];
							if (IsGridCoordinateValid(NX, NY, NZ))
							{
								const float H = N < 2 ? m_CellSize.x : (N < 4 ? m_CellSize.y : m_CellSize.z);
								const float InvH2 = 1.0f / (H * H);
								const int NeighborGridIndex = NX + ResolutionX * NY + SliceStride * NZ;
								const Vector3& Neighbor = OldVelocityData[NeighborGridIndex];
								const float NeighborValue = Component == 0 ? Neighbor.x
									: (Component == 1 ? Neighbor.y : Neighbor.z);
								const float CenterValue = Component == 0 ? OldValue.x
									: (Component == 1 ? OldValue.y : OldValue.z);
								Laplacian += (NeighborValue - CenterValue) * InvH2;
								Weight += InvH2;
							}
						}
						const float ViscosityScale = Weight > 0.0f
							? std::min(DeltaTime * KinematicViscosity, 0.24f / Weight) : 0.0f;
						Velocity += Laplacian * ViscosityScale;
						}
						*NewComponent = Velocity;
					}
				}
			}
		}
		std::vector<Vector3> Delta(NewVelocity.size());
		Vector3* DeltaData = Delta.data();
		for (size_t Index = 0; Index < Delta.size(); ++Index)
			DeltaData[Index] = NewVelocityData[Index] - OldVelocityData[Index];
		if (!ApplyFluidVelocityDelta(OldVelocityData, DeltaData, Delta.size(),
			m_GridVelocity.data(), m_Settings.Backend, m_Settings.AllowBackendFallback,
			&m_BackendResult))
		{
			if (m_Settings.AllowBackendFallback) m_GridVelocity.swap(NewVelocity);
			else { m_Initialized = false; return; }
		}
	}

	void FluidSimulation::ProjectEulerian(float DeltaTime)
	{
		const float ActiveThreshold = m_Settings.RestDensity * 0.001f;
		EnforceEulerianBoundaries();
		std::fill(m_Pressure.begin(), m_Pressure.end(), 0.0f);
		const int ResolutionX = m_Settings.Resolution.x;
		const int ResolutionY = m_Settings.Resolution.y;
		const int ResolutionZ = m_Settings.Resolution.z;
		const int SliceStride = ResolutionX * ResolutionY;
		const float* DensityData = m_GridDensity.data();
		Vector3* VelocityData = m_GridVelocity.data();
		float* DivergenceData = m_Divergence.data();
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = RowOffset + X;
					if (DensityData[Index] <= ActiveThreshold)
					{
						DivergenceData[Index] = 0.0f;
						continue;
					}
					const float RightU = VelocityData[Index].x;
					const float LeftU = X > 0 ? VelocityData[Index - 1].x : 0.0f;
					const float TopV = VelocityData[Index].y;
					const float BottomV = Y > 0 ? VelocityData[Index - ResolutionX].y : 0.0f;
					const float FrontW = VelocityData[Index].z;
					const float BackW = Z > 0 ? VelocityData[Index - SliceStride].z : 0.0f;
					DivergenceData[Index] = (RightU - LeftU) / m_CellSize.x
						+ (TopV - BottomV) / m_CellSize.y + (FrontW - BackW) / m_CellSize.z;
				}
			}
		}

		std::vector<float> NextPressure(m_Pressure.size(), 0.0f);
		const float InvCellSizeSquaredX = 1.0f / (m_CellSize.x * m_CellSize.x);
		const float InvCellSizeSquaredY = 1.0f / (m_CellSize.y * m_CellSize.y);
		const float InvCellSizeSquaredZ = 1.0f / (m_CellSize.z * m_CellSize.z);
		const float RightHandSideScale = m_Settings.RestDensity / DeltaTime;
		for (int Iteration = 0; Iteration < m_Settings.PressureIterations; ++Iteration)
		{
			const float* PressureData = m_Pressure.data();
			float* NextPressureData = NextPressure.data();
			for (int Z = 0; Z < ResolutionZ; ++Z)
			{
				for (int Y = 0; Y < ResolutionY; ++Y)
				{
					const int RowOffset = ResolutionX * Y + SliceStride * Z;
					for (int X = 0; X < ResolutionX; ++X)
					{
						const int Index = RowOffset + X;
						if (DensityData[Index] <= ActiveThreshold)
						{
							NextPressureData[Index] = 0.0f;
							continue;
						}

						float NeighborSum = 0.0f;
						float CoefficientSum = 0.0f;
						if (X > 0)
						{
							const int NeighborIndex = Index - 1;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredX * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredX;
						}
						if (X + 1 < ResolutionX)
						{
							const int NeighborIndex = Index + 1;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredX * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredX;
						}
						if (Y > 0)
						{
							const int NeighborIndex = Index - ResolutionX;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredY * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredY;
						}
						if (Y + 1 < ResolutionY)
						{
							const int NeighborIndex = Index + ResolutionX;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredY * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredY;
						}
						else if (m_Settings.OpenTopBoundary)
						{
							CoefficientSum += InvCellSizeSquaredY;
						}
						if (Z > 0)
						{
							const int NeighborIndex = Index - SliceStride;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredZ * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredZ;
						}
						if (Z + 1 < ResolutionZ)
						{
							const int NeighborIndex = Index + SliceStride;
							if (DensityData[NeighborIndex] > ActiveThreshold)
								NeighborSum += InvCellSizeSquaredZ * PressureData[NeighborIndex];
							CoefficientSum += InvCellSizeSquaredZ;
						}
						const float RightHandSide = RightHandSideScale * DivergenceData[Index];
						NextPressureData[Index] = CoefficientSum > 0.0f
							? (NeighborSum - RightHandSide) / CoefficientSum : 0.0f;
					}
				}
			}
			m_Pressure.swap(NextPressure);
		}

		const float* PressureData = m_Pressure.data();
		const float InverseRestDensity = 1.0f / m_Settings.RestDensity;
		for (int Z = 0; Z < ResolutionZ; ++Z)
		{
			for (int Y = 0; Y < ResolutionY; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = 0; X < ResolutionX; ++X)
				{
					const int Index = RowOffset + X;
					const bool LeftActive = DensityData[Index] > ActiveThreshold;
					Vector3& FaceVelocity = VelocityData[Index];
					if (X + 1 < ResolutionX)
					{
						const int RightIndex = Index + 1;
						const bool RightActive = DensityData[RightIndex] > ActiveThreshold;
						if (LeftActive || RightActive)
						{
							const float LeftPressure = LeftActive ? PressureData[Index] : 0.0f;
							const float RightPressure = RightActive ? PressureData[RightIndex] : 0.0f;
							FaceVelocity.x -= DeltaTime * (RightPressure - LeftPressure)
								* InverseRestDensity / m_CellSize.x;
						}
					}
					if (Y + 1 < ResolutionY)
					{
						const int TopIndex = Index + ResolutionX;
						const bool TopActive = DensityData[TopIndex] > ActiveThreshold;
						if (LeftActive || TopActive)
						{
							const float BottomPressure = LeftActive ? PressureData[Index] : 0.0f;
							const float TopPressure = TopActive ? PressureData[TopIndex] : 0.0f;
							FaceVelocity.y -= DeltaTime * (TopPressure - BottomPressure)
								* InverseRestDensity / m_CellSize.y;
						}
					}
					else if (m_Settings.OpenTopBoundary && LeftActive)
					{
						FaceVelocity.y += DeltaTime * PressureData[Index]
							* InverseRestDensity / m_CellSize.y;
					}
					if (Z + 1 < ResolutionZ)
					{
						const int FrontIndex = Index + SliceStride;
						const bool FrontActive = DensityData[FrontIndex] > ActiveThreshold;
						if (LeftActive || FrontActive)
						{
							const float BackPressure = LeftActive ? PressureData[Index] : 0.0f;
							const float FrontPressure = FrontActive ? PressureData[FrontIndex] : 0.0f;
							FaceVelocity.z -= DeltaTime * (FrontPressure - BackPressure)
								* InverseRestDensity / m_CellSize.z;
						}
					}
				}
			}
		}
	}

	void FluidSimulation::EnforceEulerianBoundaries()
	{
		const float ActiveThreshold = m_Settings.RestDensity * 0.001f;
		const int ResolutionX = m_Settings.Resolution.x;
		const int ResolutionY = m_Settings.Resolution.y;
		const int ResolutionZ = m_Settings.Resolution.z;
		const int SliceStride = ResolutionX * ResolutionY;
		const float* DensityData = m_GridDensity.data();
		Vector3* VelocityData = m_GridVelocity.data();
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				const int RowOffset = ResolutionX * Y + SliceStride * Z;
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = RowOffset + X;
					const bool LeftActive = DensityData[Index] > ActiveThreshold;
					Vector3& Velocity = VelocityData[Index];
					if (X + 1 == ResolutionX || (!LeftActive
						&& DensityData[Index + 1] <= ActiveThreshold)) Velocity.x = 0.0f;
					if (Y + 1 == ResolutionY)
					{
						if (!m_Settings.OpenTopBoundary || !LeftActive) Velocity.y = 0.0f;
					}
					else if (!LeftActive && DensityData[Index + ResolutionX] <= ActiveThreshold)
					{
						Velocity.y = 0.0f;
					}
					if (Z + 1 == ResolutionZ || (!LeftActive
						&& DensityData[Index + SliceStride] <= ActiveThreshold)) Velocity.z = 0.0f;
				}
			}
		}
	}

	bool FluidSimulation::IsAboveOpenTop(const Vector3& LocalPosition) const
	{
		if (!m_Settings.OpenTopBoundary || m_Settings.Gravity.SquareLength() <= 1.0e-12f)
		{
			return false;
		}
		const Vector3 Up = (-m_Settings.Gravity).Unit();
		float LipHeight = std::numeric_limits<float>::max();
		for (int Z = 0; Z < 2; ++Z)
		{
			for (int X = 0; X < 2; ++X)
			{
				const Vector3 RimPoint(X == 0 ? m_Settings.BoundsMin.x : m_Settings.BoundsMax.x,
					m_Settings.BoundsMax.y,
					Z == 0 ? m_Settings.BoundsMin.z : m_Settings.BoundsMax.z);
				LipHeight = std::min(LipHeight,
					Up.Dot(m_ContainerTransform.LocalToWorld(RimPoint)));
			}
		}
		return Up.Dot(m_ContainerTransform.LocalToWorld(LocalPosition)) > LipHeight;
	}

	void FluidSimulation::DrainEulerianOverflow(float DeltaTime)
	{
		if (!m_Settings.OpenTopBoundary || !(DeltaTime > 0.0f)
			|| m_Settings.Gravity.SquareLength() <= 1.0e-12f)
		{
			return;
		}
		const Vector3 Up = (-m_Settings.Gravity).Unit();
		const float GravityMagnitude = m_Settings.Gravity.Length();
		float LipHeight = std::numeric_limits<float>::max();
		for (int Z = 0; Z < 2; ++Z)
		{
			for (int X = 0; X < 2; ++X)
			{
				const Vector3 RimPoint(X == 0 ? m_Settings.BoundsMin.x : m_Settings.BoundsMax.x,
					m_Settings.BoundsMax.y,
					Z == 0 ? m_Settings.BoundsMin.z : m_Settings.BoundsMax.z);
				LipHeight = std::min(LipHeight,
					Up.Dot(m_ContainerTransform.LocalToWorld(RimPoint)));
			}
		}
		const float DrainLength = std::max(1.0e-6f,
			std::min(m_CellSize.x, std::min(m_CellSize.y, m_CellSize.z)));
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = GridIndex(X, Y, Z);
					float& Density = m_GridDensity[(size_t)Index];
					if (Density <= 0.0f) continue;
					const float Height = Up.Dot(m_ContainerTransform.LocalToWorld(
						GridCellCenter(X, Y, Z)));
					const float Head = Height - LipHeight;
					if (Head <= 0.0f) continue;
					const float DrainSpeed = std::sqrt(2.0f * GravityMagnitude * Head);
					const float DrainFraction = ClampFloat(
						DrainSpeed * DeltaTime / DrainLength, 0.0f, 1.0f);
					Density *= 1.0f - DrainFraction;
				}
			}
		}
	}

	uint64_t FluidSimulation::ParticleBucketKey(int X, int Y, int Z) const
	{
		const uint64_t HX = MixBucketCoordinate((uint64_t)(int64_t)X + 0x9e3779b97f4a7c15ULL);
		const uint64_t HY = MixBucketCoordinate((uint64_t)(int64_t)Y + 0x632be59bd9b4e019ULL);
		const uint64_t HZ = MixBucketCoordinate((uint64_t)(int64_t)Z + 0x94d049bb133111ebULL);
		return HX ^ (HY << 1) ^ (HZ << 7);
	}

	void FluidSimulation::RebuildParticleBuckets()
	{
		m_ParticleBuckets.clear();
		std::vector<std::pair<uint64_t, int>> Entries;
		Entries.reserve(m_Particles.size());
		for (size_t ParticleIndex = 0; ParticleIndex < m_Particles.size(); ++ParticleIndex)
		{
			const Vector3 Position = m_Particles[ParticleIndex].Position;
			const int X = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / m_Settings.SmoothingRadius);
			const int Y = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / m_Settings.SmoothingRadius);
			const int Z = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / m_Settings.SmoothingRadius);
			Entries.push_back(std::make_pair(ParticleBucketKey(X, Y, Z), (int)ParticleIndex));
		}

		std::sort(Entries.begin(), Entries.end(),
			[](const std::pair<uint64_t, int>& Left, const std::pair<uint64_t, int>& Right)
			{
				return Left.first < Right.first
					|| (Left.first == Right.first && Left.second < Right.second);
			});
		for (const std::pair<uint64_t, int>& Entry : Entries)
		{
			if (m_ParticleBuckets.empty() || m_ParticleBuckets.back().Key != Entry.first)
			{
				ParticleBucket Bucket;
				Bucket.Key = Entry.first;
				m_ParticleBuckets.push_back(Bucket);
			}
			m_ParticleBuckets.back().Indices.push_back(Entry.second);
		}
	}

	void FluidSimulation::ComputeParticleDensities()
	{
		RebuildParticleBuckets();
		const float H = m_Settings.SmoothingRadius;
		const float H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		for (FluidParticle& Particle : m_Particles)
		{
			float Density = 0.0f;
			const int BaseX = (int)std::floor((Particle.Position.x - m_Settings.BoundsMin.x) / H);
			const int BaseY = (int)std::floor((Particle.Position.y - m_Settings.BoundsMin.y) / H);
			const int BaseZ = (int)std::floor((Particle.Position.z - m_Settings.BoundsMin.z) / H);
			for (int DZ = -1; DZ <= 1; ++DZ)
			{
				for (int DY = -1; DY <= 1; ++DY)
				{
					for (int DX = -1; DX <= 1; ++DX)
					{
						const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
						auto It = std::lower_bound(m_ParticleBuckets.begin(), m_ParticleBuckets.end(), Key,
							[](const ParticleBucket& Bucket, uint64_t Value) { return Bucket.Key < Value; });
						if (It == m_ParticleBuckets.end() || It->Key != Key) continue;
						for (int NeighborIndex : It->Indices)
						{
							const FluidParticle& Neighbor = m_Particles[(size_t)NeighborIndex];
							const float R2 = (Particle.Position - Neighbor.Position).SquareLength();
							if (R2 < H2)
							{
								const float Term = H2 - R2;
								Density += Neighbor.Mass * Poly6 * Term * Term * Term;
							}
						}
					}
				}
			}
			Particle.Density = std::max(Density, 0.05f * m_Settings.RestDensity);
			Particle.Pressure = m_Settings.PressureStiffness
				* std::max(Particle.Density / m_Settings.RestDensity - 1.0f, 0.0f);
		}
	}

	void FluidSimulation::ComputeParticleAccelerations()
	{
		const float H = m_Settings.SmoothingRadius;
		const Vector3 LocalGravity = m_ContainerTransform.WorldToLocalDirection(m_Settings.Gravity);
		const float SpikyGradient = -45.0f / (Pi * std::pow(H, 6.0f));
		const float ViscosityLaplacian = 45.0f / (Pi * std::pow(H, 6.0f));
		for (size_t ParticleIndex = 0; ParticleIndex < m_Particles.size(); ++ParticleIndex)
		{
			FluidParticle& Particle = m_Particles[ParticleIndex];
			Vector3 Acceleration = LocalGravity;
			const int BaseX = (int)std::floor((Particle.Position.x - m_Settings.BoundsMin.x) / H);
			const int BaseY = (int)std::floor((Particle.Position.y - m_Settings.BoundsMin.y) / H);
			const int BaseZ = (int)std::floor((Particle.Position.z - m_Settings.BoundsMin.z) / H);
			for (int DZ = -1; DZ <= 1; ++DZ)
			{
				for (int DY = -1; DY <= 1; ++DY)
				{
					for (int DX = -1; DX <= 1; ++DX)
					{
						const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
						auto It = std::lower_bound(m_ParticleBuckets.begin(), m_ParticleBuckets.end(), Key,
							[](const ParticleBucket& Bucket, uint64_t Value) { return Bucket.Key < Value; });
						if (It == m_ParticleBuckets.end() || It->Key != Key) continue;
						for (int NeighborIndex : It->Indices)
						{
							if (NeighborIndex == (int)ParticleIndex) continue;
							const FluidParticle& Neighbor = m_Particles[(size_t)NeighborIndex];
							const Vector3 Difference = Particle.Position - Neighbor.Position;
							const float Distance = Difference.Length();
							if (!(Distance > 1.0e-6f && Distance < H)) continue;
							const float KernelDistance = H - Distance;
							const Vector3 Gradient = Difference * (SpikyGradient
								* KernelDistance * KernelDistance / Distance);
							const float PressureScale = Neighbor.Mass * (Particle.Pressure
								/ (Particle.Density * Particle.Density) + Neighbor.Pressure
								/ (Neighbor.Density * Neighbor.Density));
							Acceleration -= Gradient * PressureScale;
							const float Laplacian = ViscosityLaplacian * KernelDistance;
							Acceleration += (Neighbor.Velocity - Particle.Velocity)
								* (m_Settings.DynamicViscosity * Neighbor.Mass * Laplacian / Neighbor.Density);
						}
					}
				}
			}
			Particle.Acceleration = IsFiniteVector(Acceleration) ? Acceleration : LocalGravity;
		}
	}

	void FluidSimulation::EnforceParticleBoundary(FluidParticle& Particle) const
	{
		const float Radius = 0.5f * m_Settings.ParticleSpacing;
		const Vector3 Min = m_Settings.BoundsMin + Vector3(Radius);
		const Vector3 Max = m_Settings.BoundsMax - Vector3(Radius);
		const float TangentialScale = ClampFloat(1.0f - m_Settings.BoundaryFriction, 0.0f, 1.0f);
		if (Particle.Position.x < Min.x)
		{
			Particle.Position.x = Min.x;
			if (Particle.Velocity.x < 0.0f) Particle.Velocity.x *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.y *= TangentialScale; Particle.Velocity.z *= TangentialScale;
		}
		else if (Particle.Position.x > Max.x)
		{
			Particle.Position.x = Max.x;
			if (Particle.Velocity.x > 0.0f) Particle.Velocity.x *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.y *= TangentialScale; Particle.Velocity.z *= TangentialScale;
		}
		if (Particle.Position.y < Min.y)
		{
			Particle.Position.y = Min.y;
			if (Particle.Velocity.y < 0.0f) Particle.Velocity.y *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.x *= TangentialScale; Particle.Velocity.z *= TangentialScale;
		}
		else if (!m_Settings.OpenTopBoundary && Particle.Position.y > Max.y)
		{
			Particle.Position.y = Max.y;
			if (Particle.Velocity.y > 0.0f) Particle.Velocity.y *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.x *= TangentialScale; Particle.Velocity.z *= TangentialScale;
		}
		if (Particle.Position.z < Min.z)
		{
			Particle.Position.z = Min.z;
			if (Particle.Velocity.z < 0.0f) Particle.Velocity.z *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.x *= TangentialScale; Particle.Velocity.y *= TangentialScale;
		}
		else if (Particle.Position.z > Max.z)
		{
			Particle.Position.z = Max.z;
			if (Particle.Velocity.z > 0.0f) Particle.Velocity.z *= -m_Settings.BoundaryRestitution;
			Particle.Velocity.x *= TangentialScale; Particle.Velocity.y *= TangentialScale;
		}
	}

	void FluidSimulation::StepLagrangian(float DeltaTime)
	{
		if (m_Particles.empty())
		{
			return;
		}
		ComputeParticleDensities();
		ComputeParticleAccelerations();
		std::vector<Vector3> Velocity(m_Particles.size());
		std::vector<Vector3> Delta(m_Particles.size());
		std::vector<Vector3> Integrated(m_Particles.size());
		for (size_t Index = 0; Index < m_Particles.size(); ++Index)
		{
			Velocity[Index] = m_Particles[Index].Velocity;
			Delta[Index] = m_Particles[Index].Acceleration * DeltaTime;
		}
		if (!ApplyFluidVelocityDelta(Velocity.data(), Delta.data(), Velocity.size(),
			Integrated.data(), m_Settings.Backend, m_Settings.AllowBackendFallback,
			&m_BackendResult))
		{
			if (!m_Settings.AllowBackendFallback) { m_Initialized = false; return; }
			for (size_t Index = 0; Index < Integrated.size(); ++Index)
				Integrated[Index] = Velocity[Index] + Delta[Index];
		}
		for (size_t Index = 0; Index < m_Particles.size(); ++Index)
		{
			FluidParticle& Particle = m_Particles[Index];
			Particle.Velocity = Integrated[Index];
			Particle.Position += Particle.Velocity * DeltaTime;
			EnforceParticleBoundary(Particle);
		}
		if (m_Settings.OpenTopBoundary)
		{
			m_Particles.erase(std::remove_if(m_Particles.begin(), m_Particles.end(),
				[this](const FluidParticle& Particle)
				{
					return Particle.Position.y > m_Settings.BoundsMax.y
						|| IsAboveOpenTop(Particle.Position);
				}), m_Particles.end());
			if (m_Particles.empty())
			{
				m_ParticleBuckets.clear();
				return;
			}
		}
		ComputeParticleDensities();
	}

	void FluidSimulation::ConvertEulerianToLagrangian()
	{
		m_Particles.clear();
		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		const float Threshold = 0.0f;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = GridIndex(X, Y, Z);
					const float Density = m_GridDensity[(size_t)Index];
					if (Density > Threshold)
					{
						m_Particles.push_back(FluidParticle(GridCellCenter(X, Y, Z),
							SampleGridVector(m_GridVelocity.data(), GridCellCenter(X, Y, Z)),
							Density * CellVolume));
					}
				}
			}
		}
		ComputeParticleDensities();
	}

	void FluidSimulation::ConvertLagrangianToEulerian()
	{
		std::fill(m_GridDensity.begin(), m_GridDensity.end(), 0.0f);
		std::fill(m_GridVelocity.begin(), m_GridVelocity.end(), Vector3::Zero());
		std::vector<float> DepositedMass(m_GridDensity.size(), 0.0f);
		std::vector<Vector3> CellVelocity(m_GridVelocity.size(), Vector3::Zero());
		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		for (const FluidParticle& Particle : m_Particles)
		{
			const Vector3 GridPosition(
				(Particle.Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
				(Particle.Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
				(Particle.Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
			const int X0 = ClampInt((int)std::floor(GridPosition.x), 0, m_Settings.Resolution.x - 1);
			const int Y0 = ClampInt((int)std::floor(GridPosition.y), 0, m_Settings.Resolution.y - 1);
			const int Z0 = ClampInt((int)std::floor(GridPosition.z), 0, m_Settings.Resolution.z - 1);
			const float TX = ClampFloat(GridPosition.x - (float)X0, 0.0f, 1.0f);
			const float TY = ClampFloat(GridPosition.y - (float)Y0, 0.0f, 1.0f);
			const float TZ = ClampFloat(GridPosition.z - (float)Z0, 0.0f, 1.0f);
			for (int DZ = 0; DZ <= 1; ++DZ)
			{
				for (int DY = 0; DY <= 1; ++DY)
				{
					for (int DX = 0; DX <= 1; ++DX)
					{
						const int X = std::min(X0 + DX, m_Settings.Resolution.x - 1);
						const int Y = std::min(Y0 + DY, m_Settings.Resolution.y - 1);
						const int Z = std::min(Z0 + DZ, m_Settings.Resolution.z - 1);
						const float Weight = (DX ? TX : 1.0f - TX) * (DY ? TY : 1.0f - TY)
							* (DZ ? TZ : 1.0f - TZ);
						const int Index = GridIndex(X, Y, Z);
						const float Mass = Particle.Mass * Weight;
						DepositedMass[(size_t)Index] += Mass;
						CellVelocity[(size_t)Index] += Particle.Velocity * Mass;
					}
				}
			}
		}
		for (size_t Index = 0; Index < m_GridDensity.size(); ++Index)
		{
			if (DepositedMass[Index] > DensityEpsilon)
			{
				m_GridDensity[Index] = DepositedMass[Index] / CellVolume;
				CellVelocity[Index] /= DepositedMass[Index];
			}
		}
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		{
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			{
				for (int X = 0; X < m_Settings.Resolution.x; ++X)
				{
					const int Index = GridIndex(X, Y, Z);
					Vector3& FaceVelocity = m_GridVelocity[(size_t)Index];
					if (X + 1 < m_Settings.Resolution.x)
					{
						const int Neighbor = GridIndex(X + 1, Y, Z);
						const float Weight = DepositedMass[(size_t)Index] + DepositedMass[(size_t)Neighbor];
						if (Weight > DensityEpsilon)
						{
							FaceVelocity.x = (CellVelocity[(size_t)Index].x * DepositedMass[(size_t)Index]
								+ CellVelocity[(size_t)Neighbor].x * DepositedMass[(size_t)Neighbor]) / Weight;
						}
					}
					if (Y + 1 < m_Settings.Resolution.y)
					{
						const int Neighbor = GridIndex(X, Y + 1, Z);
						const float Weight = DepositedMass[(size_t)Index] + DepositedMass[(size_t)Neighbor];
						if (Weight > DensityEpsilon)
						{
							FaceVelocity.y = (CellVelocity[(size_t)Index].y * DepositedMass[(size_t)Index]
								+ CellVelocity[(size_t)Neighbor].y * DepositedMass[(size_t)Neighbor]) / Weight;
						}
					}
					if (Z + 1 < m_Settings.Resolution.z)
					{
						const int Neighbor = GridIndex(X, Y, Z + 1);
						const float Weight = DepositedMass[(size_t)Index] + DepositedMass[(size_t)Neighbor];
						if (Weight > DensityEpsilon)
						{
							FaceVelocity.z = (CellVelocity[(size_t)Index].z * DepositedMass[(size_t)Index]
								+ CellVelocity[(size_t)Neighbor].z * DepositedMass[(size_t)Neighbor]) / Weight;
						}
					}
				}
			}
		}
		EnforceEulerianBoundaries();
	}
}

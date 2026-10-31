#include "AerodynamicsSimulation.h"
#include "MACGridSampling.h"
#include "FluidParallelFor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <utility>

namespace Riemann
{
	namespace
	{
		const float Pi = 3.14159265358979323846f;
		const float SmallValue = 1.0e-6f;
		const size_t MaxParticleCount = (size_t)1024 * 1024;

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

		uint64_t MixBucketCoordinate(uint64_t Value)
		{
			Value ^= Value >> 30;
			Value *= 0xbf58476d1ce4e5b9ULL;
			Value ^= Value >> 27;
			Value *= 0x94d049bb133111ebULL;
			return Value ^ (Value >> 31);
		}

	}

	AerodynamicsSimulationSettings::AerodynamicsSimulationSettings()
		: Method(FluidSimulationMethod::Eulerian)
		, Backend(FluidBackend::CPU)
		, AllowBackendFallback(true)
		, BoundsMin(-2.0f, -2.0f, -2.0f)
		, BoundsMax(2.0f, 2.0f, 2.0f)
		, Resolution(16, 16, 16)
		, Gravity(Vector3::Zero())
		, AmbientDensity(1.225f)
		, AmbientPressure(101325.0f)
		, SpeedOfSound(20.0f)
		, AdiabaticExponent(1.4f)
		, DynamicViscosity(1.81e-5f)
		, ArtificialViscosity(0.05f)
		, ParticleSpacing(0.25f)
		, SmoothingRadius(0.5f)
		, BoundaryRestitution(0.2f)
		, MaxTimeStep(1.0f / 240.0f)
		, AcousticCFL(0.35f)
		, MaxSubsteps(256)
	{
	}

	AirParticle::AirParticle()
		: Position(Vector3::Zero())
		, ReferencePosition(Vector3::Zero())
		, Velocity(Vector3::Zero())
		, Acceleration(Vector3::Zero())
		, Mass(0.0f)
		, ReferenceMass(0.0f)
		, DensityNormalization(1.0f)
		, Density(0.0f)
		, Pressure(0.0f)
	{
	}

	AerodynamicSample::AerodynamicSample()
		: Velocity(Vector3::Zero())
		, PressureGradient(Vector3::Zero())
		, Density(0.0f)
		, Pressure(0.0f)
	{
	}

	AerodynamicSurfaceSample::AerodynamicSurfaceSample()
		: Position(Vector3::Zero())
		, Normal(Vector3::UnitY())
		, Velocity(Vector3::Zero())
		, Area(0.0f)
	{
	}

	AerodynamicForceResult::AerodynamicForceResult()
		: Force(Vector3::Zero())
		, Torque(Vector3::Zero())
	{
	}

	AerodynamicsSimulation::AerodynamicsSimulation()
		: m_CellSize(Vector3::Zero())
		, m_Initialized(false)
		, m_TimeRemainder(0.0f)
	{
	}

	AerodynamicsSimulation::AerodynamicsSimulation(const AerodynamicsSimulationSettings& Settings)
		: m_CellSize(Vector3::Zero())
		, m_Initialized(false)
		, m_TimeRemainder(0.0f)
	{
		Initialize(Settings);
	}

	bool AerodynamicsSimulation::Initialize(const AerodynamicsSimulationSettings& Settings)
	{
		const Vector3 Extent = Settings.BoundsMax - Settings.BoundsMin;
		const bool ValidMethod = Settings.Method == FluidSimulationMethod::Eulerian
			|| Settings.Method == FluidSimulationMethod::Lagrangian;
		if (!ValidMethod || !IsFiniteVector(Settings.BoundsMin) || !IsFiniteVector(Settings.BoundsMax)
			|| !IsFiniteVector(Extent) || !IsFiniteVector(Settings.Gravity)
			|| !std::isfinite(Settings.AmbientDensity) || !std::isfinite(Settings.AmbientPressure)
			|| !std::isfinite(Settings.SpeedOfSound) || !std::isfinite(Settings.DynamicViscosity)
			|| !std::isfinite(Settings.ArtificialViscosity)
			|| !std::isfinite(Settings.AdiabaticExponent)
			|| !std::isfinite(Settings.ParticleSpacing) || !std::isfinite(Settings.SmoothingRadius)
			|| !std::isfinite(Settings.BoundaryRestitution) || !std::isfinite(Settings.MaxTimeStep)
			|| !std::isfinite(Settings.AcousticCFL)
			|| !(Extent.x > 0.0f && Extent.y > 0.0f && Extent.z > 0.0f)
			|| Settings.Resolution.x < 3 || Settings.Resolution.y < 3 || Settings.Resolution.z < 3
			|| Settings.Resolution.x > 2048 || Settings.Resolution.y > 2048 || Settings.Resolution.z > 2048
			|| Settings.AmbientDensity < 1.0e-6f || Settings.AmbientDensity > 1.0e6f
			|| Settings.AmbientPressure < 0.0f || Settings.AmbientPressure > 1.0e12f
			|| Settings.SpeedOfSound <= 0.0f || Settings.SpeedOfSound > 1.0e6f
			|| Settings.AdiabaticExponent < 1.0f || Settings.AdiabaticExponent > 2.0f
			|| Settings.DynamicViscosity < 0.0f || Settings.DynamicViscosity > 1.0e6f
			|| Settings.ArtificialViscosity < 0.0f || Settings.ArtificialViscosity > 2.0f
			|| Settings.ParticleSpacing < 1.0e-4f || Settings.ParticleSpacing > 1.0e4f
			|| Settings.SmoothingRadius < Settings.ParticleSpacing || Settings.SmoothingRadius > 1.0e4f
			|| Settings.ParticleSpacing > Extent.x || Settings.ParticleSpacing > Extent.y
			|| Settings.ParticleSpacing > Extent.z
			|| Settings.BoundaryRestitution < 0.0f || Settings.BoundaryRestitution > 1.0f
			|| Settings.MaxTimeStep < 1.0e-6f || Settings.AcousticCFL <= 0.0f
			|| Settings.AcousticCFL > 1.0f || Settings.MaxSubsteps < 1 || Settings.MaxSubsteps > 4096)
		{
			return false;
		}

		const size_t ResolutionX = (size_t)Settings.Resolution.x;
		const size_t ResolutionY = (size_t)Settings.Resolution.y;
		const size_t ResolutionZ = (size_t)Settings.Resolution.z;
		const size_t MaxCellCount = (size_t)1024 * 1024;
		if (ResolutionX > MaxCellCount / ResolutionY
			|| ResolutionX * ResolutionY > MaxCellCount / ResolutionZ)
		{
			return false;
		}

		const double ParticleCountX = std::ceil((double)Extent.x / (double)Settings.ParticleSpacing);
		const double ParticleCountY = std::ceil((double)Extent.y / (double)Settings.ParticleSpacing);
		const double ParticleCountZ = std::ceil((double)Extent.z / (double)Settings.ParticleSpacing);
		if (ParticleCountX > 1024.0 || ParticleCountY > 1024.0 || ParticleCountZ > 1024.0
			|| ParticleCountX * ParticleCountY * ParticleCountZ > (double)MaxParticleCount)
		{
			return false;
		}
		const double ParticleMass = (double)Settings.AmbientDensity
			* (double)Settings.ParticleSpacing * (double)Settings.ParticleSpacing
			* (double)Settings.ParticleSpacing;
		if (ParticleMass < (double)std::numeric_limits<float>::min()
			|| ParticleMass > (double)std::numeric_limits<float>::max())
		{
			return false;
		}
		const double H = (double)Settings.SmoothingRadius;
		const double Poly6Coefficient = 315.0 / (64.0 * (double)Pi * std::pow(H, 9.0));
		const double GradientCoefficient = 45.0 / ((double)Pi * std::pow(H, 6.0));
		if (!std::isfinite(Poly6Coefficient) || !std::isfinite(GradientCoefficient)
			|| Poly6Coefficient < (double)std::numeric_limits<float>::min()
			|| Poly6Coefficient > (double)std::numeric_limits<float>::max()
			|| GradientCoefficient < (double)std::numeric_limits<float>::min()
			|| GradientCoefficient > (double)std::numeric_limits<float>::max())
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
		if (!IsFiniteVector(m_CellSize) || m_CellSize.x < std::numeric_limits<float>::min()
			|| m_CellSize.y < std::numeric_limits<float>::min()
			|| m_CellSize.z < std::numeric_limits<float>::min())
		{
			m_Initialized = false;
			return false;
		}
		const float MinCellSize = std::min(m_CellSize.x, std::min(m_CellSize.y, m_CellSize.z));
		const float MaxCellSize = std::max(m_CellSize.x, std::max(m_CellSize.y, m_CellSize.z));
		const double CellVolume = (double)m_CellSize.x * (double)m_CellSize.y * (double)m_CellSize.z;
		if (Settings.ParticleSpacing > MinCellSize * 1.0001f
			|| Settings.SmoothingRadius < 1.5f * MaxCellSize
			|| CellVolume < (double)std::numeric_limits<float>::min()
			|| CellVolume > (double)std::numeric_limits<float>::max())
		{
			m_Initialized = false;
			return false;
		}
		try
		{
			const size_t CellCount = ResolutionX * ResolutionY * ResolutionZ;
			m_GridDensity.assign(CellCount, Settings.AmbientDensity);
			m_GridPressure.assign(CellCount, Settings.AmbientPressure);
			m_GridVelocity.assign(CellCount, Vector3::Zero());
			m_Particles.clear();
			m_ParticleBuckets.clear();
			m_ReferenceParticleBuckets.clear();
			m_TimeRemainder = 0.0f;
			m_Initialized = true;
			if (Settings.Method == FluidSimulationMethod::Lagrangian)
			{
				SeedAmbientParticles();
				if (m_Particles.empty())
				{
					m_Initialized = false;
					return false;
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			m_GridDensity.clear();
			m_GridPressure.clear();
			m_GridVelocity.clear();
			m_Particles.clear();
			m_ParticleBuckets.clear();
			m_ReferenceParticleBuckets.clear();
			m_Initialized = false;
			return false;
		}
		return true;
	}

	void AerodynamicsSimulation::Reset()
	{
		if (!m_Initialized) return;
		std::fill(m_GridDensity.begin(), m_GridDensity.end(), m_Settings.AmbientDensity);
		std::fill(m_GridPressure.begin(), m_GridPressure.end(), m_Settings.AmbientPressure);
		std::fill(m_GridVelocity.begin(), m_GridVelocity.end(), Vector3::Zero());
		m_Particles.clear();
		m_ParticleBuckets.clear();
		m_ReferenceParticleBuckets.clear();
		m_TimeRemainder = 0.0f;
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian)
		{
			SeedAmbientParticles();
		}
	}

	bool AerodynamicsSimulation::SetMethod(FluidSimulationMethod Method, bool PreserveState)
	{
		if (!m_Initialized || (Method != FluidSimulationMethod::Eulerian
			&& Method != FluidSimulationMethod::Lagrangian))
		{
			return false;
		}
		if (Method == m_Settings.Method) return true;
		if (PreserveState)
		{
			bool Converted = false;
			try
			{
				Converted = Method == FluidSimulationMethod::Lagrangian
					? ConvertEulerianToLagrangian() : ConvertLagrangianToEulerian();
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}
			if (!Converted) return false;
			m_Settings.Method = Method;
			return true;
		}
		const FluidSimulationMethod PreviousMethod = m_Settings.Method;
		m_Settings.Method = Method;
		try
		{
			Reset();
		}
		catch (const std::bad_alloc&)
		{
			m_Settings.Method = PreviousMethod;
			return false;
		}
		return Method != FluidSimulationMethod::Lagrangian || !m_Particles.empty();
	}

	int AerodynamicsSimulation::GridIndex(int X, int Y, int Z) const
	{
		return X + m_Settings.Resolution.x * (Y + m_Settings.Resolution.y * Z);
	}

	bool AerodynamicsSimulation::IsGridCoordinateValid(int X, int Y, int Z) const
	{
		return X >= 0 && X < m_Settings.Resolution.x
			&& Y >= 0 && Y < m_Settings.Resolution.y
			&& Z >= 0 && Z < m_Settings.Resolution.z;
	}

	Vector3 AerodynamicsSimulation::GridCellCenter(int X, int Y, int Z) const
	{
		return m_Settings.BoundsMin + Vector3(((float)X + 0.5f) * m_CellSize.x,
			((float)Y + 0.5f) * m_CellSize.y, ((float)Z + 0.5f) * m_CellSize.z);
	}

	Vector3 AerodynamicsSimulation::GridFaceCenter(int X, int Y, int Z, int Component) const
	{
		Vector3 Position = GridCellCenter(X, Y, Z);
		if (Component == 0) Position.x += 0.5f * m_CellSize.x;
		if (Component == 1) Position.y += 0.5f * m_CellSize.y;
		if (Component == 2) Position.z += 0.5f * m_CellSize.z;
		return Position;
	}

	float AerodynamicsSimulation::SampleGridScalar(const std::vector<float>& Field,
		const Vector3& Position) const
	{
		if (Field.empty() || !IsFiniteVector(Position)) return 0.0f;
		const Vector3 GridPosition((Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
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
		const float C00 = Field[(size_t)GridIndex(X0, Y0, Z0)] * (1.0f - TX)
			+ Field[(size_t)GridIndex(X1, Y0, Z0)] * TX;
		const float C10 = Field[(size_t)GridIndex(X0, Y1, Z0)] * (1.0f - TX)
			+ Field[(size_t)GridIndex(X1, Y1, Z0)] * TX;
		const float C01 = Field[(size_t)GridIndex(X0, Y0, Z1)] * (1.0f - TX)
			+ Field[(size_t)GridIndex(X1, Y0, Z1)] * TX;
		const float C11 = Field[(size_t)GridIndex(X0, Y1, Z1)] * (1.0f - TX)
			+ Field[(size_t)GridIndex(X1, Y1, Z1)] * TX;
		return (C00 * (1.0f - TY) + C10 * TY) * (1.0f - TZ)
			+ (C01 * (1.0f - TY) + C11 * TY) * TZ;
	}

	float AerodynamicsSimulation::SampleGridVelocityComponent(const std::vector<Vector3>& Field,
		const Vector3& Position, int Component) const
	{
		const Vector3 GridPosition((Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
			(Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
			(Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
		if (Component == 0) return SampleMACComponent<0>(Field.data(), m_Settings.Resolution, GridPosition);
		if (Component == 1) return SampleMACComponent<1>(Field.data(), m_Settings.Resolution, GridPosition);
		return SampleMACComponent<2>(Field.data(), m_Settings.Resolution, GridPosition);
	}

	Vector3 AerodynamicsSimulation::SampleGridVelocity(const std::vector<Vector3>& Field,
		const Vector3& Position) const
	{
		if (Field.empty() || !IsFiniteVector(Position)) return Vector3::Zero();
		const Vector3 GridPosition((Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
			(Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
			(Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
		return SampleMACVelocity(Field.data(), m_Settings.Resolution, GridPosition);
	}

	void AerodynamicsSimulation::SetDensityBox(const Vector3& Min, const Vector3& Max, float Density)
	{
		if (!m_Initialized || !IsFiniteVector(Min) || !IsFiniteVector(Max)
			|| !std::isfinite(Density) || Density <= 0.0f || Density > 1.0e6f) return;
		const Vector3 ClippedMin = Min.Max(m_Settings.BoundsMin);
		const Vector3 ClippedMax = Max.Min(m_Settings.BoundsMax);
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				const Vector3 Position = GridCellCenter(X, Y, Z);
				if (Position.x >= ClippedMin.x && Position.x <= ClippedMax.x
					&& Position.y >= ClippedMin.y && Position.y <= ClippedMax.y
					&& Position.z >= ClippedMin.z && Position.z <= ClippedMax.z)
				{
					m_GridDensity[(size_t)GridIndex(X, Y, Z)] = Density;
				}
			}
			UpdateEulerianPressure();
			return;
		}
		for (AirParticle& Particle : m_Particles)
		{
			if (Particle.Position.x >= ClippedMin.x && Particle.Position.x <= ClippedMax.x
				&& Particle.Position.y >= ClippedMin.y && Particle.Position.y <= ClippedMax.y
				&& Particle.Position.z >= ClippedMin.z && Particle.Position.z <= ClippedMax.z)
			{
				Particle.Mass = Particle.ReferenceMass * Density / m_Settings.AmbientDensity;
			}
		}
		ComputeParticleDensities();
	}

	void AerodynamicsSimulation::SetDensitySphere(const Vector3& Center, float Radius, float Density)
	{
		if (!m_Initialized || !IsFiniteVector(Center) || !std::isfinite(Radius)
			|| !std::isfinite(Density) || Radius <= 0.0f || Density <= 0.0f || Density > 1.0e6f) return;
		const float RadiusSquared = Radius * Radius;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				if ((GridCellCenter(X, Y, Z) - Center).SquareLength() <= RadiusSquared)
					m_GridDensity[(size_t)GridIndex(X, Y, Z)] = Density;
			}
			UpdateEulerianPressure();
			return;
		}
		for (AirParticle& Particle : m_Particles)
		{
			if ((Particle.Position - Center).SquareLength() <= RadiusSquared)
				Particle.Mass = Particle.ReferenceMass * Density / m_Settings.AmbientDensity;
		}
		ComputeParticleDensities();
	}

	void AerodynamicsSimulation::SetVelocityBox(const Vector3& Min, const Vector3& Max,
		const Vector3& Velocity)
	{
		if (!m_Initialized || !IsFiniteVector(Min) || !IsFiniteVector(Max) || !IsFiniteVector(Velocity)) return;
		const Vector3 ClippedMin = Min.Max(m_Settings.BoundsMin);
		const Vector3 ClippedMax = Max.Min(m_Settings.BoundsMax);
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian)
		{
			for (AirParticle& Particle : m_Particles)
			{
				if (Particle.Position.x >= ClippedMin.x && Particle.Position.x <= ClippedMax.x
					&& Particle.Position.y >= ClippedMin.y && Particle.Position.y <= ClippedMax.y
					&& Particle.Position.z >= ClippedMin.z && Particle.Position.z <= ClippedMax.z)
					Particle.Velocity = Velocity;
			}
			return;
		}
		std::vector<uint8_t> Selected(m_GridDensity.size(), 0);
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const Vector3 Position = GridCellCenter(X, Y, Z);
			if (Position.x >= ClippedMin.x && Position.x <= ClippedMax.x
				&& Position.y >= ClippedMin.y && Position.y <= ClippedMax.y
				&& Position.z >= ClippedMin.z && Position.z <= ClippedMax.z)
				Selected[(size_t)GridIndex(X, Y, Z)] = 1;
		}
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const int Index = GridIndex(X, Y, Z);
			const bool Here = Selected[(size_t)Index] != 0;
			Vector3& Face = m_GridVelocity[(size_t)Index];
			if (Here || (X + 1 < m_Settings.Resolution.x && Selected[(size_t)GridIndex(X + 1, Y, Z)])) Face.x = Velocity.x;
			if (Here || (Y + 1 < m_Settings.Resolution.y && Selected[(size_t)GridIndex(X, Y + 1, Z)])) Face.y = Velocity.y;
			if (Here || (Z + 1 < m_Settings.Resolution.z && Selected[(size_t)GridIndex(X, Y, Z + 1)])) Face.z = Velocity.z;
		}
		EnforceEulerianBoundaries();
	}

	bool AerodynamicsSimulation::ApplyImpulse(const Vector3& Position, float Radius,
		const Vector3& Impulse)
	{
		if (!m_Initialized || !IsFiniteVector(Position) || !IsFiniteVector(Impulse)
			|| !std::isfinite(Radius) || Radius <= 0.0f) return false;
		const float RadiusSquared = Radius * Radius;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
			float Mass = 0.0f;
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				const int Index = GridIndex(X, Y, Z);
				if ((GridCellCenter(X, Y, Z) - Position).SquareLength() <= RadiusSquared)
					Mass += m_GridDensity[(size_t)Index] * CellVolume;
			}
			if (!(Mass > SmallValue)) return false;
			const Vector3 DeltaVelocity = Impulse / Mass;
			for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
			for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
			for (int X = 0; X < m_Settings.Resolution.x; ++X)
			{
				const int Index = GridIndex(X, Y, Z);
				if ((GridCellCenter(X, Y, Z) - Position).SquareLength() <= RadiusSquared)
					m_GridVelocity[(size_t)Index] += DeltaVelocity;
			}
			EnforceEulerianBoundaries();
			return true;
		}

		float Mass = 0.0f;
		for (const AirParticle& Particle : m_Particles)
			if ((Particle.Position - Position).SquareLength() <= RadiusSquared) Mass += Particle.Mass;
		if (!(Mass > SmallValue)) return false;
		const Vector3 DeltaVelocity = Impulse / Mass;
		for (AirParticle& Particle : m_Particles)
			if ((Particle.Position - Position).SquareLength() <= RadiusSquared)
				Particle.Velocity += DeltaVelocity;
		return true;
	}

	void AerodynamicsSimulation::Step(float DeltaTime)
	{
		if (!m_Initialized || !(DeltaTime > 0.0f) || !std::isfinite(DeltaTime)) return;
		m_TimeRemainder = std::min((double)std::numeric_limits<float>::max(),
			m_TimeRemainder + (double)DeltaTime);
		for (int Substep = 0; Substep < m_Settings.MaxSubsteps && m_TimeRemainder > 0.0; ++Substep)
		{
			const float StepLimit = ComputeStableTimeStep();
			if (!(StepLimit > 0.0f) || !std::isfinite(StepLimit)) break;
			const float TimeStep = (float)std::min(m_TimeRemainder, (double)StepLimit);
			if (m_Settings.Method == FluidSimulationMethod::Eulerian) StepEulerian(TimeStep);
			else StepLagrangian(TimeStep);
			if (!m_Initialized) break;
			m_TimeRemainder = std::max(0.0, m_TimeRemainder - (double)TimeStep);
		}
	}

	float AerodynamicsSimulation::ComputeStableTimeStep() const
	{
		float MaxSpeedSquared = 0.0f;
		float MaxDensity = m_Settings.AmbientDensity;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			for (const Vector3& Velocity : m_GridVelocity)
				MaxSpeedSquared = std::max(MaxSpeedSquared, Velocity.SquareLength());
			for (float Density : m_GridDensity) MaxDensity = std::max(MaxDensity, Density);
		}
		else
		{
			for (const AirParticle& Particle : m_Particles)
			{
				MaxSpeedSquared = std::max(MaxSpeedSquared, Particle.Velocity.SquareLength());
				MaxDensity = std::max(MaxDensity, Particle.Density);
			}
		}
		// The equation of state is monotone for the validated exponent [1, 2].
		const float MaxSpeed = std::sqrt(MaxSpeedSquared);
		const float MaxSoundSpeed = m_Settings.SpeedOfSound * std::pow(
			MaxDensity / m_Settings.AmbientDensity, 0.5f * (m_Settings.AdiabaticExponent - 1.0f));
		const float MinSpacing = m_Settings.Method == FluidSimulationMethod::Eulerian
			? std::min(m_CellSize.x, std::min(m_CellSize.y, m_CellSize.z)) : m_Settings.ParticleSpacing;
		const float AcousticStep = m_Settings.AcousticCFL * MinSpacing
			/ std::max(MaxSoundSpeed + MaxSpeed, SmallValue);
		float StableStep = std::min(m_Settings.MaxTimeStep, AcousticStep);
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian
			&& m_Settings.DynamicViscosity > 0.0f && !m_Particles.empty())
		{
			float MinimumDensity = std::numeric_limits<float>::max();
			for (const AirParticle& Particle : m_Particles)
				MinimumDensity = std::min(MinimumDensity, Particle.Density);
			const float ViscousStep = 0.125f * m_Settings.SmoothingRadius
				* m_Settings.SmoothingRadius * MinimumDensity / m_Settings.DynamicViscosity;
			StableStep = std::min(StableStep, ViscousStep);
		}
		return StableStep;
	}

	void AerodynamicsSimulation::StepEulerian(float DeltaTime)
	{
		m_DensityScratch = m_GridDensity;
		const std::vector<float>& OldDensity = m_DensityScratch;
		const std::vector<Vector3>& OldVelocity = m_GridVelocity;
		AdvectEulerianDensity(DeltaTime, OldDensity, OldVelocity);
		AdvectEulerianVelocity(DeltaTime, OldVelocity);
		UpdateEulerianPressure();
		ApplyEulerianPressureAndForces(DeltaTime);
		if (!m_Initialized) return;
		EnforceEulerianBoundaries();
	}

	void AerodynamicsSimulation::AdvectEulerianDensity(float DeltaTime,
		const std::vector<float>& OldDensity, const std::vector<Vector3>& OldVelocity)
	{
		m_GridDensity = OldDensity;
		auto TransferFace = [&](int Left, int Right, float Velocity, float Spacing)
		{
			if (Velocity == 0.0f) return;
			const int Source = Velocity > 0.0f ? Left : Right;
			const int Destination = Velocity > 0.0f ? Right : Left;
			const double Desired = std::fabs((double)Velocity) * (double)DeltaTime
				* (double)OldDensity[(size_t)Source] / (double)Spacing;
			const float Amount = (float)std::min(Desired,
				(double)std::max(0.0f, m_GridDensity[(size_t)Source]));
			m_GridDensity[(size_t)Source] -= Amount;
			m_GridDensity[(size_t)Destination] += Amount;
		};
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		{
			for (int X = m_Settings.Resolution.x - 2; X >= 0; --X)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].x;
				if (V > 0.0f) TransferFace(I, GridIndex(X + 1, Y, Z), V, m_CellSize.x);
			}
			for (int X = 0; X + 1 < m_Settings.Resolution.x; ++X)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].x;
				if (V < 0.0f) TransferFace(I, GridIndex(X + 1, Y, Z), V, m_CellSize.x);
			}
		}
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			for (int Y = m_Settings.Resolution.y - 2; Y >= 0; --Y)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].y;
				if (V > 0.0f) TransferFace(I, GridIndex(X, Y + 1, Z), V, m_CellSize.y);
			}
			for (int Y = 0; Y + 1 < m_Settings.Resolution.y; ++Y)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].y;
				if (V < 0.0f) TransferFace(I, GridIndex(X, Y + 1, Z), V, m_CellSize.y);
			}
		}
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			for (int Z = m_Settings.Resolution.z - 2; Z >= 0; --Z)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].z;
				if (V > 0.0f) TransferFace(I, GridIndex(X, Y, Z + 1), V, m_CellSize.z);
			}
			for (int Z = 0; Z + 1 < m_Settings.Resolution.z; ++Z)
			{
				const int I = GridIndex(X, Y, Z); const float V = OldVelocity[(size_t)I].z;
				if (V < 0.0f) TransferFace(I, GridIndex(X, Y, Z + 1), V, m_CellSize.z);
			}
		}
	}

	void AerodynamicsSimulation::AdvectEulerianVelocity(float DeltaTime,
		const std::vector<Vector3>& OldVelocity)
	{
		m_VelocityOutput.resize(OldVelocity.size());
		std::vector<Vector3>& Advected = m_VelocityOutput;
		FluidParallelFor((int)OldVelocity.size(), [&](int Index)
		{
			const int X = Index % m_Settings.Resolution.x;
			const int Y = (Index / m_Settings.Resolution.x) % m_Settings.Resolution.y;
			const int Z = Index / (m_Settings.Resolution.x * m_Settings.Resolution.y);
			Vector3& Value = Advected[(size_t)GridIndex(X, Y, Z)];
			for (int Component = 0; Component < 3; ++Component)
			{
				const Vector3 Face = GridFaceCenter(X, Y, Z, Component);
				const Vector3 Velocity = SampleGridVelocity(OldVelocity, Face);
				const Vector3 MidVelocity = SampleGridVelocity(OldVelocity, Face - Velocity * (0.5f * DeltaTime));
				Vector3 Source = Face - MidVelocity * DeltaTime;
				Source.x = ClampFloat(Source.x, m_Settings.BoundsMin.x, m_Settings.BoundsMax.x);
				Source.y = ClampFloat(Source.y, m_Settings.BoundsMin.y, m_Settings.BoundsMax.y);
				Source.z = ClampFloat(Source.z, m_Settings.BoundsMin.z, m_Settings.BoundsMax.z);
				const float Sampled = SampleGridVelocityComponent(OldVelocity, Source, Component);
				if (Component == 0) Value.x = Sampled;
				if (Component == 1) Value.y = Sampled;
				if (Component == 2) Value.z = Sampled;
			}
		});
		m_GridVelocity.swap(Advected);
	}

	void AerodynamicsSimulation::UpdateEulerianPressure()
	{
		for (size_t Index = 0; Index < m_GridDensity.size(); ++Index)
		{
			m_GridPressure[Index] = PressureFromDensity(m_GridDensity[Index]);
		}
	}

	float AerodynamicsSimulation::PressureFromDensity(float Density) const
	{
		const float DensityRatio = std::max(Density / m_Settings.AmbientDensity, 0.01f);
		const float PressureScale = m_Settings.AmbientDensity * m_Settings.SpeedOfSound
			* m_Settings.SpeedOfSound / m_Settings.AdiabaticExponent;
		const float GaugePressure = PressureScale
			* (std::pow(DensityRatio, m_Settings.AdiabaticExponent) - 1.0f);
		return std::max(0.0f, m_Settings.AmbientPressure + GaugePressure);
	}

	void AerodynamicsSimulation::ApplyEulerianPressureAndForces(float DeltaTime)
	{
		const std::vector<Vector3>& OldVelocity = m_GridVelocity;
		m_VelocityOutput = OldVelocity;
		std::vector<Vector3>& NewVelocity = m_VelocityOutput;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const int Index = GridIndex(X, Y, Z);
			Vector3& FaceVelocity = NewVelocity[(size_t)Index];
			for (int Component = 0; Component < 3; ++Component)
			{
				const bool Boundary = (Component == 0 && X + 1 == m_Settings.Resolution.x)
					|| (Component == 1 && Y + 1 == m_Settings.Resolution.y)
					|| (Component == 2 && Z + 1 == m_Settings.Resolution.z);
				float* Value = Component == 0 ? &FaceVelocity.x : (Component == 1 ? &FaceVelocity.y : &FaceVelocity.z);
				if (Boundary) { *Value = 0.0f; continue; }
				const int Neighbor = Component == 0 ? GridIndex(X + 1, Y, Z)
					: (Component == 1 ? GridIndex(X, Y + 1, Z) : GridIndex(X, Y, Z + 1));
				const float Spacing = Component == 0 ? m_CellSize.x : (Component == 1 ? m_CellSize.y : m_CellSize.z);
				const float FaceDensity = std::max(0.5f * (m_GridDensity[(size_t)Index]
					+ m_GridDensity[(size_t)Neighbor]), 0.05f * m_Settings.AmbientDensity);
				const float PressureGradient = (m_GridPressure[(size_t)Neighbor]
					- m_GridPressure[(size_t)Index]) / Spacing;
				const float Gravity = Component == 0 ? m_Settings.Gravity.x
					: (Component == 1 ? m_Settings.Gravity.y : m_Settings.Gravity.z);
				*Value += DeltaTime * (Gravity - PressureGradient / FaceDensity);
				if (m_Settings.DynamicViscosity > 0.0f)
				{
					float Laplacian = 0.0f, Weight = 0.0f;
					const int Offset[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 },
						{ 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };
					const Vector3& Center = OldVelocity[(size_t)Index];
					const float CenterValue = Component == 0 ? Center.x : (Component == 1 ? Center.y : Center.z);
					for (int N = 0; N < 6; ++N)
					{
						const int NX = X + Offset[N][0], NY = Y + Offset[N][1], NZ = Z + Offset[N][2];
						if (!IsGridCoordinateValid(NX, NY, NZ)) continue;
						const float H = N < 2 ? m_CellSize.x : (N < 4 ? m_CellSize.y : m_CellSize.z);
						const float InvH2 = 1.0f / (H * H);
						const Vector3& Other = OldVelocity[(size_t)GridIndex(NX, NY, NZ)];
						const float OtherValue = Component == 0 ? Other.x : (Component == 1 ? Other.y : Other.z);
						Laplacian += (OtherValue - CenterValue) * InvH2;
						Weight += InvH2;
					}
					const float KinematicViscosity = m_Settings.DynamicViscosity / FaceDensity;
					const float Scale = Weight > 0.0f ? std::min(DeltaTime * KinematicViscosity, 0.24f / Weight) : 0.0f;
					*Value += Laplacian * Scale;
				}
			}
		}
		if (m_Settings.Backend == FluidBackend::CPU)
		{
			m_GridVelocity.swap(NewVelocity);
			return;
		}
		m_VelocityDelta.resize(NewVelocity.size());
		std::vector<Vector3>& Delta = m_VelocityDelta;
		for (size_t Index = 0; Index < Delta.size(); ++Index)
			Delta[Index] = NewVelocity[Index] - OldVelocity[Index];
		if (!ApplyFluidVelocityDelta(OldVelocity.data(), Delta.data(), Delta.size(),
			m_GridVelocity.data(), m_Settings.Backend, m_Settings.AllowBackendFallback,
			&m_BackendResult))
		{
			if (m_Settings.AllowBackendFallback) m_GridVelocity.swap(NewVelocity);
			else { m_Initialized = false; return; }
		}
	}

	void AerodynamicsSimulation::EnforceEulerianBoundaries()
	{
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			Vector3& Velocity = m_GridVelocity[(size_t)GridIndex(X, Y, Z)];
			if (X + 1 == m_Settings.Resolution.x) Velocity.x = 0.0f;
			if (Y + 1 == m_Settings.Resolution.y) Velocity.y = 0.0f;
			if (Z + 1 == m_Settings.Resolution.z) Velocity.z = 0.0f;
		}
	}

	void AerodynamicsSimulation::SeedAmbientParticles()
	{
		m_Particles.clear();
		const Vector3 Extent = m_Settings.BoundsMax - m_Settings.BoundsMin;
		const size_t CountX = std::max((size_t)1,
			(size_t)std::floor((double)Extent.x / (double)m_Settings.ParticleSpacing));
		const size_t CountY = std::max((size_t)1,
			(size_t)std::floor((double)Extent.y / (double)m_Settings.ParticleSpacing));
		const size_t CountZ = std::max((size_t)1,
			(size_t)std::floor((double)Extent.z / (double)m_Settings.ParticleSpacing));
		if (CountX > MaxParticleCount / CountY || CountX * CountY > MaxParticleCount / CountZ) return;
		const Vector3 Spacing(Extent.x / (float)CountX, Extent.y / (float)CountY, Extent.z / (float)CountZ);
		m_Particles.reserve(CountX * CountY * CountZ);
		const float Mass = m_Settings.AmbientDensity * Spacing.x * Spacing.y * Spacing.z;
		for (size_t IZ = 0; IZ < CountZ; ++IZ)
		for (size_t IY = 0; IY < CountY; ++IY)
		for (size_t IX = 0; IX < CountX; ++IX)
		{
			AirParticle Particle;
			Particle.Position = Vector3(
				m_Settings.BoundsMin.x + ((float)IX + 0.5f) * Spacing.x,
				m_Settings.BoundsMin.y + ((float)IY + 0.5f) * Spacing.y,
				m_Settings.BoundsMin.z + ((float)IZ + 0.5f) * Spacing.z);
			Particle.ReferencePosition = Particle.Position;
			Particle.Mass = Particle.ReferenceMass = Mass;
			m_Particles.push_back(Particle);
		}
		InitializeParticleNormalizations();
		ComputeParticleDensities();
	}

	uint64_t AerodynamicsSimulation::ParticleBucketKey(int X, int Y, int Z) const
	{
		const uint64_t HX = MixBucketCoordinate((uint64_t)(int64_t)X + 0x9e3779b97f4a7c15ULL);
		const uint64_t HY = MixBucketCoordinate((uint64_t)(int64_t)Y + 0x632be59bd9b4e019ULL);
		const uint64_t HZ = MixBucketCoordinate((uint64_t)(int64_t)Z + 0x94d049bb133111ebULL);
		return HX ^ (HY << 1) ^ (HZ << 7);
	}

	void AerodynamicsSimulation::RebuildParticleBuckets()
	{
		m_ParticleBuckets.Prepare(m_Particles.size());
		for (size_t Index = 0; Index < m_Particles.size(); ++Index)
		{
			const Vector3& Position = m_Particles[Index].Position;
			const int X = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / m_Settings.SmoothingRadius);
			const int Y = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / m_Settings.SmoothingRadius);
			const int Z = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / m_Settings.SmoothingRadius);
			m_ParticleBuckets.Insert(ParticleBucketKey(X, Y, Z), (int)Index);
		}
	}

	void AerodynamicsSimulation::InitializeParticleNormalizations()
	{
		RebuildReferenceParticleBuckets();
		for (AirParticle& Particle : m_Particles)
		{
			const float ReferenceDensity = ComputeReferenceDensityAt(Particle.Position);
			Particle.DensityNormalization = ReferenceDensity > 0.0f
				? m_Settings.AmbientDensity / ReferenceDensity : 1.0f;
		}
	}

	void AerodynamicsSimulation::RebuildReferenceParticleBuckets()
	{
		m_ReferenceParticleBuckets.Prepare(m_Particles.size());
		for (size_t Index = 0; Index < m_Particles.size(); ++Index)
		{
			const Vector3& Position = m_Particles[Index].ReferencePosition;
			const int X = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / m_Settings.SmoothingRadius);
			const int Y = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / m_Settings.SmoothingRadius);
			const int Z = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / m_Settings.SmoothingRadius);
			m_ReferenceParticleBuckets.Insert(ParticleBucketKey(X, Y, Z), (int)Index);
		}
	}

	float AerodynamicsSimulation::ComputeReferenceDensityAt(const Vector3& Position) const
	{
		const float H = m_Settings.SmoothingRadius, H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		float ReferenceDensity = 0.0f;
		const int BaseX = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / H);
		const int BaseY = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / H);
		const int BaseZ = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / H);
		for (int DZ = -1; DZ <= 1; ++DZ)
		for (int DY = -1; DY <= 1; ++DY)
		for (int DX = -1; DX <= 1; ++DX)
		{
			const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
			auto It = m_ReferenceParticleBuckets.Find(Key);
			if (It == nullptr) continue;
			for (int NeighborIndex : It->Indices)
			{
				const AirParticle& Neighbor = m_Particles[(size_t)NeighborIndex];
				const float R2 = (Position - Neighbor.ReferencePosition).SquareLength();
				if (R2 < H2)
				{
					const float Term = H2 - R2;
					ReferenceDensity += Neighbor.ReferenceMass * Poly6 * Term * Term * Term;
				}
			}
		}
		return ReferenceDensity;
	}

	void AerodynamicsSimulation::ComputeParticleDensities()
	{
		RebuildParticleBuckets();
		const float H = m_Settings.SmoothingRadius, H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		FluidParallelFor((int)m_Particles.size(), [&](int ParticleIndex)
		{
			AirParticle& Particle = m_Particles[ParticleIndex];
			float RawDensity = 0.0f;
			const float ReferenceDensity = ComputeReferenceDensityAt(Particle.Position);
			Particle.DensityNormalization = ReferenceDensity > 0.0f
				? m_Settings.AmbientDensity / ReferenceDensity : 1.0f;
			const int BaseX = (int)std::floor((Particle.Position.x - m_Settings.BoundsMin.x) / H);
			const int BaseY = (int)std::floor((Particle.Position.y - m_Settings.BoundsMin.y) / H);
			const int BaseZ = (int)std::floor((Particle.Position.z - m_Settings.BoundsMin.z) / H);
			for (int DZ = -1; DZ <= 1; ++DZ)
			for (int DY = -1; DY <= 1; ++DY)
			for (int DX = -1; DX <= 1; ++DX)
			{
				const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
				auto It = m_ParticleBuckets.Find(Key);
				if (It == nullptr) continue;
				for (int NeighborIndex : It->Indices)
				{
					const AirParticle& Neighbor = m_Particles[(size_t)NeighborIndex];
					const float R2 = (Particle.Position - Neighbor.Position).SquareLength();
					if (R2 < H2)
					{
						const float Term = H2 - R2;
						RawDensity += Neighbor.Mass * Poly6 * Term * Term * Term;
					}
				}
			}
			Particle.Density = std::max(0.05f * m_Settings.AmbientDensity,
				RawDensity * Particle.DensityNormalization);
			Particle.Pressure = PressureFromDensity(Particle.Density);
		});
	}

	void AerodynamicsSimulation::ComputeParticleAccelerations()
	{
		const float H = m_Settings.SmoothingRadius;
		const float Spiky = -45.0f / (Pi * std::pow(H, 6.0f));
		const float Viscosity = 45.0f / (Pi * std::pow(H, 6.0f));
		const float MinGauge = -0.25f * m_Settings.AmbientDensity
			* m_Settings.SpeedOfSound * m_Settings.SpeedOfSound;
		FluidParallelFor((int)m_Particles.size(), [&](int ParticleIndex)
		{
			AirParticle& Particle = m_Particles[ParticleIndex];
			Vector3 Acceleration = m_Settings.Gravity;
			const int BaseX = (int)std::floor((Particle.Position.x - m_Settings.BoundsMin.x) / H);
			const int BaseY = (int)std::floor((Particle.Position.y - m_Settings.BoundsMin.y) / H);
			const int BaseZ = (int)std::floor((Particle.Position.z - m_Settings.BoundsMin.z) / H);
			for (int DZ = -1; DZ <= 1; ++DZ)
			for (int DY = -1; DY <= 1; ++DY)
			for (int DX = -1; DX <= 1; ++DX)
			{
				const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
				auto It = m_ParticleBuckets.Find(Key);
				if (It == nullptr) continue;
				for (int NeighborIndex : It->Indices)
				{
					if (NeighborIndex == (int)ParticleIndex) continue;
					const AirParticle& Neighbor = m_Particles[(size_t)NeighborIndex];
					const Vector3 Difference = Particle.Position - Neighbor.Position;
					const float DistanceSquared = Difference.SquareLength();
					if (DistanceSquared >= H * H) continue;
					const float Distance = std::sqrt(DistanceSquared);
					if (!(Distance > SmallValue && Distance < H)) continue;
					const float KernelDistance = H - Distance;
					const Vector3 Gradient = Difference * (Spiky * KernelDistance * KernelDistance / Distance);
					const float PressureA = std::max(Particle.Pressure - m_Settings.AmbientPressure, MinGauge);
					const float PressureB = std::max(Neighbor.Pressure - m_Settings.AmbientPressure, MinGauge);
					float ArtificialPressure = 0.0f;
					const Vector3 RelativeVelocity = Particle.Velocity - Neighbor.Velocity;
					const float ApproachingSpeed = RelativeVelocity.Dot(Difference);
					if (ApproachingSpeed < 0.0f && m_Settings.ArtificialViscosity > 0.0f)
					{
						const float Mu = H * ApproachingSpeed
							/ (Difference.SquareLength() + 0.01f * H * H);
						const float AverageDensity = 0.5f * (Particle.Density + Neighbor.Density);
						ArtificialPressure = (-m_Settings.ArtificialViscosity * m_Settings.SpeedOfSound * Mu
							+ 2.0f * m_Settings.ArtificialViscosity * Mu * Mu) / AverageDensity;
					}
					const float PressureScale = Neighbor.Mass * (PressureA / (Particle.Density * Particle.Density)
						+ PressureB / (Neighbor.Density * Neighbor.Density) + ArtificialPressure);
					Acceleration -= Gradient * PressureScale;
					Acceleration += (Neighbor.Velocity - Particle.Velocity)
						* (2.0f * m_Settings.DynamicViscosity * Neighbor.Mass * Viscosity
							* KernelDistance / (Particle.Density + Neighbor.Density));
				}
			}
			Particle.Acceleration = IsFiniteVector(Acceleration) ? Acceleration : m_Settings.Gravity;
		});
	}

	void AerodynamicsSimulation::EnforceParticleBoundary(AirParticle& Particle) const
	{
		const float Radius = 0.5f * m_Settings.ParticleSpacing;
		const Vector3 Min = m_Settings.BoundsMin + Vector3(Radius);
		const Vector3 Max = m_Settings.BoundsMax - Vector3(Radius);
		if (Particle.Position.x < Min.x) { Particle.Position.x = Min.x; if (Particle.Velocity.x < 0.0f) Particle.Velocity.x *= -m_Settings.BoundaryRestitution; }
		if (Particle.Position.x > Max.x) { Particle.Position.x = Max.x; if (Particle.Velocity.x > 0.0f) Particle.Velocity.x *= -m_Settings.BoundaryRestitution; }
		if (Particle.Position.y < Min.y) { Particle.Position.y = Min.y; if (Particle.Velocity.y < 0.0f) Particle.Velocity.y *= -m_Settings.BoundaryRestitution; }
		if (Particle.Position.y > Max.y) { Particle.Position.y = Max.y; if (Particle.Velocity.y > 0.0f) Particle.Velocity.y *= -m_Settings.BoundaryRestitution; }
		if (Particle.Position.z < Min.z) { Particle.Position.z = Min.z; if (Particle.Velocity.z < 0.0f) Particle.Velocity.z *= -m_Settings.BoundaryRestitution; }
		if (Particle.Position.z > Max.z) { Particle.Position.z = Max.z; if (Particle.Velocity.z > 0.0f) Particle.Velocity.z *= -m_Settings.BoundaryRestitution; }
	}

	void AerodynamicsSimulation::StepLagrangian(float DeltaTime)
	{
		if (m_Particles.empty()) return;
		// Positions and densities are synchronized at the end of the previous step.
		ComputeParticleAccelerations();
		m_VelocityScratch.resize(m_Particles.size());
		m_VelocityDelta.resize(m_Particles.size());
		m_VelocityOutput.resize(m_Particles.size());
		std::vector<Vector3>& Velocity = m_VelocityScratch;
		std::vector<Vector3>& Delta = m_VelocityDelta;
		std::vector<Vector3>& Integrated = m_VelocityOutput;
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
			AirParticle& Particle = m_Particles[Index];
			Particle.Velocity = Integrated[Index];
			Particle.Position += Particle.Velocity * DeltaTime;
			EnforceParticleBoundary(Particle);
		}
		ComputeParticleDensities();
	}

	float AerodynamicsSimulation::SampleLagrangianDensity(const Vector3& Position) const
	{
		const float H = m_Settings.SmoothingRadius, H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		float WeightedDensity = 0.0f, WeightSum = 0.0f;
		const int BaseX = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / H);
		const int BaseY = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / H);
		const int BaseZ = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / H);
		for (int DZ = -1; DZ <= 1; ++DZ)
		for (int DY = -1; DY <= 1; ++DY)
		for (int DX = -1; DX <= 1; ++DX)
		{
			const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
			auto It = m_ParticleBuckets.Find(Key);
			if (It == nullptr) continue;
			for (int ParticleIndex : It->Indices)
			{
				const AirParticle& Particle = m_Particles[(size_t)ParticleIndex];
				const float R2 = (Position - Particle.Position).SquareLength();
				if (R2 < H2)
				{
					const float Term = H2 - R2, Weight = Poly6 * Term * Term * Term;
					WeightedDensity += Particle.Density * Weight;
					WeightSum += Weight;
				}
			}
		}
		return WeightSum > 0.0f ? WeightedDensity / WeightSum : 0.0f;
	}

	Vector3 AerodynamicsSimulation::SampleLagrangianVelocity(const Vector3& Position) const
	{
		const float H = m_Settings.SmoothingRadius, H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * Pi * std::pow(H, 9.0f));
		Vector3 Velocity = Vector3::Zero(); float WeightSum = 0.0f;
		const int BaseX = (int)std::floor((Position.x - m_Settings.BoundsMin.x) / H);
		const int BaseY = (int)std::floor((Position.y - m_Settings.BoundsMin.y) / H);
		const int BaseZ = (int)std::floor((Position.z - m_Settings.BoundsMin.z) / H);
		for (int DZ = -1; DZ <= 1; ++DZ)
		for (int DY = -1; DY <= 1; ++DY)
		for (int DX = -1; DX <= 1; ++DX)
		{
			const uint64_t Key = ParticleBucketKey(BaseX + DX, BaseY + DY, BaseZ + DZ);
			auto It = m_ParticleBuckets.Find(Key);
			if (It == nullptr) continue;
			for (int ParticleIndex : It->Indices)
			{
				const AirParticle& Particle = m_Particles[(size_t)ParticleIndex];
				const float R2 = (Position - Particle.Position).SquareLength();
				if (R2 < H2)
				{
					const float Term = H2 - R2, Weight = Particle.Mass * Poly6 * Term * Term * Term;
					Velocity += Particle.Velocity * Weight; WeightSum += Weight;
				}
			}
		}
		return WeightSum > 0.0f ? Velocity / WeightSum : Vector3::Zero();
	}

	float AerodynamicsSimulation::SamplePressureOnly(const Vector3& Position) const
	{
		const Vector3 Clamped(ClampFloat(Position.x, m_Settings.BoundsMin.x, m_Settings.BoundsMax.x),
			ClampFloat(Position.y, m_Settings.BoundsMin.y, m_Settings.BoundsMax.y),
			ClampFloat(Position.z, m_Settings.BoundsMin.z, m_Settings.BoundsMax.z));
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
			return SampleGridScalar(m_GridPressure, Clamped);
		const float Density = SampleLagrangianDensity(Clamped);
		return PressureFromDensity(Density);
	}

	AerodynamicSample AerodynamicsSimulation::Sample(const Vector3& Position) const
	{
		AerodynamicSample Result;
		Result.Density = m_Settings.AmbientDensity;
		Result.Pressure = m_Settings.AmbientPressure;
		if (!m_Initialized || !IsFiniteVector(Position)
			|| Position.x < m_Settings.BoundsMin.x || Position.x > m_Settings.BoundsMax.x
			|| Position.y < m_Settings.BoundsMin.y || Position.y > m_Settings.BoundsMax.y
			|| Position.z < m_Settings.BoundsMin.z || Position.z > m_Settings.BoundsMax.z) return Result;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			Result.Density = SampleGridScalar(m_GridDensity, Position);
			Result.Pressure = SampleGridScalar(m_GridPressure, Position);
			Result.Velocity = SampleGridVelocity(m_GridVelocity, Position);
		}
		else
		{
			Result.Density = SampleLagrangianDensity(Position);
			Result.Pressure = SamplePressureOnly(Position);
			Result.Velocity = SampleLagrangianVelocity(Position);
		}
		const Vector3 H = m_Settings.Method == FluidSimulationMethod::Eulerian
			? m_CellSize : Vector3(m_Settings.ParticleSpacing);
		const Vector3 MinX = Position - Vector3(H.x, 0.0f, 0.0f);
		const Vector3 MaxX = Position + Vector3(H.x, 0.0f, 0.0f);
		const Vector3 MinY = Position - Vector3(0.0f, H.y, 0.0f);
		const Vector3 MaxY = Position + Vector3(0.0f, H.y, 0.0f);
		const Vector3 MinZ = Position - Vector3(0.0f, 0.0f, H.z);
		const Vector3 MaxZ = Position + Vector3(0.0f, 0.0f, H.z);
		const float DX = ClampFloat(MaxX.x, m_Settings.BoundsMin.x, m_Settings.BoundsMax.x)
			- ClampFloat(MinX.x, m_Settings.BoundsMin.x, m_Settings.BoundsMax.x);
		const float DY = ClampFloat(MaxY.y, m_Settings.BoundsMin.y, m_Settings.BoundsMax.y)
			- ClampFloat(MinY.y, m_Settings.BoundsMin.y, m_Settings.BoundsMax.y);
		const float DZ = ClampFloat(MaxZ.z, m_Settings.BoundsMin.z, m_Settings.BoundsMax.z)
			- ClampFloat(MinZ.z, m_Settings.BoundsMin.z, m_Settings.BoundsMax.z);
		Result.PressureGradient = Vector3(
			DX > 0.0f ? (SamplePressureOnly(MaxX) - SamplePressureOnly(MinX)) / DX : 0.0f,
			DY > 0.0f ? (SamplePressureOnly(MaxY) - SamplePressureOnly(MinY)) / DY : 0.0f,
			DZ > 0.0f ? (SamplePressureOnly(MaxZ) - SamplePressureOnly(MinZ)) / DZ : 0.0f);
		return Result;
	}

	float AerodynamicsSimulation::GetTotalMass() const
	{
		if (!m_Initialized) return 0.0f;
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian)
		{
			float Mass = 0.0f; for (const AirParticle& Particle : m_Particles) Mass += Particle.Mass; return Mass;
		}
		const float Volume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		float Mass = 0.0f; for (float Density : m_GridDensity) Mass += Density * Volume; return Mass;
	}

	Vector3 AerodynamicsSimulation::GetTotalMomentum() const
	{
		if (!m_Initialized) return Vector3::Zero();
		Vector3 Momentum = Vector3::Zero();
		if (m_Settings.Method == FluidSimulationMethod::Lagrangian)
		{
			for (const AirParticle& Particle : m_Particles) Momentum += Particle.Velocity * Particle.Mass;
			return Momentum;
		}
		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const int Index = GridIndex(X, Y, Z);
			Momentum += SampleGridVelocity(m_GridVelocity, GridCellCenter(X, Y, Z))
				* (m_GridDensity[(size_t)Index] * CellVolume);
		}
		return Momentum;
	}

	bool AerodynamicsSimulation::BuildDensityField(std::vector<float>* DensityField) const
	{
		if (!m_Initialized || DensityField == nullptr) return false;
		if (m_Settings.Method == FluidSimulationMethod::Eulerian)
		{
			*DensityField = m_GridDensity;
			return true;
		}

		DensityField->assign(m_GridDensity.size(), m_Settings.AmbientDensity);
		FluidParallelFor((int)m_GridDensity.size(), [&](int Index)
		{
			const int X = Index % m_Settings.Resolution.x;
			const int Y = (Index / m_Settings.Resolution.x) % m_Settings.Resolution.y;
			const int Z = Index / (m_Settings.Resolution.x * m_Settings.Resolution.y);			(*DensityField)[(size_t)GridIndex(X, Y, Z)] =
				SampleLagrangianDensity(GridCellCenter(X, Y, Z));
		});
		return true;
	}

	AerodynamicForceResult AerodynamicsSimulation::ComputeSurfaceForce(const Vector3& CenterOfMass,
		const std::vector<AerodynamicSurfaceSample>& SurfaceSamples, float DragCoefficient) const
	{
		AerodynamicForceResult Result;
		if (!m_Initialized || !IsFiniteVector(CenterOfMass) || !std::isfinite(DragCoefficient)
			|| DragCoefficient < 0.0f) return Result;
		for (const AerodynamicSurfaceSample& Surface : SurfaceSamples)
		{
			if (!IsFiniteVector(Surface.Position) || !IsFiniteVector(Surface.Normal)
				|| !IsFiniteVector(Surface.Velocity) || !std::isfinite(Surface.Area) || Surface.Area <= 0.0f) continue;
			const Vector3 Normal = Surface.Normal.SafeUnit();
			const AerodynamicSample Air = Sample(Surface.Position);
			Vector3 Force = -Normal * ((Air.Pressure - m_Settings.AmbientPressure) * Surface.Area);
			const Vector3 RelativeVelocity = Air.Velocity - Surface.Velocity;
			const float NormalSpeed = RelativeVelocity.Dot(Normal);
			if (NormalSpeed < 0.0f)
			{
				Force += RelativeVelocity * (0.5f * Air.Density * DragCoefficient
					* Surface.Area * (-NormalSpeed));
			}
			Result.Force += Force;
			Result.Torque += (Surface.Position - CenterOfMass).Cross(Force);
		}
		return Result;
	}

	AerodynamicForceResult AerodynamicsSimulation::ComputeSphereForce(const Vector3& Center, float Radius,
		const Vector3& LinearVelocity, float DragCoefficient) const
	{
		AerodynamicForceResult Result;
		if (!m_Initialized || !IsFiniteVector(Center) || !IsFiniteVector(LinearVelocity)
			|| !std::isfinite(Radius) || !std::isfinite(DragCoefficient)
			|| Radius <= 0.0f || DragCoefficient < 0.0f) return Result;
		const AerodynamicSample Air = Sample(Center);
		const float Volume = (4.0f / 3.0f) * Pi * Radius * Radius * Radius;
		const float Area = Pi * Radius * Radius;
		const Vector3 RelativeVelocity = Air.Velocity - LinearVelocity;
		Result.Force = -Air.PressureGradient * Volume;
		const float Speed = RelativeVelocity.Length();
		if (Speed > SmallValue)
			Result.Force += RelativeVelocity * (0.5f * Air.Density * DragCoefficient * Area * Speed);
		return Result;
	}

	bool AerodynamicsSimulation::ConvertEulerianToLagrangian()
	{
		if (m_GridDensity.size() > MaxParticleCount) return false;
		m_Particles.clear(); m_Particles.reserve(m_GridDensity.size());
		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const int Index = GridIndex(X, Y, Z);
			AirParticle Particle;
			Particle.Position = GridCellCenter(X, Y, Z);
			Particle.ReferencePosition = Particle.Position;
			Particle.Velocity = SampleGridVelocity(m_GridVelocity, Particle.Position);
			Particle.Mass = m_GridDensity[(size_t)Index] * CellVolume;
			Particle.ReferenceMass = m_Settings.AmbientDensity * CellVolume;
			m_Particles.push_back(Particle);
		}
		InitializeParticleNormalizations(); ComputeParticleDensities(); return true;
	}

	bool AerodynamicsSimulation::ConvertLagrangianToEulerian()
	{
		std::fill(m_GridDensity.begin(), m_GridDensity.end(), 0.0f);
		std::fill(m_GridVelocity.begin(), m_GridVelocity.end(), Vector3::Zero());
		std::vector<float> Mass(m_GridDensity.size(), 0.0f);
		std::vector<Vector3> Momentum(m_GridDensity.size(), Vector3::Zero());
		const float CellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
		for (const AirParticle& Particle : m_Particles)
		{
			const Vector3 G((Particle.Position.x - m_Settings.BoundsMin.x) / m_CellSize.x - 0.5f,
				(Particle.Position.y - m_Settings.BoundsMin.y) / m_CellSize.y - 0.5f,
				(Particle.Position.z - m_Settings.BoundsMin.z) / m_CellSize.z - 0.5f);
			const int X0 = ClampInt((int)std::floor(G.x), 0, m_Settings.Resolution.x - 1);
			const int Y0 = ClampInt((int)std::floor(G.y), 0, m_Settings.Resolution.y - 1);
			const int Z0 = ClampInt((int)std::floor(G.z), 0, m_Settings.Resolution.z - 1);
			const float TX = ClampFloat(G.x - (float)X0, 0.0f, 1.0f);
			const float TY = ClampFloat(G.y - (float)Y0, 0.0f, 1.0f);
			const float TZ = ClampFloat(G.z - (float)Z0, 0.0f, 1.0f);
			for (int DZ = 0; DZ <= 1; ++DZ)
			for (int DY = 0; DY <= 1; ++DY)
			for (int DX = 0; DX <= 1; ++DX)
			{
				const int X = std::min(X0 + DX, m_Settings.Resolution.x - 1);
				const int Y = std::min(Y0 + DY, m_Settings.Resolution.y - 1);
				const int Z = std::min(Z0 + DZ, m_Settings.Resolution.z - 1);
				const float W = (DX ? TX : 1.0f - TX) * (DY ? TY : 1.0f - TY) * (DZ ? TZ : 1.0f - TZ);
				const int Index = GridIndex(X, Y, Z); const float M = Particle.Mass * W;
				Mass[(size_t)Index] += M; Momentum[(size_t)Index] += Particle.Velocity * M;
			}
		}
		for (size_t I = 0; I < Mass.size(); ++I)
		{
			m_GridDensity[I] = Mass[I] / CellVolume;
		}
		for (int Z = 0; Z < m_Settings.Resolution.z; ++Z)
		for (int Y = 0; Y < m_Settings.Resolution.y; ++Y)
		for (int X = 0; X < m_Settings.Resolution.x; ++X)
		{
			const int I = GridIndex(X, Y, Z); Vector3& Face = m_GridVelocity[(size_t)I];
			if (X + 1 < m_Settings.Resolution.x)
			{
				const int N = GridIndex(X + 1, Y, Z);
				const float Weight = Mass[(size_t)I] + Mass[(size_t)N];
				if (Weight > 0.0f) Face.x = (Momentum[(size_t)I].x + Momentum[(size_t)N].x) / Weight;
			}
			if (Y + 1 < m_Settings.Resolution.y)
			{
				const int N = GridIndex(X, Y + 1, Z);
				const float Weight = Mass[(size_t)I] + Mass[(size_t)N];
				if (Weight > 0.0f) Face.y = (Momentum[(size_t)I].y + Momentum[(size_t)N].y) / Weight;
			}
			if (Z + 1 < m_Settings.Resolution.z)
			{
				const int N = GridIndex(X, Y, Z + 1);
				const float Weight = Mass[(size_t)I] + Mass[(size_t)N];
				if (Weight > 0.0f) Face.z = (Momentum[(size_t)I].z + Momentum[(size_t)N].z) / Weight;
			}
		}
		UpdateEulerianPressure(); EnforceEulerianBoundaries(); return true;
	}
}

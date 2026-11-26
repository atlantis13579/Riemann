#include "AerodynamicsSimulation.h"
#include "AerodynamicsBarotropicSolver.h"
#include "AerodynamicsIncompressibleSolver.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		template<class T> const std::vector<T>& EmptyField() { static const std::vector<T> Empty; return Empty; }
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
	}

	AerodynamicsSimulation::AerodynamicsSimulation() = default;
	AerodynamicsSimulation::~AerodynamicsSimulation() = default;
	AerodynamicsSimulation::AerodynamicsSimulation(const AerodynamicsSimulationSettings& Settings) { Initialize(Settings); }

	bool AerodynamicsSimulation::Initialize(const AerodynamicsSimulationSettings& Settings)
	{
		// A failed initialization must not expose a solver from another algorithm.
		const auto Requested = Settings;
		m_Barotropic.reset(); m_Incompressible.reset(); m_Settings = Requested; m_CellSize = Vector3::Zero();
		if ((Settings.Algorithm != AerodynamicsAlgorithm::Barotropic && Settings.Algorithm != AerodynamicsAlgorithm::Incompressible)
			|| (Settings.Backend != FluidBackend::CPU && Settings.Backend != FluidBackend::SIMD
				&& Settings.Backend != FluidBackend::CUDA && Settings.Backend != FluidBackend::ComputeShader && Settings.Backend != FluidBackend::Metal)) return false;
		if (Settings.Algorithm == AerodynamicsAlgorithm::Incompressible)
		{
			if (Settings.Method != FluidSimulationMethod::Eulerian || !std::isfinite(Settings.AmbientDensity) || Settings.AmbientDensity <= 0
				|| !std::isfinite(Settings.AmbientPressure) || Settings.AmbientPressure < 0) return false;
			std::unique_ptr<AerodynamicsIncompressibleSolver> Next(new AerodynamicsIncompressibleSolver());
			if (!Next->Initialize(Requested)) return false;
			m_CellSize = Next->GetCellSize(); m_Incompressible = std::move(Next);
		}
		else
		{
			// Moving solids and tracer sources are not implemented by the barotropic solver.
			if (!Settings.Emitters.empty() || !Settings.Obstacles.empty()) return false;
			std::unique_ptr<AerodynamicsBarotropicSolver> Next(new AerodynamicsBarotropicSolver());
			if (!Next->Initialize(Requested)) return false;
			m_CellSize = Next->GetCellSize(); m_Barotropic = std::move(Next);
		}
		return true;
	}
	bool AerodynamicsSimulation::IsInitialized() const
	{
		return m_Barotropic ? m_Barotropic->IsInitialized() : m_Incompressible && m_Incompressible->IsInitialized();
	}
	void AerodynamicsSimulation::Reset()
	{
		if (m_Barotropic) m_Barotropic->Reset();
		if (m_Incompressible) m_Incompressible->Reset();
	}
	void AerodynamicsSimulation::Step(float Dt)
	{
		if (m_Barotropic) m_Barotropic->Step(Dt);
		if (m_Incompressible) m_Incompressible->Step(Dt);
	}
	bool AerodynamicsSimulation::StepSynchronized(float Dt)
	{
		return m_Incompressible && m_Incompressible->StepSynchronized(Dt);
	}
	float AerodynamicsSimulation::GetStableTimeStep() const
	{
		return m_Incompressible ? m_Incompressible->GetStableTimeStep() : m_Settings.MaxTimeStep;
	}
	bool AerodynamicsSimulation::SetMovingBoundaries(const std::vector<AerodynamicsObstacle>& Obstacles, const std::vector<AerodynamicsSurfaceMesh>& Surfaces)
	{
		return m_Incompressible && m_Incompressible->SetMovingBoundaries(Obstacles, Surfaces);
	}
	bool AerodynamicsSimulation::ComputeSurfaceLoads(std::vector<std::vector<Vector3>>& Forces) const
	{
		Forces.clear(); return m_Incompressible && m_Incompressible->ComputeSurfaceLoads(Forces);
	}
	bool AerodynamicsSimulation::SetMethod(FluidSimulationMethod Method, bool PreserveState)
	{
		if (!IsInitialized()) return false;
		if (m_Incompressible) return Method == FluidSimulationMethod::Eulerian;
		if (!m_Barotropic->SetMethod(Method, PreserveState)) return false;
		m_Settings.Method = Method; return true;
	}
	FluidBackend AerodynamicsSimulation::GetBackend() const
	{
		return m_Barotropic ? m_Barotropic->GetBackend() : m_Incompressible ? m_Incompressible->GetBackend() : FluidBackend::CPU;
	}
	double AerodynamicsSimulation::GetTime() const { return m_Barotropic ? m_Barotropic->GetTime() : m_Incompressible ? m_Incompressible->GetTime() : 0; }
	double AerodynamicsSimulation::GetPendingTime() const { return m_Barotropic ? m_Barotropic->GetPendingTime() : m_Incompressible ? m_Incompressible->GetPendingTime() : 0; }
	bool AerodynamicsSimulation::ApplyImpulse(const Vector3& Position, float Radius, const Vector3& Impulse)
	{
		if (!IsInitialized() || !Finite(Position) || !Finite(Impulse) || !std::isfinite(Radius) || Radius <= 0) return false;
		if (m_Barotropic) return m_Barotropic->ApplyImpulse(Position, Radius, Impulse);
		const float Mass = m_Incompressible->GetImpulseMass(Position, Radius);
		return Mass > 0 && m_Incompressible->ApplyVelocityImpulse(Position, Radius, Impulse / Mass);
	}
	bool AerodynamicsSimulation::ApplyVelocityImpulse(const Vector3& Position, float Radius, const Vector3& Change)
	{
		if (!m_Incompressible || !Finite(Change)) return false;
		return m_Incompressible->ApplyVelocityImpulse(Position, Radius, Change * std::min(1.0f, 12.0f / std::max(1e-5f, Change.Length())));
	}
	void AerodynamicsSimulation::SetDensityBox(const Vector3& Min, const Vector3& Max, float Density)
	{
		if (m_Settings.Algorithm != AerodynamicsAlgorithm::Barotropic) throw std::logic_error("Mass density edits require barotropic gas");
		if (m_Barotropic) m_Barotropic->SetDensityBox(Min, Max, Density);
	}
	void AerodynamicsSimulation::SetDensitySphere(const Vector3& Center, float Radius, float Density)
	{
		if (m_Settings.Algorithm != AerodynamicsAlgorithm::Barotropic) throw std::logic_error("Mass density edits require barotropic gas");
		if (m_Barotropic) m_Barotropic->SetDensitySphere(Center, Radius, Density);
	}
	void AerodynamicsSimulation::SetVelocityBox(const Vector3& Min, const Vector3& Max, const Vector3& Velocity)
	{
		if (m_Settings.Algorithm != AerodynamicsAlgorithm::Barotropic) throw std::logic_error("Velocity box edits require barotropic gas");
		if (m_Barotropic) m_Barotropic->SetVelocityBox(Min, Max, Velocity);
	}
	AerodynamicSample AerodynamicsSimulation::Sample(const Vector3& Position) const
	{
		return m_Barotropic ? m_Barotropic->Sample(Position) : m_Incompressible ? m_Incompressible->Sample(Position) : AerodynamicSample();
	}
	const std::vector<float>& AerodynamicsSimulation::GetGridDensity() const { return m_Barotropic ? m_Barotropic->GetGridDensity() : m_Incompressible ? m_Incompressible->GetGridDensity() : EmptyField<float>(); }
	const std::vector<float>& AerodynamicsSimulation::GetGridPressure() const { return m_Barotropic ? m_Barotropic->GetGridPressure() : m_Incompressible ? m_Incompressible->GetGridPressure() : EmptyField<float>(); }
	const std::vector<Vector3>& AerodynamicsSimulation::GetGridVelocity() const { return m_Barotropic ? m_Barotropic->GetGridVelocity() : m_Incompressible ? m_Incompressible->GetVelocity() : EmptyField<Vector3>(); }
	const std::vector<AirParticle>& AerodynamicsSimulation::GetParticles() const { return m_Barotropic ? m_Barotropic->GetParticles() : EmptyField<AirParticle>(); }
	const std::vector<float>& AerodynamicsSimulation::GetTracerDensity() const { return m_Incompressible ? m_Incompressible->GetTracerDensity() : EmptyField<float>(); }
	const std::vector<float>& AerodynamicsSimulation::GetTemperature() const { return m_Incompressible ? m_Incompressible->GetTemperature() : EmptyField<float>(); }
	const std::vector<unsigned char>& AerodynamicsSimulation::GetSolidCells() const { return m_Incompressible ? m_Incompressible->GetSolidCells() : EmptyField<unsigned char>(); }
	const std::vector<Vector3>& AerodynamicsSimulation::GetBoundaryVelocity() const { return m_Incompressible ? m_Incompressible->GetBoundaryVelocity() : EmptyField<Vector3>(); }
	bool AerodynamicsSimulation::SetRigidObstacles(const std::vector<AerodynamicsObstacle>& Obstacles)
	{
		return m_Incompressible ? m_Incompressible->SetRigidObstacles(Obstacles) : IsInitialized() && Obstacles.empty();
	}
	bool AerodynamicsSimulation::ComputeRigidLoads(float AirDensity, std::vector<AerodynamicsRigidLoad>& Loads) const
	{
		Loads.clear(); return m_Incompressible && m_Incompressible->ComputeRigidLoads(AirDensity, Loads);
	}
	float AerodynamicsSimulation::GetDivergenceRMS() const
	{
		if (!IsInitialized()) return 0;
		if (m_Incompressible) return m_Incompressible->GetDivergenceRMS();
		if (GetMethod() != FluidSimulationMethod::Eulerian) throw std::logic_error("Grid divergence requires Eulerian discretization");
		const auto& V = GetGridVelocity(); const auto N = m_Settings.Resolution; double Sum = 0;
		for (int Z = 0; Z < N.z; ++Z) for (int Y = 0; Y < N.y; ++Y) for (int X = 0; X < N.x; ++X)
		{
			const int I = X + N.x * (Y + N.y * Z);
			const float D = (V[I].x - (X ? V[I - 1].x : 0)) / m_CellSize.x
				+ (V[I].y - (Y ? V[I - N.x].y : 0)) / m_CellSize.y
				+ (V[I].z - (Z ? V[I - N.x * N.y].z : 0)) / m_CellSize.z;
			Sum += D * D;
		}
		return (float)std::sqrt(Sum / V.size());
	}
	bool AerodynamicsSimulation::BuildDensityField(std::vector<float>* Field) const
	{
		if (!Field || !IsInitialized()) return false;
		if (m_Barotropic) return m_Barotropic->BuildDensityField(Field);
		*Field = GetGridDensity(); return true;
	}
	float AerodynamicsSimulation::GetTotalMass() const
	{
		if (!IsInitialized()) return 0;
		if (m_Barotropic) return m_Barotropic->GetTotalMass();
		double Mass = 0; for (float Density : GetGridDensity()) Mass += Density;
		return (float)(Mass * m_CellSize.x * m_CellSize.y * m_CellSize.z);
	}
	Vector3 AerodynamicsSimulation::GetTotalMomentum() const
	{
		if (!IsInitialized()) return Vector3::Zero();
		if (m_Barotropic) return m_Barotropic->GetTotalMomentum();
		const auto& V = GetGridVelocity(); const auto& Solid = GetSolidCells(); const auto N = m_Settings.Resolution;
		Vector3 Momentum = Vector3::Zero();
		for (int Z = 0; Z < N.z; ++Z) for (int Y = 0; Y < N.y; ++Y) for (int X = 0; X < N.x; ++X)
		{
			const int I = X + N.x * (Y + N.y * Z); if (Solid[I]) continue;
			Momentum += Vector3(V[I].x + (X ? V[I - 1].x : 0), V[I].y + (Y ? V[I - N.x].y : 0),
				V[I].z + (Z ? V[I - N.x * N.y].z : 0)) * 0.5f;
		}
		return Momentum * (m_Settings.AmbientDensity * m_CellSize.x * m_CellSize.y * m_CellSize.z);
	}
	AerodynamicForceResult AerodynamicsSimulation::ComputeSurfaceForce(const Vector3& CenterOfMass,
		const std::vector<AerodynamicSurfaceSample>& SurfaceSamples, float DragCoefficient) const
	{
		AerodynamicForceResult Result;
		if (!IsInitialized() || !Finite(CenterOfMass) || !std::isfinite(DragCoefficient)
			|| DragCoefficient < 0.0f) return Result;
		for (const AerodynamicSurfaceSample& Surface : SurfaceSamples)
		{
			if (!Finite(Surface.Position) || !Finite(Surface.Normal)
				|| !Finite(Surface.Velocity) || !std::isfinite(Surface.Area) || Surface.Area <= 0.0f) continue;
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
		if (!IsInitialized() || !Finite(Center) || !Finite(LinearVelocity)
			|| !std::isfinite(Radius) || !std::isfinite(DragCoefficient)
			|| Radius <= 0.0f || DragCoefficient < 0.0f) return Result;
		const AerodynamicSample Air = Sample(Center);
		const float Volume = (4.0f / 3.0f) * 3.14159265358979323846f * Radius * Radius * Radius;
		const float Area = 3.14159265358979323846f * Radius * Radius;
		const Vector3 RelativeVelocity = Air.Velocity - LinearVelocity;
		Result.Force = -Air.PressureGradient * Volume;
		const float Speed = RelativeVelocity.Length();
		if (Speed > 1.0e-6f)
			Result.Force += RelativeVelocity * (0.5f * Air.Density * DragCoefficient * Area * Speed);
		return Result;
	}
	AerodynamicsSimulationSettings::AerodynamicsSimulationSettings(AerodynamicsAlgorithm Algorithm)
		: Algorithm(Algorithm)
		, Method(FluidSimulationMethod::Eulerian)
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
		if (Algorithm == AerodynamicsAlgorithm::Incompressible)
		{
			AllowBackendFallback = false;
			BoundsMin = Vector3(-2.4f, 0, -2.4f); BoundsMax = Vector3(2.4f, 8, 2.4f);
			Resolution = Vector3i(48, 80, 48); MaxTimeStep = 1.0f / 60; MaxSubsteps = 8;
		}
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

}

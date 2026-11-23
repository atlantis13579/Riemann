#include "Test.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Renderer/AerodynamicsVolume.h"
#include "../Viewer/SceneWorld.h"
#include <cmath>
#include <stdexcept>

void TestAerodynamicsInterface()
{
	using namespace Riemann;
	AerodynamicsSimulation Simulation;
	DensityVolumeDesc Volume;
	AerodynamicsSimulationSettings Selected;
	Selected.Algorithm = AerodynamicsAlgorithm::Incompressible;
	EXPECT(Simulation.Initialize(Selected));
	for (auto Algorithm : { AerodynamicsAlgorithm::Barotropic, AerodynamicsAlgorithm::Incompressible })
	{
		AerodynamicsSimulationSettings Settings(Algorithm);
		Settings.BoundsMin = Vector3::Zero(); Settings.BoundsMax = Vector3::One(); Settings.Resolution = Vector3i(8, 8, 8);
		Settings.ParticleSpacing = 0.125f; Settings.SmoothingRadius = 0.25f; Settings.MaxTimeStep = 0.005f; Settings.MaxSubsteps = 32;
		EXPECT(Simulation.Initialize(Settings));
		EXPECT(Simulation.GetAlgorithm() == Algorithm && Simulation.GetBackend() == FluidBackend::CPU);
		EXPECT_NEAR(Simulation.GetTotalMass(), Settings.AmbientDensity, 1e-5f);
		const auto Air = Simulation.Sample(Vector3(0.5f));
		EXPECT_NEAR(Air.Density, Settings.AmbientDensity, 1e-6f);
		EXPECT_NEAR(Air.Pressure, Settings.AmbientPressure, 0.01f);
		EXPECT(Air.TracerDensity == 0 && Air.Temperature == 0 && Air.Velocity.IsZero());
		EXPECT(Simulation.ApplyImpulse(Vector3(0.5f), 0.2f, Vector3(0.001f, 0, 0)));
		EXPECT(Simulation.GetTotalMomentum().x > 0);
		EXPECT_NEAR(Simulation.GetTotalMomentum().x, 0.001f, 1e-6f);
		Simulation.Step(0.01f);
		EXPECT_NEAR(Simulation.GetTime() + Simulation.GetPendingTime(), 0.01, 1e-7);
		EXPECT(BuildAerodynamicsDensityVolume(Simulation, &Volume));
		EXPECT(Volume.Tracer == (Algorithm == AerodynamicsAlgorithm::Incompressible));
		std::vector<float> Density;
		EXPECT(Simulation.BuildDensityField(&Density) && Density.size() == 512);
		if (Algorithm == AerodynamicsAlgorithm::Incompressible)
		{
			EXPECT(Simulation.HasTracerField() && Simulation.SupportsMovingObstacles());
			for (float Rho : Density) EXPECT_NEAR(Rho, Settings.AmbientDensity, 1e-6f);
			EXPECT(!Simulation.SetMethod(FluidSimulationMethod::Lagrangian));
			EXPECT(Simulation.GetMethod() == FluidSimulationMethod::Eulerian);
			bool Rejected = false;
			try { Simulation.SetDensityBox(Vector3::Zero(), Vector3::One(), 2); } catch (const std::logic_error&) { Rejected = true; }
			EXPECT(Rejected);
			AerodynamicsObstacle Obstacle; Obstacle.Center = Vector3(0.5f); Obstacle.Radius = 0.2f;
			EXPECT(Simulation.SetRigidObstacles({ Obstacle }));
			EXPECT(Simulation.GetTotalMass() < Settings.AmbientDensity);
		}
		else
		{
			EXPECT(!Simulation.HasTracerField() && !Simulation.SupportsMovingObstacles());
			EXPECT(!Simulation.SetRigidObstacles({ AerodynamicsObstacle() }));
			EXPECT(Simulation.GetTracerDensity().empty());
		}
		Simulation.Reset();
		EXPECT(Simulation.GetTime() == 0 && Simulation.GetPendingTime() == 0);
		EXPECT(Simulation.Initialize(Simulation.GetSettings()));
		EXPECT(Simulation.GetTime() == 0);
		Settings.Backend = (FluidBackend)255;
		EXPECT(!Simulation.Initialize(Settings) && !Simulation.IsInitialized());
		EXPECT(Simulation.GetGridVelocity().empty() && Simulation.GetTracerDensity().empty());
	}
	AerodynamicsSimulationSettings Projection(AerodynamicsAlgorithm::Incompressible);
	Projection.Resolution = Vector3i(8, 8, 8); Projection.Backend = FluidBackend::SIMD; Projection.AllowBackendFallback = false;
	EXPECT(!Simulation.Initialize(Projection));
	Projection.AllowBackendFallback = true;
	EXPECT(Simulation.Initialize(Projection) && Simulation.UsedBackendFallback() && Simulation.GetBackend() == FluidBackend::CPU);
	Projection.Emitters.push_back(AerodynamicsEmitter()); Projection.Backend = FluidBackend::CPU;
	EXPECT(Simulation.Initialize(Projection)); Simulation.Step(0.01f);
	EXPECT(BuildAerodynamicsDensityVolume(Simulation, &Volume) && Volume.Tracer);
	AerodynamicsSimulationSettings Barotropic;
	EXPECT(Simulation.Initialize(Barotropic));
	EXPECT(BuildAerodynamicsDensityVolume(Simulation, &Volume) && !Volume.Tracer);
	EXPECT(Simulation.GetSolidCells().empty() && Simulation.GetTemperature().empty());
	EXPECT(Simulation.GetTime() == 0 && Simulation.GetPendingTime() == 0);
	Barotropic.Algorithm = (AerodynamicsAlgorithm)255;
	EXPECT(!Simulation.Initialize(Barotropic));

	SceneWorld Scene; std::string Error;
	EXPECT(Scene.LoadFromText(R"({"name":"Legacy gas","fieldSimulation":{"type":"smoke3d","backend":"cpu","smokeWeight":0.7,"smokeExtinction":4.0,"smokeColor":[0.1,0.2,0.3]},"objects":[]})", "legacy-gas", &Error));
	const auto& Desc = Scene.GetFieldSimulation();
	EXPECT(Desc.Type == "aerodynamics3d" && Desc.Aerodynamics.Algorithm == AerodynamicsAlgorithm::Incompressible);
	EXPECT_NEAR(Desc.Aerodynamics.TracerWeight, 0.7f, 1e-6f);
	EXPECT_NEAR(Desc.TracerExtinction, 4, 1e-6f);
	EXPECT_NEAR(Desc.TracerColor.y, 0.2f, 1e-6f);
}

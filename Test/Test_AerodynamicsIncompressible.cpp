#include "Test.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include <cmath>
#include <stdexcept>
#include "../Src/Deformation/Cloth.h"
#include "../Src/Fluid/FluidBackend.h"
#include <chrono>

void TestAerodynamicsIncompressible()
{
	using namespace Riemann;
	AerodynamicsSimulationSettings Settings(AerodynamicsAlgorithm::Incompressible);
	Settings.BoundsMin = Vector3(-1, 0, -1);
	Settings.BoundsMax = Vector3(1, 3, 1.5f);
	Settings.Resolution = Vector3i(16, 24, 18);
	Settings.PressureIterations = 100;
	AerodynamicsSimulation Simulation;
	EXPECT(Simulation.Initialize(Settings));
	Simulation.Step(0.1f);
	EXPECT(Simulation.GetDivergenceRMS() == 0);
	EXPECT(std::accumulate(Simulation.GetTracerDensity().begin(), Simulation.GetTracerDensity().end(), 0.0f) == 0);
	EXPECT(Simulation.ApplyVelocityImpulse(Vector3(0, 1, 0), 0.4f, Vector3(1, 2, -0.5f)));
	const float Before = Simulation.GetDivergenceRMS();
	Simulation.Step(1.0f / 60);
	printf("Simulation anisotropic projection divergence: %.6f -> %.6f\n", Before, Simulation.GetDivergenceRMS());
	EXPECT(Simulation.GetDivergenceRMS() < Before * 0.02f);
	Simulation.Reset();
	EXPECT(Simulation.GetTime() == 0 && Simulation.GetPendingTime() == 0);
	for (const Vector3& V : Simulation.GetGridVelocity()) EXPECT(V.SquareLength() == 0);

	AerodynamicsEmitter E;
	E.Position = Vector3(0, 0.4f, 0); E.Radius = 0.3f;
	Settings.Emitters.push_back(E);
	AerodynamicsObstacle O; O.Center = Vector3(0.1f, 1.2f, 0); O.Radius = 0.28f;
	Settings.Obstacles.push_back(O);
	EXPECT(Simulation.Initialize(Settings));
	for (int Frame = 0; Frame < 240; ++Frame) Simulation.Step(1.0f / 60);
	float UpperDensity = 0;
	for (size_t I = 0; I < Simulation.GetTracerDensity().size(); ++I)
	{
		EXPECT(std::isfinite(Simulation.GetTracerDensity()[I]) && Simulation.GetTracerDensity()[I] >= 0 && Simulation.GetTracerDensity()[I] <= E.Density + 1e-5f);
		EXPECT(std::isfinite(Simulation.GetTemperature()[I]) && Simulation.GetTemperature()[I] >= 0 && Simulation.GetTemperature()[I] <= E.Temperature + 1e-5f);
		const Vector3& V = Simulation.GetGridVelocity()[I];
		EXPECT(std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z));
		if (Simulation.GetSolidCells()[I]) EXPECT(Simulation.GetTracerDensity()[I] == 0 && V.SquareLength() == 0);
		if ((I / 16 % 24) > 12) UpperDensity += Simulation.GetTracerDensity()[I];
	}
	printf("Simulation above deflector: %.6f, divergence: %.6f\n", UpperDensity, Simulation.GetDivergenceRMS());
	EXPECT(UpperDensity > 0.1f);
	EXPECT(Simulation.GetDivergenceRMS() < 0.05f);
	Simulation.Reset();
	for (float D : Simulation.GetTracerDensity()) EXPECT(D == 0);

	Settings.MaxSubsteps = 1;
	EXPECT(Simulation.Initialize(Settings));
	Simulation.Step(0.1f);
	EXPECT_NEAR(Simulation.GetTime() + Simulation.GetPendingTime(), 0.1, 1e-6);
	EXPECT(Simulation.GetPendingTime() > 0.08);
	Simulation.Step(std::numeric_limits<float>::quiet_NaN());
	EXPECT(std::isfinite(Simulation.GetPendingTime()));
	Settings.Resolution.x = -1;
	EXPECT(!Simulation.Initialize(Settings));
	Settings.Resolution.x = 16;
	Settings.Buoyancy = std::numeric_limits<float>::infinity();
	EXPECT(!Simulation.Initialize(Settings));

}

void TestAerodynamicsBackends()
{
	using namespace Riemann;
	AerodynamicsSimulationSettings Settings(AerodynamicsAlgorithm::Incompressible);
	Settings.BoundsMin = Vector3(-1, 0, -1);
	Settings.BoundsMax = Vector3(1, 3, 1.5f);
	Settings.Resolution = Vector3i(16, 24, 18);
	Settings.PressureIterations = 120;
	AerodynamicsEmitter Source; Source.Position = Vector3(0, 0.4f, 0); Source.Radius = 0.3f;
	Settings.Emitters.push_back(Source);
	AerodynamicsObstacle Obstacle; Obstacle.Center = Vector3(0.1f, 1.2f, 0); Obstacle.Radius = 0.28f;
	Settings.Obstacles.push_back(Obstacle);
	const FluidBackend Backends[] = { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader, FluidBackend::Metal };
	const char* Names[] = { "CPU", "CUDA", "ComputeShader", "Metal" };
	std::vector<float> Reference;
	for (int B = 0; B < 4; ++B)
	{
		Settings.Backend = Backends[B];
		AerodynamicsSimulation Simulation;
		if (!Simulation.Initialize(Settings))
		{
			printf("Simulation %s unavailable (no fallback used).\n", Names[B]);
#if defined(RIEMANN_ENABLE_CUDA)
			if (B == 1) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (B == 2) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_METAL)
            if (B == 3) EXPECT(false);
#endif
			continue;
		}
		EXPECT(Simulation.GetBackend() == Backends[B]);
		Simulation.ApplyVelocityImpulse(Vector3(-0.2f, 1.8f, 0), 0.4f, Vector3(1, 2, -0.5f));
		const float Before = Simulation.GetDivergenceRMS();
		Simulation.Step(1.0f / 60);
		const float After = Simulation.GetDivergenceRMS();
		EXPECT(After < Before * 0.025f);
		const auto Air = Simulation.Sample(Vector3(-0.2f, 1.8f, 0));
		EXPECT_NEAR(Air.Density, Settings.AmbientDensity, 1e-6f);
		EXPECT(std::isfinite(Air.Pressure) && std::isfinite(Air.PressureGradient.SquareLength()));
		EXPECT(std::isfinite(Air.Velocity.SquareLength()) && Air.TracerDensity >= 0 && Air.Temperature >= 0);
		EXPECT(Simulation.GetGridPressure().size() == Simulation.GetGridDensity().size());
		Simulation.Reset();
		for (float P : Simulation.GetGridPressure()) EXPECT_NEAR(P, Settings.AmbientPressure, 1e-5f);
		for (int Frame = 0; Frame < 60; ++Frame) Simulation.Step(1.0f / 60);
		EXPECT(Simulation.IsInitialized());
		const auto& Density = Simulation.GetTracerDensity();
		const auto& Temperature = Simulation.GetTemperature();
		const auto& Velocity = Simulation.GetGridVelocity();
		double Mass = 0, Error = 0;
		for (size_t I = 0; I < Density.size(); ++I)
		{
			EXPECT(std::isfinite(Density[I]) && Density[I] >= 0 && Density[I] <= Source.Density + 1e-5f);
			EXPECT(std::isfinite(Temperature[I]) && Temperature[I] >= 0 && Temperature[I] <= Source.Temperature + 1e-5f);
			EXPECT(std::isfinite(Velocity[I].x) && std::isfinite(Velocity[I].y) && std::isfinite(Velocity[I].z));
			if (Simulation.GetSolidCells()[I]) EXPECT(Density[I] == 0 && Velocity[I].SquareLength() == 0);
			Mass += Density[I];
			if (B != 0) Error += std::fabs(Density[I] - Reference[I]);
		}
		if (B == 0) Reference = Density;
		EXPECT(Error < Mass * 0.04);
		printf("Simulation %s: divergence %.6f -> %.6f; mass %.5f; CPU relative L1 %.5f\n", Names[B], Before, After, Mass, Error / std::max(Mass, 1e-8));
		Simulation.Reset();
		for (float D : Simulation.GetTracerDensity()) EXPECT(D == 0);
		for (const Vector3& V : Simulation.GetGridVelocity()) EXPECT(V.SquareLength() == 0);
	}
}

void TestAerodynamicsMovingObstacles()
{
	using namespace Riemann;
	for (FluidBackend Backend : { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader, FluidBackend::Metal })
	{
		AerodynamicsSimulationSettings Settings(AerodynamicsAlgorithm::Incompressible);
		Settings.Backend = Backend;
		Settings.BoundsMin = Vector3(-2, -2, -2);
		Settings.BoundsMax = Vector3(2, 2, 2);
		Settings.Resolution = Vector3i(24, 24, 24);
		Settings.PressureIterations = 160;
		Settings.Vorticity = 0;
		AerodynamicsSimulation Simulation;
		if (!Simulation.Initialize(Settings))
		{
#if defined(RIEMANN_ENABLE_CUDA)
			if (Backend == FluidBackend::CUDA) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (Backend == FluidBackend::ComputeShader) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_METAL)
            if (Backend == FluidBackend::Metal) EXPECT(false);
#endif
			continue;
		}
		AerodynamicsObstacle Sphere;
		Sphere.Center = Vector3(-0.7f, 0, 0); Sphere.Radius = 0.4f;
		Sphere.LinearVelocity = Vector3(0.8f, 0.2f, 0);
		AerodynamicsObstacle Box;
		Box.Type = AerodynamicsObstacle::Shape::Box;
		Box.Center = Vector3(0.7f, 0, 0); Box.HalfExtent = Vector3(0.3f, 0.6f, 0.25f);
		Box.Rotation = Quaternion(0, 0, std::sin(0.3f), std::cos(0.3f));
		Box.AngularVelocity = Vector3(0, 0, 1.2f);
		EXPECT(Simulation.SetRigidObstacles({ Sphere, Box }));
		const auto OriginalMask = Simulation.GetSolidCells();
		const float Before = Simulation.GetDivergenceRMS();
		EXPECT(Before > 0.1f);
		Simulation.Step(1.0f / 60);
		const float After = Simulation.GetDivergenceRMS();
		EXPECT(After < Before * 0.03f);
		float FluidEnergy = 0;
		int CheckedFaces = 0;
		const auto& Velocity = Simulation.GetGridVelocity();
		for (int I = 0; I < (int)Velocity.size(); ++I)
		{
			const int G[] = { I % 24, I / 24 % 24, I / (24 * 24) };
			const int Stride[] = { 1, 24, 24 * 24 };
			EXPECT(std::isfinite(Velocity[I].SquareLength()));
			if (!OriginalMask[I]) FluidEnergy += Velocity[I].SquareLength();
			else EXPECT(Simulation.GetTracerDensity()[I] == 0);
			for (int A = 0; A < 3; ++A)
			{
				if (G[A] == 23 || (!OriginalMask[I] && !OriginalMask[I + Stride[A]])) continue;
				Vector3 P(-2 + (G[0] + 0.5f) / 6, -2 + (G[1] + 0.5f) / 6, -2 + (G[2] + 0.5f) / 6);
				P[A] += 1.0f / 12;
				const AerodynamicsObstacle& O = P.x < 0 ? Sphere : Box;
				const float Expected = (O.LinearVelocity + O.AngularVelocity.Cross(P - O.Center))[A];
				EXPECT_NEAR(Velocity[I][A], Expected, 1e-5f);
				++CheckedFaces;
			}
		}
		EXPECT(FluidEnergy > 1 && CheckedFaces > 50);
		Sphere.Center.y += 1; Box.Center.y -= 1;
		EXPECT(Simulation.SetRigidObstacles({ Sphere, Box }));
		int Released = 0, Occupied = 0;
		for (size_t I = 0; I < OriginalMask.size(); ++I)
		{
			Released += OriginalMask[I] && !Simulation.GetSolidCells()[I];
			Occupied += !OriginalMask[I] && Simulation.GetSolidCells()[I];
		}
		EXPECT(Released > 50 && Occupied > 50);
		const auto MovedMask = Simulation.GetSolidCells();
		Sphere.LinearVelocity.x = std::numeric_limits<float>::quiet_NaN();
		EXPECT(!Simulation.SetRigidObstacles({ Sphere }));
		EXPECT(MovedMask == Simulation.GetSolidCells());
		EXPECT(Simulation.SetRigidObstacles({}));
		for (unsigned char Solid : Simulation.GetSolidCells()) EXPECT(Solid == 0);
		printf("Moving smoke walls backend %d: divergence %.6f -> %.6f, fluid energy %.4f, released %d cells\n",
			(int)Backend, Before, After, FluidEnergy, Released);
	}
}

void TestAerodynamicsBoundaryCoupling()
{
	using namespace Riemann;
	Vector3 ReferenceForce;
	for (FluidBackend Backend : { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader, FluidBackend::Metal })
	{
		AerodynamicsSimulationSettings Settings(AerodynamicsAlgorithm::Incompressible);
		Settings.Backend = Backend;
		Settings.BoundsMin = Vector3(-2, 0, -2); Settings.BoundsMax = Vector3(2, 4, 2);
		Settings.Resolution = Vector3i(24, 24, 24); Settings.PressureIterations = 120;
		Settings.Vorticity = 0;
		AerodynamicsSimulation Simulation;
		if (!Simulation.Initialize(Settings))
		{
#if defined(RIEMANN_ENABLE_CUDA)
			if (Backend == FluidBackend::CUDA) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (Backend == FluidBackend::ComputeShader) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_METAL)
            if (Backend == FluidBackend::Metal) EXPECT(false);
#endif
			continue;
		}
		PhysicsWorldParam WorldSettings; WorldSettings.gravityAcc = Vector3::Zero();
		PhysicsWorld World(WorldSettings);
		RigidBodyParam BodySettings; BodySettings.rigidType = RigidType::Dynamic;
		BodySettings.sleepThreshold = 0; BodySettings.linearDamping = BodySettings.angularDamping = 1;
		Geometry* Sphere = GeometryFactory::CreateSphere(Vector3(0, 1.7f, 0), 0.5f);
		RigidBodyDynamic* Body = World.CreateRigidBody(Sphere, BodySettings)->CastDynamic();
		Body->InvMass = 0.2f;
		AerodynamicsRigidCoupling* Coupling = new AerodynamicsRigidCoupling(&Simulation);
		World.AddForceField(Coupling);
		Body->V = Vector3(1, 0, 0);
		EXPECT(Coupling->Synchronize({ Sphere }));
		Simulation.Step(1.0f / 60);
		Coupling->Update(1.0f / 60);
		AerodynamicsRigidCouplingResult Load;
		EXPECT(Coupling->Evaluate(Body, &Load));
		EXPECT(Load.Force.x < -0.1f);
		if (Backend == FluidBackend::CPU) ReferenceForce = Load.Force;
		else EXPECT((Load.Force - ReferenceForce).Length() < ReferenceForce.Length() * 0.005f);
		printf("Simulation pressure reaction backend %d: (%.4f, %.4f, %.4f) N\n", (int)Backend, Load.Force.x, Load.Force.y, Load.Force.z);
		// An off-center, rotating box must transfer angular momentum to the air.
		Geometry* Box = GeometryFactory::CreateOBB(Vector3(1.1f, 2, 0), Vector3(0.25f, 0.6f, 0.3f));
		RigidBodyDynamic* Paddle = World.CreateRigidBody(Box, BodySettings)->CastDynamic();
		Paddle->W = Vector3(0, 0, 1);
		Body->V = Vector3::Zero();
		Simulation.Reset();
		EXPECT(Coupling->Synchronize({ Sphere, Box }));
		Simulation.Step(1.0f / 60); Coupling->Update(1.0f / 60);
		EXPECT(Coupling->Evaluate(Paddle, &Load));
		EXPECT(Load.Torque.z < -0.01f);
		// A plume drives a free body, whose new pose becomes the next smoke boundary.
		World.RemoveRigidBody(Paddle);
		AerodynamicsEmitter E; E.Position = Vector3(0, 0.5f, 0); E.Radius = 0.35f; E.Velocity = Vector3(0, 3, 0);
		Settings.Emitters.push_back(E);
		EXPECT(Simulation.Initialize(Settings));
		Coupling->SetSimulation(&Simulation);
		Body->V = Body->W = Vector3::Zero();
		const Vector3 Start = Body->X;
		EXPECT(Coupling->Synchronize({ Sphere }));
		for (int Frame = 0; Frame < 150; ++Frame)
		{
			World.Simulate(1.0f / 60);
			EXPECT(Coupling->Synchronize({ Sphere }));
			Simulation.Step(1.0f / 60);
			EXPECT(Simulation.IsInitialized() && std::isfinite(Body->X.SquareLength()));
		}
		EXPECT(Body->X.y > Start.y + 0.01f);
		printf("Coupled plume backend %d: body lift %.5f m, speed %.5f m/s\n", (int)Backend, Body->X.y - Start.y, Body->V.Length());
		Simulation.Reset(); Coupling->Update(1.0f / 60);
		EXPECT(Coupling->Evaluate(Body, &Load) && Load.Force.IsZero() && Load.Torque.IsZero());
		EXPECT(World.RemoveForceField(Coupling));
	}
}

namespace Test_AerodynamicsInterface_Cases
{
void TestAerodynamicsInterface()
{
	using namespace Riemann;
	AerodynamicsSimulation Simulation;
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
	AerodynamicsSimulationSettings Barotropic;
	EXPECT(Simulation.Initialize(Barotropic));
	EXPECT(Simulation.GetSolidCells().empty() && Simulation.GetTemperature().empty());
	EXPECT(Simulation.GetTime() == 0 && Simulation.GetPendingTime() == 0);
	Barotropic.Algorithm = (AerodynamicsAlgorithm)255;
	EXPECT(!Simulation.Initialize(Barotropic));

}
}

void TestAerodynamicsInterface() { Test_AerodynamicsInterface_Cases::TestAerodynamicsInterface(); }

namespace Test_WorldAerodynamics_Cases
{
namespace
{
	using namespace Riemann;
	AerodynamicsSimulationSettings AirSettings(FluidBackend Backend)
	{
		AerodynamicsSimulationSettings S(AerodynamicsAlgorithm::Incompressible);
		S.Backend = Backend; S.AllowBackendFallback = false; S.BoundsMin = Vector3(-1); S.BoundsMax = Vector3(1);
		S.Resolution = Vector3i(20, 20, 20); S.PressureIterations = 60; S.MaxTimeStep = 1.0f / 120;
		S.Buoyancy = S.TracerWeight = S.Vorticity = 0; S.Emitters.clear(); return S;
	}
	AerodynamicsSurfaceMesh Sheet()
	{
		AerodynamicsSurfaceMesh S;
		S.Positions = { Vector3(0, -0.7f, -0.7f), Vector3(0, 0.7f, -0.7f), Vector3(0, -0.7f, 0.7f), Vector3(0, 0.7f, 0.7f) };
		S.Velocities.assign(4, Vector3::UnitX()); S.Triangles = { Index3(0, 1, 2), Index3(2, 1, 3) }; S.Thickness = 0.01f; return S;
	}
	void MovingSurface(FluidBackend Backend)
	{
		auto Settings = AirSettings(Backend); AerodynamicsSimulation Air(Settings);
		if (!Air.IsInitialized()) { printf("World aerodynamics backend %d unavailable\n", (int)Backend); return; }
		auto S = Sheet(); EXPECT(Air.SetMovingBoundaries({}, { S }));
		EXPECT(std::count(Air.GetSolidCells().begin(), Air.GetSolidCells().end(), 1) > 100);
		EXPECT(Air.StepSynchronized(1.0f / 60));
		std::vector<std::vector<Vector3>> Loads; EXPECT(Air.ComputeSurfaceLoads(Loads)); EXPECT(Loads.size() == 1 && Loads[0].size() == 4);
		Vector3 Force(0); for (const auto& F : Loads[0]) Force += F;
		EXPECT(Force.x < -0.1f); EXPECT(Air.GetTotalMomentum().x > 0.01f);
		EXPECT_NEAR(Air.GetTime(), 1.0 / 60, 1e-7); EXPECT(Air.GetPendingTime() <= 1e-7);
		printf("Moving cloth boundary backend %d: force=(%g,%g,%g), air momentum=%g\n", (int)Backend, Force.x, Force.y, Force.z, Air.GetTotalMomentum().x);
		const auto Original = Air.GetSolidCells(); auto Bad = S; Bad.Triangles[0].a = 100;
		EXPECT(!Air.SetMovingBoundaries({}, { Bad })); EXPECT(Air.GetSolidCells() == Original);
		EXPECT(Air.SetMovingBoundaries({}, {})); EXPECT(std::count(Air.GetSolidCells().begin(), Air.GetSolidCells().end(), 1) == 0);
		Air.Reset(); EXPECT(Air.ComputeSurfaceLoads(Loads)); EXPECT(Loads.empty());
	}
	void CoupledWorld(FluidBackend Backend, bool RigidAir = true, bool Contacts = true)
	{
		auto Settings = AirSettings(Backend);
		AerodynamicsEmitter E; E.Position = Vector3(-0.65f, 0, 0); E.Velocity = Vector3(2, 0, 0); E.Radius = 0.3f; E.Temperature = 0;
		Settings.Emitters = { E };
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; P.gravityAcc = Vector3::Zero(); PhysicsWorld W(P);
		auto* Air = W.CreateAerodynamics(Settings, AerodynamicsRigidCouplingSettings(), RigidAir, true); if (!Air) return;
		ClothSettings C; C.Model = ClothModel::TriangleStrain; C.Gravity = Vector3::Zero(); C.Damping = 0.98f;
		C.Collision.RigidCoupling = Contacts; C.Thickness = 0.01f; C.Substeps = 2; C.SolverIterations = 16;
		C.Rows = C.Columns = 9; C.Width = C.Height = 1.2f; C.SurfaceDensity = 0.3f; C.ParticleInverseMass = 100;
		auto* Cloth = W.CreateCloth(C, Vector3::Zero(), Vector3::UnitZ(), Vector3::UnitY());
		for (int I = 0; I < C.Columns; ++I) Cloth->PinParticle(0, I, Cloth->GetParticle(0, I).Position);
		RigidBodyParam R; R.rigidType = RigidType::Dynamic; R.linearDamping = R.angularDamping = 0.99f;
		auto* Ball = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0.18f, 0, 0), 0.15f), R);
		const Vector3 Initial = Ball->X;
		size_t ContactCount = 0;
		for (int I = 0; I < 240; ++I)
		{
			W.Simulate(1.0f / 120);
			ContactCount += Cloth->GetCollisionStats().RigidContacts;
			float Speed = 0; for (int J = 0; J < Cloth->GetParticleCount(); ++J) Speed = std::max(Speed, Cloth->GetParticleVelocity(J).Length());
			if (Speed > 30) { EXPECT(false); break; }
		}
		float Displacement = 0; for (int I = 0; I < Cloth->GetParticleCount(); ++I) Displacement = std::max(Displacement, Cloth->GetParticle(I).Position.x);
		EXPECT(Displacement > 0.01f); EXPECT(Ball->X.x > Initial.x + 0.001f);
		EXPECT(Contacts ? ContactCount > 0 : ContactCount == 0);
		EXPECT_NEAR(Air->GetTime(), 2.0, 2e-6); EXPECT(Air->GetPendingTime() <= 1e-7);
		EXPECT(std::isfinite(Ball->V.Length()) && Ball->V.Length() < 10);
		printf("Three-way world backend %d, rigidAir=%d, contacts=%zu: cloth displacement=%g, rigid displacement=%g, air time=%g\n", (int)Backend, RigidAir, ContactCount, Displacement, Ball->X.x - Initial.x, Air->GetTime());
		EXPECT(W.RemoveCloth(Cloth)); W.Simulate(1.0f / 120);
		std::vector<std::vector<Vector3>> Loads; EXPECT(Air->ComputeSurfaceLoads(Loads) && Loads.empty());
		W.Reset(); EXPECT(!W.GetAerodynamics() && !W.GetAerodynamicsCoupling());
	}
	void AreaMassAndLifetime()
	{
		PhysicsWorldParam P; PhysicsWorld W(P); ClothSettings S; S.UseSurfaceDensity = true; S.SurfaceDensity = 0.2f; S.Width = 2; S.Height = 3;
		for (int Resolution : { 5, 13 })
		{
			S.Rows = S.Columns = Resolution; auto* C = W.CreateCloth(S, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
			float Mass = 0; for (int I = 0; I < C->GetParticleCount(); ++I) Mass += 1 / C->GetParticle(I).InverseMass;
			EXPECT_NEAR(Mass, 1.2f, 1e-5f); W.RemoveCloth(C);
		}
		auto* Air = W.CreateAerodynamics(AirSettings(FluidBackend::CPU)); EXPECT(Air);
		EXPECT(W.RemoveForceField(W.GetAerodynamicsCoupling())); EXPECT(!W.GetAerodynamicsCoupling());
		W.Simulate(1.0f / 120); EXPECT(W.RemoveAerodynamics());
	}
	void SurfacePressureAccuracy()
	{
		for (float InverseMass : { 1.0f, 100.0f })
		{
			Vector3 Reference(0);
			for (auto Backend : { FluidBackend::CPU, FluidBackend::ComputeShader, FluidBackend::CUDA })
			{
				if (!IsFluidBackendAvailable(Backend)) continue;
				auto Settings = AirSettings(Backend); Settings.Resolution = Vector3i(20, 32, 20);
				Settings.PressureIterations = Backend == FluidBackend::CPU ? 200 : 40;
				AerodynamicsSimulation Air(Settings); EXPECT(Air.IsInitialized()); if (!Air.IsInitialized()) continue;
				auto S = Sheet(); S.InverseMasses.assign(S.Positions.size(), InverseMass);
				EXPECT(Air.SetMovingBoundaries({}, { S })); EXPECT(Air.StepSynchronized(1.0f / 120));
				std::vector<std::vector<Vector3>> Loads; EXPECT(Air.ComputeSurfaceLoads(Loads));
				Vector3 Force(0); for (const auto& F : Loads[0]) Force += F;
				if (Backend == FluidBackend::CPU) Reference = Force;
				const float Error = (Force - Reference).Length() / std::max(Reference.Length(), 1e-6f);
				printf("Finite surface pressure backend=%d inverseMass=%g force=%g error=%g divergence=%g\n", (int)Backend, InverseMass, Force.x, Error, Air.GetDivergenceRMS());
				EXPECT(Error < 0.03f);
			}
		}
	}
}

void TestWorldAerodynamics()
{
	SurfacePressureAccuracy();
	AreaMassAndLifetime();
	CoupledWorld(Riemann::FluidBackend::CPU, false);
	CoupledWorld(Riemann::FluidBackend::CPU, true, false);
	for (auto Backend : { Riemann::FluidBackend::CPU, Riemann::FluidBackend::ComputeShader, Riemann::FluidBackend::CUDA })
	{
		MovingSurface(Backend); CoupledWorld(Backend);
	}
}

void TestAerodynamicsPressureAccuracy() { SurfacePressureAccuracy(); }
}

void TestWorldAerodynamics() { Test_WorldAerodynamics_Cases::TestWorldAerodynamics(); }
void TestAerodynamicsPressureAccuracy() { Test_WorldAerodynamics_Cases::TestAerodynamicsPressureAccuracy(); }

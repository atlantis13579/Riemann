#include "Test.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Renderer/AerodynamicsVolume.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <numeric>

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
	DensityVolumeDesc Volume;
	BuildAerodynamicsDensityVolume(Simulation, &Volume);
	EXPECT(Volume.IsValid() && Volume.Tracer);
	EXPECT(Volume.Density == Simulation.GetTracerDensity());
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

	SceneWorld World;
	std::string Error;
	EXPECT(World.LoadFromFile(TestContentsPath() + "Scenes/12_smoke_3d.json", &Error));
	const SceneFieldSimulationDesc& Scene = World.GetFieldSimulation();
	EXPECT(Scene.Type == "aerodynamics3d" && Scene.Aerodynamics.Algorithm == AerodynamicsAlgorithm::Incompressible && Scene.Aerodynamics.Emitters.size() == 2 && Scene.RigidCoupling);
	EXPECT(Scene.Aerodynamics.Emitters[0].Velocity.y > 0 && Scene.Aerodynamics.Emitters[1].Velocity.x > 0);
	EXPECT(Scene.Aerodynamics.Obstacles.empty() && World.GetAerodynamicsRigidBodyCount() == 2);
	int StaticSmokeBoxes = 0;
	for (const SceneObjectInstance& Object : World.GetObjects())
		if (Object.Id == "smoke_body_2" && Object.GeometryPtr)
		{
			const RigidBody* Body = Object.GeometryPtr->GetParent<RigidBody>();
			StaticSmokeBoxes += Body && Body->mRigidType == RigidType::Static ? 1 : 0;
		}
	EXPECT(StaticSmokeBoxes == 1);
	EXPECT(Scene.Aerodynamics.Backend == FluidBackend::CUDA);
	EXPECT(!Scene.Aerodynamics.AllowBackendFallback);
	AerodynamicsSimulation DefaultSmoke(Scene.Aerodynamics);
#if defined(RIEMANN_ENABLE_CUDA)
	EXPECT(DefaultSmoke.IsInitialized());
	if (DefaultSmoke.IsInitialized())
	{
		DefaultSmoke.Step(1.0f / 60.0f);
		EXPECT(DefaultSmoke.GetTime() > 0.0);
		EXPECT(std::accumulate(DefaultSmoke.GetTracerDensity().begin(),
			DefaultSmoke.GetTracerDensity().end(), 0.0f) > 0.0f);
	}
#else
	EXPECT(!DefaultSmoke.IsInitialized());
#endif
	auto ValidateSceneBackend = [&](FluidBackend Backend)
	{
		AerodynamicsSimulationSettings SceneSettings = Scene.Aerodynamics;
		SceneSettings.Backend = Backend;
		SceneSettings.AllowBackendFallback = false;
		EXPECT(Simulation.Initialize(SceneSettings));
		Simulation.Step(1.0f / 60);
		EXPECT(Simulation.IsInitialized() && Simulation.GetBackend() == Backend && !Simulation.UsedBackendFallback());
		BuildAerodynamicsDensityVolume(Simulation, &Volume, Scene.VisualizationThreshold, Scene.TracerExtinction, Scene.TracerColor);
		EXPECT(Volume.IsValid());
		EXPECT(std::accumulate(Volume.Density.begin(), Volume.Density.end(), 0.0f) > 0);
	};
	ValidateSceneBackend(FluidBackend::CPU);
#if defined(RIEMANN_ENABLE_CUDA)
	ValidateSceneBackend(FluidBackend::CUDA);
#endif
#if defined(_WIN32) && defined(RIEMANN_ENABLE_COMPUTE_SHADER)
	ValidateSceneBackend(FluidBackend::ComputeShader);
#endif
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
	const FluidBackend Backends[] = { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader };
	const char* Names[] = { "CPU", "CUDA", "ComputeShader" };
	std::vector<float> Reference;
	for (int B = 0; B < 3; ++B)
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
	for (FluidBackend Backend : { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader })
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
	for (FluidBackend Backend : { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader })
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

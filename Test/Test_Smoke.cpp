#include "Test.h"
#include "../Src/Fluid/SmokeSimulation.h"
#include "../Src/Fluid/SmokeRigidCoupling.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Renderer/SmokeVolume.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <numeric>

void TestSmoke()
{
	using namespace Riemann;
	SmokeSimulationSettings Settings;
	Settings.BoundsMin = Vector3(-1, 0, -1);
	Settings.BoundsMax = Vector3(1, 3, 1.5f);
	Settings.Resolution = Vector3i(16, 24, 18);
	Settings.PressureIterations = 100;
	SmokeSimulation Smoke;
	EXPECT(Smoke.Initialize(Settings));
	Smoke.Step(0.1f);
	EXPECT(Smoke.GetDivergenceRMS() == 0);
	EXPECT(std::accumulate(Smoke.GetDensity().begin(), Smoke.GetDensity().end(), 0.0f) == 0);
	EXPECT(Smoke.ApplyImpulse(Vector3(0, 1, 0), 0.4f, Vector3(1, 2, -0.5f)));
	const float Before = Smoke.GetDivergenceRMS();
	Smoke.Step(1.0f / 60);
	printf("Smoke anisotropic projection divergence: %.6f -> %.6f\n", Before, Smoke.GetDivergenceRMS());
	EXPECT(Smoke.GetDivergenceRMS() < Before * 0.02f);
	Smoke.Reset();
	EXPECT(Smoke.GetTime() == 0 && Smoke.GetPendingTime() == 0);
	for (const Vector3& V : Smoke.GetVelocity()) EXPECT(V.SquareLength() == 0);

	SmokeEmitter E;
	E.Position = Vector3(0, 0.4f, 0); E.Radius = 0.3f;
	Settings.Emitters.push_back(E);
	SmokeObstacle O; O.Center = Vector3(0.1f, 1.2f, 0); O.Radius = 0.28f;
	Settings.Obstacles.push_back(O);
	EXPECT(Smoke.Initialize(Settings));
	for (int Frame = 0; Frame < 240; ++Frame) Smoke.Step(1.0f / 60);
	float UpperDensity = 0;
	for (size_t I = 0; I < Smoke.GetDensity().size(); ++I)
	{
		EXPECT(std::isfinite(Smoke.GetDensity()[I]) && Smoke.GetDensity()[I] >= 0 && Smoke.GetDensity()[I] <= E.Density + 1e-5f);
		EXPECT(std::isfinite(Smoke.GetTemperature()[I]) && Smoke.GetTemperature()[I] >= 0 && Smoke.GetTemperature()[I] <= E.Temperature + 1e-5f);
		const Vector3& V = Smoke.GetVelocity()[I];
		EXPECT(std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z));
		if (Smoke.GetSolidCells()[I]) EXPECT(Smoke.GetDensity()[I] == 0 && V.SquareLength() == 0);
		if ((I / 16 % 24) > 12) UpperDensity += Smoke.GetDensity()[I];
	}
	printf("Smoke above deflector: %.6f, divergence: %.6f\n", UpperDensity, Smoke.GetDivergenceRMS());
	EXPECT(UpperDensity > 0.1f);
	EXPECT(Smoke.GetDivergenceRMS() < 0.05f);
	DensityVolumeDesc Volume;
	BuildSmokeDensityVolume(Smoke, &Volume);
	EXPECT(Volume.IsValid() && Volume.Smoke);
	EXPECT(Volume.Density == Smoke.GetDensity());
	Smoke.Reset();
	for (float D : Smoke.GetDensity()) EXPECT(D == 0);

	Settings.MaxSubsteps = 1;
	EXPECT(Smoke.Initialize(Settings));
	Smoke.Step(0.1f);
	EXPECT_NEAR(Smoke.GetTime() + Smoke.GetPendingTime(), 0.1, 1e-6);
	EXPECT(Smoke.GetPendingTime() > 0.08);
	Smoke.Step(std::numeric_limits<float>::quiet_NaN());
	EXPECT(std::isfinite(Smoke.GetPendingTime()));
	Settings.Resolution.x = -1;
	EXPECT(!Smoke.Initialize(Settings));
	Settings.Resolution.x = 16;
	Settings.Buoyancy = std::numeric_limits<float>::infinity();
	EXPECT(!Smoke.Initialize(Settings));

	SceneWorld World;
	std::string Error;
	EXPECT(World.LoadFromFile(TestContentsPath() + "Scenes/12_3d_smoke.json", &Error));
	const SceneFieldSimulationDesc& Scene = World.GetFieldSimulation();
	EXPECT(Scene.Type == "smoke3d" && Scene.Smoke.Emitters.size() == 1 && Scene.RigidCoupling);
	EXPECT(Scene.Smoke.Obstacles.empty() && World.GetSmokeRigidBodyCount() == 0);
	SmokeSimulationSettings SceneSettings = Scene.Smoke;
	SceneSettings.Backend = SmokeBackend::CPU;
	EXPECT(Smoke.Initialize(SceneSettings));
	BuildSmokeDensityVolume(Smoke, &Volume, Scene.SmokeExtinction, Scene.SmokeColor);
	EXPECT(Volume.IsValid());
}

void TestSmokeBackends()
{
	using namespace Riemann;
	SmokeSimulationSettings Settings;
	Settings.BoundsMin = Vector3(-1, 0, -1);
	Settings.BoundsMax = Vector3(1, 3, 1.5f);
	Settings.Resolution = Vector3i(16, 24, 18);
	Settings.PressureIterations = 120;
	SmokeEmitter Source; Source.Position = Vector3(0, 0.4f, 0); Source.Radius = 0.3f;
	Settings.Emitters.push_back(Source);
	SmokeObstacle Obstacle; Obstacle.Center = Vector3(0.1f, 1.2f, 0); Obstacle.Radius = 0.28f;
	Settings.Obstacles.push_back(Obstacle);
	const SmokeBackend Backends[] = { SmokeBackend::CPU, SmokeBackend::CUDA, SmokeBackend::ComputeShader };
	const char* Names[] = { "CPU", "CUDA", "ComputeShader" };
	std::vector<float> Reference;
	for (int B = 0; B < 3; ++B)
	{
		Settings.Backend = Backends[B];
		SmokeSimulation Simulation;
		if (!Simulation.Initialize(Settings))
		{
			printf("Smoke %s unavailable (no fallback used).\n", Names[B]);
#if defined(RIEMANN_ENABLE_CUDA)
			if (B == 1) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (B == 2) EXPECT(false);
#endif
			continue;
		}
		EXPECT(Simulation.GetBackend() == Backends[B]);
		Simulation.ApplyImpulse(Vector3(-0.2f, 1.8f, 0), 0.4f, Vector3(1, 2, -0.5f));
		const float Before = Simulation.GetDivergenceRMS();
		Simulation.Step(1.0f / 60);
		const float After = Simulation.GetDivergenceRMS();
		EXPECT(After < Before * 0.025f);
		Simulation.Reset();
		for (int Frame = 0; Frame < 60; ++Frame) Simulation.Step(1.0f / 60);
		EXPECT(Simulation.IsInitialized());
		const auto& Density = Simulation.GetDensity();
		const auto& Temperature = Simulation.GetTemperature();
		const auto& Velocity = Simulation.GetVelocity();
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
		printf("Smoke %s: divergence %.6f -> %.6f; mass %.5f; CPU relative L1 %.5f\n", Names[B], Before, After, Mass, Error / std::max(Mass, 1e-8));
		Simulation.Reset();
		for (float D : Simulation.GetDensity()) EXPECT(D == 0);
		for (const Vector3& V : Simulation.GetVelocity()) EXPECT(V.SquareLength() == 0);
	}
}

void TestSmokeMovingObstacles()
{
	using namespace Riemann;
	for (SmokeBackend Backend : { SmokeBackend::CPU, SmokeBackend::CUDA, SmokeBackend::ComputeShader })
	{
		SmokeSimulationSettings Settings;
		Settings.Backend = Backend;
		Settings.BoundsMin = Vector3(-2, -2, -2);
		Settings.BoundsMax = Vector3(2, 2, 2);
		Settings.Resolution = Vector3i(24, 24, 24);
		Settings.PressureIterations = 160;
		Settings.Vorticity = 0;
		SmokeSimulation Smoke;
		if (!Smoke.Initialize(Settings))
		{
#if defined(RIEMANN_ENABLE_CUDA)
			if (Backend == SmokeBackend::CUDA) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (Backend == SmokeBackend::ComputeShader) EXPECT(false);
#endif
			continue;
		}
		SmokeObstacle Sphere;
		Sphere.Center = Vector3(-0.7f, 0, 0); Sphere.Radius = 0.4f;
		Sphere.LinearVelocity = Vector3(0.8f, 0.2f, 0);
		SmokeObstacle Box;
		Box.Type = SmokeObstacle::Shape::Box;
		Box.Center = Vector3(0.7f, 0, 0); Box.HalfExtent = Vector3(0.3f, 0.6f, 0.25f);
		Box.Rotation = Quaternion(0, 0, std::sin(0.3f), std::cos(0.3f));
		Box.AngularVelocity = Vector3(0, 0, 1.2f);
		EXPECT(Smoke.SetRigidObstacles({ Sphere, Box }));
		const auto OriginalMask = Smoke.GetSolidCells();
		const float Before = Smoke.GetDivergenceRMS();
		EXPECT(Before > 0.1f);
		Smoke.Step(1.0f / 60);
		const float After = Smoke.GetDivergenceRMS();
		EXPECT(After < Before * 0.03f);
		float FluidEnergy = 0;
		int CheckedFaces = 0;
		const auto& Velocity = Smoke.GetVelocity();
		for (int I = 0; I < (int)Velocity.size(); ++I)
		{
			const int G[] = { I % 24, I / 24 % 24, I / (24 * 24) };
			const int Stride[] = { 1, 24, 24 * 24 };
			EXPECT(std::isfinite(Velocity[I].SquareLength()));
			if (!OriginalMask[I]) FluidEnergy += Velocity[I].SquareLength();
			else EXPECT(Smoke.GetDensity()[I] == 0);
			for (int A = 0; A < 3; ++A)
			{
				if (G[A] == 23 || (!OriginalMask[I] && !OriginalMask[I + Stride[A]])) continue;
				Vector3 P(-2 + (G[0] + 0.5f) / 6, -2 + (G[1] + 0.5f) / 6, -2 + (G[2] + 0.5f) / 6);
				P[A] += 1.0f / 12;
				const SmokeObstacle& O = P.x < 0 ? Sphere : Box;
				const float Expected = (O.LinearVelocity + O.AngularVelocity.Cross(P - O.Center))[A];
				EXPECT_NEAR(Velocity[I][A], Expected, 1e-5f);
				++CheckedFaces;
			}
		}
		EXPECT(FluidEnergy > 1 && CheckedFaces > 50);
		Sphere.Center.y += 1; Box.Center.y -= 1;
		EXPECT(Smoke.SetRigidObstacles({ Sphere, Box }));
		int Released = 0, Occupied = 0;
		for (size_t I = 0; I < OriginalMask.size(); ++I)
		{
			Released += OriginalMask[I] && !Smoke.GetSolidCells()[I];
			Occupied += !OriginalMask[I] && Smoke.GetSolidCells()[I];
		}
		EXPECT(Released > 50 && Occupied > 50);
		const auto MovedMask = Smoke.GetSolidCells();
		Sphere.LinearVelocity.x = std::numeric_limits<float>::quiet_NaN();
		EXPECT(!Smoke.SetRigidObstacles({ Sphere }));
		EXPECT(MovedMask == Smoke.GetSolidCells());
		EXPECT(Smoke.SetRigidObstacles({}));
		for (unsigned char Solid : Smoke.GetSolidCells()) EXPECT(Solid == 0);
		printf("Moving smoke walls backend %d: divergence %.6f -> %.6f, fluid energy %.4f, released %d cells\n",
			(int)Backend, Before, After, FluidEnergy, Released);
	}
}

void TestSmokeRigidCoupling()
{
	using namespace Riemann;
	Vector3 ReferenceForce;
	for (SmokeBackend Backend : { SmokeBackend::CPU, SmokeBackend::CUDA, SmokeBackend::ComputeShader })
	{
		SmokeSimulationSettings Settings;
		Settings.Backend = Backend;
		Settings.BoundsMin = Vector3(-2, 0, -2); Settings.BoundsMax = Vector3(2, 4, 2);
		Settings.Resolution = Vector3i(24, 24, 24); Settings.PressureIterations = 120;
		Settings.Vorticity = 0;
		SmokeSimulation Smoke;
		if (!Smoke.Initialize(Settings))
		{
#if defined(RIEMANN_ENABLE_CUDA)
			if (Backend == SmokeBackend::CUDA) EXPECT(false);
#endif
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER)
			if (Backend == SmokeBackend::ComputeShader) EXPECT(false);
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
		SmokeRigidCoupling* Coupling = new SmokeRigidCoupling(&Smoke);
		World.AddForceField(Coupling);
		Body->V = Vector3(1, 0, 0);
		EXPECT(Coupling->Synchronize({ Sphere }));
		Smoke.Step(1.0f / 60);
		Coupling->Update(1.0f / 60);
		SmokeRigidLoad Load;
		EXPECT(Coupling->Evaluate(Body, Load));
		EXPECT(Load.Force.x < -0.1f);
		if (Backend == SmokeBackend::CPU) ReferenceForce = Load.Force;
		else EXPECT((Load.Force - ReferenceForce).Length() < ReferenceForce.Length() * 0.005f);
		printf("Smoke pressure reaction backend %d: (%.4f, %.4f, %.4f) N\n", (int)Backend, Load.Force.x, Load.Force.y, Load.Force.z);
		// An off-center, rotating box must transfer angular momentum to the air.
		Geometry* Box = GeometryFactory::CreateOBB(Vector3(1.1f, 2, 0), Vector3(0.25f, 0.6f, 0.3f));
		RigidBodyDynamic* Paddle = World.CreateRigidBody(Box, BodySettings)->CastDynamic();
		Paddle->W = Vector3(0, 0, 1);
		Body->V = Vector3::Zero();
		Smoke.Reset();
		EXPECT(Coupling->Synchronize({ Sphere, Box }));
		Smoke.Step(1.0f / 60); Coupling->Update(1.0f / 60);
		EXPECT(Coupling->Evaluate(Paddle, Load));
		EXPECT(Load.Torque.z < -0.01f);
		// A plume drives a free body, whose new pose becomes the next smoke boundary.
		World.RemoveRigidBody(Paddle);
		SmokeEmitter E; E.Position = Vector3(0, 0.5f, 0); E.Radius = 0.35f; E.Velocity = Vector3(0, 3, 0);
		Settings.Emitters.push_back(E);
		EXPECT(Smoke.Initialize(Settings));
		Coupling->SetSimulation(&Smoke);
		Body->V = Body->W = Vector3::Zero();
		const Vector3 Start = Body->X;
		EXPECT(Coupling->Synchronize({ Sphere }));
		for (int Frame = 0; Frame < 150; ++Frame)
		{
			World.Simulate(1.0f / 60);
			EXPECT(Coupling->Synchronize({ Sphere }));
			Smoke.Step(1.0f / 60);
			EXPECT(Smoke.IsInitialized() && std::isfinite(Body->X.SquareLength()));
		}
		EXPECT(Body->X.y > Start.y + 0.01f);
		printf("Coupled plume backend %d: body lift %.5f m, speed %.5f m/s\n", (int)Backend, Body->X.y - Start.y, Body->V.Length());
		Smoke.Reset(); Coupling->Update(1.0f / 60);
		EXPECT(Coupling->Evaluate(Body, Load) && Load.Force.IsZero() && Load.Torque.IsZero());
		EXPECT(World.RemoveForceField(Coupling));
	}
}

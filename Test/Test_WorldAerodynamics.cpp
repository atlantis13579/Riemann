#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Fluid/FluidBackend.h"
#include <algorithm>
#include <chrono>
#include "../Viewer/SceneWorld.h"

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
	float SmokeClothResponse(bool HorizontalEmitter)
	{
		SceneWorld Scene; std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/12_smoke_3d.json", &Error));
		auto* W = Scene.GetSimulation(); auto* C = W->GetCloth(0);
		EXPECT(C); if (!C) return 0;
		auto Settings = Scene.GetFieldSimulation().Aerodynamics;
		EXPECT(Settings.Emitters.size() == 2);
		if (!HorizontalEmitter) Settings.Emitters.resize(1);
		EXPECT(Scene.GetAerodynamicsRigidBodyCount() == 2);
		auto* Air = W->CreateAerodynamics(Settings); EXPECT(Air); if (!Air) return 0;
		std::vector<Vector3> Rest;
		for (int I = 0; I < C->GetParticleCount(); ++I) Rest.push_back(C->GetParticle(I).Position);
		for (int Frame = 0; Frame < 240; ++Frame)
		{
			Scene.Step(1.0f / 60);
			for (int I = 0; I < C->GetParticleCount(); ++I)
			{
				EXPECT(std::isfinite(C->GetParticle(I).Position.SquareLength()));
				EXPECT(C->GetParticleVelocity(I).Length() < 30);
				if (C->GetParticle(I).Pinned) EXPECT(C->GetParticle(I).Position == Rest[I]);
			}
		}
		float Displacement = 0; int Count = 0;
		for (int I = 0; I < C->GetParticleCount(); ++I) if (!C->GetParticle(I).Pinned)
		{
			Displacement += C->GetParticle(I).Position.x - Rest[I].x; ++Count;
		}
		Displacement /= std::max(Count, 1);
		EXPECT_NEAR(Air->GetTime(), 4.0, 4e-6); EXPECT(Air->GetPendingTime() <= 1e-7);
		printf("Smoke demo horizontal emitter=%d: mean cloth displacement=%g\n", HorizontalEmitter, Displacement);
		return Displacement;
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

void TestCoupledDemos()
{
	using namespace Riemann;
	SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
	EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
	auto* C = Scene.GetDemoCloth(); EXPECT(C && C->GetRigidAttachmentCount() == 0); if (!C) return;
	RigidBody* Ball = nullptr; for (const auto& Object : Scene.GetObjects()) if (Object.Id == "cloth_ball") Ball = Object.GeometryPtr->GetParent<RigidBody>();
	EXPECT(Ball); if (!Ball) return;
	const Vector3 Start = Ball->X;
	EXPECT(Scene.ApplyClothDemo(false, false, true, false));
	Ball->V = Vector3(0, 0, -6);
	size_t Contacts = 0;
	for (int Frame = 0; Frame < 15; ++Frame)
	{
		Scene.Step(1.0f / 60);
		Contacts += C->GetCollisionStats().RigidContacts;
	}
	EXPECT(Contacts > 0);
	Ball->V = Vector3(0, 0, 8);
	for (int Frame = 0; Frame < 30; ++Frame) Scene.Step(1.0f / 60);
	EXPECT(Ball->X.z > Start.z + 1.0f);
	EXPECT(C->GetRigidAttachmentCount() == 0);
	EXPECT(Scene.SetClothRigidInteraction(false)); EXPECT(C->GetRigidAttachmentCount() == 0);
	EXPECT(Scene.SetClothRigidInteraction(true)); EXPECT(C->GetRigidAttachmentCount() == 0);
	EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/12_smoke_3d.json", &Error));
	auto* W = Scene.GetSimulation(); EXPECT(W->GetClothCount() == 1); EXPECT(Scene.GetAerodynamicsRigidBodyCount() == 2);
	C = W->GetCloth(0); EXPECT(C->HasRigidCoupling()); EXPECT(C->GetSettings().UseSurfaceDensity);
	const auto& S = Scene.GetFieldSimulation().Aerodynamics;
	EXPECT(S.Emitters.size() == 2); EXPECT(S.Emitters[0].Velocity.y > 0 && S.Emitters[1].Velocity.x > 0);
	EXPECT(C->GetParticle(0).Position.y < 2.15f);
	if (!IsFluidBackendAvailable(S.Backend)) return;
	auto* Air = W->CreateAerodynamics(S); EXPECT(Air); if (!Air) return;
	for (int I = 0; I < 3; ++I) Scene.Step(1.0f / 60);
	const auto Before = std::chrono::steady_clock::now();
	for (int I = 0; I < 3; ++I) Scene.Step(1.0f / 60);
	const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Before).count() / 3;
	printf("Smoke demo %dx%dx%d: %g ms/frame, air time=%g\n", S.Resolution.x, S.Resolution.y, S.Resolution.z, Milliseconds, Air->GetTime());
	EXPECT_NEAR(Air->GetTime(), 0.1, 1e-6); EXPECT(Air->GetPendingTime() <= 1e-7);
	W->RemoveAerodynamics();
	const float WithoutHorizontal = SmokeClothResponse(false);
	const float WithHorizontal = SmokeClothResponse(true);
	EXPECT(WithHorizontal > 0.05f);
	EXPECT(WithHorizontal > WithoutHorizontal + 0.04f);
}

void TestAerodynamicsPerformance()
{
	using namespace Riemann;
	for (int Mode = 0; Mode < 4; ++Mode)
	{
		SceneWorld Scene; std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/12_smoke_3d.json", &Error));
		auto* W = Scene.GetSimulation(); auto S = Scene.GetFieldSimulation().Aerodynamics;
		if (!IsFluidBackendAvailable(S.Backend)) return;
		if (Mode == 0) W->RemoveCloth(W->GetCloth(0));
		EXPECT(Scene.GetAerodynamicsRigidBodyCount() == 2);
		std::unique_ptr<AerodynamicsSimulation> Standalone;
		if (Mode == 1) Standalone.reset(new AerodynamicsSimulation(S));
		auto* Air = Mode == 1 ? Standalone.get() : W->CreateAerodynamics(S);
		EXPECT(Air && Air->IsInitialized()); if (!Air || !Air->IsInitialized()) continue;
		printf("Performance mode %d: %s\n", Mode, Mode == 0 ? "world without cloth" : Mode == 1 ? "standalone smoke" : Mode == 2 ? "world with cloth" : "world with cloth and two rigid bodies");
		double Total = 0; int Substeps = 0;
		for (int Frame = 0; Frame < 75; ++Frame)
		{
			const auto Start = std::chrono::steady_clock::now();
			if (Mode == 1) Air->Step(1.0f / 60); else W->Simulate(1.0f / 60);
			if (Frame >= 15) Total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
			if (Frame >= 15) Substeps += W->GetStepInfo().SubstepCount;
		}
		printf("Performance mode %d: %.3f ms/frame, %.2f world substeps\n", Mode, Total / 60, Substeps / 60.0);
		printf("Performance mode %d: divergence=%g stableDt=%g\n", Mode, Air->GetDivergenceRMS(), Air->GetStableTimeStep());
		if (Mode == 2)
		{
			auto* C = W->GetCloth(0); float Bend = 0;
			for (const auto& B : C->GetTopology().GetBends())
			{
				Vector3 P[4], G[4]; for (int I = 0; I < 4; ++I) P[I] = C->GetParticle(B.Particles[I]).Position;
				float A = 0; if (ClothTopology::EvaluateBend(B, P, A, G)) Bend = std::max(Bend, std::fabs(A - B.RestAngle));
			}
			printf("Performance cloth maximum bend=%g radians\n", Bend);
		}
	}
}

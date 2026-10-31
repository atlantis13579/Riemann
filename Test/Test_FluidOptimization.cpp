#include "Test.h"
#include <algorithm>
#include <vector>
#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/MACGridSampling.h"

// Regression coverage: independent MAC ghost-face interpolation, anisotropic
// closed/free-surface pressure projection, and brute-force SPH density samples
// after particle motion, insertion, method conversion, and reset. Core tests
// additionally cover conservation, impulses, boundaries, rigid coupling, and
// marching cubes.

namespace
{
	float ReferenceMAC(const std::vector<Vector3>& Field, const Vector3i& Size,
		Vector3 P, int Component)
	{
		P[Component] -= 0.5f;
		const Vector3i Base((int)std::floor(P.x), (int)std::floor(P.y), (int)std::floor(P.z));
		const Vector3 T = P - Vector3((float)Base.x, (float)Base.y, (float)Base.z);
		float Result = 0.0f;
		for (int Z = 0; Z < 2; ++Z)
		for (int Y = 0; Y < 2; ++Y)
		for (int X = 0; X < 2; ++X)
		{
			Vector3i C(Base.x + X, Base.y + Y, Base.z + Z);
			if (C[Component] < 0 || C[Component] >= Size[Component]) continue;
			for (int Axis = 0; Axis < 3; ++Axis) C[Axis] = std::max(0, std::min(Size[Axis] - 1, C[Axis]));
			Result += Field[C.x + Size.x * (C.y + Size.y * C.z)][Component]
				* (X ? T.x : 1 - T.x) * (Y ? T.y : 1 - T.y) * (Z ? T.z : 1 - T.z);
		}
		return Result;
	}
}

void TestFluidOptimization()
{
	using namespace Riemann;
	const Vector3i Size(7, 5, 9);
	std::vector<Vector3> Field(Size.x * Size.y * Size.z);
	for (size_t I = 0; I < Field.size(); ++I)
		Field[I] = Vector3(std::sin((float)I), std::cos(0.31f * (float)I), std::sin(0.7f * (float)I));
	for (int I = 0; I < 2000; ++I)
	{
		const Vector3 P(-1.1f + (I % 37) * 0.27f, -1.2f + (I % 29) * 0.3f, -1.3f + (I % 41) * 0.31f);
		const Vector3 Value = SampleMACVelocity(Field.data(), Size, P);
		for (int C = 0; C < 3; ++C) EXPECT_NEAR(Value[C], ReferenceMAC(Field, Size, P, C), 5.0e-6f);
	}

	// Anisotropic closed and free-surface domains exercise the pressure nullspace
	// and the mixed Dirichlet/Neumann stencil without relying on iteration counts.
	for (int Closed = 0; Closed < 2; ++Closed)
	{
		FluidSimulationSettings Settings;
		Settings.BoundsMin = Vector3::Zero(); Settings.BoundsMax = Vector3(2, 1, 1.5f);
		Settings.Resolution = Vector3i(20, 12, 16); Settings.Gravity = Vector3::Zero();
		Settings.PressureIterations = 120; Settings.DynamicViscosity = 0;
		FluidSimulation Fluid(Settings);
		Fluid.AddFluidBox(Vector3::Zero(), Vector3(2, Closed ? 1.0f : 0.6f, 1.5f));
		const float Mass = Fluid.GetTotalMass();
		EXPECT(Fluid.ApplyImpulse(Vector3(0.6f, 0.4f, 0.75f), 0.3f, Vector3(15, 4, 1)));
		const float Before = Fluid.GetMaxDivergence();
		Fluid.Step(Settings.MaxTimeStep);
		EXPECT(Fluid.GetMaxDivergence() < 0.002f * Before);
		EXPECT_NEAR(Fluid.GetTotalMass(), Mass, Mass * 1.0e-5f);
	}

	// Validate the rebuilt particle index against an independent all-pairs sum,
	// including motion, insertion, state conversion, and reset.
	FluidSimulationSettings Settings;
	Settings.Method = FluidSimulationMethod::Lagrangian;
	Settings.BoundsMin = Vector3(-1); Settings.BoundsMax = Vector3(1);
	Settings.Resolution = Vector3i(10, 10, 10);
	Settings.ParticleSpacing = 0.15f; Settings.SmoothingRadius = 0.32f;
	FluidSimulation Particles(Settings);
	Particles.AddFluidBox(Vector3(-0.8f), Vector3(0.8f));
	for (int Step = 0; Step < 4; ++Step)
	{
		Particles.Step(1.0f / 60.0f);
		if (Step == 1) Particles.AddFluidSphere(Vector3(0, 0.8f, 0), 0.2f);
		const float H = Settings.SmoothingRadius, H2 = H * H;
		const float Poly6 = 315.0f / (64.0f * 3.14159265358979323846f * std::pow(H, 9.0f));
		for (int Sample = 0; Sample < 20; ++Sample)
		{
			const Vector3 P(-0.7f + Sample * 0.07f, 0.2f, -0.12f);
			float Density = 0;
			for (const FluidParticle& Particle : Particles.GetParticles())
			{
				const float R2 = (P - Particle.Position).SquareLength();
				if (R2 < H2) Density += Particle.Mass * Poly6 * std::pow(H2 - R2, 3.0f);
			}
			EXPECT_NEAR(Particles.SampleDensity(P), Density, 0.002f);
		}
	}
	EXPECT(Particles.SetMethod(FluidSimulationMethod::Eulerian));
	EXPECT(Particles.SetMethod(FluidSimulationMethod::Lagrangian));
	Particles.Step(0.01f);
	Particles.Reset();
	EXPECT_NEAR(Particles.SampleDensity(Vector3::Zero()), 0, 1.0e-6f);
}

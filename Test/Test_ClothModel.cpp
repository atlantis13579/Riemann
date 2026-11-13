#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.01f;
	ClothSettings Settings()
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1; S.SolverIterations = 20; return S;
	}
	ClothMesh Triangle()
	{
		ClothMesh M; M.Positions = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 1, 0) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses = { 1, 1, 1 }; return M;
	}
	ClothMesh Hinge()
	{
		auto M = Triangle(); M.Positions.push_back(Vector3(0.7f, -1, 0)); M.Triangles.emplace_back(1, 0, 3); M.InverseMasses.push_back(1); return M;
	}
	void DisableMaterial(ClothSettings& S) { S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false; }
	void Step(ClothSystem& Cloth, int Count, float StepTime = Dt) { for (int I = 0; I < Count; ++I) Cloth.Simulate(StepTime); }
	void DerivativesAndObjectivity()
	{
		const auto M = Hinge(); ClothTopology Topology(M); const auto& T = Topology.GetTriangles()[0];
		Vector3 P[3] = { Vector3(0.2f, 0.1f, -0.1f), Vector3(2.2f, 0.15f, 0.3f), Vector3(0.4f, 1.4f, 0.1f) };
		constexpr float DifferenceStep = 0.001f;
		for (int Channel = 0; Channel < 3; ++Channel)
		{
			float Value; Vector3 G[3]; EXPECT(ClothTopology::EvaluateStrain(T, P, Channel, Value, G));
			EXPECT_NEAR((G[0] + G[1] + G[2]).Length(), 0, 1e-6f);
			EXPECT_NEAR((P[0].Cross(G[0]) + P[1].Cross(G[1]) + P[2].Cross(G[2])).Length(), 0, 1e-6f);
			for (int I = 0; I < 3; ++I) for (int Axis = 0; Axis < 3; ++Axis)
			{
				Vector3 Q[3] = { P[0], P[1], P[2] }, Unused[3]; float Plus, Minus;
				Q[I][Axis] += DifferenceStep; EXPECT(ClothTopology::EvaluateStrain(T, Q, Channel, Plus, Unused));
				Q[I][Axis] -= 2 * DifferenceStep; EXPECT(ClothTopology::EvaluateStrain(T, Q, Channel, Minus, Unused));
				EXPECT_NEAR(G[I][Axis], (Plus - Minus) / (2 * DifferenceStep), 2e-4f);
			}
			Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3), 1.2f); Vector3 Q[3], QG[3]; float Rotated;
			for (int I = 0; I < 3; ++I) Q[I] = Rotation * P[I] + Vector3(3, -2, 1);
			EXPECT(ClothTopology::EvaluateStrain(T, Q, Channel, Rotated, QG)); EXPECT_NEAR(Value, Rotated, 1e-6f);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR((QG[I] - Rotation * G[I]).Length(), 0, 1e-6f);
		}
		const auto& B = Topology.GetBends()[0]; Vector3 Q[4]; for (int I = 0; I < 4; ++I) Q[I] = M.Positions[B.Particles[I]];
		Q[2] += Vector3(0.2f, -0.1f, 0.3f); Q[3] += Vector3(0.1f, 0.1f, -0.7f);
		float Angle; Vector3 G[4]; EXPECT(ClothTopology::EvaluateBend(B, Q, Angle, G));
		Vector3 Torque = Vector3::Zero(), Sum = Vector3::Zero();
		for (int I = 0; I < 4; ++I) { Sum += G[I]; Torque += Q[I].Cross(G[I]); }
		EXPECT_NEAR(Sum.Length(), 0, 1e-6f); EXPECT_NEAR(Torque.Length(), 0, 1e-6f);
		for (int I = 0; I < 4; ++I) for (int Axis = 0; Axis < 3; ++Axis)
		{
			Vector3 R[4] = { Q[0], Q[1], Q[2], Q[3] }, Unused[4]; float Plus, Minus;
			R[I][Axis] += DifferenceStep; EXPECT(ClothTopology::EvaluateBend(B, R, Plus, Unused));
			R[I][Axis] -= 2 * DifferenceStep; EXPECT(ClothTopology::EvaluateBend(B, R, Minus, Unused));
			EXPECT_NEAR(G[I][Axis], (Plus - Minus) / (2 * DifferenceStep), 2e-4f);
		}
		Q[2] = Q[0]; EXPECT(!ClothTopology::EvaluateBend(B, Q, Angle, G));
	}
	void IndependentMaterialModes()
	{
		for (bool RotateMaterial : { false, true })
		{
			auto M = Triangle(); M.MaterialCoordinates = RotateMaterial ? std::vector<Vector2>{ Vector2(0, 0), Vector2(0, -2), Vector2(1, 0) }
				: std::vector<Vector2>{ Vector2(0, 0), Vector2(2, 0), Vector2(0, 1) };
			auto S = Settings(); S.Material.StretchUCompliance = 0; S.Material.StretchVCompliance = 1; S.Material.ShearEnabled = false;
			ClothSystem Cloth(S, M); Cloth.PinParticle(0, M.Positions[0]); Cloth.PinParticle(2, M.Positions[2]);
			Cloth.SetParticlePosition(1, Vector3(2.6f, 0, 0)); Step(Cloth, 1);
			if (RotateMaterial) EXPECT(Cloth.GetParticle(1).Position.x > 2.59f); else EXPECT_NEAR(Cloth.GetParticle(1).Position.x, 2, 1e-5f);
		}
		{
			auto M = Triangle(); auto S = Settings(); S.Material.StretchEnabled = false; S.Material.ShearCompliance = 0;
			ClothSystem Cloth(S, M); Cloth.PinParticle(0, M.Positions[0]); Cloth.PinParticle(1, M.Positions[1]);
			Cloth.SetParticlePosition(2, Vector3(0.6f, 1, 0)); Step(Cloth, 1); EXPECT_NEAR(Cloth.GetParticle(2).Position.x, 0, 1e-6f);
			EXPECT_NEAR(Cloth.GetParticle(2).Position.y, 1, 1e-6f);
		}
		for (float Fold : { -0.7f, 0.7f })
		{
			auto M = Hinge(); auto S = Settings(); S.Material.StretchEnabled = S.Material.ShearEnabled = false; S.Material.BendingCompliance = 0;
			ClothSystem Cloth(S, M); for (int I = 0; I < 3; ++I) Cloth.PinParticle(I, M.Positions[I]);
			Cloth.SetParticlePosition(3, M.Positions[3] + Vector3(0, 0, Fold)); Step(Cloth, 1);
			EXPECT_NEAR(Cloth.GetParticle(3).Position.z, 0, 1e-5f);
		}
		auto Curved = Hinge(); Curved.Positions[3].z = 0.6f; ClothSystem Rest(Settings(), Curved); Step(Rest, 100);
		for (int I = 0; I < Rest.GetParticleCount(); ++I) EXPECT_NEAR((Rest.GetParticle(I).Position - Curved.Positions[I]).Length(), 0, 2e-4f);
	}
	void ComplianceAndTimeSteps()
	{
		float Results[2];
		for (int Mode = 0; Mode < 2; ++Mode)
		{
			auto S = Settings(); S.Material.StretchUCompliance = 0.02f; S.Gravity = Vector3(1, 0, 0); S.Damping = 0.97f;
			auto M = Triangle(); ClothSystem Cloth(S, M); Cloth.PinParticle(0, M.Positions[0]); Cloth.PinParticle(2, M.Positions[2]);
			Step(Cloth, Mode == 0 ? 1000 : 500, Mode == 0 ? 0.01f : 0.02f); Results[Mode] = Cloth.GetParticle(1).Position.x;
			// Equilibrium of A*C^2/(2*compliance) under unit force, where C=(x^2/4-1)/2.
			const float X = Results[Mode], Force = ((X * X / 4 - 1) / 2) * (X / 4) / S.Material.StretchUCompliance;
			printf("Cloth equilibrium dt=%.3f x=%.6f elastic force=%.6f\n", Mode == 0 ? 0.01f : 0.02f, X, Force);
			EXPECT_NEAR(Force, 1, 0.06f);
		}
		EXPECT_NEAR(Results[0], Results[1], 0.006f);
		for (auto Model : { ClothModel::DistanceLinks, ClothModel::TriangleStrain })
		{
			auto S = Settings(); S.Model = Model; S.Damping = 0.99f; auto M = Triangle(); ClothSystem Cloth(S, M);
			for (int I = 0; I < 3; ++I) Cloth.SetParticleVelocity(I, Vector3(0, 0, 2));
			for (int Frame = 0; Frame < 50; ++Frame) { Cloth.Simulate(0.006f); Cloth.Simulate(0.014f); }
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(Cloth.GetParticleVelocity(I).z, 2 * std::pow(0.99f, 60), 5e-5f);
		}
		PhysicsWorldParam P; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4; PhysicsWorld World(P);
		auto S = Settings(); S.Substeps = 2; auto* Cloth = World.CreateCloth(S, Triangle());
		for (int I = 0; I < 3; ++I) Cloth->SetParticleVelocity(I, Vector3(0, 0, 1));
		for (int Frame = 0; Frame < 60; ++Frame) World.Simulate(1.0f / 60);
		EXPECT_NEAR(Cloth->GetParticle(0).Position.z, 1, 2e-4f); EXPECT(World.RemoveCloth(Cloth));
	}
	void GeodesicTethers()
	{
		ClothMesh M; M.Positions = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(1, 1, 0), Vector3(0, 0, 0.2f), Vector3(1, 0, 0.2f) };
		M.Triangles = { Index3(0, 1, 2), Index3(1, 3, 2), Index3(2, 3, 4), Index3(3, 5, 4) };
		auto S = Settings(); DisableMaterial(S); S.Material.TethersEnabled = true;
		ClothSystem Cloth(S, M); Cloth.PinParticle(0, M.Positions[0]); Cloth.SetParticlePosition(4, Vector3(10, 0, 0)); Step(Cloth, 1);
		EXPECT(Cloth.GetTetherCount() == 5); EXPECT_NEAR(Cloth.GetParticle(4).Position.Length(), 1 + std::sqrt(1.04f), 1e-5f);
		Cloth.UnpinParticle(0); Step(Cloth, 1); EXPECT(Cloth.GetTetherCount() == 0);
		Cloth.PinParticle(5, Cloth.GetParticle(5).Position); Step(Cloth, 1); EXPECT(Cloth.GetTetherCount() == 5);
		// A second disconnected triangle has no route to the pinned first panel.
		auto Other = Triangle(); const int Offset = (int)M.Positions.size();
		for (const auto& P : Other.Positions) M.Positions.push_back(P + Vector3(0, 0, 10)); M.Triangles.emplace_back(Offset, Offset + 1, Offset + 2);
		ClothSystem Panels(S, M); Panels.PinParticle(0, M.Positions[0]); Step(Panels, 1); EXPECT(Panels.GetTetherCount() == 5);
	}
	void WindAndAnimation()
	{
		for (bool Reverse : { false, true })
		{
			auto M = Triangle(); if (Reverse) M.Triangles[0] = Index3(0, 2, 1);
			auto S = Settings(); DisableMaterial(S); S.Wind.NormalDrag = 1; S.Wind.Density = 1;
			ClothSystem Cloth(S, M); for (int I = 0; I < 3; ++I) Cloth.SetParticleVelocity(I, Vector3(0, 0, 5));
			Step(Cloth, 1); const float Expected = 5 / (1 + 0.5f * 5 * Dt / 3);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(Cloth.GetParticleVelocity(I).z, Expected, 1e-5f);
			for (int I = 0; I < 3; ++I) Cloth.SetParticleVelocity(I, Vector3(5, 0, 0)); Step(Cloth, 1);
			EXPECT_NEAR(Cloth.GetParticleVelocity(0).x, 5, 1e-5f);
			auto Wind = S.Wind; Wind.TangentialDrag = 1; Cloth.SetWind(Wind); Step(Cloth, 1); EXPECT(Cloth.GetParticleVelocity(0).x < 5);
			Wind.Velocity = Vector3(5, 0, 0); Cloth.SetWind(Wind);
			for (int I = 0; I < 3; ++I) Cloth.SetParticleVelocity(I, Wind.Velocity); Step(Cloth, 1); EXPECT_NEAR(Cloth.GetParticleVelocity(0).x, 5, 1e-5f);
		}
		auto M = Triangle(); auto S = Settings(); DisableMaterial(S); S.AnimationDrive.Enabled = true; S.AnimationDrive.Compliance = 0;
		ClothSystem Cloth(S, M); Cloth.SetAnimationWeights({ 1, 0, 1 }); Cloth.PinParticle(0, M.Positions[0]);
		auto Target = M.Positions; for (auto& P : Target) P.z += 1; Cloth.SetAnimationPose(Target, 1); Step(Cloth, 25);
		EXPECT_NEAR(Cloth.GetParticle(0).Position.z, 0.25f, 1e-6f); EXPECT_NEAR(Cloth.GetParticleVelocity(0).z, 1, 1e-5f);
		EXPECT_NEAR(Cloth.GetParticle(1).Position.z, 0, 1e-6f); EXPECT_NEAR(Cloth.GetParticle(2).Position.z, 0.25f, 1e-6f);
		auto Drive = S.AnimationDrive; Drive.MovePins = false; Cloth.SetAnimationDrive(Drive); Step(Cloth, 1);
		EXPECT_NEAR(Cloth.GetParticle(0).Position.z, 0.25f, 1e-6f);
		Drive.Compliance = 1e6f; Drive.Damping = 20; Cloth.SetAnimationDrive(Drive);
		Cloth.SetParticleVelocity(2, Vector3::Zero()); for (auto& P : Target) P.z += 100; Cloth.SetAnimationPose(Target);
		const auto Before = Cloth.GetParticle(2).Position; EXPECT(Before.z < 1); Step(Cloth, 1);
		EXPECT(std::abs(Cloth.GetParticleVelocity(2).z) < 0.001f);
	}
	void TopologyValidationAndRendering()
	{
		auto Reject = [](auto Build) { bool Rejected = false; try { Build(); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected); };
		auto M = Triangle(); M.InverseMasses.clear(); ClothSystem Cloth(Settings(), M);
		auto Mixed = Hinge(); Mixed.Positions[2].y = 0.0001f; Mixed.Positions[3].y = -1000;
		ClothTopology MixedTopology(Mixed); EXPECT(MixedTopology.GetBends().size() == 1);
		float Mass = 0; for (int I = 0; I < 3; ++I) Mass += 1 / Cloth.GetParticle(I).InverseMass; EXPECT_NEAR(Mass, 1, 1e-6f);
		auto Invalid = M; Invalid.Triangles.push_back(Invalid.Triangles[0]); Reject([&] { ClothTopology T(Invalid); });
		Invalid = M; Invalid.Triangles[0].a = -1; Reject([&] { ClothTopology T(Invalid); });
		Invalid = M; Invalid.Positions[2] = Invalid.Positions[0]; Reject([&] { ClothTopology T(Invalid); });
		Invalid = Hinge(); Invalid.Triangles[1] = Index3(0, 1, 3); Reject([&] { ClothTopology T(Invalid); });
		Invalid = M; Invalid.MaterialCoordinates.assign(3, Vector2(0)); Reject([&] { ClothTopology T(Invalid); });
		Invalid = M; Invalid.Positions.push_back(Vector3(1, 1, 1)); Reject([&] { ClothTopology T(Invalid); });
		Invalid = M; Invalid.InverseMasses = { 1, -1, 1 }; Reject([&] { ClothTopology T(Invalid); });
		auto S = Settings(); S.SurfaceDensity = 0; Reject([&] { ClothSystem C(S, M); });
		Reject([&] { Cloth.SetAnimationWeights({ 1, 2, 1 }); }); Reject([&] { Cloth.SetAnimationPose({}); });
		auto Material = Cloth.GetSettings().Material; Material.BendingCompliance = -1; Reject([&] { Cloth.SetMaterial(Material); });
		Cloth.Simulate(std::numeric_limits<float>::quiet_NaN()); EXPECT(Cloth.GetParticle(0).Position == M.Positions[0]);
		EXPECT(!Cloth.PinParticle(-1, Vector3::Zero())); EXPECT(!Cloth.PinParticle(0, Vector3(std::numeric_limits<float>::infinity())));
		// Rendering must retain 32-bit indices past 65535; no imported mesh is used.
		S = Settings(); S.Rows = S.Columns = 257; ClothSystem Large(S, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
		const auto* Render = Large.GetRenderGeometry()->GetShapeObj<TriangleMesh>(); EXPECT(!Render->Is16bitIndices());
		EXPECT(Render->GetIndices32()[3 * Render->NumTriangles - 1] == (uint32_t)Large.GetParticleCount() - 1);
		EXPECT(!Large.GetRenderGeometry()->IsQueryEnabled() && !Large.GetRenderGeometry()->IsSimulationEnabled());
	}
}
void TestClothModel()
{
	DerivativesAndObjectivity(); IndependentMaterialModes(); ComplianceAndTimeSteps(); GeodesicTethers(); WindAndAnimation(); TopologyValidationAndRendering();
}

void TestClothModelDemo()
{
	using namespace Riemann;
	const std::string Path = TestContentsPath() + "Scenes/08_cloth_sphere.json"; std::string Error;
	SceneWorld Scene; EXPECT(Scene.LoadFromFile(Path, &Error)); const size_t ObjectCount = Scene.GetObjects().size();
	EXPECT(!Scene.GetDemoCloth()); EXPECT(Scene.GetSimulation()->GetCloth(0)->GetSettings().Model == ClothModel::DistanceLinks);
	const int OriginalVertices = Scene.GetSimulation()->GetCloth(0)->GetParticleCount();
	auto* Grid = Scene.GetSimulation()->GetCloth(0);
	EXPECT(Grid->GetTopology().GetDistanceLinks().size() > Grid->GetTopology().GetEdges().size());
	EXPECT(Scene.ApplyClothParameters(ClothModel::TriangleStrain, 18, 3, 0.98f, 0.0002f, 0.0004f, 0.005f, 12));
	EXPECT(Grid->GetSettings().Model == ClothModel::TriangleStrain);
	EXPECT(Grid->GetSettings().SolverIterations == 18 && Grid->GetSettings().Substeps == 3);
	EXPECT_NEAR(Grid->GetSettings().Material.StretchUCompliance, 0.0002f, 1e-8f);
	EXPECT_NEAR(Grid->GetSettings().Material.StretchVCompliance, 0.0004f, 1e-8f);
	Scene.Step(1.0f / 60);
	EXPECT(Scene.ApplyClothParameters(ClothModel::DistanceLinks, 12, 1, 0.995f, 0.0001f, 0.0002f, 0.01f, 25));
	Scene.Step(1.0f / 60);
	Scene.SetClothModelDemo(true); EXPECT(Scene.LoadFromFile(Path, &Error)); auto* Cloth = Scene.GetDemoCloth(); EXPECT(Cloth); if (!Cloth) return;
	EXPECT(Cloth->GetTopology().GetDistanceLinks().size() >= Cloth->GetTopology().GetEdges().size());
	EXPECT(Scene.GetObjects().size() == ObjectCount); EXPECT(Cloth->GetParticleCount() < OriginalVertices);
	const auto& Topology = Cloth->GetTopology();
	EXPECT(Cloth->GetParticleCount() - (int)Topology.GetEdges().size() + (int)Topology.GetTriangles().size() == 0);
	std::vector<Vector3> Rest; for (int I = 0; I < Cloth->GetParticleCount(); ++I) Rest.push_back(Cloth->GetParticle(I).Position);
	SceneWorld Still; Still.SetClothModelDemo(true); EXPECT(Still.LoadFromFile(Path, &Error)); EXPECT(Still.ApplyClothDemo(false, false, true, false));
	constexpr float FrameTime = 1.0f / 60; float MaximumStrain = 0;
	for (int Frame = 0; Frame < 120; ++Frame)
	{
		Scene.Step(FrameTime); Still.Step(FrameTime);
		for (int I = 0; I < Cloth->GetParticleCount(); ++I)
		{
			const auto& Particle = Cloth->GetParticle(I); EXPECT(std::isfinite(Particle.Position.Length()) && Particle.Position.Length() < 100);
			if (Particle.Pinned) EXPECT(Particle.Position == Rest[I]);
			else EXPECT((Particle.Position - Vector3(0, 3, -0.75f)).Length() >= 1.3f - 1e-4f);
		}
	}
	float WindDeflection = 0;
	for (int I = 0; I < Cloth->GetParticleCount(); ++I) WindDeflection = std::max(WindDeflection, (Cloth->GetParticle(I).Position - Still.GetDemoCloth()->GetParticle(I).Position).Length());
	for (const auto& Triangle : Topology.GetTriangles())
	{
		Vector3 P[3], G[3]; for (int I = 0; I < 3; ++I) P[I] = Cloth->GetParticle(Triangle.Particles[I]).Position;
		for (int Channel = 0; Channel < 2; ++Channel)
		{
			float Strain; EXPECT(ClothTopology::EvaluateStrain(Triangle, P, Channel, Strain, G)); MaximumStrain = std::max(MaximumStrain, std::abs(Strain));
		}
	}
	printf("Cloth demo vertices=%d triangles=%zu wind deflection=%.6f maximum stretch strain=%.6f\n", Cloth->GetParticleCount(), Topology.GetTriangles().size(), WindDeflection, MaximumStrain);
	EXPECT(WindDeflection > 0.1f); EXPECT(MaximumStrain < 0.2f); EXPECT(Cloth->GetTetherCount() > 0);
	EXPECT(Scene.ApplyClothDemo(false, true, false, true)); EXPECT(Scene.GetClothDemoSoftMaterial());
	for (int Frame = 0; Frame < 120; ++Frame) Scene.Step(FrameTime);
	float PinMotion = 0;
	for (int I = 0; I < Cloth->GetParticleCount(); ++I)
	{
		const auto& P = Cloth->GetParticle(I); EXPECT(std::isfinite(P.Position.Length()) && P.Position.Length() < 100);
		if (P.Pinned) PinMotion = std::max(PinMotion, (P.Position - Rest[I]).Length());
	}
	EXPECT(PinMotion > 0.1f); EXPECT(Cloth->GetSettings().Wind.NormalDrag == 0 && !Cloth->GetSettings().Material.TethersEnabled);
	const auto* Render = Cloth->GetRenderGeometry()->GetShapeObj<TriangleMesh>();
	for (int I = 0; I < Cloth->GetParticleCount(); ++I) EXPECT(Render->Vertices[I] == Cloth->GetParticle(I).Position);
	EXPECT(Scene.GetSimulation()->RemoveCloth(Cloth)); Scene.Step(FrameTime); EXPECT(!Scene.GetDemoCloth());
	EXPECT(Scene.GetObjects().size() + 1 == ObjectCount);
	EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(Scene.GetDemoCloth());
	Cloth = Scene.GetDemoCloth();
	EXPECT(Scene.ApplyClothParameters(ClothModel::DistanceLinks, 14, 2, 0.99f, 0.0003f, 0.0006f, 0.02f, 10));
	EXPECT(Cloth->GetSettings().Model == ClothModel::DistanceLinks);
	EXPECT(Cloth->GetSettings().SolverIterations == 14 && Cloth->GetSettings().Substeps == 2);
	Scene.Step(FrameTime);
	EXPECT(Scene.ApplyClothParameters(ClothModel::TriangleStrain, 16, 4, 0.995f, 0.0001f, 0.0002f, 0.01f, 25));
	EXPECT(Cloth->GetSettings().Model == ClothModel::TriangleStrain);
	Scene.Step(FrameTime);
	Scene.SetClothModelDemo(false); EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(!Scene.GetDemoCloth());
	EXPECT(Scene.GetSimulation()->GetCloth(0)->GetParticleCount() == OriginalVertices);
	Scene.SetClothModelDemo(true); EXPECT(Scene.LoadFromText("{\"name\":\"Other\",\"objects\":[]}", "other", &Error)); EXPECT(!Scene.GetDemoCloth());
	Scene.Reset(); EXPECT(Scene.GetSimulation()->GetClothCount() == 0);
}

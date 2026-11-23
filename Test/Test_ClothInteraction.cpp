#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <stdexcept>
#include <algorithm>
#include <chrono>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.1f, Radius = 0.05f, InitialGap = 0.04f, Tolerance = 2e-5f;
	ClothSettings Settings()
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1; S.Thickness = Radius;
		S.SolverIterations = 20; S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0; return S;
	}
	ClothMesh Triangle(float Z, float Scale = 1, float InverseMass = 1)
	{
		ClothMesh M; M.Positions = { Vector3(-2 * Scale, -2 * Scale, Z), Vector3(2 * Scale, -2 * Scale, Z), Vector3(0, 2 * Scale, Z) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses.assign(3, InverseMass); return M;
	}
	ClothMesh Layers(bool Connected = false)
	{
		auto M = Triangle(0); const auto Upper = Triangle(InitialGap, 1, 2);
		M.Positions.insert(M.Positions.end(), Upper.Positions.begin(), Upper.Positions.end());
		M.InverseMasses.insert(M.InverseMasses.end(), Upper.InverseMasses.begin(), Upper.InverseMasses.end()); M.Triangles.emplace_back(3, 4, 5);
		if (Connected) M.Triangles.emplace_back(1, 0, 3); return M;
	}
	Vector3 MassMoment(const ClothSystem& C)
	{
		Vector3 Result = Vector3::Zero();
		for (int I = 0; I < C.GetParticleCount(); ++I) if (C.GetParticle(I).InverseMass > 0)
			Result += C.GetParticle(I).Position / C.GetParticle(I).InverseMass;
		return Result;
	}
	Vector3 Mean(const ClothSystem& C)
	{
		Vector3 Result = Vector3::Zero(); for (int I = 0; I < C.GetParticleCount(); ++I) Result += C.GetParticle(I).Position;
		return Result / (float)C.GetParticleCount();
	}
	void SelfCollision()
	{
		auto S = Settings(); S.Collision.SelfCollision = true; ClothSystem C(S, Layers()); const Vector3 Before = MassMoment(C);
		C.Simulate(Dt); EXPECT(C.GetCollisionStats().SelfContacts > 0); EXPECT(C.GetCollisionStats().InterContacts == 0);
		EXPECT((MassMoment(C) - Before).Length() < Tolerance);
		for (int I = 0; I < 3; ++I)
		{
			EXPECT_NEAR(C.GetParticle(I + 3).Position.z - C.GetParticle(I).Position.z, 2 * Radius, Tolerance);
			EXPECT(C.GetParticle(I).Position.z < 0 && C.GetParticle(I + 3).Position.z > InitialGap);
		}
		ClothSystem Pinned(S, Layers()); for (int I = 0; I < 3; ++I) Pinned.PinParticle(I, Pinned.GetParticle(I).Position);
		Pinned.Simulate(Dt);
		for (int I = 0; I < 3; ++I) { EXPECT(Pinned.GetParticle(I).Position == Layers().Positions[I]); EXPECT_NEAR(Pinned.GetParticle(I + 3).Position.z, 2 * Radius, Tolerance); }
		S.Rows = S.Columns = 4; S.Width = S.Height = 3;
		ClothSystem Flat(S, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
		std::vector<Vector3> Rest; for (int I = 0; I < Flat.GetParticleCount(); ++I) Rest.push_back(Flat.GetParticle(I).Position);
		Flat.Simulate(Dt); EXPECT(Flat.GetCollisionStats().SelfContacts == 0);
		for (int I = 0; I < Flat.GetParticleCount(); ++I) EXPECT(Flat.GetParticle(I).Position == Rest[I]);
		auto M = Layers(true); M.InverseMasses.assign(M.Positions.size(), 0);
		ClothSystem Excluded(S, M); S.Collision.ExcludeSelfNeighbors = false; ClothSystem Included(S, M);
		Excluded.Simulate(Dt); Included.Simulate(Dt); EXPECT(Included.GetCollisionStats().SelfContacts > Excluded.GetCollisionStats().SelfContacts);
	}
	void InterCollision()
	{
		for (bool Reverse : { false, true })
		{
			auto S = Settings(); S.Collision.InterCollision = true; ClothSystem A(S, Triangle(0)), B(S, Triangle(InitialGap, 1, 2));
			const auto Before = MassMoment(A) + MassMoment(B);
			ClothSystem::SimulateGroup(Reverse ? std::vector<ClothSystem*>{ &B, &A } : std::vector<ClothSystem*>{ &A, &B }, Dt);
			EXPECT(A.GetCollisionStats().InterContacts > 0 && A.GetCollisionStats().InterContacts == B.GetCollisionStats().InterContacts);
			EXPECT((MassMoment(A) + MassMoment(B) - Before).Length() < Tolerance);
			for (int I = 0; I < 3; ++I)
			{
				EXPECT_NEAR(B.GetParticle(I).Position.z - A.GetParticle(I).Position.z, 2 * Radius, Tolerance);
				EXPECT_NEAR(B.GetParticle(I).Position.z - InitialGap, -2 * A.GetParticle(I).Position.z, Tolerance);
			}
		}
		for (bool Enabled : { false, true })
		{
			PhysicsWorld W{ PhysicsWorldParam() }; auto S = Settings(); S.Collision.InterCollision = Enabled;
			auto* A = W.CreateCloth(S, Triangle(0)); S.Collision.InterCollision = true; auto* B = W.CreateCloth(S, Triangle(InitialGap));
			W.Simulate(Dt); EXPECT((A->GetCollisionStats().InterContacts > 0) == Enabled);
			if (!Enabled) { EXPECT_NEAR(A->GetParticle(0).Position.z, 0, Tolerance); EXPECT_NEAR(B->GetParticle(0).Position.z, InitialGap, Tolerance); }
			EXPECT(W.RemoveCloth(A)); W.Simulate(Dt); EXPECT(B->GetCollisionStats().InterContacts == 0);
		}
	}
	void FrictionAndTimeSteps()
	{
		float Slip[2];
		for (int Friction = 0; Friction < 2; ++Friction)
		{
			auto S = Settings(); S.SolverIterations = 1; S.Collision.InterCollision = true;
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 2 * Friction;
			ClothSystem A(S, Triangle(0, 3)), B(S, Triangle(InitialGap, 0.3f, 2));
			const Vector3 Before = MassMoment(A) + MassMoment(B);
			for (int I = 0; I < 3; ++I) { A.SetParticleVelocity(I, Vector3(-0.2f, 0, 0)); B.SetParticleVelocity(I, Vector3(0.4f, 0, 0)); }
			ClothSystem::SimulateGroup({ &A, &B }, Dt); Slip[Friction] = Mean(B).x - Mean(A).x;
			EXPECT((MassMoment(A) + MassMoment(B) - Before).Length() < Tolerance);
		}
		EXPECT_NEAR(Slip[0], 0.06f, Tolerance); EXPECT(std::abs(Slip[1]) < Slip[0] * 0.5f);
		printf("Cloth pair relative slip without/with friction: %g / %g\n", Slip[0], Slip[1]);
		auto S = Settings(); S.Collision.InterCollision = true; S.Gravity = Vector3(0, -1, 0);
		ClothSystem A(S, Triangle(0)); S.Substeps = 4; S.SolverIterations = 3; ClothSystem B(S, Triangle(2));
		S.Substeps = 1; S.Collision.InterCollision = false; ClothSystem Independent(S, Triangle(4));
		for (int I = 0; I < 3; ++I) { A.SetParticleVelocity(I, Vector3::UnitX()); B.SetParticleVelocity(I, Vector3::UnitX()); }
		for (int Step = 0; Step < 3; ++Step) ClothSystem::SimulateGroup({ &A, &B, &Independent }, Dt);
		EXPECT_NEAR(Mean(A).x, 3 * Dt, Tolerance); EXPECT_NEAR(Mean(B).x, 3 * Dt, Tolerance);
		EXPECT_NEAR(Mean(A).y, Mean(B).y, Tolerance); EXPECT(Mean(Independent).y < Mean(A).y - Tolerance);
		const Vector3 Before = Mean(A);
		for (bool Null : { false, true })
		{
			bool Rejected = false; try { ClothSystem::SimulateGroup({ &A, Null ? nullptr : &A }, Dt); } catch (const std::invalid_argument&) { Rejected = true; }
			EXPECT(Rejected && Mean(A) == Before);
		}
	}
	void EdgeContactsAndCompliance()
	{
		for (bool Rotated : { false, true })
		{
			auto MA = Triangle(0), MB = Triangle(InitialGap);
			MA.Positions = { Vector3(-3, -0.1f, 0), Vector3(3, -0.1f, 0), Vector3(3, 0.1f, 0) };
			MB.Positions = { Vector3(-0.1f, -3, InitialGap), Vector3(0.1f, 3, InitialGap), Vector3(-0.1f, 3, InitialGap) };
			Quaternion Q(0, 0, 0, 1); if (Rotated) Q.FromRotationAxis(Vector3(1, 2, 3), 0.7f);
			const Transform Pose(Rotated ? Vector3(3, -2, 1) : Vector3::Zero(), Q);
			for (auto* M : { &MA, &MB }) for (auto& P : M->Positions) P = Pose.LocalToWorld(P);
			// All vertices are far from the opposite face. Only crossing edges can
			// generate the initial contact, even after a common rigid transformation.
			for (int I = 0; I < 3; ++I)
			{
				EXPECT(SurfaceCollision::Closest(&MA.Positions[I], 1, MB.Positions.data(), 3).Distance > 2 * Radius);
				EXPECT(SurfaceCollision::Closest(MA.Positions.data(), 3, &MB.Positions[I], 1).Distance > 2 * Radius);
			}
			auto S = Settings(); S.Collision.InterCollision = true; ClothSystem A(S, MA), B(S, MB);
			const auto Before = MassMoment(A) + MassMoment(B); ClothSystem::SimulateGroup({ &A, &B }, Dt);
			Vector3 PA[3], PB[3]; for (int I = 0; I < 3; ++I) { PA[I] = A.GetParticle(I).Position; PB[I] = B.GetParticle(I).Position; }
			EXPECT(A.GetCollisionStats().InterContacts > 0); EXPECT((MassMoment(A) + MassMoment(B) - Before).Length() < 4 * Tolerance);
			EXPECT(SurfaceCollision::Closest(PA, 3, PB, 3).Distance >= 2 * Radius - Tolerance);
		}
		for (int Iterations : { 1, 30 }) for (float Step : { Dt, Dt / 2 })
		{
			auto S = Settings(); S.SolverIterations = Iterations; S.Collision.InterCollision = true;
			constexpr float Compliance = 0.02f, InverseMass = 2;
			S.Collision.Material.Compliance = Compliance; ClothSystem Fixed(S, Triangle(0, 3, 0));
			S.Collision.Material.Compliance = Compliance / 2; ClothSystem Free(S, Triangle(InitialGap, 0.3f, InverseMass));
			ClothSystem::SimulateGroup({ &Fixed, &Free }, Step);
			const float Expected = InitialGap + (2 * Radius - InitialGap) * InverseMass / (InverseMass + Compliance / (Step * Step));
			for (int I = 0; I < 3; ++I)
			{
				EXPECT(Fixed.GetParticle(I).Position == Triangle(0, 3, 0).Positions[I]);
				EXPECT_NEAR(Free.GetParticle(I).Position.z, Expected, Tolerance);
			}
		}
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error; const auto Path = TestContentsPath() + "Scenes/08_cloth_sphere.json";
		EXPECT(Scene.LoadFromFile(Path, &Error)); const size_t Count = Scene.GetObjects().size();
		auto* Primary = Scene.GetDemoCloth(); EXPECT(Primary); if (!Primary) return;
		EXPECT(Scene.SetClothSelfCollision(true)); EXPECT(Primary->GetSettings().Collision.SelfCollision);
		EXPECT(Scene.SetClothSelfCollision(false)); EXPECT(!Primary->GetSettings().Collision.SelfCollision);
		EXPECT(Scene.ApplyClothDemo(true, false, true, false, true, true, true, true));
		auto* A = Scene.GetDemoCloth(); auto* B = Scene.GetDemoClothPartner(); EXPECT(A && B); if (!A || !B) return;
		EXPECT(Scene.SetClothSelfCollision(false));
		EXPECT(!A->GetSettings().Collision.SelfCollision && !B->GetSettings().Collision.SelfCollision);
		EXPECT(Scene.SetClothSelfCollision(true));
		EXPECT(A->GetSettings().Collision.SelfCollision && B->GetSettings().Collision.SelfCollision);
		EXPECT(Scene.GetSimulation()->GetClothCount() == 2 && Scene.GetObjects().size() == Count + 1);
		size_t Contacts = 0;
		for (int Frame = 0; Frame < 24; ++Frame)
		{
			Scene.Step(1.0f / 60); Contacts += A->GetCollisionStats().InterContacts;
			for (auto* C : { A, B }) for (int I = 0; I < C->GetParticleCount(); ++I)
			{
				const auto& P = C->GetParticle(I); EXPECT(std::isfinite(P.Position.Length()) && P.Position.Length() < 100);
				if (P.Pinned) EXPECT(P.Position == P.PinPosition);
				EXPECT(C->GetRenderGeometry()->GetShapeObj<TriangleMesh>()->Vertices[I] == P.Position);
			}
		}
		printf("Two-layer cloth demo shared contacts: %zu\n", Contacts); EXPECT(Contacts > 0);
		EXPECT(Scene.ApplyClothDemo(true, true, true, false, true, false, false, false)); Scene.Step(1.0f / 60);
		EXPECT(Scene.GetDemoClothPartner() == B && Scene.GetObjects().size() == Count + 1);
		EXPECT(A->GetCollisionStats().InterContacts == 0 && B->GetCollisionStats().InterContacts == 0);
		EXPECT(!B->GetSettings().Collision.SelfCollision && B->GetSettings().Collision.Material.StaticFriction == 0);
		EXPECT(Scene.GetSimulation()->RemoveCloth(B)); Scene.Step(1.0f / 60);
		EXPECT(!Scene.GetDemoClothPartner() && Scene.GetObjects().size() == Count);
		EXPECT(Scene.ApplyClothDemo(true, false, true, false, true, true, false, true));
		EXPECT(Scene.GetSimulation()->RemoveCloth(A)); Scene.Step(1.0f / 60);
		EXPECT(!Scene.GetDemoCloth() && !Scene.GetDemoClothPartner()); EXPECT(Scene.GetObjects().size() + 1 == Count);
		EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(Scene.GetSimulation()->GetClothCount() == 1); EXPECT(!Scene.GetDemoClothPartner());
	}
}
void TestClothInteraction()
{
	SelfCollision(); InterCollision(); FrictionAndTimeSteps(); EdgeContactsAndCompliance(); ExistingDemo();
}

void TestClothSelfCollisionBenchmark()
{
	SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
	EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
	auto* Cloth = Scene.GetDemoCloth(); EXPECT(Cloth); if (!Cloth) return;
	EXPECT(Scene.SetClothSelfCollision(true));
	RigidBody* Ball = nullptr;
	for (const auto& Object : Scene.GetObjects()) if (Object.Id == "cloth_ball") Ball = Object.GeometryPtr->GetParent<RigidBody>();
	EXPECT(Ball); if (!Ball) return;
	Ball->V = Vector3(0, 0, -6);
	std::vector<double> Times; size_t Candidates = 0, SelfContacts = 0, RigidContacts = 0;
	for (int Frame = 0; Frame < 120; ++Frame)
	{
		if (Frame == 30) Ball->V = Vector3(0, 0, 8);
		const auto Start = std::chrono::steady_clock::now(); Scene.Step(1.0f / 60);
		Times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count());
		const auto& Stats = Cloth->GetCollisionStats(); Candidates += Stats.Candidates;
		SelfContacts += Stats.SelfContacts; RigidContacts += Stats.RigidContacts;
		for (int I = 0; I < Cloth->GetParticleCount(); ++I) EXPECT(std::isfinite(Cloth->GetParticle(I).Position.Length()));
	}
	double Total = 0; for (double Time : Times) Total += Time; std::sort(Times.begin(), Times.end());
	printf("Cloth self collision: mean %.3f ms, p95 %.3f ms, candidates %zu, self %zu, rigid %zu\n",
		Total / Times.size(), Times[Times.size() * 95 / 100], Candidates, SelfContacts, RigidContacts);
	EXPECT(RigidContacts > 0);
}

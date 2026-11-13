#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/SurfaceCollision.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.1f, Radius = 0.05f, Tolerance = 5e-5f;
	ClothSettings Settings(bool Continuous = true)
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1;
		S.Thickness = Radius; S.SolverIterations = 8;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		S.Collision.SurfaceContacts = true; S.Collision.Continuous = Continuous;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0;
		return S;
	}
	ClothMesh Triangle(float Z, float Scale = 1)
	{
		ClothMesh M; M.Positions = { Vector3(-2 * Scale, -2 * Scale, Z), Vector3(2 * Scale, -2 * Scale, Z), Vector3(0, 2 * Scale, Z) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses = { 1, 1, 1 }; return M;
	}
	Geometry* Plane(int Axis = 2)
	{
		auto* G = GeometryFactory::CreateTriangleMesh(); auto* M = G->GetShapeObj<TriangleMesh>();
		for (auto P : Triangle(0, 10).Positions) { if (Axis == 0) std::swap(P.x, P.z); M->AddVertex(P); }
		M->AddTriangle(0, 1, 2); M->Compact(); G->SetShapeBounds(M->BoundingVolume); return G;
	}
	RigidBody* Body(PhysicsWorld& W, Geometry* G)
	{
		RigidBodyParam P; P.rigidType = RigidType::Static; return W.CreateRigidBody(G, P);
	}
	void Velocity(ClothSystem& C, const Vector3& V)
	{
		for (int I = 0; I < C.GetParticleCount(); ++I) EXPECT(C.SetParticleVelocity(I, V));
	}
	Vector3 Center(const ClothSystem& C)
	{
		Vector3 Sum = Vector3::Zero(); for (int I = 0; I < C.GetParticleCount(); ++I) Sum += C.GetParticle(I).Position;
		return Sum / (float)C.GetParticleCount();
	}
	double ClothGap(const ClothSystem& A, const ClothSystem& B)
	{
		std::vector<Index3> Triangles; for (const auto& T : B.GetTopology().GetTriangles()) Triangles.push_back(T.Particles);
		std::vector<Vector3> Positions; for (int I = 0; I < B.GetParticleCount(); ++I) Positions.push_back(B.GetParticle(I).Position);
		SurfaceCollisionMesh Index(B.GetParticleCount(), Triangles); Index.Update(Positions, Positions, 0);
		const float Thickness = A.GetSettings().Thickness + B.GetSettings().Thickness;
		double Gap = std::numeric_limits<double>::infinity(); std::vector<int> Hits;
		for (const auto& T : A.GetTopology().GetTriangles())
		{
			Vector3 PA[3]; for (int I = 0; I < 3; ++I) PA[I] = A.GetParticle(T.Particles[I]).Position;
			Box3 Bounds(PA, 3); Bounds.Min -= Vector3(Thickness); Bounds.Max += Vector3(Thickness); Index.QueryTriangles(Bounds, Hits);
			for (int J : Hits)
			{
				Vector3 PB[3]; for (int I = 0; I < 3; ++I) PB[I] = Positions[Triangles[J][I]];
				Gap = std::min(Gap, SurfaceCollision::Closest(PA, 3, PB, 3).Distance - Thickness);
			}
		}
		return Gap;
	}
	void MeshAndConvexCrossing()
	{
		for (int Shape = 0; Shape < 3; ++Shape) for (bool Continuous : { false, true })
		{
			std::unique_ptr<Geometry> G(Shape == 0 ? Plane() : Shape == 1
				? GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(4, 4, 0.5f)) : GeometryFactory::CreateConvexMesh());
			if (Shape == 2)
			{
				std::vector<Vector3> Points; for (int I = 0; I < 8; ++I) Points.emplace_back(I & 1 ? 4.0f : -4.0f, I & 2 ? 4.0f : -4.0f, I & 4 ? 0.5f : -0.5f);
				auto* M = G->GetShapeObj<ConvexMesh>(); EXPECT(M->BuildFromPoints(Points.data(), (int)Points.size())); G->SetShapeBounds(M->ComputeBoundingVolume());
			}
			ClothSystem C(Settings(Continuous), Triangle(2)); EXPECT(C.AddCollider(G.get())); Velocity(C, Vector3(0, 0, -40)); C.Simulate(Dt);
			const float Height = Radius + (Shape ? 0.5f : 0);
			for (int I = 0; I < 3; ++I) if (Continuous) EXPECT(C.GetParticle(I).Position.z >= Height - Tolerance); else EXPECT_NEAR(C.GetParticle(I).Position.z, -2, Tolerance);
			EXPECT(Continuous ? C.GetCollisionStats().ContinuousContacts > 0 : C.GetCollisionStats().Contacts == 0);
			EXPECT(!C.GetCollisionStats().ProjectionLimitReached);
		}
		// Ending inside a convex must select the entry face, not its nearest exit.
		std::unique_ptr<Geometry> G(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(4, 4, 0.5f)));
		ClothSystem C(Settings(), Triangle(2)); EXPECT(C.AddCollider(G.get())); Velocity(C, Vector3(0, 0, -22.5f)); C.Simulate(Dt);
		for (int I = 0; I < 3; ++I) EXPECT(C.GetParticle(I).Position.z >= 0.55f - Tolerance);
		ClothSystem Embedded(Settings(), Triangle(0)); EXPECT(Embedded.AddCollider(G.get())); Embedded.Simulate(Dt);
		for (int I = 0; I < 3; ++I) EXPECT(std::abs(Embedded.GetParticle(I).Position.z) >= 0.55f - Tolerance);
		ClothSystem Escaping(Settings(), Triangle(0)); EXPECT(Escaping.AddCollider(G.get())); Velocity(Escaping, Vector3(0, 0, 20)); Escaping.Simulate(Dt);
		for (int I = 0; I < 3; ++I) EXPECT_NEAR(Escaping.GetParticle(I).Position.z, 2, Tolerance);
	}
	void RoundedAndMovingColliders()
	{
		for (bool Capsule : { false, true })
		{
			ClothSystem C(Settings(), Triangle(2)); Vector3 Ends[] = { Vector3(-1, 0, 0), Vector3(1, 0, 0) };
			if (!Capsule) Ends[0] = Ends[1] = Vector3::Zero();
			EXPECT(C.AddCapsuleCollider(nullptr, 0.2f, Ends[0], Ends[1])); Velocity(C, Vector3(0, 0, -40)); C.Simulate(Dt);
			Vector3 Points[3]; for (int I = 0; I < 3; ++I) Points[I] = C.GetParticle(I).Position;
			const auto Contact = SurfaceCollision::Closest(Points, 3, Ends, Capsule ? 2 : 1);
			printf("Cloth swept rounded capsule=%d distance=%g passes=%d\n", Capsule, Contact.Distance, C.GetCollisionStats().ContinuousPasses);
			EXPECT(Contact.Distance >= 0.25 - Tolerance); EXPECT(Contact.Normal.z > 0);
		}
		PhysicsWorld W{ PhysicsWorldParam() }; auto* G = Plane(); auto* B = Body(W, G); B->X.z = -2; B->UpdateGeometries();
		ClothSystem C(Settings(), Triangle(0)); EXPECT(C.AddCollider(G)); B->X.z = 2; B->UpdateGeometries(); C.Simulate(Dt);
		for (int I = 0; I < 3; ++I) EXPECT(C.GetParticle(I).Position.z >= 2 + Radius - Tolerance);
		EXPECT(C.GetCollisionStats().ContinuousContacts > 0);
	}
	void FrictionAndMaterialProjection()
	{
		std::unique_ptr<Geometry> G(Plane());
		for (bool Friction : { false, true })
		{
			auto S = Settings(); S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = Friction ? 2.0f : 0;
			S.Collision.Material.Compliance = 10; // Continuous barriers stay hard even with a soft discrete material.
			const auto M = Triangle(2); ClothSystem C(S, M); EXPECT(C.AddCollider(G.get())); Velocity(C, Vector3(10, 0, -40)); C.Simulate(Dt);
			for (int I = 0; I < 3; ++I)
			{
				EXPECT_NEAR(C.GetParticle(I).Position.z, Radius, Tolerance);
				EXPECT_NEAR(C.GetParticle(I).Position.x - M.Positions[I].x, Friction ? (2 - Radius) / 4 : 1, Tolerance);
			}
		}
		auto S = Settings(); S.AnimationDrive.Enabled = true; S.AnimationDrive.Compliance = 0; S.AnimationDrive.Damping = 0;
		ClothSystem Driven(S, Triangle(2)); EXPECT(Driven.AddCollider(G.get())); Driven.SetAnimationPose(Triangle(-2).Positions); Driven.Simulate(Dt);
		EXPECT(Driven.GetCollisionStats().ContinuousContacts > 0);
		for (int I = 0; I < 3; ++I) EXPECT(Driven.GetParticle(I).Position.z >= Radius - Tolerance);
	}
	void SurfaceInteractions()
	{
		for (bool Self : { false, true }) for (bool Reverse : { false, true })
		{
			auto S = Settings(); S.Collision.SurfaceContacts = false; S.Collision.SelfCollision = Self; S.Collision.InterCollision = !Self;
			auto Lower = Triangle(-1), Upper = Triangle(1); Upper.InverseMasses = { 2, 2, 2 };
			if (Self)
			{
				Lower.Positions.insert(Lower.Positions.end(), Upper.Positions.begin(), Upper.Positions.end()); Lower.InverseMasses.insert(Lower.InverseMasses.end(), Upper.InverseMasses.begin(), Upper.InverseMasses.end()); Lower.Triangles.emplace_back(3, 4, 5);
				ClothSystem C(S, Lower);
				for (int I = 0; I < 6; ++I) EXPECT(C.SetParticleVelocity(I, Vector3(0, 0, I < 3 ? 20.0f : -40.0f)));
				C.Simulate(Dt); EXPECT(C.GetCollisionStats().SelfContacts > 0);
				for (int I = 0; I < 3; ++I)
				{
					const float A = C.GetParticle(I).Position.z, B = C.GetParticle(I + 3).Position.z;
					EXPECT(B - A >= 2 * Radius - Tolerance); EXPECT_NEAR(A + B / 2, -0.5f, Tolerance);
				}
			}
			else
			{
				ClothSystem A(S, Lower); S.Collision.Continuous = false; ClothSystem B(S, Upper);
				Velocity(A, Vector3(0, 0, 20)); Velocity(B, Vector3(0, 0, -40));
				ClothSystem::SimulateGroup(Reverse ? std::vector<ClothSystem*>{ &B, &A } : std::vector<ClothSystem*>{ &A, &B }, Dt);
				EXPECT(A.GetCollisionStats().InterContacts > 0 && B.GetCollisionStats().ContinuousContacts > 0);
				for (int I = 0; I < 3; ++I)
				{
					EXPECT(B.GetParticle(I).Position.z - A.GetParticle(I).Position.z >= 2 * Radius - Tolerance);
					EXPECT_NEAR(A.GetParticle(I).Position.z + B.GetParticle(I).Position.z / 2, -0.5f, Tolerance);
				}
			}
		}
	}
	void RotationAndBudgets()
	{
		for (bool Continuous : { false, true })
		{
			PhysicsWorld W{ PhysicsWorldParam() }; auto* B = Body(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.1f));
			auto S = Settings(Continuous); auto M = Triangle(0, 0.15f);
			const Vector3 Radial = Vector3(1, 0, -1).Unit(), Up = Vector3::UnitY();
			for (auto& P : M.Positions) P = Radial * (2.5f + P.x) + Up * P.y;
			ClothSystem C(S, M); const auto Before = Center(C);
			EXPECT(C.AddCapsuleCollider(B, 0.15f, Vector3(2, 0, 0), Vector3(3, 0, 0)));
			B->Q.FromRotationAxis(Vector3::UnitY(), PI / 2); B->UpdateGeometries();
			if (Continuous)
			{
				auto Collision = C.GetSettings().Collision; Collision.MaximumAngularSubsteps = 1; C.SetCollisionSettings(Collision);
				bool Failed = false; try { C.Simulate(Dt); } catch (const std::runtime_error&) { Failed = true; } EXPECT(Failed); EXPECT(Center(C) == Before);
				Collision.MaximumAngularSubsteps = 64; C.SetCollisionSettings(Collision);
			}
			C.Simulate(Dt);
			printf("Cloth rotating capsule continuous=%d contacts=%zu displacement=%g\n", Continuous, C.GetCollisionStats().Contacts, (Center(C) - Before).Length());
			if (Continuous) { EXPECT(C.GetCollisionStats().ContinuousContacts > 0); EXPECT((Center(C) - Before).Length() > 0.1f); }
			else EXPECT(Center(C) == Before);
		}
		std::unique_ptr<Geometry> G(Plane()); auto S = Settings(); S.Collision.SweepIterations = 1; S.Collision.SweepRefinements = 0; S.Collision.MaximumContactRefinements = 0;
		ClothSystem C(S, Triangle(2)); EXPECT(C.AddCollider(G.get())); Velocity(C, Vector3(0, 0, -40));
		bool Failed = false; try { C.Simulate(Dt); } catch (const std::runtime_error&) { Failed = true; }
		EXPECT(Failed && C.GetCollisionStats().UnresolvedSweeps == 1); EXPECT(C.GetCollisionStats().Contacts == 0);
		for (int I = 0; I < 3; ++I) { EXPECT(C.GetParticle(I).Position == Triangle(2).Positions[I]); EXPECT_NEAR(C.GetParticleVelocity(I).z, -40, Tolerance); }
		auto Collision = C.GetSettings().Collision; Collision.SweepRefinements = 64; C.SetCollisionSettings(Collision); C.Simulate(Dt);
		for (int I = 0; I < 3; ++I) EXPECT_NEAR(C.GetParticle(I).Position.z, Radius, Tolerance);
		for (int Invalid = 0; Invalid < 7; ++Invalid)
		{
			auto Bad = Collision;
			if (Invalid == 0) Bad.SweepIterations = 0; else if (Invalid == 1) Bad.SweepRefinements = -1;
			else if (Invalid == 2) Bad.ContinuousIterations = 0; else if (Invalid == 3) Bad.MaximumColliderAngle = 0;
			else if (Invalid == 4) Bad.MaximumAngularSubsteps = 0;
			else if (Invalid == 5) Bad.MaximumContactRefinements = -1; else Bad.MaximumContactRefinementDepth = 0;
			bool Rejected = false; try { C.SetCollisionSettings(Bad); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		}
	}
	void EdgesAndMultipleContacts()
	{
		for (bool Rotated : { false, true })
		{
			auto A = Triangle(1), B = Triangle(-1);
			A.Positions = { Vector3(-3, -0.1f, 1), Vector3(3, -0.1f, 1), Vector3(3, 0.1f, 1) };
			B.Positions = { Vector3(-0.1f, -3, -1), Vector3(0.1f, 3, -1), Vector3(-0.1f, 3, -1) };
			Quaternion Q(0, 0, 0, 1); if (Rotated) Q.FromRotationAxis(Vector3(1, 2, 3), 0.7f);
			const Transform Pose(Rotated ? Vector3(3, -2, 1) : Vector3::Zero(), Q);
			for (auto* M : { &A, &B }) for (auto& P : M->Positions) P = Pose.LocalToWorld(P);
			auto S = Settings(); S.Collision.SurfaceContacts = false; S.Collision.InterCollision = true;
			ClothSystem CA(S, A), CB(S, B); const Vector3 Before = Center(CA) + Center(CB);
			Velocity(CA, Q * Vector3(0, 0, -20)); Velocity(CB, Q * Vector3(0, 0, 20)); ClothSystem::SimulateGroup({ &CA, &CB }, Dt);
			Vector3 PA[3], PB[3]; for (int I = 0; I < 3; ++I) { PA[I] = CA.GetParticle(I).Position; PB[I] = CB.GetParticle(I).Position; }
			const auto Contact = SurfaceCollision::Closest(PA, 3, PB, 3);
			EXPECT(CA.GetCollisionStats().ContinuousContacts > 0); EXPECT(Contact.Distance >= 2 * Radius - Tolerance);
			EXPECT(Contact.Normal.Dot(Q * Vector3::UnitZ()) > 0); EXPECT((Center(CA) + Center(CB) - Before).Length() < Tolerance);
		}
		std::unique_ptr<Geometry> X(Plane(0)), Z(Plane()); auto M = Triangle(2, 0.2f); for (auto& P : M.Positions) P.x += 2;
		auto S = Settings(); S.SolverIterations = 1; ClothSystem Corner(S, M); EXPECT(Corner.AddCollider(X.get()) && Corner.AddCollider(Z.get()));
		Velocity(Corner, Vector3(-40, 0, -40)); Corner.Simulate(Dt);
		for (int I = 0; I < 3; ++I) { EXPECT(Corner.GetParticle(I).Position.x >= Radius - Tolerance); EXPECT(Corner.GetParticle(I).Position.z >= Radius - Tolerance); }
		EXPECT(Corner.GetCollisionStats().ContinuousContacts >= 6);
	}
	void CoupledFailureRetry()
	{
		for (bool Adaptive : { false, true })
		{
			std::unique_ptr<Geometry> G(Plane()); auto S = Settings(); S.Collision.InterCollision = true;
			S.Collision.SweepIterations = Adaptive ? 4 : 1; S.Collision.SweepRefinements = 0; S.Collision.MaximumContactRefinements = 0;
			ClothSystem A(S, Triangle(2)); EXPECT(A.AddCollider(G.get()));
			S.Collision.Continuous = false; S.AnimationDrive.Enabled = true; S.AnimationDrive.Compliance = 0;
			auto M = Triangle(-5); for (auto& P : M.Positions) P.x += 50; ClothSystem B(S, M); B.PinParticle(0, M.Positions[0]);
			auto Target = M.Positions; for (auto& P : Target) P.x += 1; B.SetAnimationPose(Target, 2 * Dt);
			Velocity(A, Vector3(0, 0, -40)); const auto Animation = B.GetAnimationPose();
			bool Failed = false; try { ClothSystem::SimulateGroup({ &A, &B }, Dt); } catch (const std::runtime_error&) { Failed = true; } EXPECT(Failed);
			EXPECT(B.GetAnimationPose() == Animation);
			for (int I = 0; I < 3; ++I) EXPECT(B.GetParticle(I).Position == M.Positions[I]);
			EXPECT(B.GetParticle(0).PinPosition == M.Positions[0]);
			for (int I = 0; I < 3; ++I) { EXPECT(A.GetParticle(I).Position == Triangle(2).Positions[I]); EXPECT_NEAR(A.GetParticleVelocity(I).z, -40, Tolerance); }
			auto Collision = A.GetSettings().Collision;
			if (Adaptive) Collision.MaximumContactRefinements = 64; else Collision.SweepRefinements = 64;
			A.SetCollisionSettings(Collision);
			ClothSystem::SimulateGroup({ &A, &B }, Dt);
			EXPECT_NEAR(B.GetParticle(0).Position.x, M.Positions[0].x + 0.5f, Tolerance);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(A.GetParticle(I).Position.z, Radius, Tolerance);
			if (Adaptive)
			{
				printf("Cloth automatic sweep refinements=%d\n", A.GetCollisionStats().ContactRefinements);
				EXPECT(A.GetCollisionStats().ContactRefinements > 0); EXPECT(A.GetCollisionStats().UnresolvedSweeps > 0);
				EXPECT(A.GetCollisionStats().ContactRefinements == B.GetCollisionStats().ContactRefinements);
			}
		}
	}
	void ImmovableContactRollback()
	{
		std::unique_ptr<Geometry> G(Plane()); auto S = Settings();
		S.AnimationDrive.Enabled = true; S.Collision.MaximumContactRefinementDepth = 2;
		const auto M = Triangle(2); ClothSystem C(S, M); EXPECT(C.AddCollider(G.get()));
		for (int I = 0; I < 3; ++I) EXPECT(C.PinParticle(I, M.Positions[I]));
		C.SetAnimationPose(Triangle(-2).Positions, Dt); const auto Animation = C.GetAnimationPose();
		bool Failed = false; try { C.Simulate(Dt); } catch (const std::runtime_error&) { Failed = true; }
		EXPECT(Failed); EXPECT(C.GetCollisionStats().ProjectionLimitReached);
		EXPECT(C.GetCollisionStats().MaximumConstraintViolation > S.Collision.ContactTolerance);
		EXPECT(C.GetCollisionStats().ContactRefinements > 0 && C.GetCollisionStats().ContactRefinements < S.Collision.MaximumContactRefinements);
		EXPECT(C.GetAnimationPose() == Animation);
		for (int I = 0; I < 3; ++I) { EXPECT(C.GetParticle(I).Position == M.Positions[I]); EXPECT(C.GetParticle(I).PinPosition == M.Positions[I]); }
		C.SetAnimationPose(M.Positions, Dt); C.Simulate(Dt);
		EXPECT(!C.GetCollisionStats().ProjectionLimitReached); EXPECT(C.GetCollisionStats().ContactRefinements == 0);
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error; EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
		EXPECT(Scene.ApplyClothDemo(true, false, true, false, true, true, true, true, true));
		auto* C = Scene.GetDemoCloth(); auto* Partner = Scene.GetDemoClothPartner(); EXPECT(C && Partner); if (!C || !Partner) return;
		int Frame = 0;
		auto Step = [&]()
		{
			try { Scene.Step(1.0f / 60); ++Frame; }
			catch (...)
			{
				const auto& Stats = C->GetCollisionStats();
				printf("Continuous demo rejected frame=%d refinements=%d sweeps=%zu residual=%g penetration=%g\n", Frame,
					Stats.ContactRefinements, Stats.UnresolvedSweeps, Stats.MaximumConstraintViolation, Stats.MaximumSurfacePenetration); throw;
			}
		};
		for (int I = 0; I < 8; ++I) Step();
		RigidBody* Ball = nullptr; Geometry* BallShape = nullptr;
		for (const auto& Object : Scene.GetObjects()) if (Object.Id == "cloth_ball" && Object.GeometryPtr) { BallShape = Object.GeometryPtr; Ball = BallShape->GetParent<RigidBody>(); }
		EXPECT(Ball); if (!Ball) return;
		Ball->X.z = -4; Ball->UpdateGeometries(); Step();
		Ball->X.z = 4; Ball->UpdateGeometries(); Step();
		printf("Continuous two-layer demo contacts=%zu passes=%d substeps=%d limit=%d\n", C->GetCollisionStats().ContinuousContacts,
			C->GetCollisionStats().ContinuousPasses, C->GetCollisionStats().Substeps, C->GetCollisionStats().ProjectionLimitReached);
		EXPECT(C->GetCollisionStats().ContinuousContacts > 0);
		const auto* Sphere = BallShape->GetShapeObj<Sphere3>(); const Vector3 BallCenter = Ball->GetGeometryTransform(BallShape).LocalToWorld(Sphere->Center);
		// Check actual triangles independently of row residuals and iteration flags.
		double MinimumGap = std::numeric_limits<double>::infinity();
		for (auto* Cloth : { C, Partner }) for (const auto& Triangle : Cloth->GetTopology().GetTriangles())
		{
			Vector3 P[3]; for (int I = 0; I < 3; ++I) P[I] = Cloth->GetParticle(Triangle.Particles[I]).Position;
			MinimumGap = std::min(MinimumGap, SurfaceCollision::Closest(P, 3, &BallCenter, 1).Distance - Sphere->Radius - Cloth->GetSettings().Thickness);
		}
		printf("Continuous demo minimum sphere surface gap=%g\n", MinimumGap);
		EXPECT(MinimumGap >= -Tolerance);
		const double InterGap = ClothGap(*C, *Partner); printf("Continuous demo minimum cloth surface gap=%g\n", InterGap);
		EXPECT(InterGap >= -Tolerance);
		printf("Continuous demo refinements=%d residual=%g penetration=%g\n", C->GetCollisionStats().ContactRefinements,
			C->GetCollisionStats().MaximumConstraintViolation, C->GetCollisionStats().MaximumSurfacePenetration);
		EXPECT(C->GetSettings().Collision.Continuous && Partner->GetSettings().Collision.Continuous);
		for (auto* Cloth : { C, Partner }) for (int I = 0; I < Cloth->GetParticleCount(); ++I)
		{
			const auto& P = Cloth->GetParticle(I); EXPECT(std::isfinite(P.Position.SquareLength())); if (P.Pinned) EXPECT(P.Position == P.PinPosition);
		}
		EXPECT(Scene.ApplyClothDemo(true, false, true, false, true, true, true, true, false)); EXPECT(!C->GetSettings().Collision.Continuous); Scene.Step(1.0f / 60);
	}
}
void TestClothContinuous()
{
	auto Run = [](const char* Name, void(*Test)())
	{
		try { Test(); }
		catch (const std::exception& Error) { printf("ClothContinuous %s: %s\n", Name, Error.what()); EXPECT(false); }
	};
	Run("MeshAndConvexCrossing", MeshAndConvexCrossing); Run("RoundedAndMovingColliders", RoundedAndMovingColliders);
	Run("FrictionAndMaterialProjection", FrictionAndMaterialProjection); Run("SurfaceInteractions", SurfaceInteractions);
	Run("RotationAndBudgets", RotationAndBudgets); Run("EdgesAndMultipleContacts", EdgesAndMultipleContacts);
	Run("CoupledFailureRetry", CoupledFailureRetry); Run("ImmovableContactRollback", ImmovableContactRollback); Run("ExistingDemo", ExistingDemo);
}

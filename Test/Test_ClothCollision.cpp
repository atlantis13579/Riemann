#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.1f;
	ClothSettings Settings()
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1; S.Thickness = 0.05f;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		S.Collision.SurfaceContacts = true; S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0;
		return S;
	}
	ClothMesh Triangle(float Height, float Scale = 1)
	{
		ClothMesh M; M.Positions = { Vector3(-2 * Scale, -2 * Scale, Height), Vector3(2 * Scale, -2 * Scale, Height), Vector3(0, 2 * Scale, Height) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses = { 1, 1, 1 }; return M;
	}
	Geometry* PlaneMesh()
	{
		auto* Shape = GeometryFactory::CreateTriangleMesh(); auto* Mesh = Shape->GetShapeObj<TriangleMesh>();
		for (const auto& P : Triangle(0, 10).Positions) Mesh->AddVertex(P);
		Mesh->AddTriangle(0, 1, 2); Mesh->Compact(); Shape->SetShapeBounds(Mesh->BoundingVolume); return Shape;
	}
	RigidBody* StaticBody(PhysicsWorld& World, Geometry* Shape)
	{
		RigidBodyParam P; P.rigidType = RigidType::Static; return World.CreateRigidBody(Shape, P);
	}
	void FaceCapsuleResponse()
	{
		for (int Iterations : { 1, 20 })
		{
			auto S = Settings(); S.SolverIterations = Iterations; auto M = Triangle(0.1f); M.InverseMasses = { 1, 2, 4 };
			ClothSystem C(S, M); EXPECT(C.AddCapsuleCollider(nullptr, 0.3f, Vector3(0, 0, -1), Vector3::Zero()));
			C.Simulate(Dt); EXPECT(C.GetCollisionStats().Contacts >= 1);
			const float Weights[] = { 0.25f, 0.25f, 0.5f }, EffectiveMass = 1.1875f, Gap = 0.25f;
			for (int I = 0; I < 3; ++I)
			{
				if (Iterations == 1)
				{
					// The first linearization has an analytic mass-weighted correction.
					EXPECT_NEAR(C.GetParticle(I).Position.z, M.Positions[I].z + Gap * M.InverseMasses[I] * Weights[I] / EffectiveMass, 2e-6f);
					EXPECT_NEAR(C.GetParticle(I).Position.x, M.Positions[I].x, 1e-6f);
				}
			}
			if (Iterations > 1)
			{
				Vector3 P[3], Ends[] = { Vector3(0, 0, -1), Vector3::Zero() }; for (int I = 0; I < 3; ++I) P[I] = C.GetParticle(I).Position;
				EXPECT(SurfaceCollision::Closest(P, 3, Ends, 2).Distance > 0.3499);
			}
			const auto* Render = C.GetRenderGeometry()->GetShapeObj<TriangleMesh>();
			for (int I = 0; I < 3; ++I) EXPECT(Render->Vertices[I] == C.GetParticle(I).Position);
		}
		auto S = Settings(); auto M = Triangle(0.1f); ClothSystem C(S, M); C.PinParticle(2, M.Positions[2]);
		EXPECT(C.AddCapsuleCollider(nullptr, 0.3f, Vector3(0, 0, -1), Vector3::Zero())); C.Simulate(Dt);
		EXPECT(C.GetParticle(2).Position == M.Positions[2]); EXPECT(C.GetParticle(0).Position.z > M.Positions[0].z);
		for (float Length : { 1.0f, 10.0f })
		{
			auto M = Triangle(0.1f); M.InverseMasses = { 1, 2, 4 }; auto S = Settings(); S.SolverIterations = 30;
			ClothSystem Parallel(S, M); Vector3 Ends[] = { Vector3(-Length, 0, 0), Vector3(Length, 0, 0) };
			EXPECT(Parallel.AddCapsuleCollider(nullptr, 0.3f, Ends[0], Ends[1])); Parallel.Simulate(Dt);
			Vector3 P[3]; for (int I = 0; I < 3; ++I) P[I] = Parallel.GetParticle(I).Position;
			EXPECT(SurfaceCollision::Closest(P, 3, Ends, 2).Distance > 0.3499);
		}
	}
	void ShapeSnapshotsAndFrames()
	{
		Quaternion Q; Q.FromRotationAxis(Vector3(1, 2, 3), 0.7f); const Transform Pose(Vector3(3, -1, 2), Q);
		for (bool Capsule : { false, true })
		{
			PhysicsWorld World{ PhysicsWorldParam() }; auto* Shape = Capsule
				? GeometryFactory::CreateCapsule(Vector3(0, 0, -1), Vector3::Zero(), 0.3f) : GeometryFactory::CreateSphere(Vector3::Zero(), 0.3f);
			auto* Body = StaticBody(World, Shape); Body->X = Pose.pos; Body->Q = Q; Body->CM = Vector3(0.2f, -0.1f, 0.3f); Body->UpdateGeometries();
			const Transform ShapePose = Body->GetGeometryTransform(Shape); auto M = Triangle(0.1f);
			const auto* Segment = Shape->GetShapeObj<Capsule3>();
			const Vector3 Center = ShapePose.LocalToWorld(Capsule ? Segment->X1 : Shape->GetShapeObj<Sphere3>()->Center);
			const Vector3 Normal = Capsule ? ShapePose.quat * (Segment->X1 - Segment->X0).Unit() : Q * Vector3::UnitZ();
			Vector3 U, V; Normal.GetPerpVectors(U, V);
			for (auto& P : M.Positions) P = Center + U * P.x + V * P.y + Normal * P.z;
			ClothSystem C(Settings(), M); EXPECT(C.AddCollider(Shape)); EXPECT(C.AddCollider(Shape)); EXPECT(C.GetColliderCount() == 1);
			C.Simulate(Dt); EXPECT(C.GetCollisionStats().Contacts > 0);
			for (int I = 0; I < 3; ++I) EXPECT((C.GetParticle(I).Position - M.Positions[I]).Dot(Normal) > 0);
			EXPECT(C.RemoveCollider(Shape)); EXPECT(!C.RemoveCollider(Shape)); EXPECT(C.GetColliderCount() == 0);
		}
		// Copying a detached mesh must not retain pointers into the source buffers.
		ClothSystem C(Settings(), Triangle(0.02f)); { std::unique_ptr<Geometry> Shape(PlaneMesh()); EXPECT(C.AddCollider(Shape.get())); }
		C.Simulate(Dt); for (int I = 0; I < 3; ++I) EXPECT_NEAR(C.GetParticle(I).Position.z, 0.05f, 1e-5f);
	}
	void ConvexInterior()
	{
		for (bool Convex : { false, true })
		{
			std::unique_ptr<Geometry> Shape(Convex ? GeometryFactory::CreateConvexMesh() : GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1, 1, 0.5f)));
			if (Convex)
			{
				std::vector<Vector3> Points; for (int I = 0; I < 8; ++I) Points.emplace_back(I & 1 ? 1.0f : -1.0f, I & 2 ? 1.0f : -1.0f, I & 4 ? 0.5f : -0.5f);
				auto* Mesh = Shape->GetShapeObj<ConvexMesh>(); EXPECT(Mesh->BuildFromPoints(Points.data(), (int)Points.size())); Shape->SetShapeBounds(Mesh->Bounds);
			}
			ClothSystem C(Settings(), Triangle(0.4f, 0.1f)); const bool Registered = C.AddCollider(Shape.get());
			printf("Cloth closed collider convex=%d registered=%d\n", Convex, Registered); EXPECT(Registered); C.Simulate(Dt);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(C.GetParticle(I).Position.z, 0.55f, 1e-5f);
		}
	}
	void MovingFriction()
	{
		for (int Substeps : { 1, 4 })
		{
			PhysicsWorld World{ PhysicsWorldParam() }; auto* Shape = PlaneMesh(); auto* Body = StaticBody(World, Shape);
			auto S = Settings(); S.Substeps = Substeps; S.Gravity = Vector3(0, 0, -1); S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 1;
			auto M = Triangle(S.Thickness); ClothSystem C(S, M); EXPECT(C.AddCollider(Shape));
			const float Travel = 0.001f; Body->X.x += Travel; Body->UpdateGeometries(); C.Simulate(Dt);
			for (int I = 0; I < 3; ++I)
			{
				EXPECT_NEAR(C.GetParticle(I).Position.x - M.Positions[I].x, Travel, 2e-5f);
				EXPECT_NEAR(C.GetParticle(I).Position.z, S.Thickness, 1e-5f);
			}
		}
		for (int Iterations : { 1, 20 })
		{
			auto S = Settings(); S.SolverIterations = Iterations; S.Gravity = Vector3(0, 0, -1);
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0.1;
			auto M = Triangle(S.Thickness); ClothSystem C(S, M); std::unique_ptr<Geometry> Shape(PlaneMesh()); EXPECT(C.AddCollider(Shape.get()));
			for (int I = 0; I < 3; ++I) C.SetParticleVelocity(I, Vector3::UnitX()); C.Simulate(Dt);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(C.GetParticle(I).Position.x - M.Positions[I].x, 0.099f, 2e-5f);
		}
	}
	void MeshFeatures()
	{
		// Reverse vertex/triangle candidates: all cloth vertices miss this tiny mesh.
		std::unique_ptr<Geometry> Small(GeometryFactory::CreateTriangleMesh()); auto* Mesh = Small->GetShapeObj<TriangleMesh>();
		for (const auto& P : Triangle(0, 0.05f).Positions) Mesh->AddVertex(P);
		Mesh->AddTriangle(0, 1, 2); Mesh->Compact(); Small->SetShapeBounds(Mesh->BoundingVolume);
		ClothSystem Face(Settings(), Triangle(0.02f)); EXPECT(Face.AddCollider(Small.get())); Face.Simulate(Dt);
		EXPECT(Face.GetCollisionStats().Contacts >= 3);
		Vector3 Positions[3]; for (int I = 0; I < 3; ++I) Positions[I] = Face.GetParticle(I).Position;
		for (int I = 0; I < 3; ++I) EXPECT(SurfaceCollision::Closest(Positions, 3, &Mesh->Vertices[I], 1).Distance > 0.049);
		// Edge/edge candidates: neither triangle has a vertex near the other face.
		std::unique_ptr<Geometry> EdgeShape(GeometryFactory::CreateTriangleMesh()); auto* EdgeMesh = EdgeShape->GetShapeObj<TriangleMesh>();
		EdgeMesh->AddVertex(Vector3(0, -2, 0)); EdgeMesh->AddVertex(Vector3(0, 2, 0)); EdgeMesh->AddVertex(Vector3(0, 2, -2));
		EdgeMesh->AddTriangle(0, 1, 2); EdgeMesh->Compact(); EdgeShape->SetShapeBounds(EdgeMesh->BoundingVolume);
		auto M = Triangle(0.02f); M.Positions = { Vector3(-2, 0, 0.02f), Vector3(2, 0, 0.02f), Vector3(2, -0.2f, 0.02f) };
		ClothSystem Edge(Settings(), M); EXPECT(Edge.AddCollider(EdgeShape.get())); Edge.Simulate(Dt); EXPECT(Edge.GetCollisionStats().Contacts > 0);
		for (int I = 0; I < 3; ++I) Positions[I] = Edge.GetParticle(I).Position;
		EXPECT(SurfaceCollision::Closest(Positions, 3, EdgeMesh->Vertices, 3).Distance > 0.049);
	}
	void SpinningSphereFriction()
	{
		PhysicsWorld World{ PhysicsWorldParam() }; auto* Shape = GeometryFactory::CreateSphere(Vector3::Zero(), 0.25f); auto* Body = StaticBody(World, Shape);
		auto S = Settings(); S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 10;
		ClothSystem C(S, Triangle(0.29f)); EXPECT(C.AddCollider(Shape));
		const float Angle = 0.02f; Body->Q.FromRotationAxis(Vector3::UnitY(), Angle); Body->UpdateGeometries(); C.Simulate(Dt);
		const Vector3 Witness = C.GetParticle(0).Position * 0.25f + C.GetParticle(1).Position * 0.25f + C.GetParticle(2).Position * 0.5f;
		EXPECT_NEAR(Witness.x, 0.25f * std::sin(Angle), 2e-5f); EXPECT_NEAR(Witness.z, 0.3f, 2e-5f);
	}
	void LifetimeSwitchesAndValidation()
	{
		PhysicsWorld World{ PhysicsWorldParam() }; auto* Shape = GeometryFactory::CreateSphere(Vector3::Zero(), 0.3f); auto* Body = StaticBody(World, Shape);
		auto S = Settings(); S.Collision.SurfaceContacts = false; auto* C = World.CreateCloth(S, Triangle(0.1f));
		EXPECT(C->AddSphereCollider(Body, 0.3f, Vector3::Zero())); EXPECT(C->GetColliderCount() == 1); C->Simulate(Dt);
		EXPECT_NEAR(C->GetParticle(2).Position.z, 0.1f, 1e-6f); EXPECT(C->GetCollisionStats().Contacts == 0);
		S.Collision.SurfaceContacts = true; C->SetCollisionSettings(S.Collision); C->Simulate(Dt); EXPECT(C->GetCollisionStats().Contacts == 1);
		EXPECT(C->AddCollider(Shape)); EXPECT(C->GetColliderCount() == 2); EXPECT(C->RemoveSphereCollider(Body)); EXPECT(C->GetColliderCount() == 1);
		EXPECT(World.RemoveRigidBody(Body)); EXPECT(C->GetColliderCount() == 0); World.Simulate(Dt);
		const float Bad = std::numeric_limits<float>::quiet_NaN(); EXPECT(!C->AddCapsuleCollider(nullptr, Bad, Vector3::Zero(), Vector3::Zero()));
		EXPECT(!C->AddCapsuleCollider(nullptr, 1, Vector3(Bad, 0, 0), Vector3::Zero())); EXPECT(!C->AddCollider(nullptr));
		std::unique_ptr<Geometry> Plane(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitZ())); EXPECT(!C->AddCollider(Plane.get()));
		auto Invalid = S.Collision; Invalid.Material.DynamicFriction = Invalid.Material.StaticFriction + 1;
		bool Rejected = false; try { C->SetCollisionSettings(Invalid); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		EXPECT(C->GetSettings().Collision.SurfaceContacts);
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
		EXPECT(Scene.SetClothRigidInteraction(false));
		RigidBody* Ball = nullptr; for (const auto& Object : Scene.GetObjects()) if (Object.Id == "cloth_ball") Ball = Object.GeometryPtr->GetParent<RigidBody>();
		EXPECT(Ball); if (!Ball) return;
		auto* C = Scene.GetDemoCloth(); EXPECT(C); if (!C) return;
		EXPECT(Scene.ApplyClothDemo(true, false, true, false, true, true)); EXPECT(C->GetSettings().Collision.SurfaceContacts);
		size_t Contacts = 0; float MinimumRadius = std::numeric_limits<float>::infinity();
		for (int Frame = 0; Frame < 90; ++Frame)
		{
			Scene.Step(1.0f / 60); Contacts += C->GetCollisionStats().Contacts;
			for (int I = 0; I < C->GetParticleCount(); ++I)
			{
				const auto& P = C->GetParticle(I); EXPECT(std::isfinite(P.Position.Length()) && P.Position.Length() < 100);
				if (P.Pinned) EXPECT(P.Position == P.PinPosition);
				if (Frame > 30) MinimumRadius = std::min(MinimumRadius, (P.Position - Ball->X).Length());
			}
		}
		printf("Surface cloth demo contacts=%zu minimum vertex radius=%g\n", Contacts, MinimumRadius);
		EXPECT(Contacts > 0); EXPECT(MinimumRadius > 1.28f);
		const auto* Render = C->GetRenderGeometry()->GetShapeObj<TriangleMesh>();
		for (int I = 0; I < C->GetParticleCount(); ++I) EXPECT(Render->Vertices[I] == C->GetParticle(I).Position);
		EXPECT(Scene.ApplyClothDemo(false, true, true, false, true, false)); EXPECT(C->GetSettings().Collision.Material.StaticFriction == 0);
		for (int I = 0; I < 15; ++I) Scene.Step(1.0f / 60);
		EXPECT(Scene.ApplyClothDemo(false, false, true, false, false, true)); Scene.Step(1.0f / 60);
		EXPECT(C->GetCollisionStats().Contacts == 0); EXPECT(!C->GetSettings().Collision.SurfaceContacts);
	}
}

void TestClothCollision()
{
	FaceCapsuleResponse(); ShapeSnapshotsAndFrames(); ConvexInterior(); MeshFeatures(); MovingFriction(); SpinningSphereFriction(); LifetimeSwitchesAndValidation(); ExistingDemo();
}

#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include <memory>
#include "../Src/Collision/SurfaceCollision.h"
#include <cmath>
#include <chrono>
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Collision/GeometryQuery.h"

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

namespace Test_ClothCollision_Cases
{
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
}

void TestClothCollision()
{
	FaceCapsuleResponse(); ShapeSnapshotsAndFrames(); ConvexInterior(); MeshFeatures(); MovingFriction(); SpinningSphereFriction(); LifetimeSwitchesAndValidation();
}
}

void TestClothCollision() { Test_ClothCollision_Cases::TestClothCollision(); }

namespace Test_ClothContinuous_Cases
{
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
	Run("CoupledFailureRetry", CoupledFailureRetry); Run("ImmovableContactRollback", ImmovableContactRollback);
}
}

void TestClothContinuous() { Test_ClothContinuous_Cases::TestClothContinuous(); }

namespace Test_ClothInteraction_Cases
{
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
}
void TestClothInteraction()
{
	SelfCollision(); InterCollision(); FrictionAndTimeSteps(); EdgeContactsAndCompliance();
}
}

void TestClothInteraction() { Test_ClothInteraction_Cases::TestClothInteraction(); }

namespace Test_ClothRigidCoupling_Cases
{
namespace
{
	using namespace Riemann;
	void StepFrame(PhysicsWorld& World, float Dt)
	{
		const auto Before = World.GetBroadPhasePassCount();
		World.Simulate(Dt);
		EXPECT(World.GetBroadPhasePassCount() == Before + 1);
	}
	PhysicsWorldParam WorldSettings()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD;
		P.gravityAcc = Vector3::Zero(); P.xpbdSettings.PositionIterations = 24; return P;
	}
	ClothSettings Material()
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1;
		S.Thickness = 0.02f; S.SolverIterations = 24; S.Collision.RigidCoupling = true;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0; return S;
	}
	ClothMesh Triangle(float Z)
	{
		ClothMesh M; M.Positions = { Vector3(-1, -1, Z), Vector3(1, -1, Z), Vector3(0, 1, Z) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses = { 1, 1, 1 }; return M;
	}
	RigidBodyDynamic* Ball(PhysicsWorld& W, const Vector3& P, float Radius = 0.5f)
	{
		RigidBodyParam R; R.rigidType = RigidType::Dynamic; R.linearDamping = R.angularDamping = 1;
		R.sleepThreshold = R.freezeThreshold = 0;
		return W.CreateRigidBody(GeometryFactory::CreateSphere(P, Radius), R)->CastDynamic();
	}
	Vector3 Moment(const ClothSystem& C, const RigidBody& B)
	{
		Vector3 Result = B.X / B.InvMass;
		for (int I = 0; I < C.GetParticleCount(); ++I) Result += C.GetParticle(I).Position / C.GetParticle(I).InverseMass;
		return Result;
	}
	void ContactReaction()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			auto P = WorldSettings(); P.resolutionSolver = Solver; PhysicsWorld W(P);
			auto S = Material(); S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
			auto* C = W.CreateCloth(S, Triangle(0.48f)); auto* B = Ball(W, Vector3::Zero());
			const Vector3 Before = Moment(*C, *B); StepFrame(W, 1.0f / 120);
			EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(B->X.z < -0.005f);
			EXPECT_NEAR((Moment(*C, *B) - Before).Length(), 0, 2e-5f);
			Vector3 Momentum = B->V / B->InvMass;
			for (int I = 0; I < C->GetParticleCount(); ++I) Momentum += C->GetParticleVelocity(I) / C->GetParticle(I).InverseMass;
			EXPECT_NEAR(Momentum.Length(), 0, 0.003f);
			printf("Cloth rigid contact solver %d: rigid displacement %g, momentum error %g\n", (int)Solver, B->X.z, Momentum.Length());
		}
	}
	void AttachmentPullAndLifetime()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.RigidCoupling = false; S.Substeps = 4; S.Damping = 0.98f;
		auto* C = W.CreateCloth(S, Triangle(0)); auto* B = Ball(W, C->GetParticle(2).Position, 0.1f);
		C->PinParticle(0, C->GetParticle(0).Position); C->PinParticle(1, C->GetParticle(1).Position);
		const auto Id = C->AttachToRigid(BarycentricPoint::Vertex(2), B, B->X);
		C->PinParticle(0, C->GetParticle(0).Position + Vector3(0, -0.35f, 0));
		C->PinParticle(1, C->GetParticle(1).Position + Vector3(0, -0.35f, 0));
		for (int I = 0; I < 60; ++I) StepFrame(W, 1.0f / 120);
		EXPECT(B->X.y < 0.9f); EXPECT((C->GetParticle(2).Position - B->X).Length() < 0.002f);
		printf("Cloth tension pulled attached ball to y=%g\n", B->X.y);
		std::string Error; EXPECT(!W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false, &Error));
		EXPECT(!Error.empty()); EXPECT(C->RemoveRigidAttachment(Id)); EXPECT(!C->RemoveRigidAttachment(Id));
		C->AttachToRigid(BarycentricPoint::Vertex(2), B, B->X);
		EXPECT(W.RemoveRigidBody(B)); EXPECT(C->GetRigidAttachmentCount() == 0); StepFrame(W, 1.0f / 120);
		W.Reset(); EXPECT(W.GetClothCount() == 0);
	}
	void BroadPhaseMotion()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material();
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(3)); auto* B = Ball(W, Vector3::Zero());
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts == 0);
		for (int I = 0; I < C->GetParticleCount(); ++I) C->SetParticlePosition(I, Triangle(0.48f).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(B->X.z < 0);
	}
	void OffsetAttachmentTorque()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.RigidCoupling = false;
		auto* C = W.CreateCloth(S, Triangle(0)); auto* B = Ball(W, Vector3(0, 0, 0));
		const auto Initial = Moment(*C, *B);
		C->AttachToRigid(BarycentricPoint::Vertex(2), B, C->GetParticle(2).Position);
		C->ApplyParticleImpulse(2, Vector3(0, 0, 0.1f)); StepFrame(W, 1.0f / 120);
		EXPECT(B->V.z > 0); EXPECT(std::abs(B->W.x) > 0.001f);
		Vector3 Momentum = B->V / B->InvMass;
		for (int I = 0; I < C->GetParticleCount(); ++I) Momentum += C->GetParticleVelocity(I) / C->GetParticle(I).InverseMass;
		EXPECT_NEAR((Momentum - Vector3(0, 0, 0.1f)).Length(), 0, 0.002f);
		EXPECT_NEAR((Moment(*C, *B) - Initial - Vector3(0, 0, 0.1f / 120)).Length(), 0, 2e-5f);
	}
	void MixedClothColliders()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.InterCollision = true;
		W.CreateCloth(S, Triangle(3));
		S.Collision.RigidCoupling = false; S.Collision.SurfaceContacts = true;
		auto* C = W.CreateCloth(S, Triangle(0.48f)); auto* B = Ball(W, Vector3::Zero());
		EXPECT(C->AddSphereCollider(B, 0.5f, B->X));
		StepFrame(W, 1.0f / 120);
		// Joining the shared cloth stage must preserve this cloth's prescribed collider.
		EXPECT(C->GetCollisionStats().Contacts > 0); EXPECT(C->GetCollisionStats().RigidContacts == 0);
		EXPECT(B->X == Vector3::Zero());
		const Vector3 Center = (C->GetParticle(0).Position + C->GetParticle(1).Position + C->GetParticle(2).Position) / 3;
		EXPECT(Center.z > 0.49f);
	}
	void SharedBroadPhase(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver, bool ShapeOffsets)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.pbdSettings.PositionIterations = P.xpbdSettings.PositionIterations = 8;
		P.contactGeneration.UseShapeOffsets = ShapeOffsets;
		PhysicsWorld W(P);
		auto* A = Ball(W, Vector3::Zero()); auto* B = Ball(W, Vector3(4, 0, 0));
		StepFrame(W, 1.0f / 120);
		const uint64_t RigidPasses = W.GetBroadPhasePassCount();
		EXPECT(RigidPasses == 1);
		auto S = Material(); S.Substeps = 1; S.SolverIterations = 8;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(0.51f));
		DeformableBodySettings D; D.Gravity = Vector3::Zero(); D.GroundEnabled = false;
		D.Substeps = 1; D.SolverIterations = 8; D.ParticleRadius = 0.02f; D.Collision.RigidCollision = true;
		auto* Soft = W.CreateDeformableBody(D);
		for (const auto& V : Triangle(0.51f).Positions) Soft->AddParticle(V + Vector3(4, 0, 0));
		EXPECT(Soft->SetSurfaceTriangles({ Index3(0, 1, 2) }));
		StepFrame(W, 1.0f / 120);
		EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
		EXPECT(A->X.z < 0 && B->X.z < 0);
		// Adding both material systems must not add a per-system broad-phase pass.
		EXPECT(W.GetBroadPhasePassCount() - RigidPasses == RigidPasses);
		EXPECT(W.RemoveCloth(C)); EXPECT(W.RemoveDeformableBody(Soft));
		A->X = Vector3::Zero(); A->V = Vector3::Zero(); A->UpdateGeometries();
		C = W.CreateCloth(S, Triangle(3)); StepFrame(W, 1.0f / 120);
		EXPECT(C->GetCollisionStats().RigidContacts == 0);
		for (int I = 0; I < 3; ++I) C->SetParticlePosition(I, Triangle(0.51f).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts > 0);
		printf("Shared broad phase=%d solver=%d offsets=%d passes=%llu\n", (int)Broad, (int)Solver,
			(int)ShapeOffsets, (unsigned long long)RigidPasses);
	}
	void SharedSurfaceFiltering(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.collisionFilter = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask());
		PhysicsWorld W(P); auto S = Material(); S.Collision.RigidCoupling = false; S.Collision.InterCollision = true;
		S.Substeps = 1; S.SolverIterations = 4;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* A = W.CreateCloth(S, Triangle(0));
		S.Collision.FilterData = CollisionData::ForLayer(1, 0);
		auto* B = W.CreateCloth(S, Triangle(0.03f));
		StepFrame(W, 1.0f / 120); EXPECT(A->GetCollisionStats().InterContacts == 0);
		S.Collision.FilterData = CollisionData::ForLayer(1); B->SetCollisionSettings(S.Collision);
		StepFrame(W, 1.0f / 120); EXPECT(A->GetCollisionStats().InterContacts > 0);
		EXPECT(B->GetCollisionStats().InterContacts > 0);
	}
	void SharedFieldLifetime(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad; PhysicsWorld W(P);
		AerodynamicsSimulationSettings AirSettings(AerodynamicsAlgorithm::Incompressible);
		AirSettings.Backend = FluidBackend::CPU; AirSettings.Resolution = Vector3i(8, 8, 8);
		AirSettings.BoundsMin = Vector3(-1); AirSettings.BoundsMax = Vector3(1); AirSettings.Emitters.clear();
		AirSettings.MaxTimeStep = 1.0f / 120; AirSettings.PressureIterations = 8;
		auto* Air = W.CreateAerodynamics(AirSettings); EXPECT(Air); if (!Air) return;
		auto S = Material(); S.Substeps = 1; S.SolverIterations = 4;
		auto* C = W.CreateCloth(S, Triangle(3)); auto* B = Ball(W, Vector3(3, 0, 0));
		StepFrame(W, 1.0f / 120);
		auto Occupied = [&]() { size_t N = 0; for (auto Solid : Air->GetSolidCells()) N += Solid != 0; return N; };
		EXPECT(Occupied() == 0);
		B->X = Vector3::Zero(); B->UpdateGeometries();
		StepFrame(W, 1.0f / 120); EXPECT(Occupied() > 0);
		EXPECT(W.RemoveRigidBody(B)); B->ReleaseGeometries(); delete B;
		for (int I = 0; I < 3; ++I) C->SetParticlePosition(I, Triangle(0).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(Occupied() > 0);
		EXPECT(W.RemoveCloth(C)); StepFrame(W, 1.0f / 120); EXPECT(Occupied() == 0);
		EXPECT(W.RemoveAerodynamics()); Air = W.CreateAerodynamics(AirSettings);
		EXPECT(Air); StepFrame(W, 1.0f / 120); EXPECT(Occupied() == 0);
	}
	void BarotropicFieldFiltering(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.collisionFilter = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask());
		PhysicsWorld W(P);
		AerodynamicsSimulationSettings S(AerodynamicsAlgorithm::Barotropic);
		S.Backend = FluidBackend::CPU; S.Resolution = Vector3i(8, 8, 8);
		S.BoundsMin = Vector3(-1); S.BoundsMax = Vector3(1); S.Emitters.clear();
		EXPECT(W.CreateAerodynamics(S)); auto* B = Ball(W, Vector3::Zero());
		B->mGeometries[0]->SetFilterData(CollisionData::ForLayer(1, 0));
		StepFrame(W, 1.0f / 120); AerodynamicsRigidCouplingResult Load;
		W.GetAerodynamicsCoupling()->Evaluate(B, &Load); EXPECT(Load.SampleCount == 0);
		B->mGeometries[0]->SetFilterData(CollisionData::ForLayer(1));
		StepFrame(W, 1.0f / 120);
		EXPECT(W.GetAerodynamicsCoupling()->Evaluate(B, &Load)); EXPECT(Load.SampleCount > 0);
	}
	void FrameMotion(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		for (bool Accelerating : { false, true })
		{
			auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 8;
			P.gravityAcc = Accelerating ? Vector3(100, 0, 0) : Vector3::Zero();
			int Contacts = 0; P.onContact = [&](const PhysicsContactInfo&) { ++Contacts; };
			PhysicsWorld W(P);
			W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(3, 0, 0), 0.5f), RigidBodyParam())->SetRestitution(0);
			auto* B = Ball(W, Vector3::Zero()); B->V = Accelerating ? Vector3::Zero() : Vector3(20, 0, 0); B->SetRestitution(0);
			StepFrame(W, 0.25f);
			printf("Frame motion broad=%d solver=%d acceleration=%d contacts=%d x=%g\n", (int)Broad, (int)Solver, (int)Accelerating, Contacts, B->X.x);
			// SI/LCP retain their gradual penetration correction. This fixture checks
			// that later substeps detect and stop the impact, not their correction rate.
			EXPECT(Contacts > 0); EXPECT(B->X.x < 2.5f);
		}
	}
	void IndependentMaterials(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P);
		auto S = Material(); S.Collision.RigidCoupling = false; S.Collision.InterCollision = true;
		S.Substeps = 3; S.SolverIterations = 5;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		W.CreateCloth(S, Triangle(0)); auto* C = W.CreateCloth(S, Triangle(0.2f));
		for (int I = 0; I < 3; ++I) C->SetParticleVelocity(I, Vector3(0, 0, -1));
		DeformableBodySettings D; D.Gravity = Vector3::Zero(); D.GroundEnabled = false;
		D.Substeps = 2; D.SolverIterations = 5; D.Collision.InterCollision = true;
		for (float Z : { 0.0f, 0.01f })
		{
			auto* B = W.CreateDeformableBody(D);
			for (const auto& V : Triangle(Z).Positions) B->AddParticle(V + Vector3(4, 0, 0));
			EXPECT(B->SetSurfaceTriangles({ Index3(0, 1, 2) }));
		}
		StepFrame(W, 0.2f);
		EXPECT(C->GetCollisionStats().InterContacts > 0);
	}
	void TransferredMotion(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 16;
		PhysicsWorld W(P);
		auto* A = Ball(W, Vector3(-2, 0, 0)); auto* B = Ball(W, Vector3::Zero()); auto* C = Ball(W, Vector3(2, 0, 0));
		A->SetRestitution(1); B->SetRestitution(1); C->SetRestitution(1); A->V = Vector3(20, 0, 0);
		StepFrame(W, 0.3f); EXPECT(C->X.x > 2.1f);
	}
	void MovingCoupledSurface(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P); auto S = Material(); S.Substeps = 2;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(1.5f)); auto* B = Ball(W, Vector3::Zero());
		for (int I = 0; I < 3; ++I) C->SetParticleVelocity(I, Vector3(0, 0, -10));
		StepFrame(W, 0.15f); EXPECT(B->X.z < -0.01f);
	}
	void LateRegistration(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld* World = nullptr; RigidBodyDynamic* Added = nullptr; bool AddedContact = false;
		P.onContact = [&](const PhysicsContactInfo& Info)
		{
			if (!Added) Added = Ball(*World, Vector3(4, 0.49f, 0));
			else if (Info.BodyA == Added || Info.BodyB == Added) AddedContact = true;
		};
		PhysicsWorld W(P); World = &W;
		W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		Ball(W, Vector3(0, 0.49f, 0));
		StepFrame(W, 1.0f / 60); EXPECT(Added && AddedContact);
	}
	void ProjectedMotion(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad; int Contacts = 0;
		P.onContact = [&](const PhysicsContactInfo&) { ++Contacts; };
		PhysicsWorld W(P);
		auto* Reference = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(), 0.1f), RigidBodyParam());
		Reference->mGeometries[0]->SetSimulationEnabled(false);
		auto* B = Ball(W, Vector3(-5, 0, 0));
		W.AddConstraint(new FixedConstraint(Reference, B));
		W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5, 0.75f, 0), 0.5f), RigidBodyParam());
		B->X = Vector3::Zero(); B->UpdateGeometries();
		StepFrame(W, 1.0f / 60); EXPECT(Contacts > 0);
	}
	void CandidateViews()
	{
		BroadPhaseResult Result;
		constexpr int Rigids = 128, ClothA = Rigids, ClothB = Rigids + 1;
		constexpr int SoftA = Rigids + 2, SoftB = Rigids + 3, Air = Rigids + 4;
		Result.States.resize(Rigids + 5);
		Result.SetObject(ClothA, BroadPhaseObjectType::Cloth);
		Result.SetObject(ClothB, BroadPhaseObjectType::Cloth);
		Result.SetObject(SoftA, BroadPhaseObjectType::Deformable);
		Result.SetObject(SoftB, BroadPhaseObjectType::Deformable);
		Result.SetObject(Air, BroadPhaseObjectType::Aerodynamics);
		for (int I = 0; I < Rigids; ++I) for (int J = I + 1; J < Rigids; ++J) Result.Pairs.emplace_back(I, J);
		const size_t RigidCount = Result.Pairs.size();
		Result.Pairs.emplace_back(ClothB, ClothA);
		Result.Pairs.emplace_back(0, ClothA);
		Result.Pairs.emplace_back(SoftA, 1);
		Result.Pairs.emplace_back(SoftA, SoftB);
		Result.Pairs.emplace_back(Air, 0);
		Result.Pairs.emplace_back(ClothA, Air);
		// Cloth-soft interaction has no contact implementation and must not be
		// routed to either homogeneous surface detector.
		Result.Pairs.emplace_back(ClothA, SoftA);
		Result.DispatchPairs();
		EXPECT(Result.GetPairs(BroadPhasePairType::Rigid).size() == RigidCount);
		EXPECT(Result.GetPairs(BroadPhasePairType::Cloth).size() == 1);
		EXPECT(&Result.GetPairs(BroadPhasePairType::Cloth)[0] == &Result.Pairs[RigidCount]);
		EXPECT(Result.GetPairs(BroadPhasePairType::ClothRigid).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::DeformableRigid).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::Deformable).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::Aerodynamics).size() == 2);
		BroadPhaseObjectMap<int> Mapping; int Lookups = 0;
		auto Bind = [&](int Index) { ++Lookups; return Index * 2; };
		Mapping.Update(Result, Bind);
		for (int Iteration = 0; Iteration < 24; ++Iteration)
		{
			Mapping.Update(Result, Bind);
			size_t Visits = 0;
			for (const auto& Pair : Result.GetPairs(BroadPhasePairType::Cloth))
			{
				++Visits; EXPECT(Mapping[Pair.index1] == ClothB * 2); EXPECT(Mapping[Pair.index2] == ClothA * 2);
			}
			EXPECT(Visits == 1);
		}
		EXPECT(Lookups == (int)Result.States.size());
		const size_t Begin = Result.Pairs.size(); const int Added = (int)Result.States.size();
		Result.States.emplace_back();
		Result.Pairs.emplace_back(Added, ClothB); Result.Pairs.emplace_back(Air, Added); Result.Pairs.emplace_back(Added, 1);
		Result.DispatchPairs(Begin); Mapping.Update(Result, Bind);
		EXPECT(Lookups == (int)Result.States.size()); EXPECT(Mapping[Added] == Added * 2);
		EXPECT(Result.GetPairs(BroadPhasePairType::ClothRigid).size() == 2);
		EXPECT(&Result.GetPairs(BroadPhasePairType::ClothRigid)[1] == &Result.Pairs[Begin]);
		EXPECT(Result.GetPairs(BroadPhasePairType::Aerodynamics).size() == 3);
		EXPECT(Result.GetPairs(BroadPhasePairType::Rigid).size() == RigidCount + 1);
		Result.Clear(); Result.States.resize(2); Result.Pairs.emplace_back(0, 1); Result.DispatchPairs();
		Mapping.Update(Result, Bind); EXPECT(Lookups == Added + 3);
		const auto PureRigid = Result.GetPairs(BroadPhasePairType::Rigid);
		EXPECT(PureRigid.size() == 1 && PureRigid.Indices == nullptr);
		EXPECT(&PureRigid[0] == &Result.Pairs[0]); EXPECT(Result.GetPairs(BroadPhasePairType::Cloth).empty());
	}
	void ReplacedFieldGeometry(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld* World = nullptr; RigidBody* Obstacle = nullptr; bool Replaced = false;
		GeometryHandle Previous;
		P.onContact = [&](const PhysicsContactInfo&)
		{
			if (Replaced) return;
			Replaced = true;
			Previous = Obstacle->GetGeometryHandle(0);
			EXPECT(World->RemoveRigidBody(Obstacle)); Obstacle->ReleaseGeometries(); delete Obstacle;
			Obstacle = World->CreateRigidBody(GeometryFactory::CreateSphere(Vector3(4, 0.5f, 0), 0.3f), RigidBodyParam());
			const auto Current = Obstacle->GetGeometryHandle(0);
			EXPECT(Current.Index == Previous.Index && Current.Generation != Previous.Generation);
		};
		PhysicsWorld W(P); World = &W;
		W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		Ball(W, Vector3(0, 0.49f, 0));
		Obstacle = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(4, -0.5f, 0), 0.3f), RigidBodyParam());
		AerodynamicsSimulationSettings S(AerodynamicsAlgorithm::Incompressible);
		S.Backend = FluidBackend::CPU; S.Resolution = Vector3i(16); S.PressureIterations = 8;
		S.BoundsMin = Vector3(3, -1, -1); S.BoundsMax = Vector3(5, 1, 1); S.Emitters.clear();
		auto* Air = W.CreateAerodynamics(S); EXPECT(Air); if (!Air) return;
		StepFrame(W, 1.0f / 60); EXPECT(Replaced);
		const auto& Solid = Air->GetSolidCells();
		EXPECT(Solid[8 + 16 * (4 + 16 * 8)] == 0);
		EXPECT(Solid[8 + 16 * (12 + 16 * 8)] != 0);
	}
}

void TestClothRigidCoupling()
{
	ContactReaction(); AttachmentPullAndLifetime(); OffsetAttachmentTorque(); MixedClothColliders(); BroadPhaseMotion();
}

void TestUnifiedBroadPhase()
{
	CandidateViews();
	for (auto Broad : { BroadPhaseSolver::SAP, BroadPhaseSolver::ABP, BroadPhaseSolver::MBP,
		BroadPhaseSolver::AllPairs, BroadPhaseSolver::Bruteforce, BroadPhaseSolver::DynamicAABB })
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
			for (bool Offsets : { false, true }) SharedBroadPhase(Broad, Solver, Offsets);
		SharedSurfaceFiltering(Broad); SharedFieldLifetime(Broad); BarotropicFieldFiltering(Broad);
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			FrameMotion(Broad, Solver); IndependentMaterials(Broad, Solver); TransferredMotion(Broad, Solver);
		}
		MovingCoupledSurface(Broad); LateRegistration(Broad); ProjectedMotion(Broad); ReplacedFieldGeometry(Broad);
	}
}
}

void TestClothRigidCoupling() { Test_ClothRigidCoupling_Cases::TestClothRigidCoupling(); }
void TestUnifiedBroadPhase() { Test_ClothRigidCoupling_Cases::TestUnifiedBroadPhase(); }

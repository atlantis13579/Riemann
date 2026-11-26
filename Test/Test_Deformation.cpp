#include "Test.h"
#include <limits>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include <cfloat>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include "../Src/Deformation/RigidSurfaceContact.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"

using namespace Riemann;

namespace
{
	DeformableBodySettings IsolatedDeformableSettings()
	{
		DeformableBodySettings settings;
		settings.Gravity = Vector3::Zero();
		settings.Damping = 0.0f;
		settings.Substeps = 1;
		settings.SolverIterations = 12;
		settings.EdgeCompliance = 0.0f;
		return settings;
	}

	void AddUnitTetrahedronParticles(DeformableBody& body, const Vector3& offset = Vector3::Zero())
	{
		EXPECT(body.AddParticle(offset) == 0);
		EXPECT(body.AddParticle(offset + Vector3::UnitX()) == 1);
		EXPECT(body.AddParticle(offset + Vector3::UnitY()) == 2);
		EXPECT(body.AddParticle(offset + Vector3::UnitZ()) == 3);
	}

	void TestDeformableValidation()
	{
		printf("Running TestDeformableValidation\n");
		DeformableBody body(IsolatedDeformableSettings());
		body.Simulate(1.0f / 60.0f);
		body.Reset();
		EXPECT(body.GetRenderGeometry() == nullptr);
		EXPECT(body.AddParticle(Vector3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)) == -1);
		EXPECT(body.AddParticle(Vector3::Zero(), -1.0f) == -1);
		AddUnitTetrahedronParticles(body);
		EXPECT(body.AddEdge(-1, 1) == -1);
		EXPECT(body.AddEdge(0, 4) == -1);
		EXPECT(body.AddEdge(0, 0) == -1);
		EXPECT(body.AddTetrahedron(0, 1, 2, 2) == -1);
		EXPECT(body.AddTetrahedron(0, 1, 2, 4) == -1);
		EXPECT(body.GetEdges().empty());
		EXPECT(body.AddTetrahedron(0, 1, 2, 3) == 0);
		EXPECT(body.GetEdges().size() == 6);
		EXPECT(body.AddTetrahedron(3, 2, 1, 0) == 0);
		EXPECT(body.GetTetrahedra().size() == 1);
		EXPECT(body.AddEdge(1, 0) >= 0);
		EXPECT(body.GetEdges().size() == 6);
		EXPECT_FALSE(body.SetSurfaceTriangles({ Index3(0, 1, 4) }));
		EXPECT_FALSE(body.SetSurfaceTriangles({ Index3(0, 1, 1) }));
		EXPECT_FALSE(body.AddForce(10, Vector3::UnitY()));
		EXPECT_FALSE(body.SetParticleVelocity(0, Vector3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f)));
		const Vector3 position = body.GetParticle(0).Position;
		body.Simulate(0.0f);
		body.Simulate(-1.0f);
		body.Simulate(std::numeric_limits<float>::quiet_NaN());
		EXPECT_SAME(body.GetParticle(0).Position, position);

		DeformableBody flat(IsolatedDeformableSettings());
		flat.AddParticle(Vector3::Zero());
		flat.AddParticle(Vector3::UnitX());
		flat.AddParticle(Vector3::UnitY());
		flat.AddParticle(Vector3(1.0f, 1.0f, 0.0f));
		EXPECT(flat.AddTetrahedron(0, 1, 2, 3) == -1);
		EXPECT(flat.GetTetrahedra().empty());
		EXPECT(flat.GetEdges().empty());
	}

	void TestDeformableForcesAndImpulses()
	{
		printf("Running TestDeformableForcesAndImpulses\n");
		DeformableBodySettings settings = IsolatedDeformableSettings();
		settings.Substeps = 4;
		settings.Gravity = Vector3(0.0f, -10.0f, 0.0f);
		DeformableBody body(settings);
		body.AddParticle(Vector3::Zero(), 0.5f);
		body.AddParticle(Vector3(0.0f, 4.0f, 0.0f), 0.0f);
		EXPECT(body.AddForce(0, Vector3(4.0f, 0.0f, 0.0f)));
		EXPECT_FALSE(body.AddImpulse(1, Vector3::UnitX()));
		body.Simulate(0.2f);
		EXPECT_NEAR(body.GetParticle(0).Velocity.x, 0.4f, 1.0e-5f);
		EXPECT_NEAR(body.GetParticle(0).Velocity.y, -2.0f, 1.0e-5f);
		EXPECT_NEAR(body.GetParticle(0).Position.x, 0.05f, 1.0e-5f);
		EXPECT_NEAR(body.GetParticle(0).Position.y, -0.25f, 1.0e-5f);
		EXPECT_SAME(body.GetParticle(0).Force, Vector3::Zero());
		EXPECT_SAME(body.GetParticle(1).Position, Vector3(0.0f, 4.0f, 0.0f));
		body.Simulate(0.2f);
		EXPECT_NEAR(body.GetParticle(0).Velocity.x, 0.4f, 1.0e-5f);
		EXPECT(body.AddImpulse(0, Vector3(2.0f, 0.0f, 0.0f)));
		EXPECT_NEAR(body.GetParticle(0).Velocity.x, 1.4f, 1.0e-5f);

		DeformableBody radial(IsolatedDeformableSettings());
		radial.AddParticle(Vector3::Zero(), 1.0f);
		radial.AddParticle(Vector3::UnitX(), 0.25f);
		radial.AddParticle(Vector3::UnitY(), 1.0f);
		radial.PinParticle(2, Vector3::UnitY());
		EXPECT(radial.AddRadialImpulse(Vector3::Zero(), 2.0f, Vector3(0.0f, 0.0f, 6.0f)) == 2);
		const Vector3 momentum = radial.GetParticle(0).Velocity + radial.GetParticle(1).Velocity * 4.0f;
		EXPECT_SAME(momentum, Vector3(0.0f, 0.0f, 6.0f));
		EXPECT_SAME(radial.GetParticle(2).Velocity, Vector3::Zero());
		EXPECT(radial.AddRadialImpulse(Vector3::Zero(), 0.0f, Vector3::UnitY()) == 0);
	}

	void TestDeformableElasticEdges()
	{
		printf("Running TestDeformableElasticEdges\n");
		DeformableBody body(IsolatedDeformableSettings());
		body.AddParticle(Vector3::Zero(), 1.0f);
		body.AddParticle(Vector3::UnitX(), 1.0f / 3.0f);
		body.AddEdge(0, 1);
		body.SetParticlePosition(1, Vector3(3.0f, 0.0f, 0.0f));
		body.Simulate(0.1f);
		EXPECT_NEAR(body.GetParticle(0).Position.x, 1.5f, 1.0e-5f);
		EXPECT_NEAR(body.GetParticle(1).Position.x, 2.5f, 1.0e-5f);
		EXPECT_NEAR((body.GetParticle(0).Position.x + 3.0f * body.GetParticle(1).Position.x) / 4.0f, 2.25f, 1.0e-5f);

		float positions[2];
		for (int i = 0; i < 2; ++i)
		{
			DeformableBodySettings settings = IsolatedDeformableSettings();
			settings.SolverIterations = i == 0 ? 1 : 32;
			settings.EdgeCompliance = 0.001f;
			DeformableBody soft(settings);
			soft.AddParticle(Vector3::Zero());
			soft.AddParticle(Vector3::UnitX());
			soft.AddEdge(0, 1);
			soft.PinParticle(0, Vector3::Zero());
			soft.SetParticlePosition(1, Vector3(2.0f, 0.0f, 0.0f));
			soft.Simulate(0.1f);
			positions[i] = soft.GetParticle(1).Position.x;
			EXPECT_NEAR(positions[i], 2.0f - 1.0f / 1.1f, 1.0e-5f);
			EXPECT_SAME(soft.GetParticle(0).Position, Vector3::Zero());
			EXPECT_FALSE(soft.AddForce(0, Vector3::UnitX()));
			EXPECT(soft.UnpinParticle(0));
			EXPECT(soft.AddImpulse(0, Vector3::UnitX()));
		}
		EXPECT_NEAR(positions[0], positions[1], 1.0e-6f);

		for (int substeps = 1; substeps <= 4; substeps *= 4)
		{
			DeformableBodySettings settings = IsolatedDeformableSettings();
			settings.Substeps = substeps;
			settings.EdgeCompliance = 0.002f;
			settings.Gravity = Vector3(0.0f, -10.0f, 0.0f);
			settings.Damping = 1.0f;
			DeformableBody hanging(settings);
			hanging.AddParticle(Vector3::Zero(), 0.0f);
			hanging.AddParticle(-Vector3::UnitY());
			hanging.AddEdge(0, 1);
			for (int frame = 0; frame < 360; ++frame) hanging.Simulate(1.0f / 60.0f);
			EXPECT_NEAR(-hanging.GetParticle(1).Position.y, 1.02f, 0.001f);
		}
	}

	void TestDeformableSignedVolume()
	{
		printf("Running TestDeformableSignedVolume\n");
		for (int reverse = 0; reverse < 2; ++reverse)
		{
			DeformableBodySettings settings = IsolatedDeformableSettings();
			settings.EdgeCompliance = 1.0e8f;
			DeformableBody body(settings);
			AddUnitTetrahedronParticles(body);
			EXPECT(body.AddTetrahedron(0, reverse ? 2 : 1, reverse ? 1 : 2, 3) == 0);
			const float rest = reverse ? -1.0f / 6.0f : 1.0f / 6.0f;
			EXPECT_NEAR(body.GetTetrahedronVolume(0), rest, 1.0e-6f);
			for (int i = 0; i < 3; ++i) body.PinParticle(i, body.GetParticle(i).Position);
			body.SetParticlePosition(3, Vector3(0.0f, 0.0f, 2.0f));
			body.Simulate(0.1f);
			EXPECT_NEAR(body.GetTetrahedronVolume(0), rest, 1.0e-5f);
			EXPECT_NEAR(body.GetParticle(3).Position.z, 1.0f, 1.0e-5f);
			body.SetParticlePosition(3, Vector3(0.0f, 0.0f, -0.5f));
			body.Simulate(0.1f);
			EXPECT_NEAR(body.GetTetrahedronVolume(0), rest, 1.0e-5f);
		}
	}

	void TestDeformablePlasticityAndReset()
	{
		printf("Running TestDeformablePlasticityAndReset\n");
		DeformableBodySettings settings = IsolatedDeformableSettings();
		settings.YieldStrain = 0.1f;
		settings.PlasticCreep = 100.0f;
		settings.MaxPlasticStrain = 0.25f;
		settings.EdgeCompliance = 0.01f;
		settings.Damping = 2.0f;
		DeformableBody body(settings);
		body.AddParticle(Vector3::Zero());
		body.AddParticle(Vector3::UnitX());
		body.AddEdge(0, 1);
		body.PinParticle(0, Vector3::Zero());
		body.SetParticlePosition(1, Vector3(1.05f, 0.0f, 0.0f));
		body.Simulate(0.1f);
		EXPECT_NEAR(body.GetEdges()[0].RestLength, 1.0f, 1.0e-6f);
		body.Reset();
		body.AddImpulse(1, Vector3(10.0f, 0.0f, 0.0f));
		body.Simulate(0.1f);
		EXPECT_NEAR(body.GetEdges()[0].RestLength, 1.25f, 1.0e-5f);
		for (int frame = 0; frame < 100; ++frame) body.Simulate(0.1f);
		EXPECT_NEAR(body.GetParticle(1).Position.x, 1.25f, 1.0e-5f);
		body.Reset();
		EXPECT(body.GetParticle(0).Pinned);
		EXPECT_SAME(body.GetParticle(1).Position, Vector3::UnitX());
		EXPECT_SAME(body.GetParticle(1).Velocity, Vector3::Zero());
		EXPECT_NEAR(body.GetEdges()[0].RestLength, 1.0f, 1.0e-6f);
		body.AddImpulse(1, Vector3(-10.0f, 0.0f, 0.0f));
		body.Simulate(0.1f);
		EXPECT_NEAR(body.GetEdges()[0].RestLength, 0.75f, 1.0e-5f);

		for (int substeps = 1; substeps <= 4; substeps *= 4)
		{
			for (int overloaded = 0; overloaded < 2; ++overloaded)
			{
				DeformableBodySettings loadedSettings = settings;
				loadedSettings.Substeps = substeps;
				loadedSettings.EdgeCompliance = overloaded ? 0.002f : 0.0001f;
				loadedSettings.PlasticCreep = 2.0f;
				DeformableBody loaded(loadedSettings);
				loaded.AddParticle(Vector3::Zero(), 0.0f);
				loaded.AddParticle(Vector3::UnitX());
				loaded.AddEdge(0, 1);
				for (int frame = 0; frame < 180; ++frame)
				{
					loaded.AddForce(1, Vector3(100.0f, 0.0f, 0.0f));
					loaded.Simulate(1.0f / 30.0f);
				}
				EXPECT_NEAR(loaded.GetEdges()[0].RestLength, overloaded ? 1.25f : 1.0f, 0.001f);
			}
		}
	}

	void TestDeformableGround()
	{
		printf("Running TestDeformableGround\n");
		DeformableBodySettings settings = IsolatedDeformableSettings();
		settings.GroundEnabled = true;
		settings.GroundHeight = 0.0f;
		settings.ParticleRadius = 0.0f;
		settings.Restitution = 0.5f;
		settings.Friction = 0.5f;
		DeformableBody bounce(settings);
		bounce.AddParticle(Vector3(0.0f, 0.1f, 0.0f));
		bounce.SetParticleVelocity(0, Vector3(2.0f, -2.0f, 0.0f));
		bounce.Simulate(0.1f);
		EXPECT_NEAR(bounce.GetParticle(0).Position.y, 0.0f, 1.0e-6f);
		EXPECT_NEAR(bounce.GetParticle(0).Velocity.y, 1.0f, 1.0e-5f);
		EXPECT_NEAR(bounce.GetParticle(0).Velocity.x, 0.5f, 1.0e-5f);

		DeformableBody penetrating(settings);
		penetrating.AddParticle(Vector3(0.0f, -1.0f, 0.0f));
		for (int frame = 0; frame < 120; ++frame) penetrating.Simulate(1.0f / 60.0f);
		EXPECT_NEAR(penetrating.GetParticle(0).Position.y, 0.0f, 1.0e-5f);
		EXPECT_SAME(penetrating.GetParticle(0).Velocity, Vector3::Zero());

		settings.Substeps = 4;
		settings.SolverIterations = 24;
		settings.Gravity = Vector3(0.0f, -9.8f, 0.0f);
		settings.Damping = 0.5f;
		settings.Restitution = 0.0f;
		settings.ParticleRadius = 0.025f;
		DeformableBody falling(settings);
		AddUnitTetrahedronParticles(falling, Vector3(0.0f, 2.0f, 0.0f));
		falling.AddTetrahedron(0, 1, 2, 3);
		for (int frame = 0; frame < 240; ++frame) falling.Simulate(1.0f / 60.0f);
		for (int i = 0; i < falling.GetParticleCount(); ++i)
		{
			EXPECT(falling.GetParticle(i).Position.y >= 0.0249f);
			EXPECT(falling.GetParticle(i).Position.y < 1.1f);
			EXPECT(std::isfinite(falling.GetParticle(i).Velocity.SquareLength()));
		}
		EXPECT_NEAR(falling.GetTetrahedronVolume(0), 1.0f / 6.0f, 0.002f);
	}

	void TestDeformableWorldAndRendering()
	{
		printf("Running TestDeformableWorldAndRendering\n");
		PhysicsWorldParam parameters;
		PhysicsWorld world(parameters);
		DeformableBodySettings settings = IsolatedDeformableSettings();
		settings.Gravity = Vector3(0.0f, -9.8f, 0.0f);
		DeformableBody* body = world.CreateDeformableBody(settings);
		EXPECT(body != nullptr);
		if (!body) return;
		EXPECT(world.GetDeformableBodyCount() == 1);
		AddUnitTetrahedronParticles(*body, Vector3(0.0f, 2.0f, 0.0f));
		body->AddTetrahedron(0, 1, 2, 3);
		EXPECT(body->SetSurfaceTriangles({ Index3(0, 2, 1), Index3(0, 1, 3), Index3(0, 3, 2), Index3(1, 2, 3) }));
		Geometry* geometry = body->GetRenderGeometry();
		EXPECT(geometry != nullptr);
		if (!geometry) return;
		EXPECT_FALSE(geometry->IsQueryEnabled());
		EXPECT_FALSE(geometry->IsSimulationEnabled());
		TriangleMesh* mesh = geometry->GetShapeObj<TriangleMesh>();
		EXPECT(mesh != nullptr);
		if (!mesh) return;
		const Vector3 initial = body->GetParticle(0).Position;
		for (int frame = 0; frame < 10; ++frame) world.Simulate();
		EXPECT(body->GetParticle(0).Position.y < initial.y - 0.05f);
		EXPECT(body->GetRenderGeometry() == geometry);
		EXPECT(mesh->NumVertices == 4 && mesh->NumTriangles == 4);
		for (int i = 0; i < 4; ++i)
		{
			EXPECT_SAME(mesh->Vertices[i], body->GetParticle(i).Position);
			EXPECT_NEAR(mesh->mNormals[(size_t)i].Length(), 1.0f, 1.0e-5f);
		}
		EXPECT_NEAR(geometry->GetShapeBounds().Min.y, body->GetParticle(0).Position.y, 1.0e-5f);
		body->SetParticlePosition(3, body->GetParticle(3).Position + Vector3(0.0f, 0.0f, 1.0f));
		EXPECT_NEAR(geometry->GetShapeBounds().Max.z, 2.0f, 1.0e-5f);
		EXPECT_SAME(mesh->Vertices[3], body->GetParticle(3).Position);
		body->Reset();
		EXPECT_SAME(mesh->Vertices[0], initial);
		EXPECT_NEAR(geometry->GetShapeBounds().Max.z, 1.0f, 1.0e-5f);
		EXPECT(body->SetSurfaceTriangles({ Index3(0, 1, 2) }));
		EXPECT(body->GetRenderGeometry() == geometry);
		EXPECT(mesh->NumTriangles == 1);
		EXPECT(body->SetSurfaceTriangles({}));
		EXPECT(mesh->NumTriangles == 0);
		EXPECT(world.RemoveDeformableBody(body));
		EXPECT(world.GetDeformableBodyCount() == 0);
		EXPECT_FALSE(world.RemoveDeformableBody(nullptr));
		world.CreateDeformableBody(settings);
		world.Reset();
		EXPECT(world.GetDeformableBodyCount() == 0);
	}
}

void TestDeformableSystem()
{
	TestDeformableValidation();
	TestDeformableForcesAndImpulses();
	TestDeformableElasticEdges();
	TestDeformableSignedVolume();
	TestDeformablePlasticityAndReset();
	TestDeformableGround();
	TestDeformableWorldAndRendering();
}

namespace Test_DeformableAttachment_Cases
{
namespace
{
	using namespace Riemann;
	using DVector = Maths::TVector3<double>;
	template<class F> void Reject(F Function)
	{
		bool Threw = false; try { Function(); } catch (const std::exception&) { Threw = true; } EXPECT(Threw);
	}
	DeformableBodySettings FreeSettings()
	{
		DeformableBodySettings S; S.Substeps = 1; S.SolverIterations = 12; S.Gravity = Vector3::Zero(); S.Damping = 0; return S;
	}
	void WeightedKernel()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			Vector3 A(2, 3, 4), B[] = { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() };
			SurfaceContactParticle PA{ &A, Scale }, PB[4]; const double WA = 1, WB[] = { 0.1, 0.2, 0.3, 0.4 };
			double Weight = Scale; Vector3 Before = A / Scale;
			for (int I = 0; I < 4; ++I) { PB[I] = { &B[I], Scale * (I + 2) }; Weight += PB[I].InverseMass * WB[I] * WB[I]; Before += B[I] / PB[I].InverseMass; }
			BarycentricAttachmentConstraint C(&PA, &WA, 1, PB, WB, 4); const auto Error = C.GetError();
			EXPECT_NEAR(C.GetEffectiveInverseMass(), Weight, Weight * 1e-7); EXPECT(C.Solve(0.1f));
			EXPECT_NEAR(C.GetError().SquareLength(), 0, 1e-11); const auto Lambda = C.GetMultiplier();
			EXPECT_NEAR((Lambda + Error / Weight).SquareLength(), 0, 1e-12);
			Vector3 After = A / Scale; for (int I = 0; I < 4; ++I) After += B[I] / PB[I].InverseMass;
			EXPECT_NEAR((After - Before).Length(), 0, 1e-5 / Scale);
			for (int I = 0; I < 20; ++I) C.Solve(0.1f);
			EXPECT_NEAR(C.GetError().SquareLength(), 0, 1e-11);
		}
		Vector3 P[] = { Vector3(0), Vector3(4, 0, 0) }; SurfaceContactParticle Particles[] = { { &P[0], 2 }, { &P[1], 4 } };
		const double WA[] = { 0.75, 0.25 }, WB[] = { 0.25, 0.75 };
		BarycentricAttachmentConstraint Shared(Particles, WA, 2, Particles, WB, 2);
		EXPECT_NEAR(Shared.GetEffectiveInverseMass(), 1.5, 1e-12); Shared.Solve(0.1f);
		EXPECT_NEAR(P[0].x, 4.0 / 3, 1e-6); EXPECT_NEAR(P[1].x, 4.0 / 3, 1e-6);
		BarycentricAttachmentConstraint Identical(Particles, WA, 2, Particles, WA, 2);
		EXPECT(!Identical.Solve(0.1f)); EXPECT(Identical.GetEffectiveInverseMass() == 0);
	}
	void ComplianceAndValidation()
	{
		const double Weight = 1, Compliance = 0.001;
		for (float Dt : { 0.000001f, 0.01f, 1.0f })
		{
			Vector3 A(1, 2, 3), Target(0); SurfaceContactParticle PA{ &A, 2 }, PB{ &Target, 0 };
			BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1, Compliance);
			const double Alpha = Compliance / ((double)Dt * Dt); const DVector Expected = DVector(-1, -2, -3) / (2 + Alpha);
			C.Solve(Dt); const Vector3 Once = A;
			EXPECT_NEAR((C.GetMultiplier() - Expected).SquareLength(), 0, 1e-12);
			for (int I = 0; I < 32; ++I) C.Solve(Dt);
			EXPECT_NEAR((A - Once).Length(), 0, 1e-6);
			EXPECT_NEAR((C.GetError() + C.GetMultiplier() * Alpha).SquareLength(), 0, 1e-12);
			EXPECT_NEAR((C.GetForce(Dt) - C.GetMultiplier() / ((double)Dt * Dt)).SquareLength(), 0, 1e-12);
		}
		Vector3 A(1), B(0); SurfaceContactParticle PA{ &A, 0 }, PB{ &B, 0 };
		BarycentricAttachmentConstraint Fixed(&PA, &Weight, 1, &PB, &Weight, 1);
		EXPECT(!Fixed.Solve(0.1f)); EXPECT(A == Vector3(1)); EXPECT(Fixed.GetForce(0.1f).SquareLength() == 0);
		BarycentricAttachmentConstraint PrescribedSpring(&PA, &Weight, 1, &PB, &Weight, 1, Compliance);
		PrescribedSpring.Solve(0.1f); EXPECT(A == Vector3(1)); EXPECT(B == Vector3(0));
		EXPECT_NEAR((PrescribedSpring.GetForce(0.1f) + DVector(1 / Compliance)).SquareLength(), 0, 1e-12);
		Reject([&] { Fixed.Solve(0); }); Reject([&] { Fixed.GetForce(-1); });
		for (double Bad : { -1.0, 0.0, 0.9, std::numeric_limits<double>::quiet_NaN() })
			Reject([&] { BarycentricAttachmentConstraint C(&PA, &Bad, 1, &PB, &Weight, 1); });
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 5, &PB, &Weight, 1); });
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1, -1); });
		SurfaceContactParticle Conflict{ &A, 1 };
		Reject([&] { BarycentricAttachmentConstraint C(&PA, &Weight, 1, &Conflict, &Weight, 1); });
		PA.InverseMass = 1; BarycentricAttachmentConstraint C(&PA, &Weight, 1, &PB, &Weight, 1);
		B.x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { C.Solve(0.1f); }); EXPECT(A == Vector3(1));
		auto Point = BarycentricPoint::Vertex(0); EXPECT(Point.IsValid(1)); Point.Weights[3] = 1; EXPECT(!Point.IsValid(1));
		Vector3 Large[] = { Vector3(FLT_MAX, 0, 0), Vector3(FLT_MAX, 0, 0) }, Target(-FLT_MAX, 0, 0);
		SurfaceContactParticle Weighted[] = { { &Large[0], 1 }, { &Large[1], 0 } }, Anchor{ &Target, 0 };
		const double SmallWeight[] = { 0.001, 0.999 };
		BarycentricAttachmentConstraint Overflow(Weighted, SmallWeight, 2, &Anchor, &Weight, 1);
		Reject([&] { Overflow.Solve(0.1f); }); EXPECT(Large[0].x == FLT_MAX); EXPECT(Overflow.GetMultiplier().SquareLength() == 0);
	}
	void WorldAnchorAndStorage()
	{
		auto S = FreeSettings(); S.Substeps = 4; S.Gravity = Vector3(0, -9.8f, 0);
		DeformableBody Body(S);
		for (const auto& P : { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }) Body.AddParticle(P);
		Body.AddTetrahedron(0, 1, 2, 3);
		const auto Point = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
		const Vector3 Target = Body.GetPointPosition(Point); const auto Id = Body.AttachToWorld(Point, Target);
		// Reallocation must not invalidate an attachment's stored material point.
		for (int I = 0; I < 128; ++I) Body.AddParticle(Vector3((float)I), 0);
		for (int I = 0; I < 30; ++I) Body.Simulate(1.0f / 60);
		EXPECT_NEAR((Body.GetPointPosition(Point) - Target).Length(), 0, 2e-6);
		EXPECT_NEAR(Body.GetAttachmentForce(Id).y, 4 * 9.8, 0.02);
		EXPECT(Body.SetAttachmentTarget(Id, Target + Vector3::UnitZ())); Body.Simulate(0.1f);
		EXPECT_NEAR((Body.GetPointPosition(Point) - Target - Vector3::UnitZ()).Length(), 0, 2e-6);
		Body.Reset(); EXPECT(Body.GetAttachmentForce(Id).SquareLength() == 0);
		EXPECT(Body.RemoveAttachment(Id)); EXPECT(!Body.RemoveAttachment(Id));
		const auto NewId = Body.AttachToWorld(Point, Target, 0.001); EXPECT(NewId != Id);
		Reject([&] { Body.AttachToWorld(BarycentricPoint::Vertex(999), Target); });
		Reject([&] { Body.GetAttachmentForce(Id); });
	}
	void CoupledBodies()
	{
		for (bool Reverse : { false, true })
		{
			auto S = FreeSettings(); DeformableBody A(S), Independent(S); S.Substeps = 4; DeformableBody B(S);
			A.AddParticle(Vector3(0), 1); B.AddParticle(Vector3(0), 0.5f); Independent.AddParticle(Vector3(0));
			A.SetParticleVelocity(0, Vector3(2, 0, 0)); B.SetParticleVelocity(0, Vector3(-1, 0, 0));
			Independent.AddForce(0, Vector3::UnitY());
			const auto P = BarycentricPoint::Vertex(0); const auto Id = A.AttachToBody(P, &B, P);
			Reject([&] { A.Simulate(0.1f); }); Reject([&] { B.Simulate(0.1f); });
			EXPECT(A.GetParticle(0).Position == Vector3(0)); EXPECT(B.GetParticle(0).Position == Vector3(0));
			DeformableBody::SimulateGroup(Reverse ? std::vector<DeformableBody*>{ &B, &Independent, &A } : std::vector<DeformableBody*>{ &A, &Independent, &B }, 0.1f);
			EXPECT_NEAR((A.GetParticle(0).Position - B.GetParticle(0).Position).Length(), 0, 1e-6);
			EXPECT_NEAR((A.GetParticle(0).Velocity + B.GetParticle(0).Velocity * 2).Length(), 0, 1e-6);
			EXPECT_NEAR(A.GetAttachmentForce(Id).x, -20, 1e-4);
			EXPECT_NEAR(Independent.GetParticle(0).Position.y, 0.01, 1e-7); // One independent substep.
			EXPECT(!A.SetAttachmentTarget(Id, Vector3(1)));
			Reject([&] { DeformableBody::SimulateGroup({ &A, &A, &B }, 0.1f); });
			EXPECT(A.RemoveAttachments(&B) == 1); B.Simulate(0.1f); A.Simulate(0.1f);
		}
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Solver; PhysicsWorld World(Param);
			auto* A = World.CreateDeformableBody(FreeSettings()); auto* B = World.CreateDeformableBody(FreeSettings());
			A->AddParticle(Vector3(0)); B->AddParticle(Vector3(1, 0, 0)); const auto Point = BarycentricPoint::Vertex(0);
			A->AttachToBody(Point, B, Point); B->AttachToBody(Point, A, Point);
			World.Simulate(0.1f); EXPECT_NEAR(A->GetParticle(0).Position.x, 0.5, 1e-6);
			EXPECT_NEAR(B->GetParticle(0).Position.x, 0.5, 1e-6);
			EXPECT(World.RemoveDeformableBody(B)); EXPECT(A->GetAttachmentCount() == 0); World.Simulate(0.1f);
			A->AttachToBody(Point, A, Point); World.Reset(); EXPECT(World.GetDeformableBodyCount() == 0);
		}
	}
	void PinsAndLifetime()
	{
		const auto Point = BarycentricPoint::Vertex(0);
		DeformableBody A(FreeSettings()); A.AddParticle(Vector3(0));
		std::unique_ptr<DeformableBody> B(new DeformableBody(FreeSettings())); B->AddParticle(Vector3(2, 0, 0), 0.5f);
		const auto Id = A.AttachToBody(Point, B.get(), Point);
		A.PinParticle(0, Vector3(0)); DeformableBody::SimulateGroup({ &A, B.get() }, 0.1f);
		EXPECT(A.GetParticle(0).Position == Vector3(0)); EXPECT_NEAR(B->GetParticle(0).Position.Length(), 0, 1e-6);
		EXPECT_NEAR(A.GetAttachmentForce(Id).x, 400, 1e-3);
		A.UnpinParticle(0); A.SetParticlePosition(0, Vector3(0)); B->SetParticlePosition(0, Vector3(2, 0, 0));
		DeformableBody::SimulateGroup({ &A, B.get() }, 0.1f);
		EXPECT_NEAR(A.GetParticle(0).Position.x, 4.0 / 3, 1e-6); EXPECT_NEAR(B->GetParticle(0).Position.x, 4.0 / 3, 1e-6);
		A.AttachToBody(Point, B.get(), Point); EXPECT(A.RemoveAttachment(Id));
		Reject([&] { B->Simulate(0.1f); }); B.reset(); EXPECT(A.GetAttachmentCount() == 0); A.Simulate(0.1f);
		{
			DeformableBody Owner(FreeSettings()); Owner.AddParticle(Vector3(0)); Owner.AttachToBody(Point, &A, Point);
			Reject([&] { A.Simulate(0.1f); });
		}
		A.Simulate(0.1f);
		A.AddParticle(Vector3(4, 0, 0)); A.SetParticlePosition(0, Vector3(0));
		A.AttachToBody(Point, &A, BarycentricPoint::Vertex(1)); A.Simulate(0.1f);
		EXPECT_NEAR(A.GetParticle(0).Position.x, 2, 1e-6); EXPECT_NEAR(A.GetParticle(1).Position.x, 2, 1e-6);
	}
}
void TestDeformableAttachment()
{
	WeightedKernel(); ComplianceAndValidation(); WorldAnchorAndStorage(); CoupledBodies(); PinsAndLifetime();
}
}

void TestDeformableAttachment() { Test_DeformableAttachment_Cases::TestDeformableAttachment(); }

namespace Test_DeformableCollision_Cases
{
namespace
{
	using namespace Riemann;
	using DVector = Maths::TVector3<double>;
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Substeps = 1; S.SolverIterations = 24;
		S.Gravity = Vector3::Zero(); S.Damping = 0; S.GroundEnabled = false; S.ParticleRadius = 0.02f;
		S.Collision.InterCollision = true; S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0;
		return S;
	}
	void Triangle(DeformableBody& Body, float Height, float Size, float InverseMass = 1)
	{
		Body.AddParticle(Vector3(-Size, Height, -Size), InverseMass);
		Body.AddParticle(Vector3(Size, Height, -Size), InverseMass);
		Body.AddParticle(Vector3(0, Height, Size), InverseMass);
		EXPECT(Body.SetSurfaceTriangles({ Index3(0, 1, 2) }));
	}
	DVector Momentum(const DeformableBody& Body)
	{
		DVector P(0); for (const auto& V : Body.GetParticles()) if (V.InverseMass && !V.Pinned)
			P += DVector(V.Velocity.x, V.Velocity.y, V.Velocity.z) / V.InverseMass;
		return P;
	}
	double Distance(const DeformableBody& A, const DeformableBody& B)
	{
		double Result = std::numeric_limits<double>::infinity();
		for (const auto& TA : A.GetSurfaceTriangles()) for (const auto& TB : B.GetSurfaceTriangles())
		{
			Vector3 PA[3], PB[3]; for (int I = 0; I < 3; ++I) { PA[I] = A.GetParticle(TA[I]).Position; PB[I] = B.GetParticle(TB[I]).Position; }
			Result = std::min(Result, SurfaceCollision::Closest(PA, 3, PB, 3).Distance);
		}
		return Result;
	}
	void FreePair()
	{
		for (bool Reverse : { false, true }) for (int Substeps : { 1, 4 })
		{
			auto S = Settings(); DeformableBody A(S); S.Substeps = Substeps; DeformableBody B(S);
			Triangle(A, 0.03f, 1); Triangle(B, 0, 1, 0.5f);
			for (int I = 0; I < 3; ++I) { A.SetParticleVelocity(I, Vector3(0, -0.1f, 0)); B.SetParticleVelocity(I, Vector3(0, 0.05f, 0)); }
			const auto Group = Reverse ? std::vector<DeformableBody*>{ &B, &A } : std::vector<DeformableBody*>{ &A, &B };
			DeformableBody::SimulateGroup(Group, 0.01f);
			EXPECT(Distance(A, B) >= 0.04 - 2e-5);
			EXPECT_NEAR((Momentum(A) + Momentum(B)).Length(), 0, 5e-5);
			EXPECT(A.GetCollisionStats().InterContacts > 0); EXPECT(B.GetCollisionStats().InterContacts > 0);
			EXPECT(A.GetCollisionStats().SelfContacts == 0);
			const float Center = (A.GetParticle(0).Position.y + B.GetParticle(0).Position.y * 2) / 3;
			EXPECT_NEAR(Center, 0.01, 2e-5);
		}
	}
	void SelfAndInterior()
	{
		auto S = Settings(); S.Collision.InterCollision = false; S.Collision.SelfCollision = true;
		DeformableBody Body(S); Triangle(Body, 0, 1, 0);
		for (int I = 0; I < 3; ++I) Body.AddParticle(Body.GetParticle(I).Position + Vector3(0, 0.03f, 0));
		Body.SetSurfaceTriangles({ Index3(0, 1, 2), Index3(3, 4, 5) });
		Body.Simulate(0.01f);
		for (int I = 3; I < 6; ++I) EXPECT(Body.GetParticle(I).Position.y >= 0.04f - 2e-5f);
		EXPECT(Body.GetCollisionStats().SelfContacts > 0); EXPECT(Body.GetCollisionStats().InterContacts == 0);
		DeformableBody Interior(S); Triangle(Interior, 0, 1);
		Interior.AddParticle(Vector3(0, 0.005f, 0)); Interior.Simulate(0.01f);
		EXPECT_NEAR(Interior.GetParticle(3).Position.y, 0.005, 1e-7);
		EXPECT(Interior.GetCollisionStats().Contacts == 0);
	}
	void EdgeContacts()
	{
		auto S = Settings(); DeformableBody A(S), B(S);
		for (const auto& P : { Vector3(-1, 0.03f, 0), Vector3(1, 0.03f, 0), Vector3(-1, 0.03f, -0.1f) }) A.AddParticle(P);
		for (const auto& P : { Vector3(0, 0, -1), Vector3(0, 0, 1), Vector3(0.1f, 0, -1) }) B.AddParticle(P, 0);
		A.SetSurfaceTriangles({ Index3(0, 1, 2) }); B.SetSurfaceTriangles({ Index3(0, 1, 2) });
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
		EXPECT(A.GetCollisionStats().InterContacts > 0); EXPECT(Distance(A, B) >= 0.04 - 2e-5);
	}
	void FrictionAndCompliance()
	{
		for (double Friction : { 0.0, 1.0 }) for (int Iterations : { 1, 24 })
		{
			auto S = Settings(); S.SolverIterations = Iterations;
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = Friction;
			DeformableBody A(S), B(S); Triangle(A, 0.03f, 0.2f); Triangle(B, 0, 3, 0);
			for (int I = 0; I < 3; ++I) A.SetParticleVelocity(I, Vector3(0.5f, 0, 0));
			DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(A.GetParticle(I).Velocity.x, Friction ? 0 : 0.5, 2e-5);
		}
		for (int Iterations : { 1, 24 })
		{
			auto S = Settings(); S.SolverIterations = Iterations; S.Collision.Material.Compliance = 0.0001;
			DeformableBody A(S), B(S); Triangle(A, 0.03f, 0.2f); Triangle(B, 0, 3, 0);
			DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
			for (int I = 0; I < 3; ++I) EXPECT_NEAR(A.GetParticle(I).Position.y, 0.035, 2e-6);
		}
	}
	void PolicyAndValidation()
	{
		auto S = Settings(); DeformableBody A(S); S.Collision.InterCollision = false; DeformableBody B(S);
		Triangle(A, 0.03f, 1); Triangle(B, 0, 1);
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f); EXPECT(A.GetCollisionStats().Contacts == 0);
		B.SetCollisionSettings(A.GetSettings().Collision); DeformableBody::SimulateGroup({ &A, &B }, 0.01f);
		EXPECT(A.GetCollisionStats().Contacts > 0); A.Reset(); EXPECT(A.GetCollisionStats().Contacts == 0);
		auto Bad = S.Collision; Bad.ContactTolerance = 0;
		bool Rejected = false; try { A.SetCollisionSettings(Bad); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		S.Collision = Bad; Rejected = false; try { DeformableBody Invalid(S); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		DeformableBody Empty(Settings()); Empty.AddParticle(Vector3(0)); Rejected = false;
		try { Empty.Simulate(0.01f); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected);
		EXPECT(Empty.AddParticle(Vector3(1)) == 1);
		// Replacing topology and growing particle storage must invalidate every
		// old feature index; collision rows cannot outlive their physical substep.
		for (const auto& P : { Vector3(10, 0, 0), Vector3(11, 0, 0), Vector3(10, 0, 1) }) A.AddParticle(P);
		A.SetSurfaceTriangles({ Index3(3, 4, 5) });
		DeformableBody::SimulateGroup({ &A, &B }, 0.01f); EXPECT(A.GetCollisionStats().Contacts == 0);
	}
	void RigidAttachmentContactChain()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld World(P);
			auto S = Settings(); S.SolverIterations = 48; S.Substeps = 4;
			auto* A = World.CreateDeformableBody(S); S.Substeps = 1; auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0, 1); Triangle(*B, 0.03f, 1);
			for (int I = 0; I < 3; ++I) B->SetParticleVelocity(I, Vector3(0, -0.1f, 0));
			const auto Point = BarycentricPoint::Triangle(Index3(0, 1, 2), Vector3(1.0f / 3));
			const Vector3 Anchor = A->GetPointPosition(Point);
			RigidBodyParam R; R.rigidType = RigidType::Dynamic; R.linearDamping = R.angularDamping = 1; R.sleepThreshold = 0;
			auto* Rigid = World.CreateRigidBody(R, Transform(Anchor))->CastDynamic();
			Rigid->InvMass = 0.5f; Rigid->InvInertia = Matrix3(1, 1, 1);
			World.AttachDeformableToRigid(A, Point, Rigid, Anchor);
			World.Simulate(0.01f);
			const DVector Total = Momentum(*A) + Momentum(*B) + DVector(Rigid->V.x, Rigid->V.y, Rigid->V.z) * 2;
			printf("Soft contact attachment chain solver=%d momentumY=%g rigidV=%g gap=%g\n", (int)Solver, Total.y, Rigid->V.y, Distance(*A, *B));
			EXPECT_NEAR((Total - DVector(0, -0.3, 0)).Length(), 0, 2e-4);
			EXPECT(Rigid->V.y < 0); EXPECT(Distance(*A, *B) >= 0.04 - 2e-5);
			EXPECT_NEAR((A->GetPointPosition(Point) - Rigid->X).Length(), 0, 2e-5);
			EXPECT(B->GetCollisionStats().Contacts > 0);
		}
	}
	void WorldRoutes()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			PhysicsWorldParam P; PhysicsWorld World(P);
			EXPECT(World.SetSolverConfiguration(Solver, SolverBackend::CPU, false));
			auto S = Settings(); auto* A = World.CreateDeformableBody(S); auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0.03f, 1); Triangle(*B, 0, 1); World.Simulate(0.01f);
			EXPECT(Distance(*A, *B) >= 0.04 - 2e-5); EXPECT_NEAR((Momentum(*A) + Momentum(*B)).Length(), 0, 5e-5);
		}
	}
	void TemporalFrameLifetime()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
			for (int WorldSteps : { 1, 4 })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = WorldSteps;
			PhysicsWorld World(P); auto S = Settings(); S.Substeps = 2;
			auto* A = World.CreateDeformableBody(S); auto* B = World.CreateDeformableBody(S);
			Triangle(*A, 0.03f, 1); Triangle(*B, 0, 1);
			for (int I = 0; I < 3; ++I) A->AddForce(I, Vector3::UnitX());
			World.Simulate(0.04f);
			EXPECT_NEAR((Momentum(*A) + Momentum(*B)).x, 0.12, 2e-4);
			// These surfaces separate during the first physical substep. Statistics
			// must retain that collision even when later world substeps have no rows.
			EXPECT(A->GetCollisionStats().Contacts > 0);
			World.Simulate(0.04f);
			EXPECT_NEAR((Momentum(*A) + Momentum(*B)).x, 0.12, 4e-4);
			EXPECT(A->GetCollisionStats().Contacts == 0);
			S.Collision.InterCollision = false;
			auto* Held = World.CreateDeformableBody(S); Held->AddParticle(Vector3(10, 0, 0));
			const auto Id = Held->AttachToWorld(BarycentricPoint::Vertex(0), Vector3(10, 0, 0));
			Held->AddForce(0, Vector3(0, -3, 0)); World.Simulate(0.04f);
			EXPECT_NEAR(Held->GetAttachmentForce(Id).y, 3, 1e-5);
		}
	}
}
void TestDeformableCollision()
{
	FreePair(); SelfAndInterior(); EdgeContacts(); FrictionAndCompliance(); PolicyAndValidation(); WorldRoutes(); TemporalFrameLifetime(); RigidAttachmentContactChain();
}
}

void TestDeformableCollision() { Test_DeformableCollision_Cases::TestDeformableCollision(); }

namespace Test_DeformableRigidContact_Cases
{
namespace
{
	using namespace Riemann;
	using DVector = RigidPointResponse::DVector;
	DVector Double(const Vector3& V) { return DVector(V.x, V.y, V.z); }
	double Length(const DVector& V) { return std::hypot(std::hypot(V.x, V.y), V.z); }
	template<class F> void Reject(F Function)
	{
		bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected);
	}
	SurfaceContact Hit(Vector3 Normal = Vector3::UnitY())
	{
		SurfaceContact C; C.Normal = Normal; C.WeightsA = Vector3::UnitX(); return C;
	}
	void KernelResponse()
	{
		Vector3 Center(0), Point(1, -0.01f, 0); Quaternion Q = Quaternion::One();
		RigidConstraintBody Rigid{ &Center, &Q, 0.5, Matrix3(1, 1, 2) }; SurfaceContactParticle Soft{ &Point, 1 };
		auto C = Hit(); C.PointA = Point; C.PointB = Vector3(1, 0, 0);
		SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 0;
		RigidSurfaceContact Row(&Soft, 1, Rigid, Vector3::UnitX(), C, 0, M);
		const auto Initial = Double(Point) + Double(Center) * 2; Row.Solve(0.01f);
		EXPECT_NEAR(Row.GetNormalMultiplier(), 0.01 / 3.5, 1e-9);
		EXPECT_NEAR(Row.GetRigidAngularMultiplier().z, -Row.GetNormalMultiplier(), 1e-12);
		EXPECT(Length(Double(Point) + Double(Center) * 2 - Initial) < 1e-7);
		EXPECT(Q.z < 0); for (int I = 0; I < 12; ++I) Row.Solve(0.01f); EXPECT(std::abs(Row.GetNormalGap()) < 1e-7);
	}
	void CoupledTangentMass()
	{
		Vector3 Center(0), Arm(1, 2, 3), Point = Arm + Vector3(0.001f, -0.01f, 0.003f); Quaternion Q = Quaternion::One();
		RigidConstraintBody Rigid{ &Center, &Q, 0.5, Matrix3(1, 2, 3) }; SurfaceContactParticle Soft{ &Point, 1 };
		auto C = Hit(); C.PointA = Arm; C.PointB = Arm; SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 100;
		RigidPointResponse Response(Rigid, Arm); auto K = Response.Evaluate().InversePointMass; for (int I = 0; I < 3; ++I) K[I][I] += 1;
		const auto Error = Double(Point) - Double(Arm); const double N = -Error.y / K[1][1];
		const double X = Error.x + K[0][1] * N, Z = Error.z + K[2][1] * N, Determinant = K[0][0] * K[2][2] - K[0][2] * K[2][0];
		const double TX = (-X * K[2][2] + Z * K[0][2]) / Determinant, TZ = (-Z * K[0][0] + X * K[2][0]) / Determinant;
		RigidSurfaceContact Row(&Soft, 1, Rigid, Arm, C, 0, M); Row.Solve(0.01f);
		EXPECT_NEAR(Row.GetNormalMultiplier(), N, 1e-12); EXPECT_NEAR(Row.GetTangentMultiplier().x, TX, 1e-12);
		EXPECT_NEAR(Row.GetTangentMultiplier().z, TZ, 1e-12); EXPECT(!Row.IsSliding());
	}
	void SlidingDirection()
	{
		Vector3 Center(0), Arm(1, 2, 3), Point = Arm + Vector3(0.1f, -0.01f, 0.02f); Quaternion Q = Quaternion::One();
		RigidConstraintBody R{ &Center, &Q, 0.5, Matrix3(1, 2, 3) }; SurfaceContactParticle P{ &Point, 1 };
		auto C = Hit(); C.PointA = C.PointB = Arm; SurfaceContactMaterial M;
		RigidPointResponse Response(R, Arm); auto K = Response.Evaluate().InversePointMass; for (int I = 0; I < 3; ++I) K[I][I] += 1;
		const auto InitialSlip = Double(Point) - Double(Arm); RigidSurfaceContact Row(&P, 1, R, Arm, C, 0, M); Row.Solve(0.01f);
		const auto T = Row.GetTangentMultiplier(); const auto FinalSlip = InitialSlip + K * (DVector(0, Row.GetNormalMultiplier(), 0) + T);
		EXPECT(Row.IsSliding()); EXPECT_NEAR(Length(T), M.DynamicFriction * Row.GetNormalMultiplier(), 1e-12);
		// The sliding impulse opposes the post-response slip, including anisotropic
		// point mass. Clipping the free impulse radially fails this cone condition.
		EXPECT_NEAR(T.x * FinalSlip.z - T.z * FinalSlip.x, 0, 1e-12); EXPECT(T.x * FinalSlip.x + T.z * FinalSlip.z <= 0);
	}
	void KernelPolicies()
	{
		for (int Iterations : { 1, 32 })
		{
			Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One(); auto C = Hit();
			RigidConstraintBody R{ &Center, &Q, 0, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 };
			SurfaceContactMaterial M; M.Compliance = 0.0001; RigidSurfaceContact Row(&P, 1, R, Vector3::Zero(), C, 0, M);
			for (int I = 0; I < Iterations; ++I) { Row.Refresh(&P, 1, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); }
			EXPECT_NEAR(Point.y, -0.005, 1e-8);
		}
		for (double InverseMass : { 1e-200, 1.0, 1e200 })
		{
			Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One();
			RigidConstraintBody R{ &Center, &Q, InverseMass, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 }; auto C = Hit();
			RigidSurfaceContact Row(&P, 1, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); EXPECT(std::abs(Row.GetNormalGap()) < 1e-7);
		}
		Vector3 Center(0), Point(0, -0.01f, 0); Quaternion Q = Quaternion::One();
		RigidConstraintBody R{ &Center, &Q, 0, Matrix3::Zero() }; SurfaceContactParticle P{ &Point, 1 }; auto C = Hit();
		C.WeightsA = Vector3(0.5f, 0.5f, 0); SurfaceContactParticle Duplicate[] = { P, P };
		RigidSurfaceContact Row(Duplicate, 2, R, Vector3::Zero(), C, 0); Row.Solve(0.01f); EXPECT_NEAR(Point.y, 0, 1e-7);
		C = Hit(); C.Normal = Vector3::Zero(); Reject([&] { RigidSurfaceContact Bad(&P, 1, R, Vector3::Zero(), C, 0); });
		C = Hit(); SurfaceContactParticle Alias{ &Center, 1 }; Reject([&] { RigidSurfaceContact Bad(&Alias, 1, R, Vector3::Zero(), C, 0); });
		Vector3 Positions[] = { Vector3(FLT_MAX, 0, 0), Vector3(FLT_MAX, 0, 0) }; Center = Vector3(-FLT_MAX, 0, 0);
		SurfaceContactParticle Large[] = { { &Positions[0], 1 }, { &Positions[1], 0 } };
		C = Hit(-Vector3::UnitX()); C.WeightsA = Vector3(0.001f, 0.999f, 0);
		RigidSurfaceContact Overflow(Large, 2, R, Vector3::Zero(), C, 0); Reject([&] { Overflow.Solve(0.01f); });
		EXPECT(Positions[0].x == FLT_MAX); EXPECT(Center.x == -FLT_MAX); EXPECT(Overflow.GetNormalMultiplier() == 0);
		Center = Vector3::Zero(); Point = Vector3(0, -0.01f, 0); R.InverseMass = 1e308; C = Hit();
		RigidSurfaceContact MassOverflow(&P, 1, R, Vector3::Zero(), C, 0);
		Reject([&] { MassOverflow.Solve(FLT_MAX); }); EXPECT(Center == Vector3::Zero()); EXPECT(Point.y == -0.01f);
		EXPECT(MassOverflow.GetNormalMultiplier() == 0);
	}
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Substeps = 4; S.SolverIterations = 32; S.Damping = 0;
		S.Gravity = Vector3::Zero(); S.GroundEnabled = false; S.ParticleRadius = 0.02f; S.Collision.RigidCollision = true;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0; return S;
	}
	DeformableBody* Triangle(PhysicsWorld& W, const DeformableBodySettings& S, float Y, float Size = 1)
	{
		auto* B = W.CreateDeformableBody(S);
		B->AddParticle(Vector3(-Size, Y, -Size)); B->AddParticle(Vector3(Size, Y, -Size)); B->AddParticle(Vector3(0, Y, Size));
		B->SetSurfaceTriangles({ Index3(0, 1, 2) }); return B;
	}
	RigidBodyDynamic* Dynamic(PhysicsWorld& W, Geometry* Shape)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* R = W.CreateRigidBody(Shape, P)->CastDynamic(); R->InvMass = 0.5f; R->InvInertia = Matrix3(1, 1, 1); R->SetFriction(1, 1); return R;
	}
	DVector Momentum(const DeformableBody* B)
	{
		DVector Result(0); for (const auto& P : B->GetParticles()) if (!P.Pinned && P.InverseMass) Result += Double(P.Velocity) / P.InverseMass; return Result;
	}
	void WorldMomentum()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD }) for (bool Friction : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld W(P); auto S = Settings();
			S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = Friction ? 1 : 0;
			auto* Soft = Triangle(W, S, 0.51f); auto* R = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
			for (int I = 0; I < 3; ++I) Soft->SetParticleVelocity(I, Vector3(0.2f, -0.1f, 0));
			const auto Before = Momentum(Soft); W.Simulate(0.01f); const auto Total = Momentum(Soft) + Double(R->V) * 2;
			printf("Soft-rigid contact solver=%d friction=%d momentumError=%g angularZ=%g contacts=%zu\n", (int)Solver, Friction, Length(Total - Before), R->W.z, Soft->GetCollisionStats().RigidContacts);
			EXPECT(Length(Total - Before) < 5e-4); EXPECT(Soft->GetCollisionStats().RigidContacts > 0); EXPECT(R->V.y < 0);
			if (Friction) EXPECT(std::abs(R->W.z) > 1e-4); else EXPECT(std::abs(R->W.z) < 1e-5);
			EXPECT(!R->Sleeping);
		}
	}
	void ShapesAndFiltering()
	{
		for (int Shape = 0; Shape < 6; ++Shape)
		{
			printf("Soft-rigid shape=%d\n", Shape); fflush(stdout);
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
			Geometry* G = nullptr; float Y = 0.51f;
			if (Shape == 0) G = GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f);
			if (Shape == 1) G = GeometryFactory::CreateCapsule(Vector3(-0.4f, 0, 0), Vector3(0.4f, 0, 0), 0.5f);
			if (Shape == 2) G = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f));
			if (Shape == 3)
			{
				G = GeometryFactory::CreateConvexMesh(); std::vector<Vector3> V;
				for (int I = 0; I < 8; ++I) V.emplace_back(I & 1 ? 0.5f : -0.5f, I & 2 ? 0.5f : -0.5f, I & 4 ? 0.5f : -0.5f);
				auto* M = G->GetShapeObj<ConvexMesh>(); EXPECT(M->BuildFromPoints(V.data(), (int)V.size())); G->SetShapeBounds(M->Bounds);
			}
			if (Shape == 4) { G = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()); Y = 0.01f; }
			if (Shape == 5)
			{
				G = GeometryFactory::CreateTriangleMesh(); auto* M = G->GetShapeObj<TriangleMesh>();
				for (const auto& V : { Vector3(-3, 0, -3), Vector3(3, 0, -3), Vector3(0, 0, 3) }) M->AddVertex(V);
				M->AddTriangle(0, 1, 2); M->Compact(); G->SetShapeBounds(M->BoundingVolume); Y = 0.01f;
			}
			if (Shape == 2 || Shape == 3) Y = 0.4f;
			RigidBodyParam R; W.CreateRigidBody(G, R); auto* Soft = Triangle(W, S, Y, Shape >= 2 ? 0.2f : 1);
			W.Simulate(0.01f); EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
			for (const auto& V : Soft->GetParticles()) EXPECT(std::isfinite(V.Position.SquareLength()));
			if (Shape >= 2) for (const auto& V : Soft->GetParticles()) EXPECT(V.Position.y >= Y + 0.009f);
			if (Shape == 2 || Shape == 3) for (const auto& V : Soft->GetParticles()) EXPECT(V.Position.y >= 0.52f - 1e-5f);
		}
		for (int Policy = 0; Policy < 4; ++Policy)
		{
			printf("Soft-rigid filter policy=%d\n", Policy); fflush(stdout);
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::PBD; PhysicsWorld W(P); auto S = Settings();
			S.Collision.FilterData = CollisionData::ForLayer(1, Policy == 1 ? 0 : 1u << 2);
			W.SetCollisionFilter(std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask()));
			auto* Soft = Triangle(W, S, 0.51f); auto* G = GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f);
			G->SetFilterData(CollisionData::ForLayer(2, 1u << 1)); G->SetQueryEnabled(false);
			if (Policy == 2) G->SetSimulationEnabled(false); if (Policy == 3) G->SetTriggerEnabled(true);
			RigidBodyParam R; W.CreateRigidBody(G, R); W.Simulate(0.01f);
			EXPECT((Soft->GetCollisionStats().RigidContacts > 0) == (Policy == 0));
		}
	}
	void MeshFeatureContacts()
	{
		for (bool EdgeOnly : { false, true })
		{
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
			auto* Soft = W.CreateDeformableBody(S);
			const std::vector<Vector3> A = EdgeOnly
				? std::vector<Vector3>{ Vector3(-1, 0.01f, 0), Vector3(1, 0.01f, 0), Vector3(-1, 0.01f, -0.1f) }
				: std::vector<Vector3>{ Vector3(-1, 0.01f, -1), Vector3(1, 0.01f, -1), Vector3(0, 0.01f, 1) };
			const std::vector<Vector3> B = EdgeOnly
				? std::vector<Vector3>{ Vector3(0, 0, -1), Vector3(0, 0, 1), Vector3(0.1f, 0, -1) }
				: std::vector<Vector3>{ Vector3(-0.1f, 0, -0.1f), Vector3(0.1f, 0, -0.1f), Vector3(0, 0, 0.1f) };
			for (const auto& V : A) Soft->AddParticle(V); Soft->SetSurfaceTriangles({ Index3(0, 1, 2) });
			auto* G = GeometryFactory::CreateTriangleMesh(); auto* M = G->GetShapeObj<TriangleMesh>();
			for (const auto& V : B) M->AddVertex(V); M->AddTriangle(0, 1, 2); M->Compact(); G->SetShapeBounds(M->BoundingVolume);
			RigidBodyParam R; W.CreateRigidBody(G, R); W.Simulate(0.01f);
			Vector3 Current[3]; for (int I = 0; I < 3; ++I) Current[I] = Soft->GetParticle(I).Position;
			const double Distance = SurfaceCollision::Closest(Current, 3, B.data(), 3).Distance;
			printf("Soft-rigid mesh edgeOnly=%d distance=%g contacts=%zu\n", EdgeOnly, Distance, Soft->GetCollisionStats().RigidContacts);
			EXPECT(Soft->GetCollisionStats().RigidContacts > 0); EXPECT(Distance >= S.ParticleRadius - 1e-5);
		}
	}
	class SpinField : public ForceField
	{
	public:
		RigidBody* Body;
		explicit SpinField(RigidBody* B) : Body(B) {}
		void Update(float) override { Body->Q = Quaternion(0.01f, Vector3::UnitZ()); Body->UpdateGeometries(); }
		bool ApplyForce(RigidBodyDynamic*) override { return false; }
	};
	void MovingSurfaceAndPolicies()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; PhysicsWorld W(P); auto S = Settings(); S.Substeps = 1;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 10;
		auto* Soft = Triangle(W, S, 0.51f); RigidBodyParam R;
		auto* Body = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f), R); Body->SetFriction(10, 10);
		W.AddForceField(new SpinField(Body)); W.Simulate(0.01f); EXPECT(Momentum(Soft).x < -0.01);
		EXPECT(!W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
		auto Support = W.GetSupportStabilizationSettings(); Support.Enabled = true; Reject([&] { W.SetSupportStabilizationSettings(Support); });
		Reject([&] { Soft->Simulate(0.01f); }); auto C = Soft->GetSettings().Collision; C.RigidCollision = false; Soft->SetCollisionSettings(C);
		EXPECT(W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
		Reject([&] { W.CreateDeformableBody(S); });
	}
	void OffsetCenterAndWake()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD; P.islandSleeping.Enabled = true;
		PhysicsWorld W(P); auto* Soft = Triangle(W, Settings(), 0.51f);
		auto* R = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
		R->CM = R->X = Vector3::UnitX(); R->UpdateGeometries(); R->Sleeping = true;
		W.Simulate(0.01f); EXPECT(!R->Sleeping); EXPECT(R->W.z > 0);
		EXPECT(Length(Momentum(Soft) + Double(R->V) * 2) < 0.002);
		EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
	}
	void CallbackLifetime()
	{
		bool Called = false; PhysicsWorld* Active = nullptr; DeformableBody* Soft = nullptr; RigidBodyDynamic* Rigid = nullptr;
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::PBD;
		P.onTrigger = [&](const PhysicsTriggerInfo&)
		{
			if (Called) return; Called = true;
			Reject([&] { Active->RemoveRigidBody(Rigid); }); Reject([&] { Active->Reset(); });
			Reject([&] { Active->CreateDeformableBody(DeformableBodySettings()); }); Reject([&] { Active->RemoveDeformableBody(Soft); });
			RigidBodyParam R; Reject([&] { Active->CreateRigidBody(R, Transform::Identity()); });
			Reject([&] { Soft->SetCollisionSettings(Soft->GetSettings().Collision); });
			Reject([&] { Active->SetSupportStabilizationSettings(Active->GetSupportStabilizationSettings()); });
			EXPECT(!Active->SetSolverConfiguration(ResolutionPhaseSolver::PBD, SolverBackend::CPU, false));
			throw std::runtime_error("coupled contact callback fixture");
		};
		PhysicsWorld W(P); Active = &W; Soft = Triangle(W, Settings(), 0.51f);
		Rigid = Dynamic(W, GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f));
		auto* Shape = GeometryFactory::CreateSphere(Vector3::Zero(), 1); Shape->SetTriggerEnabled(true); RigidBodyParam R; W.CreateRigidBody(Shape, R);
		Reject([&] { W.Simulate(0.01f); }); EXPECT(Called);
		EXPECT(Soft->AddParticle(Vector3(4)) == 3); EXPECT(W.RemoveRigidBody(Rigid)); Rigid->ReleaseGeometries(); delete Rigid;
		W.Simulate(0.01f); EXPECT(Soft->GetCollisionStats().RigidContacts == 0);
	}
}
void TestDeformableRigidContact()
{
	KernelResponse(); CoupledTangentMass(); SlidingDirection(); KernelPolicies(); WorldMomentum(); ShapesAndFiltering(); MeshFeatureContacts(); MovingSurfaceAndPolicies(); OffsetCenterAndWake();  CallbackLifetime();
}
}

void TestDeformableRigidContact() { Test_DeformableRigidContact_Cases::TestDeformableRigidContact(); }

namespace Test_TetrahedralMaterial_Cases
{
namespace
{
	using namespace Riemann;
	using DVector = DeformationGradient::DVector;
	DVector Double(const Vector3& P) { return DVector(P.x, P.y, P.z); }
	double Length(const DVector& P) { return std::hypot(std::hypot(P.x, P.y), P.z); }
	template<class F> void Reject(F Function) { bool Threw = false; try { Function(); } catch (const std::exception&) { Threw = true; } EXPECT(Threw); }
	DeformationGradient Identity() { DeformationGradient F; for (int I = 0; I < 3; ++I) F[I][I] = 1; return F; }
	void UnitTet(DeformableBody& Body)
	{
		for (const auto& P : { Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }) Body.AddParticle(P);
		EXPECT(Body.AddTetrahedron(0, 1, 2, 3) == 0);
	}
	void StressAndCalibration()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
		{
			TetrahedralMaterial M; M.Model = Model; auto F = Identity(); auto R = TetrahedralElasticity::Evaluate(F, M);
			EXPECT_NEAR(R.EnergyDensity, 0, 1e-10); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(R.FirstPiola[I][J], 0, 1e-10);
			F[0][0] = 1.12; F[1][1] = 0.93; F[2][2] = 1.04; F[0][1] = 0.1; F[2][0] = -0.07;
			R = TetrahedralElasticity::Evaluate(F, M); constexpr double H = 1e-6;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{
				auto Plus = F, Minus = F; Plus[I][J] += H; Minus[I][J] -= H;
				const double Derivative = (TetrahedralElasticity::Evaluate(Plus, M).EnergyDensity - TetrahedralElasticity::Evaluate(Minus, M).EnergyDensity) / (2 * H);
				EXPECT_NEAR(Derivative, R.FirstPiola[I][J], 2e-5 * std::max(1.0, std::abs(Derivative)));
			}
			for (double Nu : { 0.0, 0.25, 0.49 })
			{
				M.PoissonRatio = Nu; auto Plus = Identity(), Minus = Identity(); Plus[0][0] += H; Minus[0][0] -= H;
				const auto P = TetrahedralElasticity::Evaluate(Plus, M), N = TetrahedralElasticity::Evaluate(Minus, M);
				EXPECT_NEAR((P.FirstPiola[0][0] - N.FirstPiola[0][0]) / (2 * H), M.LameLambda() + 2 * M.ShearModulus(), 1e-5 * M.YoungModulus);
				EXPECT_NEAR((P.FirstPiola[1][1] - N.FirstPiola[1][1]) / (2 * H), M.LameLambda(), 1e-5 * M.YoungModulus);
			}
			M.PoissonRatio = 0.3; const double C = std::cos(0.7), S = std::sin(0.7); auto Rotated = F;
			for (int J = 0; J < 3; ++J) { Rotated[0][J] = C * F[0][J] - S * F[1][J]; Rotated[1][J] = S * F[0][J] + C * F[1][J]; }
			const auto Q = TetrahedralElasticity::Evaluate(Rotated, M); EXPECT_NEAR(Q.EnergyDensity, R.EnergyDensity, 1e-8);
			for (int J = 0; J < 3; ++J)
			{
				EXPECT_NEAR(Q.FirstPiola[0][J], C * R.FirstPiola[0][J] - S * R.FirstPiola[1][J], 1e-8);
				EXPECT_NEAR(Q.FirstPiola[1][J], S * R.FirstPiola[0][J] + C * R.FirstPiola[1][J], 1e-8);
			}
		}
	}
	void RestShapesAndKernel()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
			for (float Scale : { 0.001f, 1.0f, 1000.0f }) for (bool Reverse : { false, true })
		{
			Vector3 Rest[] = { Vector3(0), Vector3(Scale, 0, 0), Vector3(0, Scale, 0), Vector3(0, 0, Scale) };
			if (Reverse) std::swap(Rest[1], Rest[2]); const auto Shape = TetrahedralRestShape::Create(Rest);
			const auto F = Shape.Evaluate(Rest); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(F[I][J], I == J ? 1 : 0, 1e-12);
			Vector3 P[4]; Vector3* Positions[4]; float Masses[] = { 1, 0.5f, 0.25f, 2 }; double TotalMass = 0;
			for (float& W : Masses) { W = (float)(W / ((double)Scale * Scale * Scale)); TotalMass += 1 / (double)W; }
			for (int I = 0; I < 4; ++I) { P[I] = Rest[I]; Positions[I] = &P[I]; }
			TetrahedralMaterial M; M.Model = Model; TetrahedralMaterialState State;
			for (int I = 0; I < 16; ++I) TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f * Scale);
			for (int I = 0; I < 4; ++I) EXPECT_NEAR((P[I] - Rest[I]).Length() / Scale, 0, 1e-6);
			P[1] += Vector3(0.05f, 0.01f, 0) * Scale; State = TetrahedralMaterialState();
			Vector3 Predicted[4]; for (int I = 0; I < 4; ++I) Predicted[I] = P[I];
			for (int Iteration = 0; Iteration < 64; ++Iteration)
			{
				DVector Center(0); for (int I = 0; I < 4; ++I) Center += Double(P[I]) / Masses[I];
				TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f * Scale);
				DVector Next(0), Force(0), Torque(0); const auto R = TetrahedralElasticity::Evaluate(Shape.Evaluate(P), M);
				for (int I = 0; I < 4; ++I)
				{
					Next += Double(P[I]) / Masses[I]; const auto NodalForce = (R.FirstPiola * Shape.Gradients[I]) * Shape.Volume;
					Force += NodalForce; Torque += Double(P[I]).Cross(NodalForce);
				}
				EXPECT(Length(Next - Center) / (Scale * TotalMass) < 2e-6);
				EXPECT(Length(Force) / (M.YoungModulus * Shape.Volume / Scale) < 1e-10);
				EXPECT(Length(Torque) / (M.YoungModulus * Shape.Volume) < 1e-10);
				EXPECT(Shape.Evaluate(P).Determinant() >= 0.05);
			}
			// Newton iterates are not independent physical impulses. Verify the final
			// inertia/elastic force balance and angular change about the predicted pose.
			DVector Angular(0); const auto Response = TetrahedralElasticity::Evaluate(Shape.Evaluate(P), M);
			for (int I = 0; I < 4; ++I)
			{
				const auto Correction = Double(P[I]) - Double(Predicted[I]); Angular += Double(Predicted[I]).Cross(Correction) / Masses[I];
				const auto Balance = Correction / (Masses[I] * (double)(0.01f * Scale) * (0.01f * Scale)) + (Response.FirstPiola * Shape.Gradients[I]) * Shape.Volume;
				EXPECT(Length(Balance) / (M.YoungModulus * Shape.Volume / Scale) < 2e-5);
			}
			EXPECT(Length(Angular) / (Scale * Scale * TotalMass) < 2e-6);
		}
	}
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Gravity = Vector3::Zero(); S.GroundEnabled = false; S.Damping = 0; S.Substeps = 1; S.SolverIterations = 32; S.EdgeCompliance = 0; return S;
	}
	void MaterialAssignment()
	{
		DeformableBody B(Settings()); UnitTet(B); TetrahedralMaterial M; M.Density = 6;
		const int A = B.AddMaterial(M); B.SetTetrahedronMaterial(0, A);
		for (const auto& E : B.GetEdges()) EXPECT(!E.Enabled);
		B.AddParticle(Vector3(0, 0, -1)); EXPECT(B.AddTetrahedron(0, 1, 2, 4) == 1);
		for (const auto& E : B.GetEdges()) if (E.ParticleA != 3 && E.ParticleB != 3) EXPECT(E.Enabled);
		M.Density = 12; const int C = B.AddMaterial(M); B.SetTetrahedronMaterial(1, C);
		for (const auto& E : B.GetEdges()) EXPECT(!E.Enabled);
		B.AddEdge(0, 1); EXPECT(B.GetEdges()[0].Explicit && B.GetEdges()[0].Enabled);
		B.PinParticle(3, B.GetParticle(3).Position); B.ApplyMaterialDensities();
		EXPECT_NEAR(B.GetParticle(0).InverseMass, 4.0 / 3, 1e-6); EXPECT_NEAR(B.GetParticle(3).InverseMass, 4, 1e-6);
		EXPECT_NEAR(B.GetParticle(4).InverseMass, 2, 1e-6); EXPECT(B.GetParticle(3).Pinned);
		EXPECT_NEAR(B.GetMaterialEnergy(), 0, 1e-10); B.SetTetrahedronMaterial(0, -1);
		for (int I : B.GetTetrahedra()[0].Edges) EXPECT(B.GetEdges()[I].Enabled);
		Reject([&] { B.SetTetrahedronMaterial(99, A); }); Reject([&] { B.SetTetrahedronMaterial(0, 99); });
		M.PoissonRatio = 0.5; Reject([&] { B.SetMaterial(A, M); }); EXPECT(B.GetMaterials()[A].PoissonRatio == 0.3);
		Reject([&] { B.GetTetrahedronMaterialResponse(0); });
	}
	void StaticCalibration()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
			for (double Young : { 1000.0, 10000.0 }) for (int Substeps : { 1, 2 })
		{
			auto S = Settings(); S.Substeps = Substeps; S.SolverIterations = 64; S.Damping = 12;
			DeformableBody B(S); UnitTet(B); B.PinParticle(0, Vector3::Zero()); TetrahedralMaterial M;
			M.Model = Model; M.YoungModulus = Young; M.PoissonRatio = 0.25; M.Density = 6;
			B.SetTetrahedronMaterial(0, B.AddMaterial(M)); B.ApplyMaterialDensities();
			constexpr double Stress = 1; for (int I = 0; I < 300; ++I) { B.AddForce(2, Vector3(0, (float)(Stress / 6), 0)); B.Simulate(0.01f); }
			const double Longitudinal = B.GetParticle(2).Position.y - 1, Transverse = (B.GetParticle(1).Position.x + B.GetParticle(3).Position.z) / 2 - 1;
			printf("FEM calibration model=%d E=%g substeps=%d strain=%g expected=%g lateral=%g expected=%g\n", (int)Model, Young, Substeps, Longitudinal, Stress / Young, Transverse, -M.PoissonRatio * Stress / Young);
			const auto Response = B.GetTetrahedronMaterialResponse(0);
			printf("FEM stress: xx=%g yy=%g velocityY=%g\n", Response.FirstPiola[0][0], Response.FirstPiola[1][1], B.GetParticle(2).Velocity.y);
			EXPECT_NEAR(Longitudinal, Stress / Young, Stress / Young * 0.02);
			EXPECT_NEAR(Transverse, -M.PoissonRatio * Stress / Young, M.PoissonRatio * Stress / Young * 0.02);
			EXPECT(B.GetMaterialEnergy() > 0); B.Reset(); EXPECT_NEAR(B.GetMaterialEnergy(), 0, 1e-10);
		}
	}
	void ValidationAndRecovery()
	{
		Vector3 R[] = { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }; const auto Shape = TetrahedralRestShape::Create(R);
		Vector3 P[4]; Vector3* Positions[4]; float Masses[] = { 1, 1, 1, 1 };
		for (int I = 0; I < 4; ++I) { P[I] = R[I]; Positions[I] = &P[I]; }
		TetrahedralMaterial M; TetrahedralMaterialState State;
		P[3].z = -0.1f;
		for (int I = 0; I < 64; ++I) TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f);
		EXPECT(Shape.Evaluate(P).Determinant() >= 0.05 - 1e-5);
		P[1].x = std::numeric_limits<float>::quiet_NaN(); const auto Before = P[0];
		Reject([&] { TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f); }); EXPECT(P[0] == Before);
		R[3] = R[2]; Reject([&] { TetrahedralRestShape::Create(R); });
		M.PoissonRatio = -0.1; EXPECT(!M.IsValid());
	}
}
void TestTetrahedralMaterial()
{
	StressAndCalibration(); RestShapesAndKernel(); MaterialAssignment(); StaticCalibration(); ValidationAndRecovery();
}
}

void TestTetrahedralMaterial() { Test_TetrahedralMaterial_Cases::TestTetrahedralMaterial(); }

namespace Test_TetrahedralAsset_Cases
{
using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{
		bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected);
	}
	TetrahedralAssetInput Unit(float Scale = 1)
	{
		TetrahedralAssetInput Input;
		Input.Positions = { Vector3(0), Vector3(Scale, 0, 0), Vector3(0, Scale, 0), Vector3(0, 0, Scale) };
		Input.Elements.push_back({ { 0, 1, 2, 3 }, 0 }); Input.Materials[0].Density = 6;
		return Input;
	}
	Vector3 Reconstruct(const TetrahedralAsset& Asset, const TetrahedralRenderBinding& B)
	{
		DeformationGradient::DVector P(0); const auto& E = Asset.GetElements()[B.Element];
		for (int I = 0; I < 4; ++I) { const auto& X = Asset.GetPositions()[E.Vertices[I]]; P += DeformationGradient::DVector(X.x, X.y, X.z) * B.Weights[I]; }
		return Vector3((float)P.x, (float)P.y, (float)P.z);
	}
	void PreparationAndTopology()
	{
		auto Input = Unit(); Input.Positions.push_back(-Vector3::UnitZ());
		Input.Materials.push_back(Input.Materials[0]); Input.Materials[1].Density = 12;
		Input.Elements.push_back({ { 0, 1, 2, 4 }, 1 }); Input.PinnedVertices = { 0 };
		const auto Asset = TetrahedralAsset::Prepare(Input);
		EXPECT(Asset.GetElements().size() == 2 && Asset.GetEdges().size() == 9 && Asset.GetBoundary().size() == 6);
		EXPECT(Asset.GetStats().ReorientedElements == 1 && Asset.GetStats().ConnectedComponents == 1);
		EXPECT_NEAR(Asset.GetStats().TotalVolume, 1.0 / 3, 1e-12); EXPECT_NEAR(Asset.GetStats().TotalMass, 3, 1e-12);
		for (int I = 0; I < 3; ++I) EXPECT_NEAR(1 / Asset.GetInverseMasses()[I], 0.75, 1e-7);
		EXPECT_NEAR(1 / Asset.GetInverseMasses()[3], 0.25, 1e-7); EXPECT_NEAR(1 / Asset.GetInverseMasses()[4], 0.5, 1e-7);
		EXPECT(Asset.GetPins()[0]); double BoundaryVolume = 0;
		for (const auto& F : Asset.GetBoundary()) BoundaryVolume += Asset.GetPositions()[F.a].Dot(Asset.GetPositions()[F.b].Cross(Asset.GetPositions()[F.c])) / 6.0;
		EXPECT_NEAR(BoundaryVolume, Asset.GetStats().TotalVolume, 1e-12);
		for (size_t I = 0; I < Asset.GetPositions().size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Asset.GetRenderBindings()[I]) - Asset.GetPositions()[I]).Length(), 0, 1e-7);
		const auto Face = Asset.BindPoints({ Vector3(0.2f, 0.3f, 0) }); EXPECT(Face[0].Element == 0);
		// Six tetrahedra around a cube diagonal exercise interior face cancellation
		// and connected boundary vertex links beyond a two-element fixture.
		TetrahedralAssetInput Cube;
		Cube.Positions = { Vector3(0,0,0), Vector3(1,0,0), Vector3(1,1,0), Vector3(0,1,0), Vector3(0,0,1), Vector3(1,0,1), Vector3(1,1,1), Vector3(0,1,1) };
		for (const auto& T : { std::array<int,4>{0,1,2,6}, {0,2,3,6}, {0,3,7,6}, {0,7,4,6}, {0,4,5,6}, {0,5,1,6} }) Cube.Elements.push_back({ T, 0 });
		const auto Box = TetrahedralAsset::Prepare(Cube); EXPECT(Box.GetBoundary().size() == 12); EXPECT_NEAR(Box.GetStats().TotalVolume, 1, 1e-12);
		auto Separate = Unit(); for (const auto& P : Unit().Positions) Separate.Positions.push_back(P + Vector3(3,0,0));
		Separate.Elements.push_back({ {4,5,6,7}, 0 }); EXPECT(TetrahedralAsset::Prepare(Separate).GetStats().ConnectedComponents == 2);
	}
	void InvalidAssets()
	{
		auto Bad = Unit(); Bad.Elements[0].Vertices[3] = 8; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Elements.push_back(Bad.Elements[0]); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Elements[0].Material = 1; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions.push_back(Vector3(2)); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions[3].z = 1e-7f; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions[0].x = std::numeric_limits<float>::infinity(); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.PinnedVertices = { -1 }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		// Geometric overlap without any shared connectivity must also fail.
		Bad = Unit(); for (const auto& P : Unit().Positions) Bad.Positions.push_back(P + Vector3(0.1f));
		Bad.Elements.push_back({ {4,5,6,7}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		// Two otherwise closed volumes joined at one vertex form a pinched boundary.
		Bad = Unit(); Bad.Positions.insert(Bad.Positions.end(), { -Vector3::UnitX(), -Vector3::UnitY(), -Vector3::UnitZ() });
		Bad.Elements.push_back({ {0,4,5,6}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions.push_back(Vector3(0,0,2)); Bad.Elements.push_back({ {0,1,2,4}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.RenderPositions = { Vector3(0) }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.RenderPositions = Bad.Positions; Bad.RenderTriangles = { Index3(0,1,1) }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		TetrahedralAsset Empty; Reject([&] { Empty.GetPositions(); }); Reject([&] { Empty.BindPoints({ Vector3(0) }); });
		auto Settings = TetrahedralAssetSettings(); Settings.MinimumQuality = 2; EXPECT(!Settings.IsValid());
	}
	void BindingAndAffineRendering()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			auto Input = Unit(Scale); Input.RenderPositions = { Vector3(0.2f,0.2f,0.2f)*Scale, Vector3(0.5f,0.1f,0.1f)*Scale, Vector3(0.1f,0.5f,0.1f)*Scale,
				Vector3(0.2f,0.2f,-0.05f)*Scale }; Input.RenderTriangles = { Index3(0,1,2), Index3(0,3,1) };
			Reject([&] { TetrahedralAsset::Prepare(Input); });
			TetrahedralAssetSettings Settings; Settings.AllowExtrapolation = true;
			const auto Asset = TetrahedralAsset::Prepare(Input, Settings); EXPECT(Asset.GetStats().ExtrapolatedVertices == 1);
			for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Asset.GetRenderBindings()[I]) - Input.RenderPositions[I]).Length() / Scale, 0, 2e-7);
			EXPECT(*std::min_element(Asset.GetRenderBindings()[3].Weights.begin(), Asset.GetRenderBindings()[3].Weights.end()) < 0);
			Reject([&] { Asset.BindPoints({ Vector3(0,0,-2)*Scale }); });
			std::vector<Vector3> Samples; for (int I = 0; I <= 10; ++I) for (int J = 0; J + I <= 10; ++J)
				Samples.push_back(Vector3(I/10.0f, J/10.0f, 0)*Scale);
			const auto Bindings = Asset.BindPoints(Samples); for (size_t I = 0; I < Samples.size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Bindings[I])-Samples[I]).Length()/Scale, 0, 2e-7);
			DeformableBody Body; Body.InitializeAsset(Asset); auto* Geometry = Body.GetRenderGeometry(); auto* Mesh = Geometry->GetShapeObj<TriangleMesh>();
			auto Affine = [&](const Vector3& P) { return Vector3(1.2f*P.x+0.3f*P.y, -0.2f*P.x+0.9f*P.y+0.1f*P.z, 1.1f*P.z) + Vector3(2,-1,3)*Scale; };
			for (int I = 0; I < Body.GetParticleCount(); ++I) EXPECT(Body.SetParticlePosition(I, Affine(Input.Positions[I])));
			for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Mesh->Vertices[I]-Affine(Input.RenderPositions[I])).Length()/Scale, 0, 1e-6);
			EXPECT(Body.GetSurfaceTriangles().size() == 4 && Mesh->NumTriangles == 2);
			for (const auto& N : Mesh->mNormals) EXPECT_NEAR(N.Length(), 1, 1e-6);
			EXPECT(!Geometry->IsQueryEnabled() && !Geometry->IsSimulationEnabled());
			Body.SetParticleVelocity(1, Vector3(0.1f)); const auto Velocity = Body.GetParticle(1).Velocity;
			const auto Point = BarycentricPoint::Tetrahedron({0,1,2,3}, {0.25,0.25,0.25,0.25}); Body.AttachToWorld(Point, Body.GetPointPosition(Point));
			Body.UseEmbeddedRenderMesh(false); EXPECT(Body.GetRenderGeometry() == Geometry && Mesh->NumTriangles == 4);
			EXPECT(Body.GetAttachmentCount() == 1 && Body.GetParticle(1).Velocity == Velocity);
			Body.UseEmbeddedRenderMesh(true); EXPECT(Mesh->NumTriangles == 2 && Body.GetRenderGeometry() == Geometry);
			Body.Reset(); for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Mesh->Vertices[I]-Input.RenderPositions[I]).Length()/Scale, 0, 2e-7);
		}
		// Distance alone cannot admit large barycentric weights in thin elements.
		auto Input = Unit(); Input.Positions[3].z = 0.01f; auto Settings = TetrahedralAssetSettings(); Settings.AllowExtrapolation = true; Settings.MaximumExtrapolationWeight = 1;
		const auto Thin = TetrahedralAsset::Prepare(Input, Settings); Reject([&] { Thin.BindPoints({ Vector3(0.2f,0.2f,-0.02f) }); });
	}
	void InstancesAndWorld()
	{
		auto Input = Unit(); Input.PinnedVertices = {0}; const auto Asset = TetrahedralAsset::Prepare(Input); const auto Copy = Asset;
		EXPECT(&Copy.GetPositions() == &Asset.GetPositions());
		DeformableBodySettings Settings; Settings.GroundEnabled = false;
		PhysicsWorldParam Param; PhysicsWorld World(Param); auto* A = World.CreateDeformableBody(Asset, Settings);
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitY(), 0.7f); const Transform Pose(Vector3(3,2,1), Rotation);
		auto* B = World.CreateDeformableBody(Copy, Settings, Pose);
		EXPECT(&A->GetPreparedAsset()->GetPositions() == &B->GetPreparedAsset()->GetPositions());
		for (int I = 0; I < B->GetParticleCount(); ++I) EXPECT_NEAR((B->GetParticle(I).Position-Pose.LocalToWorld(Input.Positions[I])).Length(), 0, 1e-6);
		const auto& Rest = B->GetTetrahedronRestShape(0); const auto& Tet = B->GetTetrahedra()[0]; Vector3 Positions[4];
		for (int I = 0; I < 4; ++I) Positions[I] = B->GetParticle(Tet.Particles[I]).Position;
		const auto F = Rest.Evaluate(Positions); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(F[I][J], I == J ? 1 : 0, 1e-12);
		EXPECT(std::abs(B->GetMaterialEnergy()) / (Rest.Volume * B->GetMaterials()[0].YoungModulus) < 1e-12);
		A->UnpinParticle(0); auto Material = A->GetMaterials()[0]; Material.YoungModulus *= 2; A->SetMaterial(0, Material);
		EXPECT(B->GetParticle(0).Pinned && Asset.GetPins()[0]); EXPECT_NEAR(B->GetMaterials()[0].YoungModulus, Input.Materials[0].YoungModulus, 0);
		Reject([&] { A->AddParticle(Vector3(2)); }); Reject([&] { A->AddTetrahedron(0,1,2,3); }); Reject([&] { A->SetSurfaceTriangles({}); });
		Reject([&] { A->InitializeAsset(Asset); }); EXPECT(A->AddEdge(0,1) >= 0);
		Transform Bad(Vector3(1e20f)); const size_t Count = World.GetDeformableBodies().size();
		Reject([&] { World.CreateDeformableBody(Asset, Settings, Bad); }); EXPECT(World.GetDeformableBodies().size() == Count);
		Bad = Transform::Identity(); Bad.quat.x = 2; Reject([&] { World.CreateDeformableBody(Asset, Settings, Bad); });
		A->SetTetrahedronMaterial(0, -1); for (const auto& E : A->GetEdges()) EXPECT(E.Enabled);
		A->SetTetrahedronMaterial(0, 0); EXPECT(A->GetEdges()[0].Enabled);
		for (int Frame = 0; Frame < 30; ++Frame) World.Simulate(1.0f/60);
		for (const auto* Body : {A,B}) for (const auto& P : Body->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()));
		EXPECT(World.RemoveDeformableBody(A)); EXPECT(World.RemoveDeformableBody(B));
	}
}
void TestTetrahedralAsset()
{
	PreparationAndTopology(); InvalidAssets(); BindingAndAffineRendering(); InstancesAndWorld();
}
}

void TestTetrahedralAsset() { Test_TetrahedralAsset_Cases::TestTetrahedralAsset(); }

namespace Test_ProjectiveDynamics_Cases
{
using namespace Riemann;
namespace
{
	using DVector = DeformationGradient::DVector;
	DVector Double(const Vector3& P) { return DVector(P.x,P.y,P.z); }
	template<class F> void Reject(F Function)
	{ bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected); }
	TetrahedralAsset Asset(float Scale = 1)
	{
		TetrahedralAssetInput Input; Input.Positions = { Vector3(0), Vector3(Scale,0,0), Vector3(0,Scale,0), Vector3(0,0,Scale) };
		Input.Elements.push_back({ {0,1,2,3}, 0 }); Input.Materials[0].Density = 6; Input.Materials[0].Model = TetrahedralMaterialModel::Corotated;
		return TetrahedralAsset::Prepare(Input);
	}
	void RotationsAndMaterial()
	{
		for (double X : {0.0,1e-10,1.0,100.0,-0.2})
		{
			DeformationGradient F; F[0][0] = X; F[1][1] = 1; F[2][2] = 0.8; F[0][1] = 0.3;
			const auto R = TetrahedralElasticity::ClosestRotation(F); EXPECT_NEAR(R.Determinant(),1,1e-12);
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{ double Dot = 0; for (int K = 0; K < 3; ++K) Dot += R[K][I]*R[K][J]; EXPECT_NEAR(Dot,I==J?1:0,1e-12); }
		}
		TetrahedralMaterial Material; Material.Model = TetrahedralMaterialModel::Corotated;
		DeformationGradient F; F[0][0] = 1.1; F[1][1] = 0.93; F[2][2] = 1.03; F[0][1] = 0.2;
		const auto Response = TetrahedralElasticity::Evaluate(F,Material); constexpr double H = 1e-6;
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
		{
			auto Plus=F, Minus=F; Plus[I][J]+=H; Minus[I][J]-=H;
			const double Derivative = (TetrahedralElasticity::Evaluate(Plus,Material).EnergyDensity-TetrahedralElasticity::Evaluate(Minus,Material).EnergyDensity)/(2*H);
			EXPECT_NEAR(Derivative,Response.FirstPiola[I][J],1e-5*std::max(1.0,std::abs(Derivative)));
		}
		for (double Nu : {0.0,0.25,0.49})
		{
			Material.PoissonRatio=Nu; DeformationGradient Plus,Minus;
			for (int I=0;I<3;++I) Plus[I][I]=Minus[I][I]=1;
			Plus[0][0]+=H; Minus[0][0]-=H;
			const auto P=TetrahedralElasticity::Evaluate(Plus,Material), M=TetrahedralElasticity::Evaluate(Minus,Material);
			EXPECT_NEAR((P.FirstPiola[0][0]-M.FirstPiola[0][0])/(2*H),Material.LameLambda()+2*Material.ShearModulus(),1e-5*Material.YoungModulus);
			EXPECT_NEAR((P.FirstPiola[1][1]-M.FirstPiola[1][1])/(2*H),Material.LameLambda(),1e-5*Material.YoungModulus);
		}
	}
	void GlobalEquationAndScale()
	{
		for (float Scale : {0.001f,1.0f,1000.0f})
		{
			const auto Prepared=Asset(Scale); auto Positions=Prepared.GetPositions(); Positions[1].x *= 1.1f; const auto Predicted=Positions;
			ProjectiveDynamicsElement Element; Element.Particles={0,1,2,3}; Element.Rest=Prepared.GetRestShapes()[0]; Element.ShearModulus=500;
			std::vector<ProjectiveDynamicsParticle> Particles;
			for (int I=0;I<4;++I) Particles.push_back({&Positions[I],Prepared.GetInverseMasses()[I]});
			const double Dt=0.01*Scale, Weight=2*Element.ShearModulus*Element.Rest.Volume;
			const auto Rotation=TetrahedralElasticity::ClosestRotation(Element.Rest.Evaluate(Positions.data()));
			ProjectiveDynamicsProjection Solver; Solver.Begin(Particles,{Element},Dt); EXPECT(Solver.Project()); EXPECT(Solver.GetStats().Converged);
			EXPECT_NEAR(Solver.GetStats().AcceptedStep,1,1e-12);
			auto Difference=Element.Rest.Evaluate(Positions.data());
			for (int I=0;I<3;++I) for (int J=0;J<3;++J) Difference[I][J]-=Rotation[I][J];
			DVector Momentum(0); double Residual=0,Reference=0;
			for (int I=0;I<4;++I)
			{
				const auto Change=Double(Positions[I])-Double(Predicted[I]); const double Mass=1.0/Particles[I].InverseMass;
				const auto Force=(Difference*Element.Rest.Gradients[I])*Weight;
				Residual+=(Change*(Mass/(Dt*Dt))+Force).SquareLength(); Reference+=Force.SquareLength(); Momentum+=Change*Mass;
			}
			EXPECT(std::sqrt(Residual/std::max(Reference,1e-100))<1e-4);
			EXPECT(std::hypot(std::hypot(Momentum.x,Momentum.y),Momentum.z)/(Scale*Prepared.GetStats().TotalMass)<1e-7);
			Solver.End(); Reject([&]{Solver.Project();});
			Particles[0].InverseMass=0; const auto Pinned=Positions[0]; Solver.Begin(Particles,{Element},Dt);
			for(int I=0;I<8;++I) Solver.Project(); EXPECT(Positions[0]==Pinned);
			Particles[1].Position=Particles[0].Position; Reject([&]{Solver.Begin(Particles,{Element},Dt);});
		}
	}
	void CalibrationAndSwitches()
	{
		for (bool Projective : {false,true}) for (double Young : {1000.0,10000.0}) for (int Substeps : {1,2})
		{
			DeformableBodySettings Settings; Settings.ProjectiveMaterials=Projective; Settings.Substeps=Substeps;
			Settings.SolverIterations=64; Settings.Gravity=Vector3::Zero(); Settings.Damping=12;
			DeformableBody Body(Settings); Body.InitializeAsset(Asset()); Body.PinParticle(0,Vector3(0));
			auto Material=Body.GetMaterials()[0]; Material.YoungModulus=Young; Material.PoissonRatio=0.25; Body.SetMaterial(0,Material);
			for(int Frame=0;Frame<300;++Frame) { Body.AddForce(2,Vector3(0,1.0f/6,0)); Body.Simulate(0.01f); }
			const double Axial=Body.GetParticle(2).Position.y-1, Lateral=(Body.GetParticle(1).Position.x+Body.GetParticle(3).Position.z)/2-1;
			printf("Corotated calibration PD=%d E=%g substeps=%d axial=%g lateral=%g\n",Projective,Young,Substeps,Axial,Lateral);
			EXPECT_NEAR(Axial,1/Young,0.02/Young); EXPECT_NEAR(Lateral,-Material.PoissonRatio/Young,0.02*Material.PoissonRatio/Young);
			if(Projective) EXPECT(Body.GetProjectiveMaterialStats().Converged);
			auto* Render=Body.GetRenderGeometry(); const auto Position=Body.GetParticle(2).Position;
			Body.SetProjectiveMaterialSolver(!Projective); EXPECT(Body.GetParticle(2).Position==Position && Body.GetRenderGeometry()==Render);
			Material.Model=TetrahedralMaterialModel::StableNeoHookean; Body.SetMaterial(0,Material); Body.Simulate(0.01f);
			Body.SetTetrahedronMaterial(0,-1); Body.Simulate(0.01f); Body.SetTetrahedronMaterial(0,0); Body.Reset();
			EXPECT_NEAR(Body.GetMaterialEnergy(),0,1e-10);
		}
	}
	void CoupledElementsAndLoads()
	{
		for (bool Mixed : {false,true})
		{
			TetrahedralAssetInput Input; Input.Positions={Vector3(0),Vector3::UnitX(),Vector3::UnitY(),Vector3::UnitZ(),-Vector3::UnitZ()};
			Input.Materials[0].Model=TetrahedralMaterialModel::Corotated; Input.Materials[0].Density=6; Input.Materials[0].YoungModulus=2000;
			Input.Materials.push_back(Input.Materials[0]); Input.Materials[1].YoungModulus=5000;
			if(Mixed) Input.Materials[1].Model=TetrahedralMaterialModel::SaintVenantKirchhoff;
			Input.Elements={{{0,1,2,3},0},{{0,1,2,4},1}};
			const auto Prepared=TetrahedralAsset::Prepare(Input);
			DeformableBodySettings Settings; Settings.ProjectiveMaterials=true; Settings.Substeps=2; Settings.SolverIterations=64; Settings.Damping=12;
			DeformableBody Body(Settings); Body.InitializeAsset(Prepared);
			const auto Point=BarycentricPoint::Triangle(Index3(0,1,2),Vector3(0.5f,0.25f,0.25f)); const auto Anchor=Body.GetPointPosition(Point);
			const auto Link=Body.AttachToWorld(Point,Anchor); double Reaction=0;
			constexpr int Frames=300, Samples=60;
			for(int Frame=0;Frame<Frames;++Frame)
			{
				Body.Simulate(1.0f/120); EXPECT_NEAR((Body.GetPointPosition(Point)-Anchor).Length(),0,1e-6);
				for(int I=0;I<2;++I) EXPECT(Body.GetTetrahedronMaterialResponse(I).Jacobian>0);
				if(Frame>=Frames-Samples) Reaction+=Body.GetAttachmentForce(Link).y/Samples;
			}
			const double Expected=-Settings.Gravity.y*Prepared.GetStats().TotalMass;
			printf("Projective shared-node load mixed=%d reaction=%g expected=%g\n",Mixed,Reaction,Expected);
			EXPECT_NEAR(Reaction,Expected,Expected*0.01); EXPECT(Body.GetProjectiveMaterialStats().Converged);
			// The rotation block must remain finite from a singular initial state.
			Body.ClearAttachments(); Body.Reset();
			for(int I=0;I<Body.GetParticleCount();++I) Body.SetParticlePosition(I,Vector3(0));
			for(int Frame=0;Frame<30;++Frame) Body.Simulate(1.0f/120);
			for(const auto& P:Body.GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()));
		}
	}
	void FailedPreparation()
	{
		DeformableBodySettings Settings; Settings.ProjectiveMaterials=true; Settings.Gravity=Vector3::Zero();
		DeformableBody A(Settings),B(Settings); A.InitializeAsset(Asset()); B.InitializeAsset(Asset(1000));
		const auto Normal=B.GetMaterials()[0]; auto Huge=Normal; Huge.YoungModulus=1e307; EXPECT(Huge.IsValid()); B.SetMaterial(0,Huge);
		A.AttachToBody(BarycentricPoint::Vertex(0),&B,BarycentricPoint::Vertex(0));
		Reject([&]{DeformableBody::SimulateGroup({&A,&B},0.01f);});
		bool Recovered=true;
		try { A.SetProjectiveMaterialSolver(false); B.SetMaterial(0,Normal); DeformableBody::SimulateGroup({&A,&B},0.01f); }
		catch(const std::exception&) { Recovered=false; }
		EXPECT(Recovered);
	}
}
void TestProjectiveDynamics()
{
	RotationsAndMaterial(); GlobalEquationAndScale(); CalibrationAndSwitches(); CoupledElementsAndLoads(); FailedPreparation();
}
}

void TestProjectiveDynamics() { Test_ProjectiveDynamics_Cases::TestProjectiveDynamics(); }

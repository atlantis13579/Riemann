#include "Test.h"

#include <limits>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

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

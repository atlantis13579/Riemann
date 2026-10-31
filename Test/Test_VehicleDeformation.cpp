#include "Test.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/VehicleDeformation.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	constexpr float VehicleStep = 1.0f / 60.0f;

	bool VehicleFinite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	struct VehicleFixture
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		RigidBody* Body;
		VehicleDeformation Vehicle;

		explicit VehicleFixture(const VehicleDeformationSettings& settings = VehicleDeformationSettings())
			: Vehicle(settings)
		{
			RigidBodyParam bodySettings;
			bodySettings.rigidType = RigidType::Dynamic;
			bodySettings.disableGravity = true;
			Body = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), settings.HalfExtent), bodySettings);
			EXPECT(Vehicle.Attach(Body));
		}
	};

	Vector3 FrontPoint(const VehicleDeformation& vehicle)
	{
		return Vector3(0.0f, 0.2f, vehicle.GetSettings().HalfExtent.z);
	}

	void ImpactFront(VehicleDeformation& vehicle, float impulse)
	{
		const Transform world = vehicle.GetWorldTransform();
		EXPECT(vehicle.QueueImpact(world.LocalToWorld(FrontPoint(vehicle)),
			world.LocalToWorldDirection(Vector3(0, 0, -impulse))));
	}

	float FrontDent(const VehicleDeformation& vehicle)
	{
		float dent = 0.0f;
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			if (particle.InitialPosition.z > vehicle.GetSettings().HalfExtent.z - 1.0e-4f)
				dent = std::max(dent, particle.InitialPosition.z - particle.Position.z);
		}
		return dent;
	}

	void CompareCages(const VehicleDeformation& a, const VehicleDeformation& b, float tolerance)
	{
		EXPECT(a.GetCage().GetParticleCount() == b.GetCage().GetParticleCount());
		if (a.GetCage().GetParticleCount() != b.GetCage().GetParticleCount()) return;
		for (int i = 0; i < a.GetCage().GetParticleCount(); ++i)
		{
			EXPECT((a.GetCage().GetParticle(i).Position - b.GetCage().GetParticle(i).Position).Length() < tolerance);
			EXPECT((a.GetCage().GetParticle(i).Velocity - b.GetCage().GetParticle(i).Velocity).Length() < tolerance * 100.0f);
		}
	}

	void CheckVehicleFinite(const VehicleDeformation& vehicle)
	{
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			EXPECT(VehicleFinite(particle.Position));
			EXPECT(VehicleFinite(particle.Velocity));
			if (particle.Pinned)
			{
				EXPECT_SAME(particle.Position, particle.InitialPosition);
				EXPECT_SAME(particle.Velocity, Vector3::Zero());
			}
		}
		for (int i = 0; i < (int)vehicle.GetCage().GetTetrahedra().size(); ++i)
			EXPECT(std::isfinite(vehicle.GetCage().GetTetrahedronVolume(i)));
	}

	void TestVehicleCageTopology()
	{
		for (float cabin : { 0.0f, 0.45f })
		{
			VehicleDeformationSettings settings;
			settings.CabinHeight = cabin;
			VehicleFixture fixture(settings);
			const DeformableBody& cage = fixture.Vehicle.GetCage();
			EXPECT(cage.GetParticleCount() == 135);
			EXPECT(cage.GetTetrahedra().size() == 384);
			EXPECT(cage.GetSurfaceTriangles().size() == 224);
			int pinned = 0;
			for (const DeformableParticle& particle : cage.GetParticles()) pinned += particle.Pinned ? 1 : 0;
			EXPECT(pinned == 21);
			double volume = 0.0, surfaceVolume = 0.0;
			for (int i = 0; i < (int)cage.GetTetrahedra().size(); ++i)
			{
				EXPECT(cage.GetTetrahedronVolume(i) > 0.0f);
				volume += cage.GetTetrahedronVolume(i);
			}
			std::map<std::pair<int, int>, std::pair<int, int>> edges;
			for (const Index3& triangle : cage.GetSurfaceTriangles())
			{
				const Vector3 a = cage.GetParticle(triangle.a).Position;
				const Vector3 b = cage.GetParticle(triangle.b).Position;
				const Vector3 c = cage.GetParticle(triangle.c).Position;
				EXPECT((b - a).Cross(c - a).SquareLength() > 1.0e-10f);
				surfaceVolume += double(a.Dot(b.Cross(c))) / 6.0;
				for (int side = 0; side < 3; ++side)
				{
					const int from = triangle[side], to = triangle[(side + 1) % 3];
					auto& entry = edges[{ std::min(from, to), std::max(from, to) }];
					++entry.first;
					entry.second += from < to ? 1 : -1;
				}
			}
			for (const auto& edge : edges) { EXPECT(edge.second.first == 2); EXPECT(edge.second.second == 0); }
			EXPECT_NEAR(volume, surfaceVolume, 1.0e-4);
			if (cabin == 0.0f) EXPECT_NEAR(volume, 8.0f * settings.HalfExtent.x * settings.HalfExtent.y * settings.HalfExtent.z, 1.0e-4);
			Geometry* surface = fixture.Vehicle.GetRenderGeometry();
			EXPECT(surface != nullptr);
			if (!surface) continue;
			EXPECT(surface->GetParent<RigidBody>() == nullptr);
			EXPECT_FALSE(surface->IsSimulationEnabled());
			EXPECT_FALSE(surface->IsQueryEnabled());
			const TriangleMesh* mesh = surface->GetShapeObj<TriangleMesh>();
			EXPECT(mesh->NumVertices == 135 && mesh->NumTriangles == 224);
			EXPECT_NEAR(mesh->BoundingVolume.Min.y, -settings.HalfExtent.y, 1.0e-6f);
			EXPECT_NEAR(mesh->BoundingVolume.Max.y, settings.HalfExtent.y + cabin, 1.0e-6f);
		}
	}

	void TestVehicleFrameAndContactSigns()
	{
		VehicleFixture direct, contactA, contactB;
		const Vector3 centerOfMass(0.2f, -0.1f, 0.3f);
		Quaternion localRotation, worldRotation;
		localRotation.FromRotationAxis(Vector3::UnitZ(), 0.2f);
		worldRotation.FromRotationAxis(Vector3::UnitY(), 1.57079632679f);
		const Transform localFrame(Vector3(0.3f, 0.1f, -0.2f), localRotation);
		for (VehicleFixture* fixture : { &direct, &contactA, &contactB })
		{
			fixture->Body->CM = centerOfMass;
			EXPECT(fixture->Vehicle.Attach(fixture->Body, localFrame));
		}
		contactB.Body->X = Vector3(17, 3, -9);
		contactB.Body->Q = worldRotation;
		ImpactFront(direct.Vehicle, 400.0f);
		for (VehicleFixture* fixture : { &contactA, &contactB })
		{
			const bool isA = fixture == &contactA;
			const Transform world = fixture->Vehicle.GetWorldTransform();
			PhysicsContactPoint point{};
			point.PositionWorldA = isA ? world.LocalToWorld(FrontPoint(fixture->Vehicle)) : Vector3(1000.0f);
			point.PositionWorldB = !isA ? world.LocalToWorld(FrontPoint(fixture->Vehicle)) : Vector3(1000.0f);
			point.PositionLocalA = point.PositionLocalB = Vector3(-1000.0f);
			point.Impulse = world.LocalToWorldDirection(Vector3(0, 0, isA ? 400.0f : -400.0f));
			PhysicsContactInfo info{};
			info.BodyA = isA ? fixture->Body : nullptr;
			info.BodyB = isA ? nullptr : fixture->Body;
			info.ContactPoints = &point;
			info.NumContactPoints = 1;
			fixture->Vehicle.OnContact(info);
			EXPECT(fixture->Vehicle.GetPendingImpactCount() == 1);
		}
		// Move the chassis after callback delivery. The queued impact must retain its local location.
		contactB.Body->X += Vector3(2, 1, -3);
		contactB.Body->Q = (worldRotation * worldRotation).Unit();
		for (int step = 0; step < 30; ++step)
		{
			direct.Vehicle.Simulate(VehicleStep);
			contactA.Vehicle.Simulate(VehicleStep);
			contactB.Vehicle.Simulate(VehicleStep);
			CompareCages(direct.Vehicle, contactA.Vehicle, 2.0e-4f);
			CompareCages(direct.Vehicle, contactB.Vehicle, 2.0e-4f);
		}
		EXPECT(FrontDent(direct.Vehicle) > 0.005f);
		const Transform actual = contactB.Vehicle.GetWorldTransform();
		const Vector3 expectedPosition = contactB.Body->X + contactB.Body->Q * (localFrame.pos - centerOfMass);
		EXPECT_SAME(actual.pos, expectedPosition);
		EXPECT_NEAR(std::abs(actual.quat.Dot(contactB.Body->Q * localRotation)), 1.0f, 1.0e-5f);
		EXPECT_SAME(contactB.Vehicle.GetRenderGeometry()->GetPosition(), actual.pos);
		EXPECT_NEAR(std::abs(contactB.Vehicle.GetRenderGeometry()->GetRotation().Dot(actual.quat)), 1.0f, 1.0e-5f);
		EXPECT_SAME(contactB.Body->V, Vector3::Zero());
		EXPECT_SAME(contactB.Body->W, Vector3::Zero());

		VehicleFixture moving;
		const uint64_t revision = moving.Vehicle.GetRevision();
		for (int frame = 0; frame < 60; ++frame)
		{
			moving.Body->X = Vector3(0.1f * frame, 3.0f, -2.0f);
			moving.Body->Q.FromRotationAxis(Vector3::UnitY(), frame * 0.03f);
			moving.Vehicle.Simulate(VehicleStep);
		}
		EXPECT(moving.Vehicle.GetMaxDisplacement() < 1.0e-5f);
		EXPECT(moving.Vehicle.GetMaxPlasticStrain() == 0.0f);
		EXPECT(moving.Vehicle.GetRevision() > revision);
	}

	void TestVehicleElasticPlasticAndReset()
	{
		VehicleFixture fixture;
		VehicleDeformation& vehicle = fixture.Vehicle;
		Geometry* surface = vehicle.GetRenderGeometry();
		ImpactFront(vehicle, 30.0f);
		float weakPeak = 0.0f;
		for (int step = 0; step < 180; ++step)
		{
			vehicle.Simulate(VehicleStep);
			weakPeak = std::max(weakPeak, vehicle.GetMaxDisplacement());
		}
		printf("  Vehicle elastic: peak=%.6f remaining=%.6f plastic=%.6f\n", weakPeak, vehicle.GetMaxDisplacement(), vehicle.GetMaxPlasticStrain());
		EXPECT(weakPeak > 0.001f);
		EXPECT(vehicle.GetMaxDisplacement() < 0.001f);
		EXPECT(vehicle.GetMaxPlasticStrain() < 1.0e-5f);
		vehicle.Reset();
		ImpactFront(vehicle, 400.0f);
		float strongPeak = 0.0f;
		for (int step = 0; step < 180; ++step)
		{
			vehicle.Simulate(VehicleStep);
			strongPeak = std::max(strongPeak, FrontDent(vehicle));
		}
		printf("  Vehicle plastic: peak dent=%.6f remaining=%.6f plastic=%.6f\n", strongPeak, FrontDent(vehicle), vehicle.GetMaxPlasticStrain());
		EXPECT(strongPeak > 0.02f);
		EXPECT(FrontDent(vehicle) > 0.01f);
		EXPECT(vehicle.GetMaxPlasticStrain() > 1.0e-4f);
		for (int hit = 0; hit < 6; ++hit)
		{
			ImpactFront(vehicle, 400.0f);
			for (int step = 0; step < 60; ++step) vehicle.Simulate(VehicleStep);
			CheckVehicleFinite(vehicle);
		}
		EXPECT(vehicle.GetMaxPlasticStrain() <= vehicle.GetSettings().Material.MaxPlasticStrain + 1.0e-5f);
		EXPECT(vehicle.GetMaxDisplacement() < 1.0f);
		for (int i = 0; i < (int)vehicle.GetCage().GetTetrahedra().size(); ++i) EXPECT(vehicle.GetCage().GetTetrahedronVolume(i) > 0.0f);
		EXPECT(surface == vehicle.GetRenderGeometry());
		ImpactFront(vehicle, 400.0f);
		EXPECT(vehicle.GetPendingImpactCount() > 0);
		fixture.Body->X = Vector3(5, 2, -3);
		const Transform world = vehicle.GetWorldTransform();
		vehicle.Reset();
		EXPECT(vehicle.GetPendingImpactCount() == 0);
		EXPECT(vehicle.GetMaxDisplacement() == 0.0f);
		EXPECT(vehicle.GetMaxPlasticStrain() == 0.0f);
		EXPECT(fixture.Body->Parent == &vehicle && vehicle.GetBody() == fixture.Body);
		EXPECT_SAME(vehicle.GetRenderGeometry()->GetPosition(), world.pos);
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			EXPECT_SAME(particle.Position, particle.InitialPosition);
			EXPECT_SAME(particle.PreviousPosition, particle.InitialPosition);
			EXPECT_SAME(particle.Velocity, Vector3::Zero());
			EXPECT_SAME(particle.Force, Vector3::Zero());
		}
		for (const DeformableEdge& edge : vehicle.GetCage().GetEdges())
		{
			EXPECT(edge.RestLength == edge.InitialRestLength);
			EXPECT(edge.Lambda == 0.0f);
		}
		for (const DeformableTetrahedron& tetrahedron : vehicle.GetCage().GetTetrahedra())
		{
			EXPECT(tetrahedron.RestVolume == tetrahedron.InitialRestVolume);
			EXPECT(tetrahedron.Lambda == 0.0f);
		}
		vehicle.Simulate(VehicleStep);
		EXPECT(vehicle.GetMaxDisplacement() < 1.0e-5f);
	}

	void TestVehicleBudgetAndLifetime()
	{
		VehicleFixture one, many;
		ImpactFront(one.Vehicle, 400.0f);
		for (int i = 0; i < 4; ++i) ImpactFront(many.Vehicle, 400.0f);
		one.Vehicle.Simulate(VehicleStep);
		many.Vehicle.Simulate(VehicleStep);
		CompareCages(one.Vehicle, many.Vehicle, 1.0e-4f);
		EXPECT(one.Vehicle.GetPendingImpactCount() == 0 && many.Vehicle.GetPendingImpactCount() == 0);
		VehicleFixture fixture;
		VehicleDeformation& vehicle = fixture.Vehicle;
		const Vector3 point = vehicle.GetWorldTransform().LocalToWorld(FrontPoint(vehicle));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3(0, 0, -19.0f)));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3::Zero()));
		EXPECT_FALSE(vehicle.QueueImpact(Vector3(1000.0f), Vector3(0, 0, -100.0f)));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3(std::numeric_limits<float>::infinity(), 0, 0)));
		ImpactFront(vehicle, 400.0f);
		vehicle.Simulate(0.0f);
		vehicle.Simulate(-1.0f);
		vehicle.Simulate(std::numeric_limits<float>::quiet_NaN());
		EXPECT(vehicle.GetPendingImpactCount() == 1);
		EXPECT(vehicle.GetMaxDisplacement() == 0.0f);
		GeometryAggregate otherOwner;
		RigidBodyParam fixed;
		RigidBody* staticBody = fixture.World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(10, 0, 0), 0.2f), fixed);
		EXPECT_FALSE(vehicle.Attach(nullptr));
		EXPECT_FALSE(vehicle.Attach(staticBody));
		{
			VehicleDeformation second;
			EXPECT_FALSE(second.Attach(fixture.Body));
			EXPECT(fixture.Body->Parent == &vehicle);
		}
		fixture.Body->X = Vector3(2, 3, 4);
		vehicle.Detach();
		EXPECT(fixture.Body->Parent == nullptr);
		EXPECT(vehicle.GetBody() == nullptr && vehicle.GetPendingImpactCount() == 0);
		EXPECT_SAME(vehicle.GetRenderGeometry()->GetPosition(), fixture.Body->X);
		fixture.Body->Parent = &otherOwner;
		EXPECT_FALSE(vehicle.Attach(fixture.Body));
		EXPECT(fixture.Body->Parent == &otherOwner);
		fixture.Body->Parent = nullptr;
		{
			VehicleDeformation temporary;
			EXPECT(temporary.Attach(fixture.Body));
		}
		EXPECT(fixture.Body->Parent == nullptr);
		EXPECT(vehicle.Attach(fixture.Body));
		fixture.Body->Parent = &otherOwner;
		vehicle.Detach();
		EXPECT(fixture.Body->Parent == &otherOwner);
		fixture.Body->Parent = nullptr;
	}
}

void TestVehicleDeformation()
{
	printf("Running TestVehicleDeformation\n");
	TestVehicleCageTopology();
	TestVehicleFrameAndContactSigns();
	TestVehicleElasticPlasticAndReset();
	TestVehicleBudgetAndLifetime();
}

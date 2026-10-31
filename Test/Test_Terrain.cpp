#include "Test.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	void InitTerrain(HeightField3& field, int width, int height, float scale = 0.01f)
	{
		field.Init(Box3(Vector3::Zero(), Vector3((float)width - 1, 1.0f, (float)height - 1)), width, height);
		field.AllocMemory();
		field.HeightScale = scale;
		field.RecalculateBounds();
	}

	Geometry* CreateTerrain(float halfExtent, int count)
	{
		Geometry* geometry = GeometryFactory::CreateHeightField(
			Box3(Vector3(-halfExtent, -1.0f, -halfExtent), Vector3(halfExtent, 1.0f, halfExtent)), count, count);
		HeightField3* field = geometry->GetShapeObj<HeightField3>();
		field->AllocMemory();
		field->HeightScale = 0.01f;
		field->RecalculateBounds();
		geometry->SetShapeBounds(field->BV);
		return geometry;
	}

	RigidBodyDynamic* CreateTerrainTestBody(PhysicsWorld& world, const Vector3& center)
	{
		RigidBodyParam settings;
		settings.rigidType = RigidType::Dynamic;
		settings.sleepThreshold = 0.0f;
		return world.CreateRigidBody(GeometryFactory::CreateSphere(center, 0.2f), settings)->CastDynamic();
	}
}

void TestTerrainEditing()
{
	printf("Running TestTerrainEditing\n");
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float huge = std::numeric_limits<float>::max();

	{
		HeightField3 field;
		InitTerrain(field, 4, 3, 0.25f);
		std::vector<float> values = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
		uint64_t revision = field.GetRevision();
		EXPECT(field.SetHeightsRect(-1, -1, 3, 3, values) == 4);
		EXPECT(field.GetRevision() == revision + 1);
		EXPECT_NEAR(field.GetHeight(0), 5.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(1), 6.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(4), 8.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(5), 9.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(2), 0.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(8), 0.0f, 1e-6f);
		EXPECT_NEAR(field.GetLastEditBounds().Min.x, 0.0f, 1e-6f);
		EXPECT_NEAR(field.GetLastEditBounds().Max.x, 2.0f, 1e-6f);
		EXPECT_NEAR(field.GetLastEditBounds().Min.y, 0.0f, 1e-6f);
		EXPECT_NEAR(field.GetLastEditBounds().Max.y, 9.0f, 1e-6f);
		revision = field.GetRevision();
		EXPECT(field.SetHeightsRect(-1, -1, 3, 3, values) == 0);
		EXPECT(field.GetRevision() == revision);

		// Even an invalid source sample clipped out of the destination rejects the batch.
		std::fill(values.begin(), values.end(), 2.0f);
		values[0] = nan;
		EXPECT(field.SetHeightsRect(-1, -1, 3, 3, values) == 0);
		EXPECT(field.GetRevision() == revision);
		EXPECT_NEAR(field.GetHeight(0), 5.0f, 1e-6f);
		EXPECT_NEAR(field.GetHeight(5), 9.0f, 1e-6f);
		values[0] = 2.0f;
		values.back() = nan;
		EXPECT(field.SetHeightsRect(-1, -1, 3, 3, values) == 0);
		EXPECT_NEAR(field.GetHeight(0), 5.0f, 1e-6f);
		EXPECT(field.SetHeightsRect(0, 0, 2, 3, values) == 0);
		EXPECT(field.SetHeightsRect(std::numeric_limits<int>::max(), 0, 1, 1, { 1.0f }) == 0);
		EXPECT(field.SetHeightsRect(std::numeric_limits<int>::min(), 0, 1, 1, { 1.0f }) == 0);
		EXPECT(field.GetRevision() == revision);

		EXPECT(field.SetHeight(3, 2, huge));
		EXPECT_NEAR(field.GetHeight(11), 32767.0f * 0.25f, 1e-6f);
		revision = field.GetRevision();
		EXPECT(field.SetHeight(3, 2, huge));
		EXPECT(field.GetRevision() == revision);
		EXPECT_FALSE(field.SetHeight(3, 2, nan));
		EXPECT(field.GetRevision() == revision);
		EXPECT(field.SetHeight(3, 2, -huge));
		EXPECT_NEAR(field.GetHeight(11), -32768.0f * 0.25f, 1e-6f);
		EXPECT(field.GetLastEditBounds().Max.y >= 32767.0f * 0.25f);
		EXPECT(field.GetLastEditBounds().Min.y <= -32768.0f * 0.25f);
	}

	{
		HeightField3 field;
		InitTerrain(field, 3, 3);
		EXPECT(field.SetHeight(1, 1, 1.0f));
		uint64_t revision = field.GetRevision();
		EXPECT(field.FillHeightCircle(1, 1, 0.25f, 3.0f, 0.5f, false) == 1);
		EXPECT_NEAR(field.GetHeight(4), 2.0f, 1e-5f);
		EXPECT(field.GetRevision() == revision + 1);
		revision = field.GetRevision();
		EXPECT(field.FillHeightCircle(1, 1, 0.25f, 2.0f, 1.0f, false) == 0);
		EXPECT(field.ModifyHeightCircle(1, 1, 0.0f, 1.0f) == 0);
		EXPECT(field.ModifyHeightCircle(nan, 1, 1.0f, 1.0f) == 0);
		EXPECT(field.FillHeightCircle(1, 1, 1.0f, nan) == 0);
		EXPECT(field.SmoothHeightCircle(1, 1, 1.0f, nan) == 0);
		EXPECT(field.SmoothHeightCircle(1, 1, 1.0f, 1.0f, 0) == 0);
		EXPECT(field.GetRevision() == revision);
	}

	{
		HeightField3 field;
		InitTerrain(field, 7, 7);
		EXPECT(field.SetHeight(3, 3, 9.0f));
		const uint64_t revision = field.GetRevision();
		EXPECT(field.SmoothHeightCircle(3, 3, 2.0f, 1.0f, 1, false) == 9);
		EXPECT(field.GetRevision() == revision + 1);
		for (int z = 2; z <= 4; ++z)
		for (int x = 2; x <= 4; ++x)
			EXPECT_NEAR(field.GetHeight(z * 7 + x), 1.0f, 1e-5f);
		EXPECT_NEAR(field.GetHeight(3 * 7 + 1), 0.0f, 1e-5f);
		EXPECT(field.SmoothHeightCircle(3, 3, 2.5f, 0.75f, 4, false) > 0);
		for (int z = 0; z < 7; ++z)
		for (int x = 0; x < 7; ++x)
		{
			EXPECT_NEAR(field.GetHeight(z * 7 + x), field.GetHeight(z * 7 + 6 - x), 1e-5f);
			EXPECT_NEAR(field.GetHeight(z * 7 + x), field.GetHeight((6 - z) * 7 + x), 1e-5f);
		}

		HeightField3 corner;
		InitTerrain(corner, 3, 3);
		EXPECT(corner.SetHeight(0, 0, 4.0f));
		EXPECT(corner.SmoothHeightCircle(0, 0, 0.25f, 1.0f, 1, false) == 1);
		EXPECT_NEAR(corner.GetHeight(0), 1.0f, 1e-5f);
		EXPECT_NEAR(corner.GetHeight(1), 0.0f, 1e-5f);
	}

	{
		HeightField3 field;
		InitTerrain(field, 4, 4);
		field.Cells[5].Tessellation0 = 0x80;
		const uint64_t revision = field.GetRevision();
		EXPECT(field.SetTriangleHole(1, 1, 0, true));
		EXPECT(field.GetRevision() == revision + 1);
		EXPECT(field.IsTriangleHole(1, 1, 0));
		EXPECT((field.Cells[5].Tessellation0 & 0x80) != 0);
		Vector3 triangle[6];
		EXPECT(field.GetCellTriangle(1, 1, triangle) == 3);
		float minHeight = -1.0f, maxHeight = 1.0f;
		EXPECT(field.GetHeightRange(1, 1, minHeight, maxHeight));
		EXPECT_NEAR(minHeight, 0.0f, 1e-5f);
		EXPECT_NEAR(maxHeight, 0.0f, 1e-5f);
		EXPECT(field.GetLastEditBounds().Min.x <= 1.0f && field.GetLastEditBounds().Max.x >= 2.0f);
		EXPECT(field.SetTriangleHole(1, 1, 0, true));
		EXPECT(field.GetRevision() == revision + 1);
		EXPECT(field.SetTriangleHole(1, 1, 0, false));
		EXPECT((field.Cells[5].Tessellation0 & 0x80) != 0);
		EXPECT(field.GetCellTriangle(1, 1, triangle) == 6);

		HeightField3 large;
		InitTerrain(large, 257, 257);
		EXPECT(large.SetHeight(256, 256, 3.0f));
		uint32_t indices[6];
		EXPECT(large.GetCellTriangle(255, 255, indices) == 6);
		EXPECT(*std::max_element(indices, indices + 6) == 257u * 257u - 1);
		EXPECT(large.GetCellTriangle(255, 255, triangle) == 6);
		bool foundCorner = false;
		for (const Vector3& point : triangle)
			foundCorner = foundCorner || (point - Vector3(256, 3, 256)).SquareLength() < 1e-6f;
		EXPECT(foundCorner);
	}

	{
		PhysicsWorld world{ PhysicsWorldParam() };
		Geometry* terrain = CreateTerrain(2.0f, 5);
		Quaternion bodyRotation, localRotation;
		bodyRotation.FromRotationAxis(Vector3::UnitY(), 0.7f);
		localRotation.FromRotationAxis(Vector3::UnitZ(), 0.35f);
		const Transform bodyTransform(Vector3(20, 3, -7), bodyRotation);
		const Transform shapeTransform(bodyTransform.LocalToWorld(Vector3(2, 0.5f, -1)), bodyRotation * localRotation);
		terrain->SetTransform(shapeTransform.pos, shapeTransform.quat);
		Geometry* other = GeometryFactory::CreateSphere(bodyTransform.LocalToWorld(Vector3(-4, 0, 0)), 0.2f);
		RigidBodyParam settings;
		settings.rigidType = RigidType::Static;
		RigidBody* body = world.CreateRigidBody(std::vector<Geometry*>{ terrain, other }, settings, bodyTransform);
		EXPECT(body != nullptr);
		EXPECT_SAME(body->GetGeometryTransform(terrain).pos, shapeTransform.pos);
		const Vector3 center = shapeTransform.LocalToWorld(Vector3::Zero());
		EXPECT(world.ModifyHeightField(terrain, center, 1.5f, 1.0f, false) == 9);
		EXPECT_NEAR(terrain->GetShapeObj<HeightField3>()->GetHeight(12), 1.0f, 1e-5f);
		RayCastOption option;
		option.MaxDist = 20.0f;
		RayCastResult result;
		const Vector3 rayOrigin = shapeTransform.LocalToWorld(Vector3(0.25f, 5.0f, 0.25f));
		const Vector3 rayDirection = -shapeTransform.LocalToWorldDirection(Vector3::UnitY());
		EXPECT(world.GetGeometryQuery()->RayCastQuery(rayOrigin, rayDirection, option, &result));
		EXPECT_NEAR(result.hitTimeMin, 4.0f, 0.005f);
		EXPECT(world.FillHeightField(terrain, center, 1.5f, 2.0f, 1.0f, false) == 9);
		result.Reset();
		EXPECT(world.GetGeometryQuery()->RayCastQuery(rayOrigin, rayDirection, option, &result));
		EXPECT_NEAR(result.hitTimeMin, 3.0f, 0.005f);
		EXPECT(world.SmoothHeightField(terrain, center, 1.5f, 1.0f, 2, false) > 0);
		EXPECT(world.SetHeightFieldHeights(terrain, 2, 2, 1, 1, { 0.5f }) == 1);
		EXPECT_NEAR(terrain->GetShapeObj<HeightField3>()->GetHeight(12), 0.5f, 1e-5f);
	}

	{
		PhysicsWorldParam parameters;
		parameters.gravityAcc = Vector3(0, -10, 0);
		parameters.integrateMethod = IntegrateMethod::SymplecticEuler;
		PhysicsWorld world(parameters);
		Geometry* terrain = CreateTerrain(10.0f, 21);
		RigidBodyParam terrainSettings;
		terrainSettings.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(terrain, terrainSettings) != nullptr);
		RigidBodyDynamic* nearBody = CreateTerrainTestBody(world, Vector3(0, 0.2f, 0));
		RigidBodyDynamic* farBody = CreateTerrainTestBody(world, Vector3(8, 0.2f, 0));
		RigidBodyDynamic* frozenBody = CreateTerrainTestBody(world, Vector3(0.5f, 0.2f, 0));
		nearBody->Sleep();
		farBody->Sleep();
		frozenBody->Freeze();
		const Vector3 initialNear = nearBody->X, initialFar = farBody->X, initialFrozen = frozenBody->X;
		const uint64_t revision = terrain->GetShapeObj<HeightField3>()->GetRevision();
		EXPECT(world.SetHeightFieldHeight(terrain, 10, 10, 0.0f));
		EXPECT(nearBody->Sleeping && farBody->Sleeping && frozenBody->Sleeping);
		EXPECT(terrain->GetShapeObj<HeightField3>()->GetRevision() == revision);
		EXPECT(world.ModifyHeightField(terrain, Vector3::Zero(), 2.0f, -2.0f, false) == 13);
		EXPECT(!nearBody->Sleeping);
		EXPECT(farBody->Sleeping);
		EXPECT(frozenBody->Sleeping && frozenBody->Freezing);
		for (int frame = 0; frame < 30; ++frame) world.Simulate();
		EXPECT(nearBody->X.y < initialNear.y - 0.5f);
		EXPECT_SAME(farBody->X, initialFar);
		EXPECT_SAME(frozenBody->X, initialFrozen);
		EXPECT(farBody->Sleeping && frozenBody->Sleeping);
	}
}

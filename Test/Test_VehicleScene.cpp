#include "Test.h"

#include <algorithm>
#include <vector>

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/VehicleDeformation.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	constexpr float VehicleStep = 1.0f / 60.0f;

	Geometry* VehicleSceneGeometry(const SceneWorld& scene, const std::string& id)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
			if (object.Id == id) return object.GeometryPtr;
		return nullptr;
	}

	Transform VehicleGeometryTransform(const Geometry* geometry)
	{
		const RigidBody* body = geometry->GetParent<RigidBody>();
		return body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
	}

	bool VehicleFinite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	void CheckVehicleTransform(const Transform& actual, const Transform& expected)
	{
		EXPECT_SAME(actual.pos, expected.pos);
		EXPECT_NEAR(std::abs(actual.quat.Dot(expected.quat)), 1.0f, 1.0e-5f);
	}

	void CheckVehicleRendering(const SceneWorld& scene, const char* id, VehicleDeformation* vehicle,
		float wheelRadius, float wheelWidth, float wheelTrack, float wheelbase, float wheelHeight)
	{
		EXPECT(vehicle != nullptr);
		if (!vehicle) return;
		Geometry* surface = VehicleSceneGeometry(scene, std::string(id) + "_body");
		EXPECT(surface != nullptr && surface == vehicle->GetRenderGeometry());
		if (!surface) return;
		EXPECT(surface->GetParent<RigidBody>() == nullptr);
		EXPECT_FALSE(surface->IsSimulationEnabled());
		EXPECT_FALSE(surface->IsQueryEnabled());
		const Transform frame = vehicle->GetWorldTransform();
		CheckVehicleTransform(VehicleGeometryTransform(surface), frame);
		const TriangleMesh* mesh = surface->GetShapeObj<TriangleMesh>();
		EXPECT(mesh != nullptr);
		if (!mesh) return;
		const DeformableBody& cage = vehicle->GetCage();
		EXPECT(mesh->NumVertices == (uint32_t)cage.GetParticleCount());
		EXPECT(mesh->NumTriangles == cage.GetSurfaceTriangles().size());
		EXPECT(mesh->NumTriangles > 0);
		EXPECT(mesh->mNormals.size() == mesh->NumVertices);
		if (mesh->NumVertices == 0 || mesh->NumVertices != (uint32_t)cage.GetParticleCount()) return;
		for (uint32_t i = 0; i < mesh->NumVertices; ++i)
		{
			EXPECT(VehicleFinite(mesh->Vertices[i]));
			EXPECT_SAME(mesh->Vertices[i], cage.GetParticle((int)i).Position);
			EXPECT(VehicleFinite(cage.GetParticle((int)i).Velocity));
			if (i < mesh->mNormals.size())
			{
				EXPECT(VehicleFinite(mesh->mNormals[i]));
				const float squared = mesh->mNormals[i].SquareLength();
				EXPECT(squared < 1.0e-8f || std::abs(squared - 1.0f) < 1.0e-3f);
			}
		}
		const Box3 bounds(mesh->Vertices, mesh->NumVertices);
		EXPECT_SAME(mesh->BoundingVolume.Min, bounds.Min);
		EXPECT_SAME(mesh->BoundingVolume.Max, bounds.Max);
		const Box3 worldBounds = Box3::Transform(bounds, frame.pos, frame.quat);
		EXPECT_SAME(surface->GetBounds().Min, worldBounds.Min);
		EXPECT_SAME(surface->GetBounds().Max, worldBounds.Max);
		for (uint32_t i = 0; i < mesh->NumTriangles; ++i)
		{
			uint32_t a, b, c;
			mesh->GetVertIndices(i, a, b, c);
			EXPECT(a < mesh->NumVertices && b < mesh->NumVertices && c < mesh->NumVertices);
			if (a < mesh->mNormals.size() && b < mesh->mNormals.size() && c < mesh->mNormals.size())
			{
				EXPECT_NEAR(mesh->mNormals[a].SquareLength(), 1.0f, 1.0e-3f);
				EXPECT_NEAR(mesh->mNormals[b].SquareLength(), 1.0f, 1.0e-3f);
				EXPECT_NEAR(mesh->mNormals[c].SquareLength(), 1.0f, 1.0e-3f);
			}
		}

		int wheelIndex = 0;
		for (int side = -1; side <= 1; side += 2)
		for (int end = -1; end <= 1; end += 2)
		{
			Geometry* wheel = VehicleSceneGeometry(scene, std::string(id) + "_wheel_" + std::to_string(wheelIndex++));
			EXPECT(wheel != nullptr);
			if (!wheel) continue;
			EXPECT(wheel->GetParent<RigidBody>() == nullptr);
			EXPECT_FALSE(wheel->IsSimulationEnabled());
			EXPECT_FALSE(wheel->IsQueryEnabled());
			const Cylinder3* cylinder = wheel->GetShapeObj<Cylinder3>();
			EXPECT(cylinder != nullptr);
			if (!cylinder) continue;
			EXPECT_NEAR(cylinder->Radius, wheelRadius, 1.0e-6f);
			EXPECT_NEAR(cylinder->GetHeight(), wheelWidth, 1.0e-6f);
			const Transform wheelPose = VehicleGeometryTransform(wheel);
			const Vector3 expectedCenter = frame.LocalToWorld(Vector3(side * wheelTrack * 0.5f, wheelHeight, end * wheelbase * 0.5f));
			EXPECT_SAME(wheelPose.LocalToWorld(cylinder->GetCenter()), expectedCenter);
			const Vector3 axle = wheelPose.LocalToWorldDirection(cylinder->GetUnitAxis());
			EXPECT_NEAR(std::abs(axle.Dot(frame.LocalToWorldDirection(Vector3::UnitX()))), 1.0f, 1.0e-5f);
		}
	}

	const char* VehicleFrameJson()
	{
		return R"json({
			"name": "Vehicle local frame regression",
			"physics": {"gravity": [0, 0, 0], "resolutionSolver": "pbd"},
			"objects": [{"id": "frame_car", "type": "box", "body": "dynamic", "position": [7, 3, -4],
				"rotationAxis": [0, 1, 0], "rotationDegrees": 90, "halfExtent": [1, 0.5, 2], "simulation": false, "invMass": 1}],
			"vehicles": [{"id": "framed", "chassis": "frame_car", "halfExtent": [0.8, 0.4, 1.8],
				"cells": [2, 2, 4], "cabinHeight": 0.2, "shellMass": 4,
				"position": [0.3, 0.6, -0.2], "rotationAxis": [0, 0, 1], "rotationDegrees": 30,
				"impactRadius": 1, "impulseScale": 0.1, "minImpactImpulse": 5, "maxDeformationImpulse": 10,
				"wheelRadius": 0.3, "wheelWidth": 0.2, "wheelTrack": 1.8, "wheelbase": 2.4, "wheelHeight": -0.2,
				"material": {"substeps": 2, "iterations": 8, "compliance": 0.0001, "volumeCompliance": 0.0001,
					"damping": 3, "yieldStrain": 0.05, "plasticCreep": 4, "maxPlasticStrain": 0.5}}]
		})json";
	}

	void TestVehicleSceneFrames()
	{
		SceneWorld scene;
		std::string error;
		const bool loaded = scene.LoadFromText(VehicleFrameJson(), "vehicle_frame.json", &error);
		if (!loaded) printf("Vehicle frame scene failed: %s\n", error.c_str());
		EXPECT(loaded);
		if (!loaded) return;
		EXPECT(scene.GetVehicleCount() == 1);
		EXPECT(scene.GetVehicle(-1) == nullptr);
		EXPECT(scene.GetVehicle(1) == nullptr);
		VehicleDeformation* vehicle = scene.GetVehicle(0);
		Geometry* chassis = VehicleSceneGeometry(scene, "frame_car");
		EXPECT(vehicle != nullptr && chassis != nullptr);
		if (!vehicle || !chassis) return;
		RigidBody* body = chassis->GetParent<RigidBody>();
		EXPECT(body != nullptr && vehicle->GetBody() == body);
		if (!body) return;
		Quaternion localRotation;
		localRotation.FromRotateZ(3.14159265358979323846f / 6.0f);
		const Vector3 localOffset(0.3f, 0.6f, -0.2f);
		CheckVehicleTransform(vehicle->GetWorldTransform(), Transform(body->BodyLocalToWorld(localOffset), body->Q * localRotation));
		CheckVehicleRendering(scene, "framed", vehicle, 0.3f, 0.2f, 1.8f, 2.4f, -0.2f);
		const uint32_t revision = scene.GetRenderRevision();
		for (int frame = 0; frame < 20; ++frame)
		{
			body->X = Vector3(7.0f + frame * 0.2f, 3.0f + frame * 0.03f, -4.0f);
			body->Q.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.15f * frame);
			scene.Step(VehicleStep);
			CheckVehicleTransform(vehicle->GetWorldTransform(), Transform(body->BodyLocalToWorld(localOffset), body->Q * localRotation));
			CheckVehicleRendering(scene, "framed", vehicle, 0.3f, 0.2f, 1.8f, 2.4f, -0.2f);
			EXPECT(vehicle->GetMaxDisplacement() < 1.0e-5f);
			EXPECT(vehicle->GetMaxPlasticStrain() < 1.0e-6f);
		}
		EXPECT(scene.GetRenderRevision() > revision);
	}

	void TestInvalidVehicleScenes()
	{
		const char* valid[] = {
			"\"vehicles\": [", "\"chassis\": \"frame_car\"", "\"body\": \"dynamic\"",
			"\"cells\": [2, 2, 4]", "\"cells\": [2, 2, 4]", "\"halfExtent\": [0.8, 0.4, 1.8]",
			"\"wheelRadius\": 0.3", "\"minImpactImpulse\": 5",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": 30",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": 30",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": 30",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": 30",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": 30",
			"\"objects\": ["
		};
		const char* invalid[] = {
			"\"vehicles\": {} , \"unused\": [", "\"chassis\": \"missing_chassis\"", "\"body\": \"static\"",
			"\"cells\": [2.5, 2, 4]", "\"cells\": [0, 2, 4]", "\"halfExtent\": [0, 0.4, 1.8]",
			"\"wheelRadius\": -0.3", "\"minImpactImpulse\": -5",
			"\"rotation\": [0, 0, 1]",
			"\"rotation\": [0, 0, 0, \"bad\"]",
			"\"rotation\": [0, 0, 0, 1e400]",
			"\"rotationAxis\": [0, 0, 0], \"rotationDegrees\": 30",
			"\"rotationAxis\": [0, 0, 1], \"rotationDegrees\": \"bad\"",
			"\"objects\": [{\"id\": \"framed_wheel_0\", \"type\": \"sphere\", \"body\": \"static\", \"radius\": 0.1}, "
		};
		for (size_t index = 0; index < sizeof(valid) / sizeof(valid[0]); ++index)
		{
			std::string json = VehicleFrameJson();
			const size_t position = json.find(valid[index]);
			EXPECT(position != std::string::npos);
			if (position == std::string::npos) continue;
			json.replace(position, std::string(valid[index]).size(), invalid[index]);
			SceneWorld scene;
			std::string error;
			EXPECT_FALSE(scene.LoadFromText(json, "invalid_vehicle.json", &error));
			EXPECT(!error.empty());
			EXPECT(error.find("invalid_vehicle.json") != std::string::npos);
			scene.Reset();
			EXPECT(scene.GetVehicleCount() == 0);
			EXPECT(scene.GetObjects().empty());
		}
	}

	bool LoadVehicleDemo(SceneWorld& scene)
	{
		std::string error;
		const bool loaded = scene.LoadFromFile(TestContentsPath() + "Scenes/19_vehicle_deformation.json", &error);
		if (!loaded) printf("Vehicle demo scene failed: %s\n", error.c_str());
		EXPECT(loaded);
		return loaded;
	}

	void ExportVehicleCage(VehicleDeformation& vehicle, const char* name)
	{
		// Both snapshots use cage-local coordinates so rigid travel does not hide the dent.
		Geometry* geometry = vehicle.GetRenderGeometry();
		TriangleMesh* mesh = geometry ? geometry->GetShapeObj<TriangleMesh>() : nullptr;
		const std::string path = TestOutputPath(name);
		if (!mesh || !mesh->ExportObj(path.c_str()))
			printf("Vehicle QA snapshot could not be written: %s\n", path.c_str());
	}

	float FrontDent(const VehicleDeformation& vehicle)
	{
		float dent = 0.0f;
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			if (particle.InitialPosition.z > vehicle.GetSettings().HalfExtent.z * 0.75f)
				dent = std::max(dent, particle.InitialPosition.z - particle.Position.z);
		}
		return dent;
	}
}

void TestVehicleScene()
{
	printf("Running TestVehicleScene\n");
	TestVehicleSceneFrames();
	TestInvalidVehicleScenes();
	SceneWorld scene;
	if (!LoadVehicleDemo(scene)) return;
	EXPECT(scene.GetVehicleCount() == 1);
	EXPECT(scene.GetSimulation()->GetDeformableBodyCount() == 0);
	VehicleDeformation* vehicle = scene.GetVehicle(0);
	Geometry* chassis = VehicleSceneGeometry(scene, "car");
	Geometry* wall = VehicleSceneGeometry(scene, "wall");
	EXPECT(vehicle != nullptr && chassis != nullptr && wall != nullptr);
	if (!vehicle || !chassis || !wall) return;
	RigidBody* body = chassis->GetParent<RigidBody>();
	EXPECT(body != nullptr && body == vehicle->GetBody());
	if (!body) return;
	const Vector3 initialPosition = body->X;
	const float initialForwardSpeed = body->V.z;
	const size_t objectCount = scene.GetObjects().size();
	const uint32_t initialRevision = scene.GetRenderRevision();
	EXPECT(initialForwardSpeed > 5.0f);
	EXPECT_NEAR(1.0f / body->InvMass, 80.0f, 0.01f);
	CheckVehicleRendering(scene, "crash_car", vehicle, 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);
	ExportVehicleCage(*vehicle, "vehicle_before.obj");
	int firstImpactFrame = -1, firstDamageFrame = -1;
	float peakWallImpulse = 0.0f, peakDent = 0.0f, peakDisplacement = 0.0f;
	for (int frame = 0; frame < 300; ++frame)
	{
		scene.Step(VehicleStep);
		EXPECT(VehicleFinite(body->X) && VehicleFinite(body->V) && VehicleFinite(body->W));
		EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1.0e-3f);
		const PBDSolver* solver = scene.GetSimulation()->GetPositionSolver();
		EXPECT(solver != nullptr);
		float wallImpulse = 0.0f;
		if (solver)
		{
			const std::vector<Geometry*>& geometries = solver->GetContactGeometries();
			for (const ContactManifold* manifold : solver->GetContactManifolds())
			{
				if (!manifold || manifold->indexA < 0 || manifold->indexB < 0
					|| (size_t)manifold->indexA >= geometries.size() || (size_t)manifold->indexB >= geometries.size()) continue;
				Geometry* a = geometries[(size_t)manifold->indexA];
				Geometry* b = geometries[(size_t)manifold->indexB];
				if (!((a == chassis && b == wall) || (a == wall && b == chassis))) continue;
				for (int contact = 0; contact < manifold->NumContactPointCount; ++contact)
					wallImpulse += manifold->ContactPoints[contact].totalImpulseNormal;
			}
		}
		if (wallImpulse > 0.1f && firstImpactFrame < 0) firstImpactFrame = frame;
		peakWallImpulse = std::max(peakWallImpulse, wallImpulse);
		peakDent = std::max(peakDent, FrontDent(*vehicle));
		peakDisplacement = std::max(peakDisplacement, vehicle->GetMaxDisplacement());
		if (vehicle->GetMaxDisplacement() > 0.01f && firstDamageFrame < 0) firstDamageFrame = frame;
		if (frame < 10) EXPECT(vehicle->GetMaxDisplacement() < 1.0e-4f);
		if (frame % 30 == 0) CheckVehicleRendering(scene, "crash_car", vehicle, 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);
		EXPECT(vehicle->GetPendingImpactCount() == 0);
	}
	printf("Vehicle crash: impact frame=%d damage frame=%d, peak impulse=%.4f, displacement=%.4f front dent=%.4f remaining=%.4f plastic=%.5f, final speed=%.4f z=%.4f\n",
		firstImpactFrame, firstDamageFrame, peakWallImpulse, peakDisplacement, peakDent, FrontDent(*vehicle),
		vehicle->GetMaxPlasticStrain(), body->V.Length(), body->X.z);
	EXPECT(firstImpactFrame >= 0 && firstImpactFrame < 120);
	EXPECT(firstDamageFrame >= firstImpactFrame && firstDamageFrame < 180);
	EXPECT(peakWallImpulse > 10.0f);
	EXPECT(peakDent > 0.02f);
	EXPECT(FrontDent(*vehicle) > 0.01f);
	EXPECT(vehicle->GetMaxPlasticStrain() > 1.0e-4f);
	EXPECT(std::abs(body->V.z) < initialForwardSpeed * 0.5f);
	EXPECT(body->X.z > initialPosition.z + 1.0f);
	EXPECT(body->X.z < 1.5f);
	EXPECT(scene.GetRenderRevision() > initialRevision);
	CheckVehicleRendering(scene, "crash_car", vehicle, 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);

	ExportVehicleCage(*vehicle, "vehicle_after.obj");
	const Transform poseBeforeReset(body->X, body->Q);
	const uint64_t beforeReset = vehicle->GetRevision();
	const uint32_t sceneBeforeReset = scene.GetRenderRevision();
	vehicle->Reset();
	EXPECT(vehicle->GetRevision() > beforeReset);
	EXPECT(vehicle->GetBody() == body);
	CheckVehicleTransform(Transform(body->X, body->Q), poseBeforeReset);
	EXPECT(vehicle->GetPendingImpactCount() == 0);
	EXPECT_NEAR(vehicle->GetMaxDisplacement(), 0.0f, 1.0e-7f);
	EXPECT_NEAR(vehicle->GetMaxPlasticStrain(), 0.0f, 1.0e-7f);
	for (const DeformableParticle& particle : vehicle->GetCage().GetParticles())
	{
		EXPECT_SAME(particle.Position, particle.InitialPosition);
		EXPECT_SAME(particle.Velocity, Vector3::Zero());
		EXPECT_SAME(particle.Force, Vector3::Zero());
	}
	CheckVehicleRendering(scene, "crash_car", vehicle, 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);
	scene.Step(VehicleStep);
	EXPECT(scene.GetRenderRevision() > sceneBeforeReset);
	EXPECT(vehicle->GetPendingImpactCount() == 0);

	for (int reload = 0; reload < 3; ++reload)
	{
		if (!LoadVehicleDemo(scene)) return;
		EXPECT(scene.GetVehicleCount() == 1);
		EXPECT(scene.GetObjects().size() == objectCount);
		vehicle = scene.GetVehicle(0);
		chassis = VehicleSceneGeometry(scene, "car");
		EXPECT(vehicle != nullptr && chassis != nullptr);
		if (!vehicle || !chassis) return;
		EXPECT_SAME(VehicleGeometryTransform(chassis).pos, initialPosition);
		EXPECT_NEAR(vehicle->GetMaxDisplacement(), 0.0f, 1.0e-7f);
		EXPECT_NEAR(vehicle->GetMaxPlasticStrain(), 0.0f, 1.0e-7f);
		scene.Step(VehicleStep);
		CheckVehicleRendering(scene, "crash_car", vehicle, 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);
	}
	scene.Reset();
	EXPECT(scene.GetVehicleCount() == 0);
	EXPECT(scene.GetVehicle(0) == nullptr);
	EXPECT(scene.GetObjects().empty());
	if (LoadVehicleDemo(scene))
	{
		scene.Step(VehicleStep);
		CheckVehicleRendering(scene, "crash_car", scene.GetVehicle(0), 0.4f, 0.25f, 2.1f, 2.8f, -0.25f);
	}
}

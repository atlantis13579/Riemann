#include "Test.h"

#include <algorithm>
#include <chrono>
#include <iterator>

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Cloth/PBDBackend.h"
#include "../Src/Deformation/VehicleDeformation.h"
#include "../Src/Cloth/XPBDSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	bool BackendSceneFinite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	Geometry* BackendSceneGeometry(const SceneWorld& scene, const char* id)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
			if (object.Id == id) return object.GeometryPtr;
		return nullptr;
	}

	bool LoadBackendDemo(SceneWorld& scene, const char* fileName, const char* backendName)
	{
		const std::string path = TestContentsPath() + "Scenes/" + fileName;
		std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
		EXPECT(!!file);
		if (!file) return false;
		std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		const size_t physics = text.find("\"physics\"");
		const size_t opening = physics == std::string::npos ? std::string::npos : text.find('{', physics);
		EXPECT(opening != std::string::npos);
		if (opening == std::string::npos) return false;
		// Use the shipped scene unchanged apart from the requested hardware backend.
		text.insert(opening + 1, std::string("\"solverBackend\":\"") + backendName + "\",\"allowBackendFallback\":false,");
		std::string error;
		const bool loaded = scene.LoadFromText(text, path, &error);
		if (!loaded) printf("Backend demo %s/%s failed: %s\n", backendName, fileName, error.c_str());
		EXPECT(loaded);
		return loaded;
	}

	void CheckBackendSceneBodies(const SceneWorld& scene)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
		{
			const Geometry* geometry = object.GeometryPtr;
			EXPECT(geometry != nullptr);
			if (!geometry) continue;
			const RigidBody* body = geometry->GetParent<RigidBody>();
			const Transform pose = body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
			EXPECT(BackendSceneFinite(pose.pos));
			EXPECT_NEAR(pose.quat.SquareLength(), 1.0f, 1.0e-3f);
			if (body)
			{
				EXPECT(BackendSceneFinite(body->X) && BackendSceneFinite(body->V) && BackendSceneFinite(body->W));
				EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1.0e-3f);
			}
		}
	}

	float BackendVehicleWallImpulse(const PBDSolver& solver, const Geometry* car, const Geometry* wall)
	{
		float impulse = 0.0f;
		const std::vector<Geometry*>& geometries = solver.GetContactGeometries();
		for (const ContactManifold* manifold : solver.GetContactManifolds())
		{
			if (!manifold || manifold->indexA < 0 || manifold->indexB < 0
				|| (size_t)manifold->indexA >= geometries.size() || (size_t)manifold->indexB >= geometries.size()) continue;
			const Geometry* a = geometries[(size_t)manifold->indexA];
			const Geometry* b = geometries[(size_t)manifold->indexB];
			if (!((a == car && b == wall) || (a == wall && b == car))) continue;
			for (int i = 0; i < manifold->NumContactPointCount; ++i)
				impulse += manifold->ContactPoints[i].totalImpulseNormal;
		}
		return impulse;
	}

	void RunBackendDemo(SolverBackend backend, const char* backendName, const char* fileName,
		ResolutionPhaseSolver mode, bool vehicleDemo)
	{
		const auto started = std::chrono::steady_clock::now();
		SceneWorld scene;
		if (!LoadBackendDemo(scene, fileName, backendName)) return;
		PhysicsWorld* world = scene.GetSimulation();
		EXPECT(world->GetResolutionSolver() == mode);
		EXPECT(world->GetSolverBackend() == backend);
		const PBDSolver* solver = world->GetPositionSolver();
		EXPECT(solver != nullptr);
		if (!solver) return;
		EXPECT(solver->GetBackend() == backend);
		if (mode == ResolutionPhaseSolver::XPBD)
		{
			const XPBDSolver* compliantSolver = dynamic_cast<const XPBDSolver*>(solver);
			EXPECT(compliantSolver != nullptr);
			if (compliantSolver) EXPECT_NEAR(compliantSolver->GetXPBDSettings().ContactCompliance, 0.003f, 1.0e-7f);
		}
		VehicleDeformation* vehicle = vehicleDemo ? scene.GetVehicle(0) : nullptr;
		Geometry* car = vehicleDemo ? BackendSceneGeometry(scene, "car") : nullptr;
		Geometry* wall = vehicleDemo ? BackendSceneGeometry(scene, "wall") : nullptr;
		if (vehicleDemo)
		{
			EXPECT(scene.GetVehicleCount() == 1 && vehicle != nullptr && car != nullptr && wall != nullptr);
			if (!vehicle || !car || !wall) return;
		}
		const uint32_t originalRevision = scene.GetRenderRevision();
		int contactFrames = 0, firstImpact = -1, firstDamage = -1;
		float peakImpulse = 0.0f, peakDamage = 0.0f, maxSoftDistance = 0.0f, maxHardError = 0.0f;
		for (int frame = 0; frame < 60; ++frame)
		{
			scene.Step(1.0f / 60.0f);
			CheckBackendSceneBodies(scene);
			if (solver->GetContactCount() > 0) ++contactFrames;
			for (const ContactManifold* manifold : solver->GetContactManifolds())
			{
				EXPECT(manifold != nullptr);
				if (!manifold) continue;
				for (int i = 0; i < manifold->NumContactPointCount; ++i)
				{
					const Contact& contact = manifold->ContactPoints[i];
					EXPECT(BackendSceneFinite(contact.Normal));
					EXPECT(std::isfinite(contact.totalImpulseNormal) && contact.totalImpulseNormal >= 0.0f);
				}
			}
			if (mode == ResolutionPhaseSolver::XPBD)
			{
				const std::vector<ConstraintControlState> controls = scene.GetConstraintControls();
				EXPECT(controls.size() == 2);
				if (controls.size() == 2)
				{
					maxSoftDistance = std::max(maxSoftDistance, controls[0].CurrentValue);
					maxHardError = std::max(maxHardError, std::abs(controls[1].CurrentValue - 1.5f));
				}
			}
			if (vehicle)
			{
				const float impulse = BackendVehicleWallImpulse(*solver, car, wall);
				if (impulse > 0.1f && firstImpact < 0) firstImpact = frame;
				if (vehicle->GetMaxDisplacement() > 0.01f && firstDamage < 0) firstDamage = frame;
				peakImpulse = std::max(peakImpulse, impulse);
				peakDamage = std::max(peakDamage, vehicle->GetMaxDisplacement());
				for (const DeformableParticle& particle : vehicle->GetCage().GetParticles())
					EXPECT(BackendSceneFinite(particle.Position) && BackendSceneFinite(particle.Velocity));
				EXPECT(vehicle->GetPendingImpactCount() == 0);
				if (frame < 10) EXPECT(vehicle->GetMaxDisplacement() < 1.0e-4f);
			}
		}
		EXPECT(contactFrames > 0);
		const PBDBackendStatistics& stats = solver->GetBackendStatistics();
		EXPECT(stats.RequestedBackend == backend && stats.Backend == backend && !stats.UsedFallback);
		EXPECT(stats.PositionDispatches > 0 && stats.ReconstructionDispatches > 0 && stats.VelocityDispatches > 0);
		EXPECT(stats.ContactRows > 0 && stats.MaxBatchSize > 0);
		if (mode == ResolutionPhaseSolver::XPBD)
		{
			EXPECT(maxSoftDistance > 1.65f && maxSoftDistance < 2.3f);
			EXPECT(maxHardError < 0.01f);
		}
		if (vehicle)
		{
			// Damage must arrive through a real wall contact and the world's vehicle callback.
			EXPECT(firstImpact >= 0 && firstDamage >= firstImpact && firstDamage < 60);
			EXPECT(peakImpulse > 10.0f && peakDamage > 0.02f);
			EXPECT(vehicle->GetMaxPlasticStrain() > 1.0e-4f);
			EXPECT(std::abs(vehicle->GetBody()->V.z) < 4.0f);
			EXPECT(scene.GetRenderRevision() > originalRevision);
			printf("  %s vehicle: impact=%d damage=%d impulse=%.4f deformation=%.4f plastic=%.5f speed=%.4f\n",
				backendName, firstImpact, firstDamage, peakImpulse, peakDamage, vehicle->GetMaxPlasticStrain(), vehicle->GetBody()->V.Length());
		}
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
		printf("  %s/%s: 60 frames, contacts=%d dispatches=(%llu,%llu,%llu) max batch=%u, %.3f s\n",
			backendName, fileName, contactFrames, (unsigned long long)stats.PositionDispatches,
			(unsigned long long)stats.ReconstructionDispatches, (unsigned long long)stats.VelocityDispatches, stats.MaxBatchSize, seconds);
	}
}

void TestPBDBackendScenes()
{
	printf("Running TestPBDBackendScenes\n");
	const SolverBackend backends[] = { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA };
	const char* names[] = { "CPU", "SIMD", "ComputeShader", "CUDA" };
	for (size_t i = 0; i < 4; ++i)
	{
		if (!IsPBDBackendAvailable(backends[i]))
		{
			printf("  %s demo smoke skipped: backend unavailable\n", names[i]);
			continue;
		}
		RunBackendDemo(backends[i], names[i], "16_pbd_rigid.json", ResolutionPhaseSolver::PBD, false);
		RunBackendDemo(backends[i], names[i], "17_xpbd_rigid.json", ResolutionPhaseSolver::XPBD, false);
		RunBackendDemo(backends[i], names[i], "19_vehicle_deformation.json", ResolutionPhaseSolver::PBD, true);
	}
}

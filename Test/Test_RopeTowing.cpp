#include "Test.h"

#include <algorithm>
#include <cfloat>
#include <vector>

#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	constexpr float TowingStep = 1.0f / 60.0f;
	constexpr int TowingParticles = 31;
	constexpr int TowingSides = 8;
	constexpr float TowingRadius = 0.06f;

	Geometry* TowingGeometry(const SceneWorld& scene, const char* id)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
			if (object.Id == id) return object.GeometryPtr;
		return nullptr;
	}

	Transform TowingTransform(const Geometry* geometry)
	{
		const RigidBody* body = geometry->GetParent<RigidBody>();
		return body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
	}

	bool TowingFinite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	void CheckTowingBodies(const SceneWorld& scene)
	{
		for (const SceneObjectInstance& object : scene.GetObjects())
		{
			EXPECT(object.GeometryPtr != nullptr);
			if (!object.GeometryPtr) continue;
			EXPECT(TowingFinite(TowingTransform(object.GeometryPtr).pos));
			const RigidBody* body = object.GeometryPtr->GetParent<RigidBody>();
			if (!body) continue;
			EXPECT(TowingFinite(body->X));
			EXPECT(TowingFinite(body->V));
			EXPECT(TowingFinite(body->W));
			EXPECT_NEAR(body->Q.SquareLength(), 1.0f, 1.0e-3f);
		}
	}

	std::vector<Vector3> TowingCenters(const Geometry* surface, int count, int sides)
	{
		std::vector<Vector3> centers;
		const TriangleMesh* mesh = surface ? surface->GetShapeObj<TriangleMesh>() : nullptr;
		if (!mesh || mesh->NumVertices < (uint32_t)(count * sides)) return centers;
		const Transform transform = TowingTransform(surface);
		for (int ring = 0; ring < count; ++ring)
		{
			Vector3 center = Vector3::Zero();
			for (int side = 0; side < sides; ++side) center += mesh->Vertices[ring * sides + side];
			centers.push_back(transform.LocalToWorld(center / (float)sides));
		}
		return centers;
	}

	void CheckTowingTube(Geometry* surface, int count, int sides, float radius, bool checkWinding)
	{
		EXPECT(surface != nullptr);
		if (!surface) return;
		EXPECT(surface->GetParent<RigidBody>() == nullptr);
		EXPECT_FALSE(surface->IsSimulationEnabled());
		EXPECT_FALSE(surface->IsQueryEnabled());
		const TriangleMesh* mesh = surface->GetShapeObj<TriangleMesh>();
		EXPECT(mesh != nullptr);
		if (!mesh) return;
		EXPECT(mesh->NumVertices == (uint32_t)(count * sides + 2 * (sides + 1)));
		EXPECT(mesh->NumTriangles == (uint32_t)(2 * count * sides));
		EXPECT(mesh->mNormals.size() == mesh->NumVertices);
		if (mesh->NumVertices == 0 || mesh->mNormals.size() != mesh->NumVertices) return;
		for (uint32_t vertex = 0; vertex < mesh->NumVertices; ++vertex)
		{
			EXPECT(TowingFinite(mesh->Vertices[vertex]));
			EXPECT(TowingFinite(mesh->mNormals[vertex]));
			EXPECT_NEAR(mesh->mNormals[vertex].SquareLength(), 1.0f, 1.0e-3f);
		}
		const Box3 bounds(mesh->Vertices, mesh->NumVertices);
		EXPECT_SAME(mesh->BoundingVolume.Min, bounds.Min);
		EXPECT_SAME(mesh->BoundingVolume.Max, bounds.Max);
		EXPECT_SAME(surface->GetShapeBounds().Min, bounds.Min);
		EXPECT_SAME(surface->GetShapeBounds().Max, bounds.Max);
		const Box3 worldBounds = Box3::Transform(bounds, surface->GetPosition(), surface->GetRotation());
		EXPECT_SAME(surface->GetBounds().Min, worldBounds.Min);
		EXPECT_SAME(surface->GetBounds().Max, worldBounds.Max);

		const std::vector<Vector3> centers = TowingCenters(surface, count, sides);
		EXPECT(centers.size() == (size_t)count);
		for (int ring = 0; ring < (int)centers.size(); ++ring)
		{
			const Vector3 localCenter = TowingTransform(surface).WorldToLocal(centers[ring]);
			for (int side = 0; side < sides; ++side)
			{
				const int vertex = ring * sides + side;
				const Vector3 radial = mesh->Vertices[vertex] - localCenter;
				EXPECT_NEAR(radial.Length(), radius, 2.0e-5f);
				EXPECT_NEAR(radial.Dot(mesh->mNormals[vertex]), radius, 2.0e-5f);
			}
		}
		for (uint32_t triangle = 0; triangle < mesh->NumTriangles; ++triangle)
		{
			uint32_t a, b, c;
			mesh->GetVertIndices(triangle, a, b, c);
			EXPECT(a < mesh->NumVertices && b < mesh->NumVertices && c < mesh->NumVertices);
			if (!checkWinding || a >= mesh->NumVertices || b >= mesh->NumVertices || c >= mesh->NumVertices) continue;
			const Vector3 faceNormal = (mesh->Vertices[b] - mesh->Vertices[a]).Cross(mesh->Vertices[c] - mesh->Vertices[a]);
			EXPECT(faceNormal.Dot(mesh->mNormals[a] + mesh->mNormals[b] + mesh->mNormals[c]) > 0.0f);
		}
	}

	bool LoadTowingDemo(SceneWorld& scene)
	{
		std::string error;
		const bool loaded = scene.LoadFromFile(TestContentsPath() + "Scenes/18_rope_towing.json", &error);
		if (!loaded) printf("Rope towing scene failed: %s\n", error.c_str());
		EXPECT(loaded);
		return loaded;
	}

	void TestTowingSlack()
	{
		// Isolate unilateral rope behavior from ground friction, gravity and inertia.
		const char* json = R"json({
			"name": "Slack towing regression",
			"physics": {"gravity": [0, 0, 0], "resolutionSolver": "pbd"},
			"objects": [
				{"id": "handle", "type": "sphere", "body": "static", "position": [0, 1, 0], "radius": 0.1, "simulation": false},
				{"id": "crate", "type": "box", "body": "dynamic", "position": [1, 1, 0], "halfExtent": [0.2, 0.2, 0.2], "invMass": 1}
			],
			"ropes": [{
				"id": "slack", "start": [0, 1, 0], "end": [1, 1, 0],
				"segments": 1, "iterations": 16, "restLength": 2, "radius": 0.04,
				"gravity": [0, 0, 0], "damping": 1, "tensionOnly": true,
				"pinStart": true, "mouseBody": "handle", "mouseParticle": 0,
				"body": "crate", "bodyAnchor": [1, 1, 0], "renderStyle": "tube", "tubeSides": 8
			}]
		})json";
		SceneWorld scene;
		std::string error;
		const bool loaded = scene.LoadFromText(json, "slack_towing.json", &error);
		EXPECT(loaded);
		if (!loaded) return;
		Geometry* crate = TowingGeometry(scene, "crate");
		Geometry* handle = TowingGeometry(scene, "handle");
		Geometry* surface = TowingGeometry(scene, "slack_surface");
		EXPECT(crate != nullptr && handle != nullptr && surface != nullptr);
		if (!crate || !handle || !surface) return;
		RigidBody* body = crate->GetParent<RigidBody>();
		EXPECT(body != nullptr);
		if (!body) return;
		const Vector3 position = body->X;
		const Quaternion rotation = body->Q;
		EXPECT(scene.SelectMouseControlGeometry(handle));
		for (int frame = 1; frame <= 60; ++frame)
		{
			const Vector3 target(0.5f * frame / 60.0f, 1.0f, 0.0f);
			EXPECT(scene.SetMouseControlPosition(target, TowingStep));
			scene.Step(TowingStep);
			EXPECT_SAME(body->X, position);
			EXPECT_SAME(body->V, Vector3::Zero());
			EXPECT_SAME(body->W, Vector3::Zero());
			EXPECT_NEAR(std::abs(body->Q.Dot(rotation)), 1.0f, 1.0e-6f);
			const std::vector<Vector3> centers = TowingCenters(surface, 2, TowingSides);
			EXPECT(centers.size() == 2);
			if (centers.size() == 2) EXPECT_SAME(centers.front(), target);
		}
		CheckTowingTube(surface, 2, TowingSides, 0.04f, true);
	}
}

void TestRopeTowingDemo()
{
	printf("Running TestRopeTowingDemo\n");
	TestTowingSlack();
	SceneWorld scene;
	if (!LoadTowingDemo(scene)) return;
	EXPECT(scene.GetSimulation()->GetRopeCount() == 1);
	Geometry* handle = TowingGeometry(scene, "handle");
	Geometry* crate = TowingGeometry(scene, "crate");
	Geometry* post = TowingGeometry(scene, "post");
	Geometry* surface = TowingGeometry(scene, "tow_rope_surface");
	EXPECT(handle != nullptr && crate != nullptr && post != nullptr && surface != nullptr);
	if (!handle || !crate || !post || !surface) return;
	RigidBody* body = crate->GetParent<RigidBody>();
	const AxisAlignedBox3* postShape = post->GetShapeObj<AxisAlignedBox3>();
	EXPECT(body != nullptr && postShape != nullptr);
	if (!body || !postShape) return;
	const size_t objectCount = scene.GetObjects().size();
	const Vector3 initialCrate = body->X;
	const Vector3 initialHandle = TowingTransform(handle).pos;
	const Transform postTransform = TowingTransform(post);
	CheckTowingTube(surface, TowingParticles, TowingSides, TowingRadius, true);
	const std::vector<Vector3> initialVertices = surface->GetShapeObj<TriangleMesh>()->mVertices;
	const uint32_t initialRevision = scene.GetRenderRevision();
	EXPECT(scene.IsMouseControlGeometry(handle));
	EXPECT(scene.SelectMouseControlGeometry(handle));

	float minimumPostDistance = FLT_MAX;
	float minimumHeight = FLT_MAX;
	float maximumAngularSpeed = 0.0f;
	float maximumLinearSpeed = 0.0f;
	float maximumAnchorGap = 0.0f;
	struct TowingFrame
	{
		int Frame;
		const char* Phase;
		Vector3 Position, Velocity, AngularVelocity, End, Anchor, Handle;
		Quaternion Rotation;
		float LastLength, PreviousLength;
	};
	std::vector<TowingFrame> frames;
	const char* phase = "settle";
	size_t maximumAngularFrame = 0, maximumLinearFrame = 0;
	int blockedChordFrames = 0;
	int frameIndex = 0;
	auto step = [&]()
	{
		scene.Step(TowingStep);
		CheckTowingBodies(scene);
		const std::vector<Vector3> centers = TowingCenters(surface, TowingParticles, TowingSides);
		EXPECT(centers.size() == TowingParticles);
		if (centers.size() != TowingParticles) return;
		EXPECT_SAME(centers.front(), scene.GetMouseControlPosition());
		EXPECT_SAME(centers.front(), TowingTransform(handle).pos);
		TowingFrame sample;
		sample.Frame = frameIndex + 1;
		sample.Phase = phase;
		sample.Position = body->X;
		sample.Rotation = body->Q;
		sample.Velocity = body->V;
		sample.AngularVelocity = body->W;
		sample.End = centers.back();
		sample.Anchor = body->BodyLocalToWorld(Vector3(-0.65f, 0.4f, 0.35f));
		maximumAnchorGap = std::max(maximumAnchorGap, (sample.End - sample.Anchor).Length());
		sample.Handle = centers.front();
		sample.LastLength = (centers[TowingParticles - 1] - centers[TowingParticles - 2]).Length();
		sample.PreviousLength = (centers[TowingParticles - 2] - centers[TowingParticles - 3]).Length();
		if (body->W.Length() > maximumAngularSpeed)
		{
			maximumAngularSpeed = body->W.Length();
			maximumAngularFrame = frames.size();
		}
		if (body->V.Length() > maximumLinearSpeed)
		{
			maximumLinearSpeed = body->V.Length();
			maximumLinearFrame = frames.size();
		}
		frames.push_back(sample);
		for (size_t i = 0; i < centers.size(); ++i)
		{
			EXPECT(TowingFinite(centers[i]));
			minimumHeight = std::min(minimumHeight, centers[i].y);
			if (i > 0)
			{
				const float squared = postShape->SqrDistanceToSegment(postTransform.WorldToLocal(centers[i - 1]),
					postTransform.WorldToLocal(centers[i]));
				minimumPostDistance = std::min(minimumPostDistance, std::sqrt(std::max(0.0f, squared)));
			}
		}
		if (postShape->IntersectSegment(postTransform.WorldToLocal(centers.front()),
			postTransform.WorldToLocal(centers.back()))) ++blockedChordFrames;
		if (++frameIndex % 30 == 0) CheckTowingTube(surface, TowingParticles, TowingSides, TowingRadius, false);
	};

	for (int frame = 0; frame < 120; ++frame) step();
	const Vector3 settledCrate = body->X;
	auto drag = [&](const Vector3& target, int frames)
	{
		const Vector3 start = scene.GetMouseControlPosition();
		for (int frame = 1; frame <= frames; ++frame)
		{
			const Vector3 position = start + (target - start) * ((float)frame / frames);
			EXPECT(scene.SetMouseControlPosition(position, TowingStep));
			step();
		}
	};
	phase = "cross-post";
	drag(Vector3(-3.5f, 1.0f, -3.0f), 150);
	phase = "pull-left";
	drag(Vector3(-7.0f, 1.0f, -3.0f), 180);
	phase = "hold";
	scene.ReleaseMouseControl();
	for (int frame = 0; frame < 120; ++frame) step();
	const Vector3 displacement = body->X - settledCrate;
	printf("Rope towing: crate delta=(%.4f, %.4f, %.4f), max angular=%.4f, max linear=%.4f, max anchor gap=%.5f, post clearance=%.5f, min height=%.5f, wrapped frames=%d\n",
		displacement.x, displacement.y, displacement.z, maximumAngularSpeed, maximumLinearSpeed, maximumAnchorGap,
		minimumPostDistance, minimumHeight, blockedChordFrames);
	auto printPeak = [&](const char* label, size_t peak)
	{
		if (frames.empty()) return;
		printf("Rope towing %s peak: frame=%d phase=%s speed=%.4f angular=%.4f\n", label,
			frames[peak].Frame, frames[peak].Phase, frames[peak].Velocity.Length(), frames[peak].AngularVelocity.Length());
		const size_t first = peak > 2 ? peak - 2 : 0;
		const size_t last = std::min(peak + 3, frames.size());
		for (size_t i = first; i < last; ++i)
		{
			const TowingFrame& f = frames[i];
			printf("  frame=%d %s X=(%.5f,%.5f,%.5f) Q=(%.5f,%.5f,%.5f,%.5f) V=(%.5f,%.5f,%.5f) W=(%.5f,%.5f,%.5f)\n",
				f.Frame, f.Phase, f.Position.x, f.Position.y, f.Position.z, f.Rotation.x, f.Rotation.y, f.Rotation.z, f.Rotation.w,
				f.Velocity.x, f.Velocity.y, f.Velocity.z, f.AngularVelocity.x, f.AngularVelocity.y, f.AngularVelocity.z);
			printf("    end=(%.5f,%.5f,%.5f) anchor=(%.5f,%.5f,%.5f) gap=%.5f handle=(%.5f,%.5f,%.5f) lastLengths=(%.5f,%.5f)\n",
				f.End.x, f.End.y, f.End.z, f.Anchor.x, f.Anchor.y, f.Anchor.z, (f.End - f.Anchor).Length(),
				f.Handle.x, f.Handle.y, f.Handle.z, f.LastLength, f.PreviousLength);
		}
	};
	if (maximumAngularSpeed >= 10.0f || maximumLinearSpeed >= 10.0f || maximumAnchorGap >= 0.1f)
	{
		printPeak("angular", maximumAngularFrame);
		if (maximumLinearFrame != maximumAngularFrame) printPeak("linear", maximumLinearFrame);
	}
	EXPECT(displacement.x < -0.5f);
	EXPECT(maximumAngularSpeed > 0.05f);
	// The handle moves at at most 2 m/s; a slow tow must not spin the crate violently.
	EXPECT(maximumAngularSpeed < 10.0f);
	EXPECT(maximumLinearSpeed < 10.0f);
	EXPECT(maximumAnchorGap < 0.1f);
	EXPECT(blockedChordFrames > 10);
	EXPECT(minimumPostDistance >= TowingRadius - 0.015f);
	EXPECT(minimumHeight >= TowingRadius - 0.015f);
	EXPECT(scene.GetRenderRevision() > initialRevision);
	const TriangleMesh* mesh = surface->GetShapeObj<TriangleMesh>();
	EXPECT(mesh->mVertices.size() == initialVertices.size());
	if (!initialVertices.empty() && !mesh->mVertices.empty())
		EXPECT((mesh->mVertices[0] - initialVertices[0]).Length() > 1.0f);
	CheckTowingTube(surface, TowingParticles, TowingSides, TowingRadius, false);

	// Repeated loads replace all rope surfaces and mouse targets with fresh state.
	for (int reload = 0; reload < 3; ++reload)
	{
		if (!LoadTowingDemo(scene)) return;
		EXPECT(scene.GetObjects().size() == objectCount);
		EXPECT(scene.GetSimulation()->GetRopeCount() == 1);
		handle = TowingGeometry(scene, "handle");
		crate = TowingGeometry(scene, "crate");
		surface = TowingGeometry(scene, "tow_rope_surface");
		EXPECT(handle != nullptr && crate != nullptr && surface != nullptr);
		if (!handle || !crate || !surface) return;
		EXPECT_SAME(TowingTransform(handle).pos, initialHandle);
		EXPECT_SAME(TowingTransform(crate).pos, initialCrate);
		EXPECT(scene.SelectMouseControlGeometry(handle));
		EXPECT(scene.SetMouseControlPosition(initialHandle + Vector3(-0.1f, 0.0f, 0.0f), TowingStep));
		for (int frame = 0; frame < 10; ++frame)
		{
			scene.Step(TowingStep);
			CheckTowingBodies(scene);
		}
		CheckTowingTube(surface, TowingParticles, TowingSides, TowingRadius, false);
	}
	scene.Reset();
	EXPECT(scene.GetObjects().empty());
	EXPECT(scene.GetSimulation()->GetRopeCount() == 0);
	EXPECT_FALSE(scene.SetMouseControlPosition(initialHandle, TowingStep));
	if (LoadTowingDemo(scene))
	{
		scene.Step(TowingStep);
		CheckTowingBodies(scene);
		CheckTowingTube(TowingGeometry(scene, "tow_rope_surface"), TowingParticles, TowingSides, TowingRadius, false);
	}
}

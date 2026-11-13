#include "Test.h"
#include <memory>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

namespace
{
	using namespace Riemann;
	const float GeometryTolerance = 1.0e-4f;

	void AddQuad(TriangleMesh& Mesh, const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D)
	{
		const uint32_t Base = Mesh.NumVertices;
		Mesh.AddVertex(A); Mesh.AddVertex(B); Mesh.AddVertex(C); Mesh.AddVertex(D);
		Mesh.AddTriangle(Base, Base + 1, Base + 2); Mesh.AddTriangle(Base, Base + 2, Base + 3);
	}
	std::unique_ptr<Geometry> MakeSurface(bool Corner)
	{
		std::unique_ptr<Geometry> Shape(GeometryFactory::CreateTriangleMesh());
		TriangleMesh& Mesh = *Shape->GetShapeObj<TriangleMesh>();
		AddQuad(Mesh, Vector3(-3, 0, -3), Vector3(3, 0, -3), Vector3(3, 0, 3), Vector3(-3, 0, 3));
		if (Corner) AddQuad(Mesh, Vector3(0, 0, -3), Vector3(0, 3, -3), Vector3(0, 3, 3), Vector3(0, 0, 3));
		Mesh.Compact(); Mesh.BuildBVH(); Shape->UpdateVolumeProperties();
		return Shape;
	}
	GeometryWorldState State(Geometry* Shape, uint32_t Index)
	{
		GeometryWorldState Result;
		Result.Geom = Shape; Result.ShapeToWorld = *Shape->GetTransform(); Result.WorldBounds = Shape->GetBounds();
		Result.Handle = GeometryHandle(Index, 1);
		return Result;
	}
	void TestFacePatchesAndSeams()
	{
		auto Floor = MakeSurface(false);
		std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3(0, 0.45f, 0), Vector3(0.5f)));
		MeshContactGenerator Generator;
		std::vector<ContactManifold> Patches;
		Generator.Generate(0, 1, State(Floor.get(), 0), State(Box.get(), 1), Patches);
		EXPECT(Patches.size() == 1);
		if (Patches.empty()) return;
		EXPECT(Patches[0].NumContactPointCount == 4);
		float MinX = FLT_MAX, MaxX = -FLT_MAX, MinZ = FLT_MAX, MaxZ = -FLT_MAX;
		for (int Index = 0; Index < Patches[0].NumContactPointCount; ++Index)
		{
			const Contact& C = Patches[0].ContactPoints[Index];
			EXPECT(C.Normal.Dot(Vector3::UnitY()) > 0.999f);
			EXPECT_NEAR(C.PenetrationDepth, 0.05f, GeometryTolerance);
			EXPECT_NEAR(C.PositionWorldA.y, 0.0f, GeometryTolerance);
			EXPECT_NEAR(C.PositionWorldB.y, -0.05f, GeometryTolerance);
			MinX = std::min(MinX, C.PositionWorldA.x); MaxX = std::max(MaxX, C.PositionWorldA.x);
			MinZ = std::min(MinZ, C.PositionWorldA.z); MaxZ = std::max(MaxZ, C.PositionWorldA.z);
		}
		EXPECT(MaxX - MinX > 0.99f && MaxZ - MinZ > 0.99f);
		const uint32_t FloorPatch = Patches[0].PatchId;
		for (int Step = -8; Step <= 8; ++Step)
		{
			const float X = Step * 0.1f;
			std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Vector3(X, 0.45f, -X), 0.5f));
			Generator.Generate(0, 1, State(Floor.get(), 0), State(Sphere.get(), 1), Patches);
			EXPECT(Patches.size() == 1);
			if (Patches.empty()) continue;
			EXPECT(Patches[0].PatchId == FloorPatch);
			for (int Index = 0; Index < Patches[0].NumContactPointCount; ++Index)
				EXPECT(Patches[0].ContactPoints[Index].Normal.Dot(Vector3::UnitY()) > 0.999f);
		}
		std::unique_ptr<Geometry> Capsule(GeometryFactory::CreateCapsule(Vector3(-1, 0.2f, 0), Vector3(1, 0.2f, 0), 0.25f));
		Generator.Generate(0, 1, State(Floor.get(), 0), State(Capsule.get(), 1), Patches);
		EXPECT(Patches.size() == 1 && Patches[0].NumContactPointCount >= 2);
	}

	void TestMultiplePatchCache()
	{
		auto Corner = MakeSurface(true);
		std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3(0.25f, 0.25f, 0), Vector3(0.5f)));
		const GeometryWorldState A = State(Corner.get(), 0), B = State(Box.get(), 1);
		MeshContactGenerator Generator;
		std::vector<ContactManifold> Patches;
		Generator.Generate(0, 1, A, B, Patches);
		EXPECT(Patches.size() >= 2);
		bool Floor = false, Wall = false;
		ContactManifoldManager Cache;
		Cache.BeginFrame(1);
		for (ContactManifold& Patch : Patches)
		{
			Floor = Floor || Patch.ContactPoints[0].Normal.Dot(Vector3::UnitY()) > 0.99f;
			Wall = Wall || Patch.ContactPoints[0].Normal.Dot(Vector3::UnitX()) > 0.99f;
			Cache.Store(A, B, Patch, 1.0f / 60.0f);
		}
		EXPECT(Floor && Wall && Cache.GetPairCount() == 1);
		const auto Ids = Cache.GetPatchIds(A, B);
		EXPECT(Ids.size() == Patches.size());
		Cache.BeginFrame(2);
		for (uint32_t Id : Ids)
		{
			ContactManifold Refreshed; Refreshed.PatchId = Id;
			EXPECT(Cache.Refresh(A, B, &Refreshed));
			EXPECT(Refreshed.NumContactPointCount > 0);
		}
		if (!Ids.empty())
		{
			Cache.RetainPatches(A, B, { Ids[0] });
			EXPECT(Cache.GetPatchIds(A, B).size() == 1);
		}
	}

	void TestBoundaryContactsAndTransforms()
	{
		auto Floor = MakeSurface(false);
		MeshContactGenerator Generator;
		std::vector<ContactManifold> Patches;
		// An exposed boundary must keep its radial normal instead of snapping to up.
		std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Vector3(3.3f, 0.3f, 0), 0.5f));
		Generator.Generate(0, 1, State(Floor.get(), 0), State(Sphere.get(), 1), Patches);
		EXPECT(!Patches.empty());
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
		{
			const Contact& C = Patch.ContactPoints[Index];
			EXPECT(C.Normal.x > 0.7f && C.Normal.y > 0.7f);
			EXPECT_NEAR(C.PositionWorldA.x, 3.0f, GeometryTolerance);
			EXPECT_NEAR(C.PenetrationDepth, 0.5f - std::sqrt(0.18f), GeometryTolerance);
		}
		// A deeply crossing vertical capsule requires more than its radius as depth.
		std::unique_ptr<Geometry> Capsule(GeometryFactory::CreateCapsule(Vector3(0, -0.6f, 0), Vector3(0, 1, 0), 0.25f));
		Generator.Generate(0, 1, State(Floor.get(), 0), State(Capsule.get(), 1), Patches);
		float Deepest = 0.0f;
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index) Deepest = std::max(Deepest, Patch.ContactPoints[Index].PenetrationDepth);
		EXPECT_NEAR(Deepest, 0.85f, GeometryTolerance);

		Vector3 Vertices[8]; Box3::GetVertices(Vector3(-0.5f), Vector3(0.5f), Vertices);
		std::unique_ptr<Geometry> Hull(GeometryFactory::CreateConvexMesh());
		EXPECT(Hull->GetShapeObj<ConvexMesh>()->BuildFromPoints(Vertices, 8));
		Hull->UpdateVolumeProperties(); Hull->SetPosition(Vector3(0, 0.45f, 0));
		std::unique_ptr<Geometry> Cylinder(GeometryFactory::CreateCylinder(Vector3(0, -0.05f, 0), Vector3(0, 0.95f, 0), 0.5f));
		for (Geometry* Shape : { Hull.get(), Cylinder.get() })
		{
			Generator.Generate(0, 1, State(Floor.get(), 0), State(Shape, 1), Patches);
			EXPECT(!Patches.empty());
			for (const ContactManifold& Patch : Patches)
			for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
			{
				const Contact& C = Patch.ContactPoints[Index];
				EXPECT(C.Normal.y > 0.999f);
				EXPECT_NEAR(C.PositionWorldA.y, 0.0f, GeometryTolerance);
				EXPECT_NEAR(C.PositionWorldB.y, -0.05f, GeometryTolerance);
			}
		}
		// Compare boundary coverage to the independent primitive overlap query.
		for (int Sample = 0; Sample < 80; ++Sample)
		{
			const float Angle = 0.17f + (Sample % 8) * 0.19f;
			const Vector3 Center(2.8f + (Sample % 5) * 0.14f, -0.25f + (Sample % 7) * 0.11f, 2.7f + (Sample % 4) * 0.13f);
			std::unique_ptr<Geometry> TestBox(GeometryFactory::CreateOBB(Center, Vector3(0.5f), Quaternion().FromRotateZ(Angle)));
			const bool Overlapping = TestBox->Intersect(*TestBox->GetTransform(), Floor.get(), *Floor->GetTransform());
			Generator.Generate(0, 1, State(Floor.get(), 0), State(TestBox.get(), 1), Patches);
			if (Overlapping != !Patches.empty()) printf("  mesh boundary coverage sample=%d overlap=%d patches=%zu\n", Sample, (int)Overlapping, Patches.size());
			EXPECT(Overlapping == !Patches.empty());
			for (const ContactManifold& Patch : Patches)
			for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
			{
				const Contact& C = Patch.ContactPoints[Index];
				const Vector3 Local = TestBox->GetTransform()->WorldToLocal(C.PositionWorldB).Abs();
				EXPECT(Local.x <= 0.5f + GeometryTolerance && Local.y <= 0.5f + GeometryTolerance && Local.z <= 0.5f + GeometryTolerance);
				EXPECT(std::max(Local.x, std::max(Local.y, Local.z)) >= 0.5f - GeometryTolerance);
				EXPECT_NEAR(C.PositionWorldA.y, 0.0f, GeometryTolerance);
				EXPECT(std::abs(C.PositionWorldA.x) <= 3.0f + GeometryTolerance && std::abs(C.PositionWorldA.z) <= 3.0f + GeometryTolerance);
			}
		}
		const Transform Pose(Vector3(10, 4, -2), Quaternion().FromRotateZ(0.4f));
		Floor->SetTransform(Pose.pos, Pose.quat);
		std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Pose.LocalToWorld(Vector3(0, 0.45f, 0)), Vector3(0.5f), Pose.quat));
		Generator.Generate(0, 1, State(Floor.get(), 0), State(Box.get(), 1), Patches);
		EXPECT(Patches.size() == 1 && Patches[0].NumContactPointCount == 4);
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
		{
			const Contact& C = Patch.ContactPoints[Index];
			EXPECT(C.Normal.Dot(Pose.LocalToWorldDirection(Vector3::UnitY())) > 0.999f);
			EXPECT_NEAR(Pose.WorldToLocal(C.PositionWorldA).y, 0.0f, GeometryTolerance);
			EXPECT_NEAR(C.PenetrationDepth, 0.05f, GeometryTolerance);
		}
		Generator.Generate(1, 0, State(Box.get(), 1), State(Floor.get(), 0), Patches);
		EXPECT(Patches.size() == 1);
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
			EXPECT(Patch.ContactPoints[Index].Normal.Dot(Pose.LocalToWorldDirection(-Vector3::UnitY())) > 0.999f);
	}

	void TestTriangleCandidates()
	{
		std::unique_ptr<Geometry> Shape(GeometryFactory::CreateTriangleMesh());
		TriangleMesh& Mesh = *Shape->GetShapeObj<TriangleMesh>();
		for (int X = 0; X < 12; ++X)
		for (int Z = 0; Z < 12; ++Z)
			AddQuad(Mesh, Vector3((float)X, 0, (float)Z), Vector3(X + 1.0f, 0, (float)Z),
				Vector3(X + 1.0f, 0, Z + 1.0f), Vector3((float)X, 0, Z + 1.0f));
		Mesh.Compact(); Mesh.BuildBVH(); Shape->UpdateVolumeProperties();
		for (int Step = -1; Step < 14; ++Step)
		{
			const Box3 Bounds(Vector3(Step + 0.3f, -0.1f, 3.4f), Vector3(Step + 1.7f, 0.1f, 7.2f));
			std::vector<uint32_t> Candidates, Expected;
			Mesh.QueryTriangles(Bounds, Candidates);
			for (uint32_t Index = 0; Index < Mesh.NumTriangles; ++Index)
				if (Bounds.Intersect(Box3(Mesh.GetVertex(Index, 0), Mesh.GetVertex(Index, 1), Mesh.GetVertex(Index, 2)))) Expected.push_back(Index);
			EXPECT(Candidates == Expected);
		}
	}

	void TestClosedComponents()
	{
		std::unique_ptr<Geometry> Shape(GeometryFactory::CreateTriangleMesh());
		TriangleMesh& Mesh = *Shape->GetShapeObj<TriangleMesh>();
		Mesh.AddAABB(Vector3(-3, -1, -3), Vector3(0, 0, 3));
		Mesh.AddAABB(Vector3(0, -1, -3), Vector3(3, 1, 3));
		Mesh.Compact(); Mesh.BuildBVH(); Shape->UpdateVolumeProperties();
		MeshContactGenerator Generator;
		std::vector<ContactManifold> Patches;
		std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Vector3(-0.1f, 0.2f, 0), 0.3f));
		Generator.Generate(0, 1, State(Shape.get(), 0), State(Sphere.get(), 1), Patches);
		bool Floor = false, Wall = false;
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
		{
			const Vector3 Normal = Patch.ContactPoints[Index].Normal;
			Floor = Floor || Normal.y > 0.99f;
			Wall = Wall || Normal.x < -0.99f;
			EXPECT(Normal.x <= GeometryTolerance && Normal.y >= -GeometryTolerance);
		}
		EXPECT(Floor && Wall);
		std::unique_ptr<Geometry> Capsule(GeometryFactory::CreateCapsule(Vector3(-1, -0.3f, 0), Vector3(-1, 0.1f, 0), 0.2f));
		Generator.Generate(0, 1, State(Shape.get(), 0), State(Capsule.get(), 1), Patches);
		float Deepest = 0.0f;
		for (const ContactManifold& Patch : Patches)
		for (int Index = 0; Index < Patch.NumContactPointCount; ++Index)
		{
			const Contact& C = Patch.ContactPoints[Index];
			EXPECT(C.Normal.y > 0.99f);
			Deepest = std::max(Deepest, C.PenetrationDepth);
		}
		EXPECT_NEAR(Deepest, 0.5f, GeometryTolerance);
		// Fully embedded shapes have no triangle inside their own bounds.
		std::unique_ptr<Geometry> Embedded(GeometryFactory::CreateSphere(Vector3(-1, -0.5f, 0), 0.1f));
		Generator.Generate(0, 1, State(Shape.get(), 0), State(Embedded.get(), 1), Patches);
		EXPECT(Patches.size() == 1);
		if (!Patches.empty()) EXPECT_NEAR(Patches[0].ContactPoints[0].PenetrationDepth, 0.6f, GeometryTolerance);
		MeshContactSettings Sheets; Sheets.ClosedComponentsAreSolids = false;
		MeshContactGenerator SheetGenerator(Sheets);
		SheetGenerator.Generate(0, 1, State(Shape.get(), 0), State(Embedded.get(), 1), Patches);
		EXPECT(Patches.empty());
	}

	void TestTerrainHolesAndSupport()
	{
		std::unique_ptr<Geometry> Terrain(GeometryFactory::CreateHeightField(Box3(Vector3(-2, -1, -2), Vector3(2, 1, 2)), 2, 2));
		HeightField3* Field = Terrain->GetShapeObj<HeightField3>();
		Field->AllocMemory();
		for (int X = 0; X < 2; ++X) for (int Z = 0; Z < 2; ++Z) Field->SetHeight(X, Z, 0.0f);
		Terrain->UpdateVolumeProperties();
		std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Vector3(0, 0.45f, 0), 0.5f));
		MeshContactGenerator Generator;
		std::vector<ContactManifold> Patches;
		Generator.Generate(0, 1, State(Terrain.get(), 0), State(Sphere.get(), 1), Patches);
		EXPECT(!Patches.empty());
		ContactManifoldManager Cache;
		for (ContactManifold& Patch : Patches) Cache.Store(State(Terrain.get(), 0), State(Sphere.get(), 1), Patch, 1.0f / 60.0f);
		Field->SetTriangleHole(0, 0, 0, true); Field->SetTriangleHole(0, 0, 1, true);
		for (ContactManifold& Patch : Patches) EXPECT(!Cache.Refresh(State(Terrain.get(), 0), State(Sphere.get(), 1), &Patch));
		Generator.Generate(0, 1, State(Terrain.get(), 0), State(Sphere.get(), 1), Patches);
		EXPECT(Patches.empty());

		for (ResolutionPhaseSolver Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
			ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (NarrowPhaseSolver Narrow : { NarrowPhaseSolver::GJKEPA, NarrowPhaseSolver::PCM })
		{
			PhysicsWorldParam Settings; Settings.gravityAcc = Vector3(0, -9.8f, 0); Settings.resolutionSolver = Solver; Settings.narrowphase = Narrow;
			PhysicsWorld World(Settings);
			Geometry* Ground = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4), Vector3(4, 1, 4)), 9, 9);
			HeightField3* Grid = Ground->GetShapeObj<HeightField3>();
			Grid->AllocMemory();
			for (int X = 0; X < 9; ++X) for (int Z = 0; Z < 9; ++Z) Grid->SetHeight(X, Z, 0.0f);
			Ground->UpdateVolumeProperties();
			World.CreateRigidBody(Ground, RigidBodyParam());
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.sleepThreshold = 0.0f;
			RigidBody* Body = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, 2, 0), Vector3(0.5f)), Dynamic);
			for (int Frame = 0; Frame < 180; ++Frame) World.Simulate();
			printf("  triangle terrain solver=%d narrow=%d y=%.5f speed=%.5f\n", (int)Solver, (int)Narrow, Body->X.y, Body->V.Length());
			EXPECT(Body->X.y > 0.4f && Body->X.y < 0.6f);
			EXPECT(Body->V.Length() < 0.2f);
			World.SetTriangleContactsEnabled(false); EXPECT(!World.GetTriangleContactsEnabled());
			World.SetTriangleContactsEnabled(true); EXPECT(World.GetTriangleContactsEnabled());
		}
	}
}

void TestMeshContacts()
{
	TestFacePatchesAndSeams();
	TestMultiplePatchCache();
	TestBoundaryContactsAndTransforms();
	TestTriangleCandidates();
	TestClosedComponents();
	TestTerrainHolesAndSupport();
}

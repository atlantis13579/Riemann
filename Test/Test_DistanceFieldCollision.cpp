#include "Test.h"
#include <memory>
#include <chrono>
#include "../Src/Collision/DistanceFieldCollision.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/DistanceFieldShape.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include "../Src/RigidBodyDynamics/MeshContactGenerator.h"
#include "../Src/RigidBodyDynamics/ContinuousCollisionDetection.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include <algorithm>
#include <future>
#include <limits>
#include <random>
#include "../Src/Geometry/RuntimeDistanceField.h"

namespace
{
	using namespace Riemann;
	template<class Function>
	DistanceFieldShape Cook(float Radius, int Cells, Function Value)
	{
		SignedDistanceField3 Grid;
		EXPECT(Grid.Reset(Box3(Vector3(-Radius), Vector3(Radius)), Cells, Cells, Cells, 2 * Radius / Cells));
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Grid.At(X, Y, Z) = Value(Grid.GridPointToWorld(X, Y, Z));
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Grid));
		DistanceFieldShape Shape; EXPECT(Shape.Build(Runtime)); return Shape;
	}
	float BoxDistance(const Vector3& P, const Vector3& Extent)
	{
		const Vector3 Delta(std::abs(P.x) - Extent.x, std::abs(P.y) - Extent.y, std::abs(P.z) - Extent.z);
		return Delta.Max(Vector3::Zero()).Length() + std::min(0.0f, std::max(Delta.x, std::max(Delta.y, Delta.z)));
	}
	GeometryWorldState State(Geometry* G)
	{
		GeometryWorldState S; S.Geom = G; S.ShapeToWorld = *G->GetTransform(); S.WorldBounds = G->GetBounds(); return S;
	}
	std::vector<ContactManifold> Manifolds(Geometry* A, Geometry* B, bool Persistent = false)
	{
		std::unique_ptr<NarrowPhase> Narrow(Persistent ? NarrowPhase::Create_PCM() : NarrowPhase::Create_GJKEPA());
		GeometryWorldState States[] = { State(A), State(B) };
		const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
		std::vector<ContactManifold*> Output;
		Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Output);
		std::vector<ContactManifold> Result; for (auto* M : Output) Result.push_back(*M); return Result;
	}
	Geometry* Primitive(int Kind, Vector3 Center, float Scale = 1.0f)
	{
		if (Kind == 0) return GeometryFactory::CreateSphere(Center, 0.5f * Scale);
		if (Kind == 1) return GeometryFactory::CreateOBB(Center, Vector3(0.5f * Scale));
		if (Kind == 2) return GeometryFactory::CreateCapsule(Center - Vector3::UnitX() * Scale, Center + Vector3::UnitX() * Scale, 0.5f * Scale);
		if (Kind == 3) return GeometryFactory::CreateCylinder(Center - Vector3::UnitY() * Scale / 2, Center + Vector3::UnitY() * Scale / 2, 0.5f * Scale);
		Geometry* Shape = GeometryFactory::CreateConvexMesh(); Vector3 Vertices[8];
		Box3::GetVertices(Vector3(-0.5f * Scale), Vector3(0.5f * Scale), Vertices);
		EXPECT(Shape->GetShapeObj<ConvexMesh>()->BuildFromPoints(Vertices, 8, Vector3::Zero(), 1.0e-5f * Scale)); Shape->UpdateVolumeProperties(); Shape->SetPosition(Center); return Shape;
	}
	std::unique_ptr<Geometry> MeshGround(float HalfExtent = 3.0f)
	{
		std::unique_ptr<Geometry> Ground(GeometryFactory::CreateTriangleMesh());
		auto* Mesh = Ground->GetShapeObj<TriangleMesh>();
		for (const Vector3& P : { Vector3(-HalfExtent, 0, -HalfExtent), Vector3(HalfExtent, 0, -HalfExtent),
			Vector3(HalfExtent, 0, HalfExtent), Vector3(-HalfExtent, 0, HalfExtent) }) Mesh->AddVertex(P);
		Mesh->AddTriangle(0, 1, 2); Mesh->AddTriangle(0, 2, 3);
		Mesh->Compact(); Mesh->BuildBVH(); Ground->UpdateVolumeProperties(); return Ground;
	}
	std::unique_ptr<Geometry> TerrainGround(float Scale = 1.0f)
	{
		std::unique_ptr<Geometry> Ground(GeometryFactory::CreateHeightField(Box3(Vector3(-3, -1, -3) * Scale, Vector3(3, 1, 3) * Scale), 2, 2));
		auto* Terrain = Ground->GetShapeObj<HeightField3>(); Terrain->HeightScale = 0.001f * Scale; Terrain->AllocMemory();
		for (int Z = 0; Z < 2; ++Z) for (int X = 0; X < 2; ++X) EXPECT(Terrain->SetHeight(X, Z, 0));
		Ground->UpdateVolumeProperties(); return Ground;
	}
	void TestMassAndOwnership()
	{
		const auto Sphere = Cook(1.5f, 32, [](const Vector3& P) { return P.Length() - 1.0f; });
		MassParameters Mass; EXPECT(Sphere.CalculateVolumeProperties(&Mass, 2.0f));
		const float Volume = 4.0f * 3.14159265358979323846f / 3;
		EXPECT_NEAR(Mass.Volume, Volume, 0.03f); EXPECT_NEAR(Mass.Mass, 2 * Volume, 0.06f);
		EXPECT(Mass.CenterOfMass.Length() < 1.0e-5f);
		EXPECT_NEAR(Mass.InertiaMat[0][0], 0.4f * Mass.Mass, 0.04f);
		EXPECT_NEAR(Mass.BoundingVolume.Max.x, 1.0f, 0.001f);
		EXPECT_NEAR(Mass.BoundingVolume.Min.x, -1.0f, 0.001f);
		const auto& Original = *Sphere.GetField().GetField();
		const Vector3 Shift(1000.125f, 3000.25f, -1800.75f);
		SignedDistanceField3 Shifted;
		EXPECT(Shifted.Reset(Box3(Original.GetBounds().Min + Shift, Original.GetBounds().Max + Shift), 32, 32, 32, Original.GetCellSize()));
		for (int Z = 0; Z <= 32; ++Z) for (int Y = 0; Y <= 32; ++Y) for (int X = 0; X <= 32; ++X) Shifted.At(X, Y, Z) = Original.At(X, Y, Z);
		RuntimeDistanceField ShiftedRuntime; EXPECT(ShiftedRuntime.Build(Shifted));
		DistanceFieldShape ShiftedShape; EXPECT(ShiftedShape.Build(ShiftedRuntime));
		MassParameters ShiftedMass; EXPECT(ShiftedShape.CalculateVolumeProperties(&ShiftedMass, 2.0f));
		EXPECT((ShiftedMass.CenterOfMass - Shift).Length() < 0.001f);
		EXPECT_NEAR(ShiftedMass.InertiaMat[0][0], Mass.InertiaMat[0][0], 0.001f);

		std::unique_ptr<Geometry> GeometryObject(GeometryFactory::CreateDistanceField(Sphere, Vector3(2, 3, 4)));
		EXPECT(GeometryObject->GetShapeType() == PrimitiveType::DISTANCE_FIELD);
		EXPECT(GeometryObject->GetShapeObj<DistanceFieldShape>()->GetField().GetField() == Sphere.GetField().GetField());
		EXPECT_NEAR(GeometryObject->GetMassParameters()->Mass, Volume, 0.03f);
		float Time; EXPECT(Sphere.IntersectRay(Vector3(0, 2, 0), -Vector3::UnitY(), &Time)); EXPECT_NEAR(Time, 1.0f, 0.003f);
		SignedDistanceField3 Open; EXPECT(Open.Reset(Box3(Vector3(-1), Vector3(1)), 2, 2, 2, 1.0f));
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Open));
		DistanceFieldShape Copy = Sphere; EXPECT(!Copy.Build(Runtime)); EXPECT(Copy.IsValid());
	}
	void TestContacts()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			const auto Floor = Cook(3 * Scale, 32, [=](const Vector3& P) { return BoxDistance(P, Vector3(2, 1, 2) * Scale); });
			std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Floor, Vector3(0, -Scale, 0)));
			for (int Kind = 0; Kind < 5; ++Kind)
			{
				std::unique_ptr<Geometry> Other(Primitive(Kind, Vector3(0, 0.4f * Scale, 0), Scale));
				for (bool Reverse : { false, true })
				{
					Geometry* A = Reverse ? Other.get() : Field.get(); Geometry* B = Reverse ? Field.get() : Other.get();
					EXPECT(A->Intersect(B)); auto Contacts = Manifolds(A, B);
					printf("SDF shape=%d scale=%g reverse=%d patches=%zu\n", Kind, Scale, Reverse, Contacts.size());
					EXPECT(!Contacts.empty());
					float Deepest = 0.0f;
					for (const auto& M : Contacts) for (int I = 0; I < M.NumContactPointCount; ++I)
					{
						const auto& C = M.ContactPoints[I];
						EXPECT(C.Normal.y * (Reverse ? -1 : 1) > 0.9f);
						Deepest = std::max(Deepest, C.PenetrationDepth / Scale);
						EXPECT(C.PenetrationDepth / Scale >= -0.001f && C.PenetrationDepth / Scale <= 0.106f);
						EXPECT_NEAR((C.PositionWorldA - C.PositionWorldB).Dot(C.Normal) / Scale, C.PenetrationDepth / Scale, 0.001f);
					}
					EXPECT_NEAR(Deepest, 0.1f, 0.006f);
				}
				Other->SetPosition(Vector3(0, Scale, 0)); EXPECT(!Field->Intersect(Other.get())); EXPECT(Manifolds(Field.get(), Other.get()).empty());
			}
		}
	}
	void TestCavitiesAndTransforms()
	{
		const auto Torus = Cook(1.5f, 40, [](const Vector3& P)
		{
			const float Radial = std::sqrt(P.x * P.x + P.z * P.z) - 0.9f; return std::sqrt(Radial * Radial + P.y * P.y) - 0.25f;
		});
		std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Torus));
		std::unique_ptr<Geometry> Ball(GeometryFactory::CreateSphere(Vector3::Zero(), 0.2f));
		EXPECT(!Field->Intersect(Ball.get()));
		Vector3 SweepPoint, SweepNormal; float SweepTime;
		Ball->SetPosition(Vector3(0, 2, 0));
		EXPECT(!Ball->Sweep(-Vector3::UnitY(), Field.get(), &SweepPoint, &SweepNormal, &SweepTime));
		Ball->SetPosition(Vector3(0.9f, 2, 0));
		EXPECT(Ball->Sweep(-Vector3::UnitY(), Field.get(), &SweepPoint, &SweepNormal, &SweepTime));
		EXPECT_NEAR(SweepTime, 1.55f, 0.006f); EXPECT(SweepNormal.y > 0.99f);
		Ball->SetPosition(Vector3::Zero());

		std::unique_ptr<Geometry> HoleBox(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.1f)));
		EXPECT(!Field->Intersect(HoleBox.get()));
		std::unique_ptr<Geometry> CrossingSlab(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1.2f, 0.1f, 1.2f)));
		EXPECT(Field->Intersect(CrossingSlab.get())); EXPECT(!Manifolds(Field.get(), CrossingSlab.get()).empty());

		Ball->SetPosition(Vector3(0.9f, 0.35f, 0)); EXPECT(Field->Intersect(Ball.get()));
		auto Original = Manifolds(Field.get(), Ball.get()); EXPECT(!Original.empty());
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitZ(), 0.6f);
		const Transform Common(Vector3(3, 2, -4), Rotation);
		Field->SetTransform(Common.pos, Common.quat); Ball->SetPosition(Common.LocalToWorld(Vector3(0.9f, 0.35f, 0)));
		auto Moved = Manifolds(Field.get(), Ball.get()); EXPECT(Moved.size() == Original.size());
		if (!Moved.empty() && !Original.empty())
		{
			EXPECT_NEAR(Moved[0].ContactPoints[0].PenetrationDepth, Original[0].ContactPoints[0].PenetrationDepth, 0.002f);
			EXPECT((Moved[0].ContactPoints[0].Normal - Rotation * Original[0].ContactPoints[0].Normal).Length() < 0.02f);
		}
		Ball->SetPosition(Common.LocalToWorld(Vector3(0.9f, 0, 0)));
		auto Embedded = Manifolds(Field.get(), Ball.get()); EXPECT(!Embedded.empty());
		if (!Embedded.empty()) EXPECT(Embedded[0].ContactPoints[0].PenetrationDepth > 0.4f);
	}
	void TestContainment()
	{
		const auto Sphere = Cook(1.5f, 32, [](const Vector3& P) { return P.Length() - 1.0f; });
		std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere));
		const auto Scaled = Cook(1.5f, 32, [](const Vector3& P) { return (P.Length() - 1.0f) * 1.0e-9f; });
		std::unique_ptr<Geometry> ScaledField(GeometryFactory::CreateDistanceField(Scaled));
		std::unique_ptr<Geometry> SurfaceBox(GeometryFactory::CreateOBB(Vector3(0, 1.4f, 0), Vector3(0.5f)));
		EXPECT(ScaledField->Intersect(SurfaceBox.get()));
		auto ScaledContacts = Manifolds(ScaledField.get(), SurfaceBox.get()); EXPECT(!ScaledContacts.empty());

		const auto Inner = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
		std::unique_ptr<Geometry> InnerField(GeometryFactory::CreateDistanceField(Inner));
		for (bool Reverse : { false, true })
		{
			auto Pair = Manifolds(Reverse ? InnerField.get() : Field.get(), Reverse ? Field.get() : InnerField.get());
			EXPECT(Pair.size() == 1);
			if (!Pair.empty()) EXPECT(Pair[0].ContactPoints[0].PenetrationDepth > 1.45f);
		}
		for (float HalfExtent : { 0.1f, 2.0f })
		{
			std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(HalfExtent)));
			EXPECT(Field->Intersect(Box.get())); auto Contacts = Manifolds(Field.get(), Box.get());
			EXPECT(Contacts.size() == 1);
			if (!Contacts.empty())
			{
				const auto& C = Contacts[0].ContactPoints[0];
				EXPECT(C.PenetrationDepth > (HalfExtent < 1.0f ? 1.0f : 2.9f));
				EXPECT_NEAR(C.Normal.Length(), 1.0f, 0.001f);
				DistanceFieldSample Sample; EXPECT(Sphere.GetField().Sample(C.PositionWorldA, Sample));
				EXPECT(std::abs(Sample.Value) < 0.0001f);
			}
		}
	}
	void TestQueriesAndCCD()
	{
		const auto Field = Cook(1.5f, 32, [](const Vector3& P) { return P.Length() - 1.0f; });
		for (auto Type : { SceneQueryType::StaticAABB, SceneQueryType::DynamicAABB })
		{
			PhysicsWorldParam Param; Param.sceneQueryType = Type;
			PhysicsWorld World(Param);
			Geometry* Shape = GeometryFactory::CreateDistanceField(Field, Vector3(4, 0, 0));
			World.CreateRigidBody(Shape, RigidBodyParam());
			auto* Query = World.GetGeometryQuery(); RayCastOption Ray; Ray.MaxDist = 10; RayCastResult RayHit;
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayHit));
			EXPECT_NEAR(RayHit.hitTimeMin, 3.0f, 0.003f); EXPECT(RayHit.hitNormal.x < -0.99f);
			SweepOption Sweep; Sweep.MaxDist = 10; SweepResult Hit;
			EXPECT(Query->SphereCastQuery(Vector3::Zero(), 0.5f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT_NEAR(Hit.hitTimeMin, 2.5f, 0.005f); EXPECT(Hit.hitNormal.x < -0.99f);
			Sweep.MaxDist = 2; EXPECT(!Query->SphereCastQuery(Vector3::Zero(), 0.5f, Vector3::UnitX(), Sweep, &Hit));
			IntersectOption Overlap; IntersectResult Overlaps;
			EXPECT(Query->IntersectQuerySphere(Vector3(3, 0, 0), 0.5f, Overlap, &Overlaps));
			auto Deny = std::unique_ptr<CollisionFilter>(CollisionFilter::CreateCallback([](const CollisionData&, const CollisionData&) { return false; }));
			Ray.Filter = Sweep.Filter = Overlap.Filter = Deny.get(); Sweep.MaxDist = 10;
			EXPECT(!Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayHit));
			EXPECT(!Query->SphereCastQuery(Vector3::Zero(), 0.5f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT(!Query->IntersectQuerySphere(Vector3(3, 0, 0), 0.5f, Overlap, &Overlaps));
			RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
			Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
			auto* Body = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f), Moving);
			World.Simulate(1.0f / 60.0f);
			EXPECT(World.GetLastCCDHitCount() > 0); EXPECT(Body->X.x <= 2.51f);
		}
		std::unique_ptr<Geometry> A(GeometryFactory::CreateDistanceField(Field));
		std::unique_ptr<Geometry> B(GeometryFactory::CreateSphere(Vector3(4, 0, 0), 0.5f));
		Vector3 P, N; float T;
		EXPECT(A->Sweep(Vector3::UnitX(), B.get(), &P, &N, &T)); EXPECT_NEAR(T, 2.5f, 0.005f); EXPECT(N.x < -0.99f);
	}
	void TestSimulation()
	{
		const auto Floor = Cook(3.0f, 32, [](const Vector3& P) { return BoxDistance(P, Vector3(2, 1, 2)); });
		const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (int Kind = 0; Kind <= 5; ++Kind)
		{
			PhysicsWorldParam Param; Param.gravityAcc = Vector3(0, -9.8f, 0); Param.resolutionSolver = Solver; Param.narrowphase = NarrowPhaseSolver::PCM;
			PhysicsWorld World(Param);
			World.CreateRigidBody(Kind == 5 ? GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY())
				: GeometryFactory::CreateDistanceField(Floor, Vector3(0, -1, 0)), RigidBodyParam());
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.sleepThreshold = 0.0f;
			Geometry* Shape = Kind == 5 ? GeometryFactory::CreateDistanceField(Sphere, Vector3(0, 2, 0)) : Primitive(Kind, Vector3(0, 2, 0));
			Shape->SetContactOffsets(0.05f, 0.0f);
			auto* Body = World.CreateRigidBody(Shape, Dynamic);
			for (int Step = 0; Step < 180; ++Step) World.Simulate(1.0f / 60.0f);
			printf("SDF simulation solver=%d shape=%d y=%g speed=%g\n", (int)Solver, Kind, Body->X.y, Body->V.Length());
			EXPECT_NEAR(Body->X.y, 0.5f, 0.035f); EXPECT(Body->V.Length() < 0.1f);
		}
	}
}
void TestDistanceFieldCollision()
{
	TestMassAndOwnership(); TestContacts(); TestCavitiesAndTransforms(); TestQueriesAndCCD(); TestContainment(); TestSimulation();
}


void TestDistanceFieldMesh()
{
	using namespace Riemann;
	for (float Scale : { 0.001f, 1.0f, 1000.0f })
	{
		const auto Sphere = Cook(0.75f * Scale, 24, [=](const Vector3& P) { return P.Length() - 0.5f * Scale; });
		std::unique_ptr<Geometry> Shape(GeometryFactory::CreateDistanceField(Sphere, Vector3(0, 0.45f * Scale, 0)));
		for (bool Terrain : { false, true })
		{
			auto Ground = Terrain ? TerrainGround(Scale) : MeshGround(3 * Scale);
			for (bool Reverse : { false, true })
			{
				Geometry* A = Reverse ? Shape.get() : Ground.get(); Geometry* B = Reverse ? Ground.get() : Shape.get();
				EXPECT(A->Intersect(B)); const auto Patches = Manifolds(A, B);
				printf("SDF mesh scale=%g terrain=%d reverse=%d patches=%zu\n", Scale, Terrain, Reverse, Patches.size());
				EXPECT(Patches.size() == 1);
				for (const auto& Patch : Patches) for (int I = 0; I < Patch.NumContactPointCount; ++I)
				{
					const auto& C = Patch.ContactPoints[I];
					EXPECT(C.Normal.y * (Reverse ? -1 : 1) > 0.999f);
					EXPECT_NEAR(C.PenetrationDepth / Scale, 0.05f, 0.002f);
					EXPECT_NEAR((C.PositionWorldA - C.PositionWorldB).Dot(C.Normal) / Scale, C.PenetrationDepth / Scale, 0.0001f);
					EXPECT(C.FeatureA != UINT32_MAX && C.FeatureB != UINT32_MAX);
				}
			}
		}
	}
	const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
	std::unique_ptr<Geometry> Shape(GeometryFactory::CreateDistanceField(Sphere));
	auto Ground = MeshGround();
	for (int Step = -4; Step <= 4; ++Step)
	{
		Shape->SetPosition(Vector3(Step * 0.2f, 0.45f, -Step * 0.2f));
		const auto Patches = Manifolds(Ground.get(), Shape.get()); EXPECT(Patches.size() == 1);
		for (const auto& Patch : Patches) for (int I = 0; I < Patch.NumContactPointCount; ++I) EXPECT(Patch.ContactPoints[I].Normal.y > 0.999f);
	}
	Shape->SetPosition(Vector3(0, -0.45f, 0));
	auto Below = Manifolds(Ground.get(), Shape.get()); EXPECT(Below.size() == 1);
	if (!Below.empty()) EXPECT(Below[0].ContactPoints[0].Normal.y < -0.999f);
	Shape->SetPosition(Vector3(3.3f, 0.3f, 0));
	auto Boundary = Manifolds(Ground.get(), Shape.get()); EXPECT(!Boundary.empty());
	float Deepest = 0;
	for (const auto& Patch : Boundary) for (int I = 0; I < Patch.NumContactPointCount; ++I)
	{
		const auto& C = Patch.ContactPoints[I];
		printf("SDF edge normal=(%g,%g,%g) depth=%g\n", C.Normal.x, C.Normal.y, C.Normal.z, C.PenetrationDepth);
		EXPECT(C.Normal.x > 0.5f && C.Normal.y > 0.5f);
		Deepest = std::max(Deepest, C.PenetrationDepth);
	}
	EXPECT_NEAR(Deepest, 0.5f - std::sqrt(0.18f), 0.006f);
	Shape->SetPosition(Vector3(0, 0.52f, 0)); EXPECT(!Ground->Intersect(Shape.get()));
	std::vector<DistanceFieldContact> Separated;
	auto MarginTerrain = TerrainGround();
	for (auto* Surface : { Ground.get(), MarginTerrain.get() })
	{
		EXPECT(DistanceFieldCollision::Generate(Surface, *Surface->GetTransform(), Shape.get(), *Shape->GetTransform(), 0.03f, Separated));
		for (const auto& Contact : Separated)
		{
			EXPECT_NEAR(Contact.Separation, 0.02f, 0.001f); EXPECT(Contact.Normal.y > 0.999f);
		}
	}

	Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitX(), 0.7f);
	const Transform Common(Vector3(-4, 3, 8), Rotation);
	const Transform FieldPose(Common.LocalToWorld(Vector3(0, 0.45f, 0)), Rotation);
	EXPECT(Ground->Intersect(Common, Shape.get(), FieldPose));
	Ground->SetTransform(Common.pos, Common.quat); Shape->SetTransform(FieldPose.pos, FieldPose.quat);
	auto Moved = Manifolds(Ground.get(), Shape.get()); EXPECT(Moved.size() == 1);
	if (!Moved.empty())
	{
		EXPECT((Moved[0].ContactPoints[0].Normal - Rotation * Vector3::UnitY()).Length() < 0.002f);
		EXPECT_NEAR(Moved[0].ContactPoints[0].PenetrationDepth, 0.05f, 0.002f);
	}
	Shape->SetTransform(Vector3::Zero(), Quaternion::One());
	auto Tiny = MeshGround(0.01f); EXPECT(Tiny->Intersect(Shape.get()));
	auto TinyContacts = Manifolds(Tiny.get(), Shape.get()); EXPECT(!TinyContacts.empty());
	if (!TinyContacts.empty()) EXPECT(TinyContacts[0].ContactPoints[0].PenetrationDepth > 0.49f);
	const auto Torus = Cook(1.5f, 40, [](const Vector3& P)
	{
		const float Radial = std::sqrt(P.x * P.x + P.z * P.z) - 0.9f; return std::sqrt(Radial * Radial + P.y * P.y) - 0.25f;
	});
	std::unique_ptr<Geometry> Ring(GeometryFactory::CreateDistanceField(Torus));
	EXPECT(!Tiny->Intersect(Ring.get())); Tiny->SetPosition(Vector3(0.9f, 0, 0)); EXPECT(Tiny->Intersect(Ring.get()));
	std::unique_ptr<Geometry> CavityMesh(GeometryFactory::CreateTriangleMesh());
	auto* SmallMesh = CavityMesh->GetShapeObj<TriangleMesh>(); SmallMesh->AddAABB(Vector3(-0.1f), Vector3(0.1f));
	SmallMesh->Compact(); SmallMesh->BuildBVH(); CavityMesh->UpdateVolumeProperties();
	EXPECT(!CavityMesh->Intersect(Ring.get())); EXPECT(Manifolds(CavityMesh.get(), Ring.get()).empty());
	auto Corner = MeshGround(); auto* CornerMesh = Corner->GetShapeObj<TriangleMesh>();
	const uint32_t Base = CornerMesh->NumVertices;
	for (const Vector3& P : { Vector3(0, 0, -3), Vector3(0, 3, -3), Vector3(0, 3, 3), Vector3(0, 0, 3) }) CornerMesh->AddVertex(P);
	CornerMesh->AddTriangle(Base, Base + 1, Base + 2); CornerMesh->AddTriangle(Base, Base + 2, Base + 3);
	CornerMesh->Compact(); CornerMesh->BuildBVH(); Corner->UpdateVolumeProperties(); Shape->SetPosition(Vector3(0.3f, 0.3f, 0));
	auto CornerContacts = Manifolds(Corner.get(), Shape.get()); EXPECT(CornerContacts.size() == 2);
	for (const auto& Patch : CornerContacts) for (int I = 0; I < Patch.NumContactPointCount; ++I)
		EXPECT(Patch.ContactPoints[I].Normal.x > 0.999f || Patch.ContactPoints[I].Normal.y > 0.999f);
	Shape->SetPosition(Vector3::Zero());
	std::unique_ptr<Geometry> Closed(GeometryFactory::CreateTriangleMesh());
	auto* ClosedMesh = Closed->GetShapeObj<TriangleMesh>(); ClosedMesh->AddAABB(Vector3(-2), Vector3(2));
	ClosedMesh->Compact(); ClosedMesh->BuildBVH(); Closed->UpdateVolumeProperties();
	auto Embedded = Manifolds(Closed.get(), Shape.get()); EXPECT(Embedded.size() == 1); EXPECT(Closed->Intersect(Shape.get()));
	if (!Embedded.empty()) EXPECT_NEAR(Embedded[0].ContactPoints[0].PenetrationDepth, 2.5f, 0.01f);
	auto Terrain = TerrainGround(); Shape->SetPosition(Vector3(0, 0.45f, 0));
	EXPECT(Terrain->Intersect(Shape.get()));
	MeshContactGenerator CachedTerrain;
	std::vector<ContactManifold> CachedContacts;
	auto Regenerate = [&]() { CachedTerrain.Generate(0, 1, State(Terrain.get()), State(Shape.get()), CachedContacts); };
	Regenerate(); EXPECT(CachedContacts.size() == 1);
	auto* Grid = Terrain->GetShapeObj<HeightField3>();
	EXPECT(Grid->SetTriangleHole(0, 0, 0, true)); EXPECT(Grid->SetTriangleHole(0, 0, 1, true));
	EXPECT(!Terrain->Intersect(Shape.get())); EXPECT(Manifolds(Terrain.get(), Shape.get(), true).empty());
	Regenerate(); EXPECT(CachedContacts.empty());
	EXPECT(Grid->SetTriangleHole(0, 0, 0, false)); EXPECT(Grid->SetTriangleHole(0, 0, 1, false));
	for (int Z = 0; Z < 2; ++Z) for (int X = 0; X < 2; ++X) EXPECT(Grid->SetHeight(X, Z, 0.2f));
	Terrain->UpdateVolumeProperties(); auto Raised = Manifolds(Terrain.get(), Shape.get(), true); EXPECT(Raised.size() == 1);
	if (!Raised.empty()) { printf("SDF raised height=%g depth=%g\n", Grid->GetHeight(0), Raised[0].ContactPoints[0].PenetrationDepth); EXPECT_NEAR(Raised[0].ContactPoints[0].PenetrationDepth, 0.25f, 0.003f); }
	Regenerate(); EXPECT(CachedContacts.size() == 1);
	if (!CachedContacts.empty()) EXPECT_NEAR(CachedContacts[0].ContactPoints[0].PenetrationDepth, 0.25f, 0.003f);
}

void TestDistanceFieldMeshSimulation()
{
	using namespace Riemann;
	const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
	const auto Box = Cook(0.75f, 24, [](const Vector3& P) { return BoxDistance(P, Vector3(0.5f)); });
	for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
	for (bool Terrain : { false, true }) for (bool Round : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3(0, -9.8f, 0); Param.resolutionSolver = Solver; Param.narrowphase = NarrowPhaseSolver::PCM;
		PhysicsWorld World(Param);
		auto Ground = Terrain ? TerrainGround() : MeshGround();
		World.CreateRigidBody(Ground.release(), RigidBodyParam());
		RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.sleepThreshold = 0;
		auto* CollisionShape = GeometryFactory::CreateDistanceField(Round ? Sphere : Box, Vector3(0, 2, 0));
		// Explicit margins select scale-aware slop instead of the legacy SI allowance.
		CollisionShape->SetContactOffsets(0.05f, 0.0f);
		auto* Body = World.CreateRigidBody(CollisionShape, Dynamic);
		for (int Step = 0; Step < 180; ++Step) World.Simulate(1.0f / 60.0f);
		printf("SDF mesh simulation solver=%d terrain=%d sphere=%d y=%g speed=%g\n", (int)Solver, Terrain, Round, Body->X.y, Body->V.Length());
		EXPECT_NEAR(Body->X.y, 0.5f, 0.005f); EXPECT(Body->V.Length() < 0.1f);
	}
}



void TestDistanceFieldSweeps()
{
	using namespace Riemann;
	for (float Scale : { 0.001f, 1.0f, 1000.0f })
	{
		const auto Sphere = Cook(1.5f * Scale, 24, [=](const Vector3& P) { return P.Length() - Scale; });
		for (int Kind = 0; Kind < 5; ++Kind) for (bool Reverse : { false, true })
		{
			std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere, Reverse ? Vector3::Zero() : Vector3(4 * Scale, 0, 0)));
			std::unique_ptr<Geometry> Other(Primitive(Kind, Reverse ? Vector3(4 * Scale, 0, 0) : Vector3::Zero(), Scale));
			Geometry* A = Reverse ? Field.get() : Other.get(); Geometry* B = Reverse ? Other.get() : Field.get();
			const Vector3 Direction = A->GetTransform()->WorldToLocalDirection(Vector3::UnitX() * Scale);
			DistanceFieldSweepResult Hit;
			EXPECT(DistanceFieldCollision::SweepBounded(A, *A->GetTransform(), B, *B->GetTransform(), Direction, 5.0f, Hit));
			const float Expected = Kind == 2 ? 1.5f : 2.5f;
			printf("SDF sweep scale=%g kind=%d reverse=%d t=%g conservative=%d nodes=%llu\n", Scale, Kind, Reverse, Hit.Time, Hit.Conservative, (unsigned long long)Hit.Stats.NodeVisits);
			EXPECT(!Hit.Conservative); EXPECT_NEAR(Hit.Time, Expected, 0.006f); EXPECT(A->GetTransform()->LocalToWorldDirection(Hit.Normal).x < -0.99f);
			EXPECT(!DistanceFieldCollision::SweepBounded(A, *A->GetTransform(), B, *B->GetTransform(), Direction, Expected - 0.05f, Hit));
			Vector3 P, N; float T;
			EXPECT(A->Sweep(Direction, B, &P, &N, &T)); EXPECT_NEAR(T, Expected, 0.006f);
		}
	}
	const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
	std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere));
	std::unique_ptr<Geometry> OtherField(GeometryFactory::CreateDistanceField(Sphere, Vector3(3, 0, 0)));
	DistanceFieldSweepResult Hit;
	EXPECT(DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), OtherField.get(), *OtherField->GetTransform(), Vector3::UnitX(), 4.0f, Hit));
	printf("SDF pair sweep t=%g conservative=%d nodes=%llu\n", Hit.Time, Hit.Conservative, (unsigned long long)Hit.Stats.NodeVisits);
	EXPECT(!Hit.Conservative); EXPECT_NEAR(Hit.Time, 2.0f, 0.006f);
	OtherField->SetPosition(Vector3(3, 0.6f, 0));
	EXPECT(DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), OtherField.get(), *OtherField->GetTransform(), Vector3::UnitX(), 4.0f, Hit));
	printf("SDF oblique pair t=%g conservative=%d iterations=%u nodes=%llu\n", Hit.Time, Hit.Conservative, Hit.Iterations, (unsigned long long)Hit.Stats.NodeVisits);
	printf("SDF oblique error=%g normal=%g,%g,%g\n", Hit.DistanceError, Hit.Normal.x, Hit.Normal.y, Hit.Normal.z);
	EXPECT(!Hit.Conservative); EXPECT_NEAR(Hit.Time, 2.2f, 0.012f); EXPECT(Hit.Normal.x < -0.7f && Hit.Normal.y < -0.5f);
	for (bool Terrain : { false, true })
	{
		auto Ground = Terrain ? TerrainGround() : MeshGround(); Field->SetPosition(Vector3(0, 2, 0));
		EXPECT(DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), Ground.get(), *Ground->GetTransform(), -Vector3::UnitY(), 4, Hit));
		EXPECT(!Hit.Conservative); EXPECT_NEAR(Hit.Time, 1.5f, 0.003f); EXPECT(Hit.Normal.y > 0.99f);
		if (Terrain)
		{
			auto* Grid = Ground->GetShapeObj<HeightField3>(); Grid->SetTriangleHole(0, 0, 0, true); Grid->SetTriangleHole(0, 0, 1, true);
			EXPECT(!DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), Ground.get(), *Ground->GetTransform(), -Vector3::UnitY(), 4, Hit));
		}
	}
	Field->SetPosition(Vector3::Zero());
	std::unique_ptr<Geometry> Plane(GeometryFactory::CreatePlane(Vector3(0, -2, 0), Vector3::UnitY()));
	EXPECT(DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), Plane.get(), *Plane->GetTransform(), -Vector3::UnitY(), 4.0f, Hit));
	EXPECT_NEAR(Hit.Time, 1.5f, 0.003f); EXPECT(Hit.Normal.y > 0.99f);
	EXPECT(!DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), Plane.get(), *Plane->GetTransform(), Vector3::UnitY(), 4.0f, Hit));
	std::unique_ptr<Geometry> Embedded(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.1f)));
	EXPECT(DistanceFieldCollision::SweepBounded(Embedded.get(), *Embedded->GetTransform(), Field.get(), *Field->GetTransform(), Vector3::Zero(), 0, Hit));
	EXPECT(Hit.InitialOverlap && Hit.Time == 0 && !Hit.Conservative);
	const auto Torus = Cook(1.5f, 32, [](const Vector3& P)
	{
		const float R = std::sqrt(P.x * P.x + P.z * P.z) - 0.9f; return std::sqrt(R * R + P.y * P.y) - 0.25f;
	});
	std::unique_ptr<Geometry> Ring(GeometryFactory::CreateDistanceField(Torus));
	std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3(0, 2, 0), Vector3(0.1f)));
	EXPECT(!DistanceFieldCollision::SweepBounded(Box.get(), *Box->GetTransform(), Ring.get(), *Ring->GetTransform(), -Vector3::UnitY(), 4.0f, Hit));
	Box->SetPosition(Vector3(0.9f, 2, 0));
	EXPECT(DistanceFieldCollision::SweepBounded(Box.get(), *Box->GetTransform(), Ring.get(), *Ring->GetTransform(), -Vector3::UnitY(), 4.0f, Hit));
	EXPECT(!Hit.Conservative); EXPECT_NEAR(Hit.Time, 1.65f, 0.008f);
	Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitZ(), 0.7f);
	const Transform Common(Vector3(2, -3, 4), Rotation);
	Ring->SetTransform(Common.pos, Common.quat); Box->SetTransform(Common.LocalToWorld(Vector3(0.9f, 2, 0)), Common.quat);
	EXPECT(DistanceFieldCollision::SweepBounded(Box.get(), *Box->GetTransform(), Ring.get(), *Ring->GetTransform(), -Vector3::UnitY(), 4.0f, Hit));
	EXPECT_NEAR(Hit.Time, 1.65f, 0.008f); EXPECT(Hit.Normal.y > 0.99f);
	RuntimeDistanceFieldSettings Budget; Budget.MaximumSearchNodes = 1;
	RuntimeDistanceField Limited; EXPECT(Limited.Build(*Sphere.GetField().GetField(), Budget));
	DistanceFieldShape LimitedShape; EXPECT(LimitedShape.Build(Limited));
	std::unique_ptr<Geometry> LimitedGeometry(GeometryFactory::CreateDistanceField(LimitedShape, Vector3(3, 0, 0)));
	Embedded->SetPosition(Vector3::Zero());
	EXPECT(DistanceFieldCollision::SweepBounded(Embedded.get(), *Embedded->GetTransform(), LimitedGeometry.get(), *LimitedGeometry->GetTransform(), Vector3::UnitX(), 4, Hit));
	EXPECT(Hit.Conservative && Hit.Status == DistanceFieldSearchStatus::WorkLimit); EXPECT(Hit.Time <= 2.4f);
	Field->SetPosition(Vector3::Zero());
	EXPECT(DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), LimitedGeometry.get(), *LimitedGeometry->GetTransform(), Vector3::UnitX(), 4, Hit));
	EXPECT(Hit.Conservative && Hit.Status == DistanceFieldSearchStatus::WorkLimit); EXPECT(Hit.Time <= 2.0f);
	EXPECT(!DistanceFieldCollision::SweepBounded(Field.get(), *Field->GetTransform(), LimitedGeometry.get(), *LimitedGeometry->GetTransform(), -Vector3::UnitX(), 4, Hit));
}


void TestDistanceFieldCCD()
{
	using namespace Riemann;
	const float Dt = 1.0f / 60.0f;
	const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
	for (int Kind = 0; Kind < 6; ++Kind)
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		Geometry* Target = nullptr;
		if (Kind == 0) Target = GeometryFactory::CreateSphere(Vector3(4, 0, 0), 0.5f);
		if (Kind == 1) Target = GeometryFactory::CreateOBB(Vector3(4, 0, 0), Vector3(0.5f));
		if (Kind == 2) Target = GeometryFactory::CreateDistanceField(Sphere, Vector3(4, 0, 0));
		if (Kind == 3) Target = GeometryFactory::CreatePlane(Vector3(4, 0, 0), -Vector3::UnitX());
		if (Kind >= 4)
		{
			auto Ground = Kind == 4 ? MeshGround() : TerrainGround(); Target = Ground.release();
			Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitZ(), 3.14159265358979323846f / 2);
			Target->SetTransform(Vector3(4, 0, 0), Rotation);
		}
		World.CreateRigidBody(Target, RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere), Moving)->CastDynamic();
		ContinuousCollisionHit CheckHit;
		EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Target->GetParent<RigidBody>() }, Dt, &CheckHit));
		printf("SDF CCD witness kind=%d p=%g,%g,%g n=%g,%g,%g\n", Kind, CheckHit.Position.x, CheckHit.Position.y, CheckHit.Position.z, CheckHit.Normal.x, CheckHit.Normal.y, CheckHit.Normal.z);
		World.Simulate(Dt);
		printf("SDF CCD target=%d x=%g v=%g w=%g hits=%d\n", Kind, Body->X.x, Body->V.Length(), Body->W.Length(), World.GetLastCCDHitCount());
		EXPECT(World.GetLastCCDHitCount() > 0); EXPECT(Body->HadCCDContact);
		EXPECT_NEAR(Body->X.x, Kind < 3 ? 3.0f : 3.5f, 0.01f); EXPECT(Body->V.Length() < 1.0f); EXPECT(Body->W.Length() < 1.0f);
	}
	const auto Bar = Cook(1.5f, 32, [](const Vector3& P) { return BoxDistance(P, Vector3(1.0f, 0.1f, 0.1f)); });
	for (bool FieldMoves : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		Geometry* Target = FieldMoves ? GeometryFactory::CreateSphere(Vector3(0, 0.8f, 0), 0.1f)
			: GeometryFactory::CreateDistanceField(Cook(0.15f, 16, [](const Vector3& P) { return P.Length() - 0.1f; }), Vector3(0, 0.8f, 0));
		auto* Stationary = World.CreateRigidBody(Target, RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.angularVelocity = Vector3(0, 0, (3.14159265358979323846f / 2) / Dt); Moving.sleepThreshold = 0;
		Geometry* Shape = FieldMoves ? GeometryFactory::CreateDistanceField(Bar) : GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1.0f, 0.1f, 0.1f));
		auto* Body = World.CreateRigidBody(Shape, Moving)->CastDynamic();
		ContinuousCollisionHit Hit;
		EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Stationary }, Dt, &Hit));
		printf("SDF angular CCD fieldMoves=%d fraction=%g conservative=%d\n", FieldMoves, Hit.Fraction, Hit.ConservativeHit);
		EXPECT(Hit.AngularHit && !Hit.ConservativeHit && Hit.Fraction > 0.7f && Hit.Fraction < 0.95f);
		World.Simulate(Dt); EXPECT(World.GetLastCCDHitCount() > 0 && Body->CCDMotionFraction < 0.95f);
	}
	// A static infinite plane contributes no angular speed bound. In particular,
	// zero angular speed times its unbounded radius must not produce NaN.
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		auto* Plane = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3(0, 0.8f, 0), -Vector3::UnitY()), RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.angularVelocity = Vector3(0, 0, (3.14159265358979323846f / 2) / Dt); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Bar), Moving)->CastDynamic(); ContinuousCollisionHit Hit;
		EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Plane }, Dt, &Hit));
		EXPECT(Hit.AngularHit && !Hit.ConservativeHit && Hit.Fraction > 0.4f && Hit.Fraction < 0.7f);
		EXPECT_NEAR(Hit.Position.y, 0.8f, 0.002f); EXPECT(Hit.Normal.y < -0.99f);
		World.Simulate(Dt); EXPECT(World.GetLastCCDHitCount() > 0 && Body->CCDMotionFraction < 0.7f);
	}
	// Combined translation/rotation clears this obstacle; translation at the
	// initial orientation alone would report a spurious hit.
	for (bool FieldMoves : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		Geometry* Target = FieldMoves ? GeometryFactory::CreateSphere(Vector3(-0.9f, 0.7f, 0), 0.1f)
			: GeometryFactory::CreateDistanceField(Cook(0.15f, 16, [](const Vector3& P) { return P.Length() - 0.1f; }), Vector3(-0.9f, 0.7f, 0));
		auto* Stationary = World.CreateRigidBody(Target, RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.angularVelocity = Vector3(0, 0, (3.14159265358979323846f / 2) / Dt); Moving.linearVelocity = Vector3(0, 2 / Dt, 0); Moving.sleepThreshold = 0;
		Geometry* Shape = FieldMoves ? GeometryFactory::CreateDistanceField(Bar) : GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1.0f, 0.1f, 0.1f));
		auto* Body = World.CreateRigidBody(Shape, Moving)->CastDynamic(); ContinuousCollisionHit Hit;
		EXPECT(!ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Stationary }, Dt, &Hit));
		World.Simulate(Dt); EXPECT(World.GetLastCCDHitCount() == 0); EXPECT_NEAR(Body->X.y, 2.0f, 0.001f);
	}
	// Relative-frame sweeps must restore target motion in world-space witnesses.
	// Exercise both body orderings of the trigger path as well as solid impacts.
	for (bool Trigger : { false, true }) for (bool Reverse : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere), Moving)->CastDynamic();
		Moving.linearVelocity = Vector3(-60, 0, 0);
		auto* Target = GeometryFactory::CreateSphere(Vector3(4, 0, 0), 0.5f);
		Target->SetTriggerEnabled(Trigger); Target->SetTriggerCCDEnabled(Trigger);
		auto* Other = World.CreateRigidBody(Target, Moving);
		std::vector<RigidBody*> Bodies = Reverse ? std::vector<RigidBody*>{ Other, Body } : std::vector<RigidBody*>{ Body, Other };
		ContinuousCollisionHit Hit;
		if (Trigger)
		{
			std::vector<ContinuousCollisionHit> Hits; ContinuousCollisionDetection::FindTriggerHits(Bodies, Dt, &Hits);
			EXPECT(Hits.size() == 1); if (Hits.empty()) continue; Hit = Hits.front();
		}
		else EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, Bodies, Dt, &Hit));
		EXPECT_NEAR(Hit.Fraction, 3.0f / 11, 0.001f);
		EXPECT_NEAR(Hit.Position.x, 30.0f / 11 + 0.5f, 0.001f);
		EXPECT(std::abs(Hit.Position.y) < 0.001f && std::abs(Hit.Position.z) < 0.001f);
	}
	// A compound's local shape offset and center of mass must both survive CCD.
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		auto* Wall = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), -Vector3::UnitX()), RigidBodyParam());
		auto* Front = GeometryFactory::CreateDistanceField(Sphere, Vector3(-4, 0, 0));
		auto* Back = GeometryFactory::CreateSphere(Vector3(-6, 0, 0), 0.5f);
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody({ Front, Back }, Moving, Transform(Vector3(-5, 0, 0)))->CastDynamic();
		ContinuousCollisionHit Hit; EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Wall }, Dt, &Hit));
		EXPECT(Hit.Shape == Front); EXPECT_NEAR(Hit.Fraction, 0.35f, 0.001f); EXPECT_NEAR(Hit.Position.x, 0.0f, 0.001f);
		World.Simulate(Dt); EXPECT_NEAR(Body->GetGeometryTransform(Front).pos.x, -0.501f, 0.003f);
		EXPECT(Body->V.Length() < 0.1f && Body->W.Length() < 0.1f);
	}
	// Unresolved nested searches must propagate to world motion limiting.
	{
		RuntimeDistanceFieldSettings Budget; Budget.MaximumSearchNodes = 1;
		RuntimeDistanceField Limited; EXPECT(Limited.Build(*Sphere.GetField().GetField(), Budget));
		DistanceFieldShape LimitedShape; EXPECT(LimitedShape.Build(Limited));
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		auto* Target = World.CreateRigidBody(GeometryFactory::CreateDistanceField(LimitedShape, Vector3(3, 0, 0)), RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere), Moving)->CastDynamic(); ContinuousCollisionHit Hit;
		EXPECT(ContinuousCollisionDetection::FindEarliestHit(Body, { Body, Target }, Dt, &Hit));
		EXPECT(Hit.ConservativeHit && Hit.Fraction <= 0.2f);
		World.Simulate(Dt); EXPECT(World.GetLastCCDHitCount() > 0); EXPECT(Body->X.x <= 2.0f);
	}
	for (bool Deny : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); PhysicsWorld World(Param);
		if (Deny) World.SetCollisionFilter(std::shared_ptr<CollisionFilter>(CollisionFilter::CreateCallback([](const CollisionData&, const CollisionData&) { return false; })));
		World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(4, 0, 0), Vector3(0.5f)), RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.motionType = MorionType::Continuous;
		Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0;
		auto* Body = World.CreateRigidBody(GeometryFactory::CreateDistanceField(Sphere), Moving);
		World.Simulate(Dt); EXPECT((World.GetLastCCDHitCount() == 0) == Deny);
		if (Deny) EXPECT(Body->X.x > 9.9f);
	}
}

void TestDistanceFieldMargins()
{
	using namespace Riemann;
	for (float Scale : { 0.001f, 1.0f, 1000.0f })
	{
		const auto Sphere = Cook(0.75f * Scale, 24, [=](const Vector3& P) { return P.Length() - 0.5f * Scale; });
		for (int Kind = 0; Kind <= 5; ++Kind) for (bool Reverse : { false, true })
		{
			std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere));
			std::unique_ptr<Geometry> Other(Kind == 5 ? GeometryFactory::CreateDistanceField(Sphere, Vector3(5 * Scale, 0, 0)) : Primitive(Kind, Vector3(5 * Scale, 0, 0), Scale));
			Geometry* A = Reverse ? Other.get() : Field.get(); Geometry* B = Reverse ? Field.get() : Other.get();
			const float Gap = (Kind == 2 ? 3.0f : 4.0f) * Scale;
			std::vector<DistanceFieldContact> Contacts;
			const bool Found = DistanceFieldCollision::Generate(A, *A->GetTransform(), B, *B->GetTransform(), Gap + 0.01f * Scale, Contacts);
			if (!Found) printf("SDF wide margin missing scale=%g kind=%d reverse=%d\n", Scale, Kind, Reverse);
			EXPECT(Found && !Contacts.empty());
			for (const auto& C : Contacts)
			{
				EXPECT_NEAR(C.Separation / Scale, Gap / Scale, 0.004f);
				EXPECT(C.Normal.x * (Reverse ? -1 : 1) > 0.99f);
				DistanceFieldSample Sample; EXPECT(Sphere.GetField().Sample(Reverse ? C.PointB : C.PointA, Sample)); EXPECT(std::abs(Sample.Value / Scale) < 0.0001f);
				EXPECT_NEAR((C.PointB - C.PointA).Dot(C.Normal) / Scale, C.Separation / Scale, 0.0001f);
			}
			EXPECT(!DistanceFieldCollision::Generate(A, *A->GetTransform(), B, *B->GetTransform(), Gap - 0.05f * Scale, Contacts));
			EXPECT(!A->Intersect(B));
		}
		for (float Side : { -1.0f, 1.0f }) for (bool Reverse : { false, true })
		{
			std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere, Vector3(0, 3 * Side, 0.3f) * Scale));
			auto Ground = MeshGround(0.01f * Scale); MeshContactGenerator Generator;
			std::vector<ContactManifold> Patches;
			Generator.Generate(0, 1, State(Reverse ? Field.get() : Ground.get()), State(Reverse ? Ground.get() : Field.get()), Patches, 2.6f * Scale);
			EXPECT(!Patches.empty());
			MeshContactSettings SingleSide; SingleSide.TwoSidedMeshes = false; MeshContactGenerator SingleGenerator(SingleSide);
			std::vector<ContactManifold> SinglePatches;
			SingleGenerator.Generate(0, 1, State(Ground.get()), State(Field.get()), SinglePatches, 2.6f * Scale);
			EXPECT(SinglePatches.empty() == (Side < 0.0f));
			for (const auto& Patch : Patches) for (int I = 0; I < Patch.NumContactPointCount; ++I)
			{
				const auto& C = Patch.ContactPoints[I]; EXPECT(C.PenetrationDepth < -2.5f * Scale && C.PenetrationDepth > -2.6f * Scale);
				EXPECT(C.Normal.y * Side * (Reverse ? -1 : 1) > 0.99f);
				const Vector3 OnMesh = Reverse ? C.PositionWorldB : C.PositionWorldA;
				EXPECT(std::abs(OnMesh.y) < 0.0001f * Scale && std::abs(OnMesh.x) <= 0.0101f * Scale && std::abs(OnMesh.z) <= 0.0101f * Scale);
			}
		}
	}
	const auto Sphere = Cook(0.75f, 24, [](const Vector3& P) { return P.Length() - 0.5f; });
	for (bool Reverse : { false, true })
	{
		std::unique_ptr<Geometry> Field(GeometryFactory::CreateDistanceField(Sphere));
		std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3(5, 0, 0), Vector3(0.5f)));
		Field->SetContactOffsets(2.1f, 0.1f); Box->SetContactOffsets(2.1f, 0.1f);
		GeometryWorldState States[] = { State(Reverse ? Box.get() : Field.get()), State(Reverse ? Field.get() : Box.get()) };
		const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
		std::unique_ptr<NarrowPhase> Narrow(NarrowPhase::Create_PCM()); Narrow->SetContactGeneration(ContactGenerationSettings(), 1.0f / 60.0f);
		std::vector<ContactManifold*> Patches;
		Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Patches);
		EXPECT(!Patches.empty());
		for (const auto* Patch : Patches) for (int I = 0; I < Patch->NumContactPointCount; ++I)
		{
			const auto& C = Patch->ContactPoints[I]; EXPECT_NEAR(C.PenetrationDepth, -3.8f, 0.004f);
			EXPECT_NEAR((C.PositionWorldB - C.PositionWorldA).Dot(C.Normal), 4.0f, 0.004f);
			EXPECT((Reverse ? C.FeatureB : C.FeatureA) != UINT32_MAX);
		}
	}

	for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
	for (bool FieldMoves : { false, true }) for (bool Enabled : { false, true })
	{
		PhysicsWorldParam Param; Param.gravityAcc = Vector3::Zero(); Param.resolutionSolver = Solver; Param.contactGeneration.SpeculativeContacts = Enabled;
		PhysicsWorld World(Param);
		Geometry* Static = FieldMoves ? GeometryFactory::CreateOBB(Vector3(4, 0, 0), Vector3(0.5f)) : GeometryFactory::CreateDistanceField(Sphere, Vector3(4, 0, 0));
		World.CreateRigidBody(Static, RigidBodyParam());
		RigidBodyParam Moving; Moving.rigidType = RigidType::Dynamic; Moving.linearVelocity = Vector3(600, 0, 0); Moving.sleepThreshold = 0; Moving.maxContactImpulse = FLT_MAX;
		Geometry* Dynamic = FieldMoves ? GeometryFactory::CreateDistanceField(Sphere) : GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f));
		auto* Body = World.CreateRigidBody(Dynamic, Moving);
		World.Simulate(1.0f / 60.0f);
		printf("SDF speculative solver=%d fieldMoves=%d enabled=%d x=%g vx=%g\n", (int)Solver, FieldMoves, Enabled, Body->X.x, Body->V.x);
		EXPECT(Enabled ? (Body->X.x <= 3.01f && Body->X.x > 2.9f) : Body->X.x > 9.9f);
		EXPECT(World.GetLastCCDHitCount() == 0);
	}
}

namespace Test_RuntimeDistanceField_Cases
{
namespace
{
	using namespace Riemann;
	const float ValueTolerance = 2.0e-5f;
	template<class Function>
	SignedDistanceField3 MakeField(float Radius, int Cells, Function Value)
	{
		SignedDistanceField3 Field;
		const float Spacing = 2 * Radius / Cells;
		EXPECT(Field.Reset(Box3(Vector3(-Radius), Vector3(Radius)), Cells, Cells, Cells, Spacing));
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Field.At(X, Y, Z) = Value(Field.GridPointToWorld(X, Y, Z));
		return Field;
	}
	float Multilinear(const Vector3& P)
	{
		return 1 + 2 * P.x - 3 * P.y + 0.5f * P.z + P.x * P.y + 2 * P.y * P.z - 0.25f * P.x * P.z + P.x * P.y * P.z;
	}
	Vector3 MultilinearGradient(const Vector3& P)
	{
		return Vector3(2 + P.y - 0.25f * P.z + P.y * P.z, -3 + P.x + 2 * P.z + P.x * P.z, 0.5f + 2 * P.y - 0.25f * P.x + P.x * P.y);
	}
	class SphereDistanceTarget : public DistanceFieldDistanceTarget
	{
	public:
		Vector3 Center; float Radius;
		SphereDistanceTarget(const Vector3& Position, float Size) : Center(Position), Radius(Size) {}
		bool Evaluate(const Vector3& Position, DistanceFieldTargetPoint& Result) const override
		{
			const Vector3 Delta = Position - Center; const float Length = Delta.Length();
			Result.Normal = Length > 0.0f ? Delta / Length : Vector3::UnitX();
			Result.Point = Center + Result.Normal * Radius; Result.Distance = std::max(0.0f, Length - Radius);
			Result.SupportingPlane = true; return true;
		}
	};
	void TestTargetDistance()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			RuntimeDistanceField Field;
			EXPECT(Field.Build(MakeField(1.5f * Scale, 24, [=](const Vector3& P) { return P.Length() - Scale; })));
			SphereDistanceTarget Target(Vector3(2 * Scale, 0, 0), 0.25f * Scale);
			DistanceFieldTargetDistance Result; DistanceFieldQueryStats Stats;
			EXPECT(Field.DistanceToTarget(Target, Result, &Stats));
			printf("SDF target scale=%g lower=%g upper=%g nodes=%llu subdivisions=%llu\n", Scale, Result.LowerBound / Scale, Result.UpperBound / Scale,
				(unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions);
			EXPECT(Result.Converged && Result.Status == DistanceFieldSearchStatus::Complete);
			EXPECT(Result.LowerBound <= 0.75f * Scale && Result.UpperBound >= 0.75f * Scale);
			EXPECT_NEAR(Result.UpperBound / Scale, 0.75f, 0.002f);
			EXPECT(Result.Normal.x < -0.99f);
			DistanceFieldSample AtSurface; EXPECT(Field.Sample(Result.Position, AtSurface)); EXPECT(std::abs(AtSurface.Value / Scale) < 0.0001f);
			Target.Center = Vector3(Scale, 0, 0);
			EXPECT(Field.DistanceToTarget(Target, Result)); EXPECT(Result.UpperBound / Scale < 0.001f);
		}
		RuntimeDistanceField Plane;
		EXPECT(Plane.Build(MakeField(1.0f, 16, [](const Vector3& P) { return P.x * 1.0e-9f; })));
		SphereDistanceTarget Target(Vector3(2, 0.17f, -0.31f), 0.25f);
		DistanceFieldTargetDistance Result; DistanceFieldQueryStats Stats;
		EXPECT(Plane.DistanceToTarget(Target, Result, &Stats)); EXPECT(Result.Converged);
		EXPECT(Result.LowerBound <= 1.75f && Result.UpperBound >= 1.75f);
		EXPECT_NEAR(Result.UpperBound, 1.75f, 0.002f); EXPECT(Stats.Subdivisions < 1000);
		RuntimeDistanceFieldSettings Limited; Limited.MaximumSearchNodes = 1;
		RuntimeDistanceField Short; EXPECT(Short.Build(*Plane.GetField(), Limited));
		Short.DistanceToTarget(Target, Result);
		EXPECT(!Result.Converged && Result.Status == DistanceFieldSearchStatus::WorkLimit);
		EXPECT(Result.LowerBound <= 1.75f && Result.UpperBound >= 1.75f);
	}

	void TestSamplingAndOwnership()
	{
		SignedDistanceField3 Source = MakeField(1.0f, 8, Multilinear);
		RuntimeDistanceField Runtime;
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.IsValid() && !Runtime.HasClosedBoundary());
		RuntimeDistanceField Copy = Runtime;
		std::mt19937 Random(2026); std::uniform_real_distribution<float> Coordinate(-1.0f, 1.0f);
		for (int Index = 0; Index < 1000; ++Index)
		{
			const Vector3 Point(Coordinate(Random), Coordinate(Random), Coordinate(Random));
			DistanceFieldSample Sample;
			EXPECT(Runtime.Sample(Point, Sample));
			EXPECT_NEAR(Sample.Value, Multilinear(Point), ValueTolerance);
			EXPECT((Sample.Gradient - MultilinearGradient(Point)).Length() < ValueTolerance);
			EXPECT(Sample.Gradient.Length() <= Runtime.GetGradientBound() + ValueTolerance);
		}
		for (const Vector3& Point : { Vector3(-1), Vector3(1), Vector3(-1, 0.5f, 1), Vector3(0, 1, 0) })
		{
			DistanceFieldSample Sample; EXPECT(Runtime.Sample(Point, Sample));
			EXPECT_NEAR(Sample.Value, Multilinear(Point), ValueTolerance);
			EXPECT((Sample.Gradient - MultilinearGradient(Point)).Length() < ValueTolerance);
		}
		DistanceFieldSample Sample; Sample.Value = 123;
		EXPECT(!Runtime.Sample(Vector3(1.001f, 0, 0), Sample)); EXPECT(Sample.Value == 123);
		EXPECT(!Runtime.Sample(Vector3(std::numeric_limits<float>::quiet_NaN()), Sample));
		Source.At(4, 4, 4) = -999;
		EXPECT(Runtime.Sample(Vector3::Zero(), Sample)); EXPECT_NEAR(Sample.Value, 1.0f, ValueTolerance);
		EXPECT(Runtime.Build(MakeField(1.0f, 4, [](const Vector3&) { return 3.0f; })));
		EXPECT(Copy.Sample(Vector3::Zero(), Sample)); EXPECT_NEAR(Sample.Value, 1.0f, ValueTolerance);
		EXPECT(Runtime.GetSurfaceCellCount() == 0);
		DistanceFieldRayHit Hit; EXPECT(!Runtime.RayCast(Vector3(-2, 0, 0), Vector3::UnitX(), 4.0f, Hit));
		DistanceFieldSurfacePoint Surface; EXPECT(!Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface));
		EXPECT(Surface.Converged && Surface.Status == DistanceFieldSearchStatus::Complete);
		Runtime.Clear(); EXPECT(!Runtime.IsValid() && !Runtime.Sample(Vector3::Zero(), Sample));
	}
	void TestTranslatedGridBoundaries()
	{
		const int Cells = 16;
		const float Spacing = 0.1f;
		const Vector3 Origin(100.13f, -72.71f, 35.33f);
		SignedDistanceField3 Field;
		EXPECT(Field.Reset(Box3(Origin, Origin + Vector3(Cells * Spacing)), Cells, Cells, Cells, Spacing));
		std::mt19937 Random(47); std::uniform_real_distribution<float> Value(-2.0f, 2.0f);
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Field.At(X, Y, Z) = Value(Random);
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Field));
		for (int Z = 1; Z < Cells; ++Z) for (int Y = 1; Y < Cells; ++Y) for (int X = 1; X < Cells; ++X)
		{
			const Vector3 Vertex = Field.GridPointToWorld(X, Y, Z);
			DistanceFieldSample Sample; EXPECT(Runtime.Sample(Vertex, Sample));
			EXPECT(Sample.Value == Field.At(X, Y, Z));
			const Index3 At(X, Y, Z);
			for (int Axis = 0; Axis < 3; ++Axis) for (int Side : { -1, 1 })
			{
				Vector3 Point = Vertex;
				Point[Axis] = std::nextafter(Vertex[Axis], Side < 0 ? -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity());
				Index3 Neighbor = At; Neighbor[Axis] += Side;
				const Vector3 Endpoint = Field.GridPointToWorld(Neighbor.a, Neighbor.b, Neighbor.c);
				const double Weight = ((double)Point[Axis] - Vertex[Axis]) / ((double)Endpoint[Axis] - Vertex[Axis]);
				const float Expected = (float)((1.0 - Weight) * Field.At(X, Y, Z) + Weight * Field.At(Neighbor.a, Neighbor.b, Neighbor.c));
				EXPECT(Runtime.Sample(Point, Sample));
				EXPECT_NEAR(Sample.Value, Expected, std::numeric_limits<float>::epsilon());
			}
		}
	}
	void TestHierarchy()
	{
		const int Cells = 48;
		SignedDistanceField3 Source = MakeField(1.5f, Cells, [](const Vector3& P) { return P.Length() - 1.0f; });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.HasClosedBoundary() && Runtime.GetNodeCount() > 1 && Runtime.GetSurfaceCellCount() > 0);
		for (float Range : { 0.0f, 0.1f, 0.3f })
		{
			const Box3 Bounds(Vector3(0.8f, -0.15f, -0.15f), Vector3(1.2f, 0.15f, 0.15f));
			std::vector<Index3> Found; DistanceFieldQueryStats Stats;
			Runtime.QueryCells(Bounds, -Range, Range, Found, &Stats);
			std::vector<int> Expected, Actual;
			for (const Index3& C : Found) Actual.push_back((C.c * Cells + C.b) * Cells + C.a);
			for (int Z = 0; Z < Cells; ++Z) for (int Y = 0; Y < Cells; ++Y) for (int X = 0; X < Cells; ++X)
			{
				const Box3 B(Source.GridPointToWorld(X, Y, Z), Source.GridPointToWorld(X + 1, Y + 1, Z + 1));
				if (!B.Intersect(Bounds)) continue;
				float Min = std::numeric_limits<float>::max(), Max = -Min;
				for (int DZ = 0; DZ <= 1; ++DZ) for (int DY = 0; DY <= 1; ++DY) for (int DX = 0; DX <= 1; ++DX)
				{
					const float V = Source.At(X + DX, Y + DY, Z + DZ); Min = std::min(Min, V); Max = std::max(Max, V);
				}
				if (Min <= Range && Max >= -Range) Expected.push_back((Z * Cells + Y) * Cells + X);
			}
			std::sort(Actual.begin(), Actual.end()); std::sort(Expected.begin(), Expected.end());
			EXPECT(Actual == Expected && !Actual.empty());
			EXPECT(Stats.CellTests < (uint64_t)Cells * Cells * Cells / 20);
			printf("SDF range=%g selected=%zu cell tests=%llu full grid=%d\n", Range, Actual.size(), (unsigned long long)Stats.CellTests, Cells * Cells * Cells);
		}
	}
	void TestRayRoots()
	{
		RuntimeDistanceField Runtime;
		// All three roots occur in one cell; a march that only checks its ends misses two.
		SignedDistanceField3 Source; EXPECT(Source.Reset(Box3(Vector3::Zero(), Vector3(1)), 1, 1, 1, 1.0f));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.2f) * (Y - 0.5f) * (Z - 0.8f);
		EXPECT(Runtime.Build(Source));
		DistanceFieldRayHit Hit;
		for (float Start : { 0.0f, 0.3f, 0.6f })
		{
			const float Target = Start == 0.0f ? 0.2f : (Start < 0.5f ? 0.5f : 0.8f);
			EXPECT(Runtime.RayCast(Vector3(Start), Vector3(1), 1.0f - Start, Hit, false));
			EXPECT_NEAR(Hit.Time, Target - Start, 0.0002f);
			EXPECT(Hit.NormalValid);
			EXPECT(!Runtime.RayCast(Vector3(Start), Vector3(1), Target - Start - 0.01f, Hit, false));
		}
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * (Y - 0.5f);
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 1.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance);
		EXPECT(!Hit.NormalValid);
		EXPECT(!Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 0.49f, Hit));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * (Y - 0.5f) + 1.0e-6f;
		EXPECT(Runtime.Build(Source));
		EXPECT(!Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 1.0f, Hit));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * 1.0e-9f;
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.RayCast(Vector3(1, 0.5f, 0.5f), -Vector3::UnitX(), 1.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance);
		EXPECT(Runtime.Build(MakeField(1.0f, 8, [](const Vector3& P) { return P.x - 1.0f; })));
		EXPECT(Runtime.RayCast(Vector3(2, 0, 0), Vector3(-2, 0, 0), 0.5f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance); EXPECT(Hit.Normal.x > 0.99f);
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit));
		EXPECT(Hit.InitialOverlap && Hit.Time == 0.0f);
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit, false));
		EXPECT_NEAR(Hit.Time, 1.0f, ValueTolerance);
		EXPECT(!Runtime.RayCast(Vector3::Zero(), Vector3::Zero(), 2.0f, Hit));
	}
	void TestSphereQueries()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			RuntimeDistanceField Runtime;
			EXPECT(Runtime.Build(MakeField(1.5f * Scale, 48, [&](const Vector3& P) { return P.Length() - Scale; })));
			for (float Y : { 0.0f, 0.3f, 0.8f })
			{
				DistanceFieldRayHit Hit; DistanceFieldQueryStats Stats;
				EXPECT(Runtime.RayCast(Vector3(-2, Y, 0) * Scale, Vector3::UnitX(), 4 * Scale, Hit, true, &Stats));
				EXPECT_NEAR(Hit.Time / Scale, 2 - std::sqrt(1 - Y * Y), 0.004f);
				EXPECT(Hit.NormalValid && Hit.Normal.Dot(Hit.Position.Unit()) > 0.995f);
				EXPECT(Stats.CellTests < 100);
			}
			for (const Vector3& P : { Vector3(2, 0, 0), Vector3(1.1f, 0.3f, -0.2f), Vector3(0.2f, 0, 0), Vector3::Zero() })
			{
				DistanceFieldSurfacePoint Surface; DistanceFieldQueryStats Stats;
				EXPECT(Runtime.ClosestSurfacePoint(P * Scale, Surface, 4 * Scale, &Stats));
				EXPECT(Surface.Converged && Surface.Status == DistanceFieldSearchStatus::Complete);
				EXPECT_NEAR(Surface.Distance / Scale, std::abs(P.Length() - 1.0f), 0.004f);
				DistanceFieldSample Sample; EXPECT(Runtime.Sample(Surface.Position, Sample));
				EXPECT(std::abs(Sample.Value) / Scale < 2.0e-5f);
				EXPECT(Surface.NormalValid && Surface.Normal.Dot(Surface.Position.Unit()) > 0.995f);
				printf("SDF closest scale=%g query=%g distance=%g error=%g nodes=%llu splits=%llu converged=%d\n", Scale,
					P.Length(), Surface.Distance / Scale, Surface.ErrorBound / Scale, (unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions, Surface.Converged);
			}
			DistanceFieldSurfacePoint Miss;
			EXPECT(!Runtime.ClosestSurfacePoint(Vector3(2 * Scale, 0, 0), Miss, 0.9f * Scale));
			EXPECT(Miss.Converged);
		}
	}
	void TestDistanceBounds()
	{
		RuntimeDistanceField Runtime;
		const auto Plane = MakeField(2.0f, 8, [](const Vector3& P) { return P.x + P.y + P.z; });
		EXPECT(Runtime.Build(Plane));
		std::mt19937 Random(1207); std::uniform_real_distribution<float> Coordinate(-1.0f, 1.0f);
		for (int Index = 0; Index < 100; ++Index)
		{
			const Vector3 Point(Coordinate(Random), Coordinate(Random), Coordinate(Random));
			DistanceFieldSurfacePoint Surface;
			EXPECT(Runtime.ClosestSurfacePoint(Point, Surface));
			const float Exact = std::abs(Point.x + Point.y + Point.z) / std::sqrt(3.0f);
			EXPECT(Surface.Converged);
			EXPECT(std::abs(Surface.Distance - Exact) <= Surface.ErrorBound + ValueTolerance);
			EXPECT(std::abs(Surface.Position.x + Surface.Position.y + Surface.Position.z) < ValueTolerance);
		}
		RuntimeDistanceFieldSettings Strict; Strict.RelativeDistanceTolerance = 1.0e-10f;
		EXPECT(Runtime.Build(Plane, Strict));
		DistanceFieldSurfacePoint Surface;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3(1), Surface));
		EXPECT(!Surface.Converged && Surface.Status == DistanceFieldSearchStatus::PrecisionLimit);
	}
	void TestConcavityAndBudgets()
	{
		const auto Shell = MakeField(1.5f, 48, [](const Vector3& P) { return std::max(0.5f - P.Length(), P.Length() - 1.0f); });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Shell));
		DistanceFieldSurfacePoint Surface;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface));
		EXPECT_NEAR(Surface.Distance, 0.5f, 0.004f); EXPECT(Surface.Converged);
		EXPECT(Surface.Normal.Dot(Surface.Position.Unit()) < -0.99f);
		DistanceFieldRayHit Hit;
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance); EXPECT(!Hit.InitialOverlap && Hit.Normal.x < -0.99f);
		RuntimeDistanceFieldSettings Budget; Budget.MaximumSearchNodes = 1;
		EXPECT(Runtime.Build(Shell, Budget));
		Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface);
		EXPECT(!Surface.Converged && Surface.Status == DistanceFieldSearchStatus::WorkLimit);
		const auto Torus = MakeField(1.5f, 48, [](const Vector3& P)
		{
			const float Ring = std::sqrt(P.x * P.x + P.z * P.z) - 0.9f;
			return std::sqrt(Ring * Ring + P.y * P.y) - 0.25f;
		});
		EXPECT(Runtime.Build(Torus));
		EXPECT(!Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT(Runtime.RayCast(Vector3(0.9f, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT_NEAR(Hit.Time, 1.75f, 0.006f);
		DistanceFieldQueryStats Stats;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface, 4.0f, &Stats));
		printf("SDF torus distance=%g error=%g nodes=%llu splits=%llu converged=%d\n", Surface.Distance, Surface.ErrorBound,
			(unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions, Surface.Converged);
		EXPECT_NEAR(Surface.Distance, 0.65f, 0.006f); EXPECT(Surface.Converged);
	}
	void TestValidationAndMeshCooking()
	{
		SignedDistanceField3 Field;
		EXPECT(!Field.Reset(Box3(Vector3(-1), Vector3(1)), std::numeric_limits<int>::max(), 1, 1, 1.0f));
		EXPECT(!Field.Reset(Box3(Vector3(-1), Vector3(1)), 2, 2, 2, std::numeric_limits<float>::infinity()));
		Field = MakeField(1.0f, 8, [](const Vector3& P) { return P.y; });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Field));
		const auto* Original = Runtime.GetField();
		Field.At(0, 0, 0) = std::numeric_limits<float>::quiet_NaN();
		std::string Error; EXPECT(!Runtime.Build(Field, RuntimeDistanceFieldSettings(), &Error));
		EXPECT(!Error.empty() && Runtime.GetField() == Original);
		EXPECT(Field.Reset(Box3(Vector3(-1), Vector3(1)), 2, 2, 2, 0.5f));
		EXPECT(!Runtime.Build(Field, RuntimeDistanceFieldSettings(), &Error));
		// Accepted metadata roundoff must not permit extrapolation outside the cells.
		SignedDistanceField3 RoundedBounds;
		const float AboveOne = std::nextafter(1.0f, 2.0f);
		EXPECT(RoundedBounds.Reset(Box3(Vector3(-1), Vector3(AboveOne)), 2, 2, 2, 1.0f));
		RuntimeDistanceField RoundedRuntime; EXPECT(RoundedRuntime.Build(RoundedBounds));
		EXPECT(RoundedRuntime.GetBounds().Max == Vector3(1));
		DistanceFieldSample RoundedSample;
		EXPECT(RoundedRuntime.Sample(Vector3(1), RoundedSample));
		EXPECT(!RoundedRuntime.Sample(Vector3(AboveOne), RoundedSample));
		RuntimeDistanceFieldSettings Settings; Settings.RelativeValueTolerance = -1;
		EXPECT(!Runtime.Build(*Original, Settings, &Error));
		EXPECT(Runtime.GetField() == Original);

		StaticMesh Mesh; Mesh.AddAABB(Vector3(-0.5f), Vector3(0.5f)); Mesh.Compact();
		std::vector<uint32_t> Indices;
		for (uint32_t T = 0; T < Mesh.NumTriangles; ++T)
		{
			uint32_t A, B, C; Mesh.GetVertIndices(T, A, B, C);
			// Winding-number cooking requires a consistently oriented source surface.
			if ((Mesh.Vertices[B] - Mesh.Vertices[A]).Cross(Mesh.Vertices[C] - Mesh.Vertices[A]).Dot(Mesh.Vertices[A]) < 0.0f) std::swap(B, C);
			Indices.push_back(A); Indices.push_back(B); Indices.push_back(C);
		}
		SignedDistanceFieldBuildOptions Options; Options.Resolution = 24;
		EXPECT(SignedDistanceField3::BuildFromTriangleSoup(Mesh.Vertices, Mesh.NumVertices, Indices.data(), Mesh.NumTriangles, Field, Options));
		EXPECT(Runtime.Build(Field)); EXPECT(Runtime.HasClosedBoundary());
		DistanceFieldRayHit Hit; EXPECT(Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT_NEAR(Hit.Time, 1.5f, 0.01f);
		std::vector<std::future<bool>> Jobs;
		for (int Worker = 0; Worker < 4; ++Worker) Jobs.push_back(std::async(std::launch::async, [Runtime]()
		{
			for (int Step = 0; Step < 100; ++Step)
			{
				DistanceFieldRayHit Hit; DistanceFieldSample Sample;
				if (!Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit)
					|| !Runtime.Sample(Vector3::Zero(), Sample) || Sample.Value >= 0.0f || std::abs(Hit.Time - 1.5f) > 0.01f) return false;
			}
			return true;
		}));
		for (auto& Job : Jobs) EXPECT(Job.get());
	}
}

void TestRuntimeDistanceField()
{
	printf("Running runtime SDF interpolation, hierarchy, cubic roots, closest surfaces and cooking\n");
	TestTargetDistance(); TestSamplingAndOwnership(); TestTranslatedGridBoundaries(); TestHierarchy(); TestRayRoots(); TestSphereQueries(); TestDistanceBounds(); TestConcavityAndBudgets(); TestValidationAndMeshCooking();
}
}

void TestRuntimeDistanceField() { Test_RuntimeDistanceField_Cases::TestRuntimeDistanceField(); }

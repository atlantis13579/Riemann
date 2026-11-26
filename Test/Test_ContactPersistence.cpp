#include "Test.h"
#include <memory>
#include <stdexcept>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include "../Src/RigidBodyDynamics/Jacobian.h"
#include "../Src/RigidBodyDynamics/WarmStart.h"
#include <limits>
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/FrictionPatch.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include <algorithm>
#include "../Src/Destruction/DestructionSet.h"

namespace
{
	using namespace Riemann;
	const float TimeStep = 1.0f / 60.0f;
	const float NumericalTolerance = 1.0e-5f;

	Contact MakeContact(float X, uint32_t Feature = 0)
	{
		Contact C;
		C.PositionWorldA = Vector3(X, 0.0f, 0.0f);
		C.PositionWorldB = Vector3(X, -0.01f, 0.0f);
		C.PositionLocalA = C.PositionWorldA;
		C.PositionLocalB = C.PositionWorldB;
		C.Normal = Vector3::UnitY();
		C.Tangent = Vector3::UnitX();
		C.Binormal = C.Normal.Cross(C.Tangent);
		C.PenetrationDepth = 0.01f;
		C.FeatureA = Feature;
		C.FeatureB = Feature;
		C.totalImpulseNormal = 2.0f;
		C.totalImpulseTangent = 0.2f;
		C.totalImpulseBinormal = -0.1f;
		return C;
	}

	void TestAnchorCache()
	{
		std::unique_ptr<Geometry> ShapeA(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		std::unique_ptr<Geometry> ShapeB(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		GeometryWorldState A, B;
		A.Geom = ShapeA.get(); B.Geom = ShapeB.get();
		A.Handle = GeometryHandle(0, 1); B.Handle = GeometryHandle(1, 1);
		ContactManifoldManager Cache;
		ContactManifold M;
		M.AddNewContact(0, 1, MakeContact(0.0f));
		M.AddNewContact(0, 1, MakeContact(0.1f, 1));
		Cache.BeginFrame(1);
		Cache.Store(A, B, M, TimeStep);
		const uint64_t Id = M.ContactPoints[0].PersistentId;
		EXPECT(Id != 0 && Cache.GetPairCount() == 1);
		Cache.BeginFrame(2);
		ContactManifold Refreshed;
		A.ShapeToWorld.pos = B.ShapeToWorld.pos = Vector3(100.0f, 20.0f, -40.0f);
		EXPECT(Cache.Refresh(A, B, &Refreshed));
		EXPECT(Refreshed.NumContactPointCount == 2);
		EXPECT(Refreshed.ContactPoints[0].PersistentId == Id);
		EXPECT_NEAR(Refreshed.ContactPoints[0].PositionWorldA.x, 100.0f, NumericalTolerance);
		ContactManifold Reversed;
		EXPECT(Cache.Refresh(B, A, &Reversed));
		EXPECT(Reversed.ContactPoints[0].Normal.Dot(Refreshed.ContactPoints[0].Normal) < 0.0f);
		EXPECT((Reversed.ContactPoints[0].PositionWorldA - Refreshed.ContactPoints[0].PositionWorldB).Length() < NumericalTolerance);
		const Contact& Forward = Refreshed.ContactPoints[0];
		const Contact& Backward = Reversed.ContactPoints[0];
		EXPECT((Forward.Normal.Cross(Forward.Tangent) - Forward.Binormal).Length() < NumericalTolerance);
		EXPECT((Backward.Normal.Cross(Backward.Tangent) - Backward.Binormal).Length() < NumericalTolerance);
		const Vector3 ForwardImpulse = Forward.Normal * Forward.totalImpulseNormal
			+ Forward.Tangent * Forward.totalImpulseTangent + Forward.Binormal * Forward.totalImpulseBinormal;
		const Vector3 BackwardImpulse = Backward.Normal * Backward.totalImpulseNormal
			+ Backward.Tangent * Backward.totalImpulseTangent + Backward.Binormal * Backward.totalImpulseBinormal;
		EXPECT((ForwardImpulse + BackwardImpulse).Length() < NumericalTolerance);

		A.ShapeToWorld = B.ShapeToWorld = Transform::Identity();
		ContactManifold Fresh;
		Fresh.AddNewContact(0, 1, MakeContact(0.001f));
		Fresh.AddNewContact(0, 1, MakeContact(0.002f));
		EXPECT(Cache.Match(A, B, &Fresh, true) == 1);
		EXPECT(Fresh.ContactPoints[0].PersistentId == Id);
		EXPECT_NEAR(Fresh.ContactPoints[0].totalImpulseNormal, 2.0f, NumericalTolerance);
		Cache.Store(A, B, Fresh, TimeStep);
		EXPECT(Cache.Match(A, B, &Fresh, true) == 0);
		++A.Handle.Generation;
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		ShapeA->MarkCollisionDirty();
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeA->UpdateVolumeProperties();
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeB->SetTriggerEnabled(true);
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		ShapeB->SetPosition(Vector3::UnitX());
		EXPECT(!Cache.Refresh(A, B, &Refreshed));
		Cache.Store(A, B, M, TimeStep);
		Cache.Invalidate(ShapeB.get());
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		Cache.BeginFrame(2 + ContactPersistenceSettings().RetentionFrames + 1);
		EXPECT(Cache.GetPairCount() == 0);
		Cache.Store(A, B, M, TimeStep);
		ContactManifold Empty;
		Cache.Store(A, B, Empty, TimeStep);
		EXPECT(Cache.GetPairCount() == 0);

		ContactPersistenceSettings Invalid;
		Invalid.MatchDistance = -1.0f;
		bool Rejected = false;
		try { ContactManifoldManager InvalidCache(Invalid); }
		catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
	}

	void TestScaledBoxFeatures()
	{
		// Changing length units must not turn a face witness into an edge or corner.
		for (float Scale : { 0.0001f, 1.0f, 10000.0f })
		for (int Axis = 0; Axis < 3; ++Axis)
		for (float Sign : { -1.0f, 1.0f })
		for (bool Reverse : { false, true })
		for (bool Persistent : { false, true })
		{
			Vector3 Center = Vector3::Zero(); Center[Axis] = Sign * Scale;
			std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f * Scale)));
			std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Center, 0.6f * Scale));
			Box->SetContactOffsets(0.01f * Scale, 0);
			GeometryWorldState States[2];
			States[0].Geom = Reverse ? Sphere.get() : Box.get();
			States[1].Geom = Reverse ? Box.get() : Sphere.get();
			for (int Index = 0; Index < 2; ++Index)
			{
				States[Index].Handle = GeometryHandle(Index, 1);
				States[Index].ShapeToWorld = *States[Index].Geom->GetTransform();
				States[Index].WorldBounds = States[Index].Geom->GetBounds();
			}
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
			std::unique_ptr<NarrowPhase> Narrow(Persistent ? NarrowPhase::Create_PCM() : NarrowPhase::Create_GJKEPA());
			ContactGenerationSettings Settings; Settings.LengthScale = Scale;
			Narrow->SetContactGeneration(Settings, TimeStep); Narrow->BeginFrame(1);
			std::vector<ContactManifold*> Contacts;
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			EXPECT(!Contacts.empty());
			const uint32_t Face = 1u << (2 * Axis + (Sign > 0 ? 1 : 0));
			for (const auto* Manifold : Contacts)
			for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
			{
				const auto& Point = Manifold->ContactPoints[Index];
				EXPECT((Reverse ? Point.FeatureB : Point.FeatureA) == Face);
				EXPECT((Reverse ? Point.FeatureA : Point.FeatureB) == 0);
			}
		}
	}

	void TestSameFrameAccumulation()
	{
		std::unique_ptr<Geometry> ShapeA(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		std::unique_ptr<Geometry> ShapeB(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)));
		GeometryWorldState A, B;
		A.Geom = ShapeA.get(); B.Geom = ShapeB.get();
		ContactManifoldManager Cache;
		Cache.BeginFrame(1);
		ContactManifold First;
		First.AddNewContact(0, 1, MakeContact(0.0f));
		Cache.Store(A, B, First, TimeStep);
		ContactManifold Second;
		Second.AddNewContact(0, 1, MakeContact(0.001f));
		Second.AddNewContact(0, 1, MakeContact(0.002f));
		Cache.Store(A, B, Second, TimeStep, true, true);
		Cache.BeginFrame(2);
		EXPECT(Cache.Match(A, B, &Second, true) == 2);
		EXPECT_NEAR(Second.ContactPoints[0].totalImpulseNormal, 4.0f, NumericalTolerance);
		EXPECT_NEAR(Second.ContactPoints[1].totalImpulseNormal, 2.0f, NumericalTolerance);
		Cache.BeginFrame(1);
		EXPECT(Cache.GetPairCount() == 0);
	}

	void TestReductionAndImpulseRetraction()
	{
		ContactManifold M;
		for (int Index = 0; Index < 12; ++Index)
		{
			Contact C = MakeContact((float)Index);
			C.PenetrationDepth = (float)Index;
			M.AddNewContact(0, 1, C);
		}
		EXPECT(M.NumContactPointCount == MAX_CONTACT_POINTS);
		bool Deepest = false;
		for (int Index = 0; Index < M.NumContactPointCount; ++Index)
			Deepest = Deepest || M.ContactPoints[Index].PenetrationDepth == 11.0f;
		EXPECT(Deepest);

		RigidBodyParam StaticSettings, DynamicSettings;
		DynamicSettings.rigidType = RigidType::Dynamic;
		std::unique_ptr<RigidBody> A(RigidBody::CreateRigidBody(StaticSettings, Transform::Identity()));
		std::unique_ptr<RigidBody> B(RigidBody::CreateRigidBody(DynamicSettings, Transform::Identity()));
		GeneralizedVelocity Velocities[2] = {};
		Contact C = MakeContact(0.0f);
		C.PositionLocalA = C.PositionLocalB = Vector3::Zero();
		C.PenetrationDepth = 0.0f;
		C.CachedDt = TimeStep;
		ContactJacobianSolver Constraint(Velocities, 0, 1, A.get(), B.get());
		Constraint.SetupPositionPass(&C, TimeStep);
		WarmStart::Apply(Constraint, TimeStep, ContactPersistenceSettings());
		EXPECT(Constraint.m_jN.m_totalLambda > 0.0f);
		EXPECT(Velocities[1].v.y > 0.0f);
		Constraint.Solve();
		EXPECT_NEAR(Constraint.m_jN.m_totalLambda, 0.0f, NumericalTolerance);
		EXPECT(Velocities[1].v.Length() < NumericalTolerance);
	}

	void TestWarmWorldAndSwitches()
	{
		for (NarrowPhaseSolver Mode : { NarrowPhaseSolver::GJKEPA, NarrowPhaseSolver::PCM })
		{
			PhysicsWorldParam Settings;
			Settings.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
			Settings.narrowphase = Mode;
			PhysicsWorld World(Settings);
			RigidBodyParam StaticSettings, DynamicSettings;
			DynamicSettings.rigidType = RigidType::Dynamic;
			DynamicSettings.sleepThreshold = 0.0f;
			World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), StaticSettings);
			RigidBody* Ball = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0.0f, 0.49f, 0.0f), 0.5f), DynamicSettings);
			int WarmFrames = 0;
			for (int Frame = 0; Frame < 120; ++Frame)
			{
				World.Simulate();
				WarmFrames += World.GetWarmStartContactCount() > 0;
				EXPECT(std::isfinite(Ball->X.y) && Ball->X.y > 0.4f && Ball->X.y < 0.6f);
			}
			EXPECT(WarmFrames > 0);
			World.SetSequentialImpulseWarmStarting(false);
			World.Simulate();
			EXPECT(World.GetWarmStartContactCount() == 0);
			EXPECT(World.SetNarrowPhaseSolver(NarrowPhaseSolver::PCM));
			World.Simulate();
			EXPECT(World.GetCachedContactPairCount() > 0);
			EXPECT(World.RemoveRigidBody(Ball));
			Ball->ReleaseGeometries(); delete Ball;
			EXPECT(World.GetCachedContactPairCount() == 0);
			World.Reset();
			EXPECT(World.GetCachedContactPairCount() == 0);
		}
	}

}

void TestContactPersistence()
{
	TestAnchorCache();
	TestScaledBoxFeatures();
	TestSameFrameAccumulation();
	TestReductionAndImpulseRetraction();
	TestWarmWorldAndSwitches();

}

namespace Test_ContactOffsets_Cases
{
namespace
{
	using namespace Riemann;
	const float TimeStep = 1.0f / 60.0f;
	const float Tolerance = 1.0e-5f;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse,
		ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	GeometryWorldState State(Geometry* Shape)
	{
		GeometryWorldState Result;
		Result.Geom = Shape; Result.ShapeToWorld = *Shape->GetTransform(); Result.WorldBounds = Shape->GetBounds();
		return Result;
	}
	void TestSphereBoxWitnesses()
	{
		// Known face, edge/corner, embedded and separated witnesses. Rotate and
		// reverse each pair so this also checks the world-space normal convention.
		const Vector3 Centers[] = { Vector3(0.0001f, 0.999f, 0.0002f), Vector3(5.2f, 0.7f, 5.2f),
			Vector3(0), Vector3(-4.9f, 0.1f, 0), Vector3(0, 1.1f, 0) };
		const Vector3 Closest[] = { Vector3(0.0001f, 0.5f, 0.0002f), Vector3(5, 0.5f, 5),
			Vector3(0, 0.5f, 0), Vector3(-5, 0.1f, 0), Vector3(0, 0.5f, 0) };
		const Vector3 Normals[] = { Vector3::UnitY(), Vector3(1).Unit(), Vector3::UnitY(), -Vector3::UnitX(), Vector3::UnitY() };
		for (float Scale : { 0.001f, 1.0f, 1000.0f }) for (bool Reverse : { false, true }) for (bool Offsets : { false, true })
		for (size_t I = 0; I < sizeof(Centers) / sizeof(Centers[0]); ++I)
		{
			Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.7f);
			const Transform Frame(Vector3(2, -3, 1) * Scale, Rotation);
			const Vector3 Center = Frame.LocalToWorld(Centers[I] * Scale);
			std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Frame.pos, Vector3(5, 0.5f, 5) * Scale, Rotation));
			std::unique_ptr<Geometry> Sphere(GeometryFactory::CreateSphere(Center, 0.5f * Scale));
			Box->SetContactOffsets(0.15f * Scale, 0);
			ContactGenerationSettings Settings; Settings.LengthScale = Scale; Settings.UseShapeOffsets = Offsets;
			std::unique_ptr<NarrowPhase> Narrow(NarrowPhase::Create_GJKEPA()); Narrow->SetContactGeneration(Settings, TimeStep);
			GeometryWorldState States[] = { State(Reverse ? Sphere.get() : Box.get()), State(Reverse ? Box.get() : Sphere.get()) };
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
			std::vector<ContactManifold*> Contacts;
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			if (I == 4 && !Offsets) { EXPECT(Contacts.empty()); continue; }
			EXPECT(Contacts.size() == 1); if (Contacts.empty()) continue;
			EXPECT(Contacts[0]->NumContactPointCount == 1); const auto& Contact = Contacts[0]->ContactPoints[0];
			const Vector3 Normal = Frame.LocalToWorldDirection(Normals[I]);
			const Vector3 BoxPoint = Reverse ? Contact.PositionWorldB : Contact.PositionWorldA;
			const Vector3 SpherePoint = Reverse ? Contact.PositionWorldA : Contact.PositionWorldB;
			EXPECT((BoxPoint - Frame.LocalToWorld(Closest[I] * Scale)).Length() < Tolerance * Scale);
			EXPECT((SpherePoint - (Center - Normal * (0.5f * Scale))).Length() < Tolerance * Scale);
			EXPECT((Contact.Normal - Normal * (Reverse ? -1.0f : 1.0f)).Length() < Tolerance);
		}
	}
	void TestOffsetGeometry()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			std::unique_ptr<Geometry> A(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f * Scale)));
			std::unique_ptr<Geometry> B(GeometryFactory::CreateOBB(Vector3(1.05f * Scale, 0, 0), Vector3(0.5f * Scale)));
			const Box3 Original = A->GetBounds();
			A->SetContactOffsets(0.1f * Scale, 0.02f * Scale);
			B->SetContactOffsets(0.1f * Scale, 0.02f * Scale);
			EXPECT(A->GetBounds() == Original);
			ContactGenerationSettings Settings; Settings.LengthScale = Scale;
			std::unique_ptr<NarrowPhase> Narrow(NarrowPhase::Create_GJKEPA());
			Narrow->SetContactGeneration(Settings, TimeStep);
			GeometryWorldState States[] = { State(A.get()), State(B.get()) };
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
			std::vector<ContactManifold*> Contacts;
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			EXPECT(Contacts.size() == 1);
			if (!Contacts.empty())
			{
				EXPECT(Contacts[0]->NumContactPointCount == 4);
				for (int Index = 0; Index < Contacts[0]->NumContactPointCount; ++Index)
				{
					const Contact& C = Contacts[0]->ContactPoints[Index];
					EXPECT_NEAR(C.PenetrationDepth / Scale, -0.01f, Tolerance);
					EXPECT_NEAR((C.PositionWorldB - C.PositionWorldA).Dot(C.Normal) / Scale, 0.05f, Tolerance);
					EXPECT(C.SolverSlop >= 0.0f);
				}
			}
			B->SetPosition(Vector3(2 * Scale, 0, 0)); States[1] = State(B.get());
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			EXPECT(Contacts.empty());
			bool Rejected = false;
			try { A->SetContactOffsets(0.01f, 0.02f); }
			catch (const std::invalid_argument&) { Rejected = true; }
			EXPECT(Rejected);
			EXPECT_NEAR(A->GetContactOffset(), 0.1f * Scale, Tolerance);
		}
	}
	Geometry* MakeFloor(int Kind, float Scale)
	{
		if (Kind == 0) return GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
		if (Kind == 1) return GeometryFactory::CreateOBB(Vector3(0, -Scale, 0), Vector3(4, 1, 4) * Scale);
		if (Kind == 2)
		{
			Geometry* Shape = GeometryFactory::CreateTriangleMesh();
			TriangleMesh* Mesh = Shape->GetShapeObj<TriangleMesh>();
			Mesh->AddAABB(Vector3(-4, -2, -4) * Scale, Vector3(4, 0, 4) * Scale);
			Mesh->Compact(); Mesh->BuildBVH(); Shape->SetShapeBounds(Mesh->BoundingVolume);
			return Shape;
		}
		Geometry* Shape = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4) * Scale, Vector3(4, 1, 4) * Scale), 3, 3);
		HeightField3* Grid = Shape->GetShapeObj<HeightField3>();
		Grid->AllocMemory();
		for (int X = 0; X < 3; ++X) for (int Z = 0; Z < 3; ++Z) Grid->SetHeight(X, Z, 0.0f);
		Shape->UpdateVolumeProperties();
		return Shape;
	}
	Geometry* MakeShape(int Kind, const Vector3& Position, float Scale)
	{
		if (Kind == 0) return GeometryFactory::CreateSphere(Position, 0.5f * Scale);
		if (Kind == 1) return GeometryFactory::CreateOBB(Position, Vector3(0.5f * Scale));
		if (Kind == 2) return GeometryFactory::CreateCapsule(Position - Vector3::UnitX() * Scale, Position + Vector3::UnitX() * Scale, 0.5f * Scale);
		Vector3 Points[8]; Box3::GetVertices(Vector3(-0.5f * Scale), Vector3(0.5f * Scale), Points);
		Geometry* Shape = GeometryFactory::CreateConvexMesh();
		EXPECT(Shape->GetShapeObj<ConvexMesh>()->BuildFromPoints(Points, 8, Vector3::Zero(), Tolerance * Scale));
		Shape->UpdateVolumeProperties(); Shape->SetPosition(Position);
		return Shape;
	}
	void TestScaledShapePairs()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		for (int FloorKind = 0; FloorKind < 4; ++FloorKind)
		for (int ShapeKind = 0; ShapeKind < 4; ++ShapeKind)
		for (float Gap : { -0.05f, 0.0f, 0.05f, 0.25f })
		for (bool Reverse : { false, true })
		{
			std::unique_ptr<Geometry> Floor(MakeFloor(FloorKind, Scale));
			std::unique_ptr<Geometry> Shape(MakeShape(ShapeKind, Vector3(0, 0.5f + Gap, 0) * Scale, Scale));
			Floor->SetContactOffsets(0.1f * Scale, 0.02f * Scale);
			Shape->SetContactOffsets(0.1f * Scale, 0.02f * Scale);
			GeometryWorldState States[] = { State(Reverse ? Shape.get() : Floor.get()), State(Reverse ? Floor.get() : Shape.get()) };
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] };
			ContactGenerationSettings Settings; Settings.LengthScale = Scale;
			std::unique_ptr<NarrowPhase> Narrow(NarrowPhase::Create_GJKEPA());
			Narrow->SetContactGeneration(Settings, TimeStep);
			std::vector<ContactManifold*> Contacts;
			Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			bool Valid = (Gap > 0.2f) == Contacts.empty();
			for (const ContactManifold* Patch : Contacts)
			for (int Index = 0; Index < Patch->NumContactPointCount; ++Index)
			{
				const Contact& C = Patch->ContactPoints[Index];
				Valid = Valid && std::abs(C.PenetrationDepth / Scale - (0.04f - Gap)) < 0.003f
					&& C.Normal.y * (Reverse ? -1 : 1) > 0.99f;
			}
			if (!Valid) printf("Scaled shape failure scale=%g floor=%d shape=%d gap=%g reverse=%d patches=%zu depth=%g\n",
				Scale, FloorKind, ShapeKind, Gap, Reverse, Contacts.size(), Contacts.empty() ? 0 : Contacts[0]->ContactPoints[0].PenetrationDepth / Scale);
			EXPECT(Valid);
		}
	}
	void TestRestOffsets()
	{
		for (ResolutionPhaseSolver Solver : Solvers)
		for (SolverBackend Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader })
		for (float Rest : { -0.05f, 0.05f })
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
			PhysicsWorldParam Param;
			Param.gravityAcc = Vector3(0, -9.8f, 0);
			Param.resolutionSolver = Solver; Param.solverBackend = Backend; Param.allowBackendFallback = false;
			Param.narrowphase = NarrowPhaseSolver::PCM;
			PhysicsWorld World(Param);
			RigidBodyParam Static; Static.rigidType = RigidType::Static;
			Geometry* Floor = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
			Floor->SetContactOffsets(0.1f, Rest);
			World.CreateRigidBody(Floor, Static);
			Geometry* Sphere = GeometryFactory::CreateSphere(Vector3(0, 2, 0), 0.5f);
			Sphere->SetContactOffsets(0.1f, Rest);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.sleepThreshold = 0.0f;
			RigidBodyDynamic* Body = World.CreateRigidBody(Sphere, Dynamic)->CastDynamic();
			for (int Step = 0; Step < 180; ++Step) World.Simulate(TimeStep);
			printf("Rest offset solver=%d backend=%d rest=%g height=%g\n", (int)Solver, (int)Backend, Rest, Body->X.y);
			EXPECT_NEAR(Body->X.y, 0.5f + 2 * Rest, 0.008f);
			EXPECT(Body->V.Length() < 0.05f);
			Body->Sleep();
			Sphere->SetContactOffsets(0.1f, -Rest);
			World.Simulate(TimeStep);
			EXPECT(!Body->Sleeping);
			for (int Step = 0; Step < 90; ++Step) World.Simulate(TimeStep);
			EXPECT_NEAR(Body->X.y, 0.5f, 0.008f);
		}
	}
	void TestSurfaceRestOffsets()
	{
		for (ResolutionPhaseSolver Solver : Solvers)
		for (int FloorKind = 1; FloorKind < 4; ++FloorKind)
		for (int ShapeKind = 1; ShapeKind < 4; ++ShapeKind)
		for (float Rest : { -0.05f, 0.05f })
		{
			PhysicsWorldParam Param; Param.gravityAcc = Vector3(0, -9.8f, 0);
			Param.resolutionSolver = Solver; Param.narrowphase = NarrowPhaseSolver::PCM;
			Param.contactSolverIterations = 128; // Isolate gap accuracy from the low-iteration convergence tradeoff.
			PhysicsWorld World(Param);
			Geometry* Floor = MakeFloor(FloorKind, 1.0f); Floor->SetContactOffsets(0.1f, Rest);
			World.CreateRigidBody(Floor, RigidBodyParam());
			Geometry* Shape = MakeShape(ShapeKind, Vector3(0, 2, 0), 1.0f); Shape->SetContactOffsets(0.1f, Rest);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.sleepThreshold = 0.0f;
			RigidBody* Body = World.CreateRigidBody(Shape, Dynamic);
			for (int Step = 0; Step < 360; ++Step) World.Simulate(TimeStep);
			const float SurfaceY = Body->BodyLocalToWorld(Shape->GetSupport(Body->WorldToBodyLocalDirection(-Vector3::UnitY()))).y;
			const bool Valid = std::abs(SurfaceY - 2 * Rest) < 0.012f && std::abs(Body->V.y) < 0.1f;
			if (!Valid) printf("Surface rest failure solver=%d floor=%d shape=%d rest=%g y=%g surface=%g v=%g,%g,%g\n",
				(int)Solver, FloorKind, ShapeKind, Rest, Body->X.y, SurfaceY, Body->V.x, Body->V.y, Body->V.z);
			EXPECT(Valid);
		}
	}
	void TestOpenGapAndPolicies()
	{
		for (ResolutionPhaseSolver Solver : Solvers)
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Solver;
			PhysicsWorld World(Param);
			Geometry* Floor = MakeFloor(0, 1.0f); Floor->SetContactOffsets(1.0f, 0.0f);
			World.CreateRigidBody(Floor, RigidBodyParam())->SetRestitution(1.0f);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic;
			Dynamic.linearVelocity = Vector3(20, -1, 0); Dynamic.linearDamping = 1.0f; Dynamic.sleepThreshold = 0.0f;
			RigidBody* Body = World.CreateRigidBody(MakeShape(0, Vector3(0, 0.7f, 0), 1.0f), Dynamic);
			Body->SetRestitution(1.0f);
			for (int Step = 0; Step < 3; ++Step) World.Simulate(TimeStep);
			EXPECT_NEAR(Body->V.x, 20.0f, 0.001f);
			EXPECT_NEAR(Body->V.y, -1.0f, 0.001f);
			EXPECT_NEAR(Body->X.y, 0.65f, 0.001f);
		}
		for (bool Sensor : { false, true })
		for (ResolutionPhaseSolver Solver : Solvers)
		{
			int Events = 0;
			PhysicsWorldParam Param; Param.resolutionSolver = Solver; Param.contactGeneration.SpeculativeContacts = true;
			Param.collisionFilter.reset(CollisionFilter::CreateLayerMask());
			Param.onContact = [&](const PhysicsContactInfo&) { ++Events; };
			Param.onTrigger = [&](const PhysicsTriggerInfo&) { ++Events; };
			PhysicsWorld World(Param);
			Geometry* Wall = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 4, 4));
			Wall->SetContactOffsets(2.0f, 0.0f);
			if (Sensor) Wall->SetTriggerEnabled(true);
			else { CollisionData Data; Data.v1 = 0; Wall->SetFilterData(Data); }
			World.CreateRigidBody(Wall, RigidBodyParam());
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic; Dynamic.linearVelocity = Vector3(600, 0, 0);
			RigidBody* Body = World.CreateRigidBody(MakeShape(0, Vector3(-5, 0, 0), 1.0f), Dynamic);
			World.Simulate(TimeStep);
			EXPECT(Body->X.x > 1.0f && Events == 0);
		}
	}
	void TestSpeculativeRotation()
	{
		for (ResolutionPhaseSolver Solver : Solvers)
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Solver; Param.contactGeneration.SpeculativeContacts = true;
			PhysicsWorld World(Param);
			World.CreateRigidBody(MakeFloor(1, 1.0f), RigidBodyParam());
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic;
			Dynamic.angularVelocity = Vector3(0, 0, 60); Dynamic.maxContactImpulse = FLT_MAX;
			Geometry* Shape = GeometryFactory::CreateOBB(Vector3(0, 1, 0), Vector3(2, 0.1f, 0.1f));
			RigidBody* Body = World.CreateRigidBody(Shape, Dynamic);
			World.Simulate(TimeStep);
			const float Bottom = Body->GetGeometryBounds(Shape).Min.y;
			printf("Speculative rotation solver=%d bottom=%g angular=%g\n", (int)Solver, Bottom, Body->W.z);
			EXPECT(Bottom > -0.02f);
			EXPECT(Body->W.z < 50.0f);
			EXPECT(World.GetLastCCDHitCount() == 0);
		}
	}
	void TestSpeculativeMotion()
	{
		const BroadPhaseSolver BroadPhases[] = { BroadPhaseSolver::SAP, BroadPhaseSolver::ABP, BroadPhaseSolver::MBP,
			BroadPhaseSolver::AllPairs, BroadPhaseSolver::Bruteforce, BroadPhaseSolver::DynamicAABB };
		for (BroadPhaseSolver Broad : BroadPhases)
		for (ResolutionPhaseSolver Solver : Solvers)
		for (SolverBackend Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader })
		for (bool Enabled : { false, true })
		for (bool Mesh : { false, true })
		for (bool TriangleContacts : { false, true })
		for (float Start : { -5.0f, -0.55f })
		{
			if (!Mesh && !TriangleContacts) continue;
			if ((Broad != BroadPhaseSolver::SAP && Backend != SolverBackend::CPU) || !PhysicsWorld::IsSolverBackendAvailable(Solver, Backend) || (!Enabled && (Start > -1.0f || Backend != SolverBackend::CPU))) continue;
			PhysicsWorldParam Param;
			Param.broadphase = Broad; Param.resolutionSolver = Solver; Param.solverBackend = Backend; Param.allowBackendFallback = false;
			Param.contactGeneration.SpeculativeContacts = Enabled;
			Param.meshContactSettings.Enabled = TriangleContacts;
			PhysicsWorld World(Param);
			Geometry* Wall;
			if (Mesh)
			{
				Wall = GeometryFactory::CreateTriangleMesh();
				TriangleMesh* Surface = Wall->GetShapeObj<TriangleMesh>();
				Surface->AddAABB(Vector3(-0.05f, -4, -4), Vector3(0.05f, 4, 4));
				Surface->Compact(); Surface->BuildBVH(); Wall->SetShapeBounds(Surface->BoundingVolume);
			}
			else Wall = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 4, 4));
			RigidBodyParam Static; Static.rigidType = RigidType::Static;
			World.CreateRigidBody(Wall, Static);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic;
			Dynamic.linearVelocity = Vector3(600, 0, 0); Dynamic.sleepThreshold = 0.0f;
			Dynamic.maxContactImpulse = FLT_MAX;
			RigidBodyDynamic* Body = World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(Start, 0, 0), 0.5f), Dynamic)->CastDynamic();
			World.Simulate(TimeStep);
			printf("Speculative broad=%d solver=%d backend=%d enabled=%d mesh=%d triangle=%d start=%g x=%g v=%g\n", (int)Broad, (int)Solver, (int)Backend, Enabled, Mesh, TriangleContacts, Start, Body->X.x, Body->V.x);
			EXPECT(World.GetLastCCDHitCount() == 0);
			EXPECT(Enabled ? Body->X.x <= -0.54f && Body->X.x >= -0.57f : Body->X.x > 1.0f);
			if (Enabled)
			{
				for (int Step = 0; Step < 3; ++Step) World.Simulate(TimeStep);
				EXPECT(Body->X.x <= -0.53f);
				EXPECT(std::abs(Body->V.x) < 0.1f);
			}
		}
	}
}

void TestContactOffsets()
{
	printf("Running contact offsets, signed rest gaps, scaled proximity and speculative motion\n");
	TestOffsetGeometry();
	TestSphereBoxWitnesses();
	TestScaledShapePairs();
	TestRestOffsets();
	TestSurfaceRestOffsets();
	TestSpeculativeMotion();
	TestOpenGapAndPolicies();
	TestSpeculativeRotation();

}
}

void TestContactOffsets() { Test_ContactOffsets_Cases::TestContactOffsets(); }

namespace Test_MeshContacts_Cases
{
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
}

void TestMeshContacts() { Test_MeshContacts_Cases::TestMeshContacts(); }

namespace Test_FrictionPatch_Cases
{
namespace
{
	using namespace Riemann;
	const float Dt = 1.0f / 60.0f;
	const float DynamicFriction = 0.3f, StaticFriction = 0.8f;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	const SolverBackend Backends[] = { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA };

	RigidBodyDynamic* MakeBody(PhysicsWorld& World, Geometry* Shape)
	{
		RigidBodyParam Param; Param.rigidType = RigidType::Dynamic;
		Param.sleepThreshold = 0; Param.maxContactImpulse = 100;
		Param.linearDamping = Param.angularDamping = 1;
		auto* Body = World.CreateRigidBody(Shape, Param)->CastDynamic();
		Body->InvMass = 1; Body->InvInertia = Matrix3(0, 0, 0);
		Body->SetFriction(DynamicFriction, StaticFriction);
		return Body;
	}
	RigidBody* MakeFloor(PhysicsWorld& World, const Vector3& Normal = Vector3::UnitY())
	{
		auto* Body = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Normal), RigidBodyParam());
		Body->SetFriction(DynamicFriction, StaticFriction);
		return Body;
	}
	Contact Point(const Vector3& Position, float NormalImpulse)
	{
		Contact C;
		C.PositionWorldA = C.PositionWorldB = Position;
		C.Normal = Vector3::UnitY(); C.Tangent = Vector3::UnitX(); C.Binormal = -Vector3::UnitZ();
		C.totalImpulseNormal = NormalImpulse; C.SolverSlop = 0.001f;
		return C;
	}
	void TestAnchors()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Contact C = Point(Vector3::Zero(), 2);
		std::vector<FrictionContact> Inputs = { { Floor->mGeometries[0], Body->mGeometries[0], 0, &C } };
		Solver.BeginFrame(1); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		Body->V.x = 1.0f;
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 0.0f, 1e-5f);
		EXPECT_NEAR(std::abs(C.totalImpulseTangent), 1.0f, 1e-5f);
		EXPECT(Solver.GetStatistics().SlidingAnchors == 0);
		Body->V.x = 3.0f;
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 3.4f, 1e-5f); // Release the old 1 Ns, then apply the 0.6 Ns kinetic bound.
		EXPECT(Solver.GetStatistics().SlidingAnchors == 1);
		Solver.BeginFrame(2); Solver.BeginSolve(); Body->V = Vector3::Zero();
		Solver.Prepare(Inputs, Dt); Solver.Solve(Dt); Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		EXPECT_NEAR(Body->V.x, 0.0f, 1e-5f);
		EXPECT(Solver.GetStatistics().SlidingAnchors == 0);
		// Fresh narrow-phase points move with the body; the sticking anchor must not.
		Body->X.x = 0.001f; C.PositionWorldA.x = C.PositionWorldB.x = Body->X.x;
		Solver.BeginFrame(3); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt); Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		EXPECT_NEAR(Body->V.x, -Settings.PositionBias * Body->X.x / Dt, 1e-5f);
		// A contact gap cannot carry friction even when the normal solver anticipates impact.
		C.PenetrationDepth = -0.01f; Body->V.x = 10;
		Solver.BeginFrame(4); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt); Solver.Solve(Dt);
		EXPECT_NEAR(Body->V.x, 10, 1e-5f);
		EXPECT(C.totalImpulseTangent == 0 && C.totalImpulseBinormal == 0);
		C.PenetrationDepth = 0;
		Solver.BeginFrame(7); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 0);
		Body->mGeometries[0]->SetSimulationEnabled(false);
		Solver.BeginFrame(8); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().Patches == 0);
		Body->mGeometries[0]->SetSimulationEnabled(true);
		Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 0);
		Settings.PositionBias = std::numeric_limits<float>::quiet_NaN();
		EXPECT(!Settings.IsValid());
		bool Rejected = false;
		try { Body->SetFriction(1, 0); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
		Body->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		EXPECT(Body->GetFrictionDynamic() == 0 && Body->GetFrictionStatic() == 0);
	}

	void TestPatchMoment()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1, 0.5f, 0.5f)));
		Body->InvInertia = Matrix3(1, 1, 1); Body->W = Vector3(0, 3, 0);
		Contact A = Point(Vector3(-1, 0, 0), 1), B = Point(Vector3(1, 0, 0), 1);
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Solver.BeginFrame(1); Solver.BeginSolve();
		Solver.Prepare({ { Floor->mGeometries[0], Body->mGeometries[0], 0, &A },
			{ Floor->mGeometries[0], Body->mGeometries[0], 0, &B } }, Dt);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		EXPECT(Solver.GetStatistics().Patches == 1 && Solver.GetStatistics().Anchors == 2);
		EXPECT_NEAR(Body->W.y, 2.4f, 1e-4f);
		EXPECT(Body->V.Length() < 1e-4f);
		// A pinned center of mass can still rotate and must receive friction torque.
		Body->InvMass = 0; Body->W = Vector3(0, 3, 0); Body->V = Vector3::Zero();
		Solver.Reset(); Solver.BeginFrame(2); Solver.BeginSolve();
		Solver.Prepare({ { Floor->mGeometries[0], Body->mGeometries[0], 0, &A },
			{ Floor->mGeometries[0], Body->mGeometries[0], 0, &B } }, Dt);
		Solver.Solve(Dt);
		EXPECT_NEAR(Body->W.y, 2.4f, 1e-4f);
		EXPECT(Body->V.Length() < 1e-4f);
	}

	void TestSpatialPersistence()
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		auto* Floor = MakeFloor(World);
		auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
		Body->InvInertia = Matrix3(1, 2, 3);
		FrictionPatchSettings Settings; Settings.Enabled = true;
		FrictionPatchSolver Solver(Settings);
		Contact C = Point(Vector3(0.2f, 0, 0.1f), 2);
		std::vector<FrictionContact> Inputs = { { Floor->mGeometries[0], Body->mGeometries[0], 0, &C } };
		Body->V = Vector3(8, 0, 6);
		Solver.BeginFrame(1); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		const Vector3 Velocity = Body->V, Angular = Body->W;
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3).Unit(), 0.7f);
		const Vector3 Translation(4, -2, 7);
		Body->X = Rotation * Body->X + Translation; Floor->X = Translation;
		Body->Q = Floor->Q = Rotation; Body->V = Rotation * Vector3(8, 0, 6); Body->W = Vector3::Zero();
		C.PositionWorldA = C.PositionWorldB = Rotation * C.PositionWorldA + Translation;
		C.Normal = Rotation * C.Normal; C.Tangent = Rotation * C.Tangent; C.Binormal = Rotation * C.Binormal;
		Solver.BeginFrame(2); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
		for (int Iteration = 0; Iteration < 16; ++Iteration) Solver.Solve(Dt);
		EXPECT((Body->V - Rotation * Velocity).Length() < 1e-4f);
		EXPECT((Body->W - Rotation * Angular).Length() < 1e-4f);
		std::swap(Inputs[0].ShapeA, Inputs[0].ShapeB);
		C.Normal = -C.Normal; C.Tangent = -C.Tangent;
		Solver.BeginFrame(3); Solver.BeginSolve(); Solver.Prepare(Inputs, Dt);
		EXPECT(Solver.GetStatistics().RetainedAnchors == 1);
	}

	void TestSolverRoutes()
	{
		for (auto Type : Solvers)
		for (auto Backend : Backends)
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Type, Backend)) continue;
			for (bool Joined : { false, true })
			for (float Angle : { 0.0f, 0.4f, 0.785398163f, 1.3f })
			{
				PhysicsWorld World{ PhysicsWorldParam() };
				auto* Floor = MakeFloor(World);
				auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
				auto* Partner = Joined ? MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 1.5f, 0), 0.5f)) : nullptr;
				std::vector<PositionConstraint*> Joints;
				if (Partner) Joints.push_back(World.CreateFixedConstraint(Body, Partner));
				std::vector<Geometry*> Geometries = { Floor->mGeometries[0], Body->mGeometries[0] };
				Contact C = Point(Vector3::Zero(), 0); C.PositionLocalB = Vector3(0, -0.5f, 0);
				ContactManifold M; M.AddNewContact(0, 1, C); std::vector<ContactManifold*> Contacts = { &M };
				const Vector3 Direction(std::cos(Angle), 0, std::sin(Angle));
				FrictionPatchSettings Friction; Friction.Enabled = true;
				std::unique_ptr<ConstraintSolver> Native;
				std::unique_ptr<PBDSolver> Position;
				if (Type == ResolutionPhaseSolver::SequentialImpulse) Native.reset(ConstraintSolver::CreateSequentialImpulseSolver());
				else if (Type == ResolutionPhaseSolver::LCPGlobal)
				{
					LCPSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Native.reset(CreateGlobalLCPSolver(Settings));
				}
				else
				{
					XPBDSolverSettings Settings; Settings.Backend = Backend; Settings.AllowBackendFallback = false;
					Position.reset(Type == ResolutionPhaseSolver::PBD ? new PBDSolver(Settings) : new XPBDSolver(Settings));
				}
				if (Native) Native->SetFrictionPatchSettings(Friction);
				if (Position) Position->SetFrictionPatchSettings(Friction);
				auto Step = [&](uint64_t Frame)
				{
					if (Partner) Partner->V = Body->V;
					for (auto* Joint : Joints) Joint->BeginStep(Dt, Type == ResolutionPhaseSolver::XPBD);
					if (Native)
					{
						Native->BeginFrame(Frame); Native->PreResolve(Geometries, Joints);
						Native->ResolveConstraints(Geometries, Contacts, Joints, Dt, 32); Native->PostResolve(Geometries);
					}
					else
					{
						Position->BeginStep(Partner ? std::vector<RigidBodyDynamic*>{ Body, Partner } : std::vector<RigidBodyDynamic*>{ Body }, Dt);
						Position->AddContacts(Geometries, Contacts);
						for (int Iteration = 0; Iteration < 32; ++Iteration)
						{
							Position->SolveJointVelocities(Joints, Dt);
							Position->SolveVelocities(Dt);
						}
						Position->EndStep(Dt);
					}
					if (Partner) EXPECT((Body->V - Partner->V).Length() < 0.001f);
				};
				Body->V = Direction * 10.0f + Vector3(0, -2, 0);
				Step(1);
				EXPECT((Body->V - Direction * 9.4f).Length() < 0.001f);
				Body->V = Vector3(0, -2, 0); Step(2);
				Body->V = Direction * 0.8f + Vector3(0, -2, 0); Step(3);
				EXPECT(Body->V.Length() < 0.001f); // Above the kinetic bound, below the static bound.
				Body->V = Direction * 3.0f + Vector3(0, -2, 0); Step(4);
				EXPECT((Body->V - Direction * 2.4f).Length() < 0.001f);
			}
			printf("Friction route solver=%d backend=%d: rotation, static load, slip and restick checked\n", (int)Type, (int)Backend);
		}
	}

	void TestSlopeAndDemos()
	{
		{
			PhysicsWorldParam Param; Param.resolutionSolver = ResolutionPhaseSolver::XPBD;
			Param.gravityAcc = Vector3(0, -10, 0); Param.frictionPatches.Enabled = true;
			Param.xpbdSettings.ContactCompliance = 0.001f;
			PhysicsWorld World(Param); MakeFloor(World);
			auto* Body = MakeBody(World, GeometryFactory::CreateSphere(Vector3(0, 0.5f, 0), 0.5f));
			for (int Frame = 0; Frame < 120; ++Frame) World.Simulate(Dt);
			EXPECT_NEAR(Body->X.y, 0.5f - Param.xpbdSettings.PenetrationSlop - 10.0f * Param.xpbdSettings.ContactCompliance, 0.001f);
		}
		for (auto Type : Solvers)
		for (float Angle : { 0.45f, 0.8f })
		{
			PhysicsWorldParam Param; Param.resolutionSolver = Type; Param.gravityAcc = Vector3(0, -10, 0);
			Param.frictionPatches.Enabled = true;
			float ReportedNormal = 0.0f;
			Param.onContact = [&](const PhysicsContactInfo& Info)
			{
				for (int Index = 0; Index < Info.NumContactPoints; ++Index) ReportedNormal += Info.ContactPoints[Index].NormalImpulse;
			};
			PhysicsWorld World(Param);
			const Vector3 Normal(-std::sin(Angle), std::cos(Angle), 0), Tangent(std::cos(Angle), std::sin(Angle), 0);
			MakeFloor(World, Normal);
			auto* Body = MakeBody(World, GeometryFactory::CreateOBB(Normal * 0.5f, Vector3(0.5f, 0.5f, 0.5f)));
			Body->Q.FromRotationAxis(Vector3::UnitZ(), Angle);
			for (int Frame = 0; Frame < 120; ++Frame) { ReportedNormal = 0; World.Simulate(Dt); }
			printf("Friction slope solver=%d angle=%g drift=%g speed=%g\n", (int)Type, Angle, Body->X.Dot(Tangent), Body->V.Dot(Tangent));
			if (Angle < 0.5f)
			{
				EXPECT(std::abs(Body->X.Dot(Tangent)) < 0.02f);
				EXPECT(std::abs(Body->V.Dot(Tangent)) < 0.02f);
				EXPECT_NEAR(ReportedNormal, 10.0f * std::cos(Angle) * Dt, 0.002f);
				// Static anchors must stop continuing creep, not merely slow it down.
				const Vector3 Settled = Body->X;
				for (int Frame = 0; Frame < 120; ++Frame) World.Simulate(Dt);
				EXPECT(std::abs((Body->X - Settled).Dot(Tangent)) < 0.001f);
			}
			else EXPECT(Body->V.Dot(Tangent) < -1.0f);
		}
}
}

void TestFrictionPatches()
{
	TestAnchors(); TestPatchMoment(); TestSpatialPersistence(); TestSolverRoutes(); TestSlopeAndDemos();
}
}

void TestFrictionPatches() { Test_FrictionPatch_Cases::TestFrictionPatches(); }

namespace Test_CollisionFiltering_Cases
{
namespace
{
	using namespace Riemann;
	const float TimeStep = 1.0f / 60.0f;
	const float Tolerance = 1.0e-5f;
	const uint32_t QueryLayer = 0;
	const uint32_t IgnoredLayer = 1;
	const uint32_t SolidLayer = 2;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse,
		ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };

	std::shared_ptr<CollisionFilter> LayerPolicy()
	{
		return std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask());
	}

	RigidBody* AddBox(PhysicsWorld& World, const Vector3& Center, bool Dynamic, uint32_t Layer, uint32_t Mask = UINT32_MAX)
	{
		Geometry* Shape = GeometryFactory::CreateOBB(Center, Vector3(0.5f));
		Shape->SetFilterData(CollisionData::ForLayer(Layer, Mask));
		RigidBodyParam Param;
		Param.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		Param.sleepThreshold = 0.0f;
		return World.CreateRigidBody(Shape, Param);
	}

	void TestPolicyContracts()
	{
		auto Filter = LayerPolicy();
		CollisionData Default;
		EXPECT(Filter->IsCollidable(Default, Default));
		const CollisionData Last = CollisionData::ForLayer(CollisionData::LayerCount - 1);
		EXPECT(Filter->IsCollidable(Default, Last));
		EXPECT(!Filter->IsCollidable(Default, CollisionData::ForLayer(CollisionData::LayerCount)));
		EXPECT(!Filter->IsCollidable(CollisionData::ForLayer(UINT32_MAX), Default));
		CollisionData Limited = CollisionData::ForLayer(QueryLayer, 1u << SolidLayer);
		EXPECT(Filter->IsCollidable(Limited, CollisionData::ForLayer(SolidLayer)));
		EXPECT(!Filter->IsCollidable(Limited, CollisionData::ForLayer(IgnoredLayer)));
		EXPECT(!Filter->IsCollidable(Limited, CollisionData::ForLayer(SolidLayer, 0)));
		EXPECT(!Filter->IsCollidable(CollisionData::ForLayer(SolidLayer, 0), Limited));

		unsigned char Table[] = { 1, 0, 0, 1 };
		std::unique_ptr<CollisionFilter> Legacy(CollisionFilter::CreateCollisionTable(2, Table));
		EXPECT(Legacy->IsCollidable(Default, CollisionData::ForLayer(1)));
		EXPECT(!Legacy->IsCollidable(Default, CollisionData::ForLayer(2)));
		bool Rejected = false;
		try { std::unique_ptr<CollisionFilter> Invalid(CollisionFilter::CreateCollisionTable(0, nullptr)); }
		catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
		Rejected = false;
		try { std::unique_ptr<CollisionFilter> Invalid(CollisionFilter::CreateCallback({})); }
		catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
	}

	void TestQueryParity()
	{
		for (SceneQueryType Type : { SceneQueryType::StaticAABB, SceneQueryType::DynamicAABB })
		{
			PhysicsWorldParam Param;
			Param.sceneQueryType = Type;
			Param.collisionFilter = LayerPolicy();
			PhysicsWorld World(Param);
			Geometry* Ignored = AddBox(World, Vector3(2, 0, 0), false, IgnoredLayer, 0)->mGeometries[0];
			RigidBody* NearBody = AddBox(World, Vector3(4, 0, 0), true, SolidLayer);
			Geometry* Near = NearBody->mGeometries[0];
			Geometry* Far = AddBox(World, Vector3(6, 0, 0), false, SolidLayer)->mGeometries[0];
			RigidBody* Source = AddBox(World, Vector3(-6, 0, 0), true, QueryLayer);
			GeometryQuery* Query = World.GetGeometryQuery();
			RayCastOption Ray;
			Ray.MaxDist = 10.0f;
			RayCastResult RayResult;
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Near);
			EXPECT_NEAR(RayResult.hitTimeMin, 3.5f, Tolerance);

			const int BoxCount = GeometryFactory::ObjectCount[(int)PrimitiveType::BOX];
			const int SphereCount = GeometryFactory::ObjectCount[(int)PrimitiveType::SPHERE];
			const int CapsuleCount = GeometryFactory::ObjectCount[(int)PrimitiveType::CAPSULE];
			SweepOption Sweep;
			Sweep.MaxDist = Ray.MaxDist;
			SweepResult Hit;
			EXPECT(Query->BoxCastQuery(Vector3::Zero(), Vector3(0.25f), Vector3::UnitX(), Sweep, &Hit));
			EXPECT(Hit.hitGeom == Near);
			EXPECT_NEAR(Hit.hitTimeMin, 3.25f, Tolerance);
			EXPECT_NEAR(Hit.hitPosition.x, 3.5f, Tolerance);
			EXPECT(Hit.hitNormal.Dot(-Vector3::UnitX()) > 0.99f);
			EXPECT(Query->SphereCastQuery(Vector3::Zero(), 0.25f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT(Hit.hitGeom == Near);
			EXPECT(Query->SphereCastQuery(Vector3::UnitX(), 0.25f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT_NEAR(Hit.hitTimeMin, 2.25f, Tolerance);
			EXPECT_NEAR(Hit.hitPosition.x, 3.5f, Tolerance);
			EXPECT(Query->CapsuleCastQuery(Vector3::Zero(), 0.25f, 0.25f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT(Hit.hitGeom == Near);
			Sweep.MaxDist = 1.0f;
			EXPECT(!Query->BoxCastQuery(Vector3::Zero(), Vector3(0.25f), Vector3::UnitX(), Sweep, &Hit));
			Sweep.MaxDist = Ray.MaxDist;
			Sweep.Type = SweepOption::SWEEP_PENETRATE;
			EXPECT(Query->BoxCastQuery(Vector3::Zero(), Vector3(0.25f), Vector3::UnitX(), Sweep, &Hit));
			EXPECT(Hit.hitGeom == Near && Hit.hitGeometries.size() == 2);
			EXPECT(std::find(Hit.hitGeometries.begin(), Hit.hitGeometries.end(), Ignored) == Hit.hitGeometries.end());

			IntersectOption Overlap;
			Overlap.maxOverlaps = 10;
			IntersectResult Overlaps;
			EXPECT(Query->IntersectQueryBox(Vector3(4, 0, 0), Vector3(4), Overlap, &Overlaps));
			EXPECT(Overlaps.overlaps && Overlaps.overlapGeoms.size() == 2);
			EXPECT(Query->IntersectQuerySphere(Vector3(4, 0, 0), 4.0f, Overlap, &Overlaps));
			EXPECT(Overlaps.overlapGeoms.size() == 2);
			EXPECT(Query->IntersectQueryCapsule(Vector3(4, -1, 0), Vector3(4, 1, 0), 4.0f, Overlap, &Overlaps));
			EXPECT(Overlaps.overlapGeoms.size() == 2);
			EXPECT(GeometryFactory::ObjectCount[(int)PrimitiveType::BOX] == BoxCount);
			EXPECT(GeometryFactory::ObjectCount[(int)PrimitiveType::SPHERE] == SphereCount);
			EXPECT(GeometryFactory::ObjectCount[(int)PrimitiveType::CAPSULE] == CapsuleCount);

			auto Deny = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateCallback(
				[](const CollisionData&, const CollisionData&) { return false; }));
			Ray.Filter = Sweep.Filter = Overlap.Filter = Deny.get();
			EXPECT(!Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(!Query->SphereCastQuery(Vector3::Zero(), 0.25f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT(!Query->IntersectQuerySphere(Vector3(4, 0, 0), 4.0f, Overlap, &Overlaps));
			EXPECT(!Overlaps.overlaps && Overlaps.overlapGeoms.empty());
			Ray.Filter = Sweep.Filter = Overlap.Filter = nullptr;
			auto Allow = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateCallback(
				[](const CollisionData&, const CollisionData&) { return true; }));
			Ray.Filter = Allow.get();
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Near);
			Ray.FilterData = CollisionData::ForLayer(QueryLayer, 0);
			EXPECT(!Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			Ray.Filter = nullptr;
			Ray.FilterData = CollisionData();
			Ray.SourceBody = Sweep.SourceBody = Overlap.SourceBody = Source;
			Source->IgnoreCollisionWith(NearBody);
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Far);
			EXPECT(Query->SphereCastQuery(Vector3::Zero(), 0.25f, Vector3::UnitX(), Sweep, &Hit));
			EXPECT(Hit.hitGeom == Far);
			EXPECT(Query->IntersectQuerySphere(Vector3(4, 0, 0), 4.0f, Overlap, &Overlaps));
			EXPECT(Overlaps.overlapGeoms.size() == 1 && Overlaps.overlapGeoms[0] == Far);
			Source->IgnoreCollisionWith(NearBody, false);
			Source->CollisionGroup = NearBody->CollisionGroup = Source;
			Source->EnableCollisionWithinGroup = false;
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Far);
			Source->CollisionGroup = NearBody->CollisionGroup = nullptr;
			Near->SetQueryEnabled(false);
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Far);
			Near->SetQueryEnabled(true);
			World.SetCollisionFilteringEnabled(false);
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Ignored);
			World.SetCollisionFilteringEnabled(true);
			// Live shape edits are visible to queries before the next simulation step.
			Ignored->SetFilterData(CollisionData::ForLayer(IgnoredLayer));
			EXPECT(Query->RayCastQuery(Vector3::Zero(), Vector3::UnitX(), Ray, &RayResult));
			EXPECT(RayResult.hitGeom == Ignored);
		}
	}

	void TestDiscreteFiltering()
	{
		const BroadPhaseSolver BroadPhases[] = { BroadPhaseSolver::SAP, BroadPhaseSolver::ABP,
			BroadPhaseSolver::MBP, BroadPhaseSolver::AllPairs, BroadPhaseSolver::Bruteforce, BroadPhaseSolver::DynamicAABB };
		for (BroadPhaseSolver Broad : BroadPhases)
		for (ResolutionPhaseSolver Solver : Solvers)
		for (NarrowPhaseSolver Narrow : { NarrowPhaseSolver::GJKEPA, NarrowPhaseSolver::PCM })
		{
			int Contacts = 0;
			PhysicsWorldParam Param;
			Param.broadphase = Broad;
			Param.resolutionSolver = Solver;
			Param.narrowphase = Narrow;
			Param.collisionFilter = LayerPolicy();
			Param.onContact = [&](const PhysicsContactInfo&) { ++Contacts; };
			PhysicsWorld World(Param);
			AddBox(World, Vector3::Zero(), false, SolidLayer);
			RigidBodyDynamic* Body = AddBox(World, Vector3(0.8f, 0, 0), true, QueryLayer, 0)->CastDynamic();
			Geometry* Shape = Body->mGeometries[0];
			World.Simulate(TimeStep);
			EXPECT(Contacts == 0);
			EXPECT_NEAR(Body->X.x, 0.8f, Tolerance);
			Shape->SetFilterData(CollisionData::ForLayer(QueryLayer));
			World.Simulate(TimeStep);
			EXPECT(Contacts > 0);
			Body->Sleep();
			Contacts = 0;
			Shape->SetFilterData(CollisionData::ForLayer(QueryLayer, 0));
			const Vector3 Rest = Body->X;
			World.Simulate(TimeStep);
			EXPECT(Contacts == 0 && !Body->Sleeping);
			EXPECT((Body->X - Rest).Length() < Tolerance);
			EXPECT(Body->V.Length() < Tolerance);
		}
	}

	void TestCCDAndTriggers()
	{
		for (ResolutionPhaseSolver Solver : Solvers)
		for (bool Allowed : { false, true })
		for (bool Angular : { false, true })
		{
			PhysicsWorldParam Param;
			Param.resolutionSolver = Solver;
			Param.collisionFilter = LayerPolicy();
			PhysicsWorld World(Param);
			RigidBodyParam Static;
			Static.rigidType = RigidType::Static;
			Geometry* Wall = GeometryFactory::CreatePlane(Vector3::Zero(), -Vector3::UnitX());
			Wall->SetFilterData(CollisionData::ForLayer(SolidLayer, Allowed ? UINT32_MAX : 0));
			World.CreateRigidBody(Wall, Static);
			RigidBodyParam Moving;
			Moving.rigidType = RigidType::Dynamic;
			Moving.motionType = MorionType::Continuous;
			Moving.sleepThreshold = 0.0f;
			Moving.angularDamping = 1.0f;
			Geometry* Shape;
			if (Angular)
			{
				Moving.angularVelocity = Vector3(0, 0, 120);
				Shape = GeometryFactory::CreateCapsule(Vector3(-0.5f, -1, 0), Vector3(-0.5f, 1, 0), 0.1f);
			}
			else
			{
				Moving.linearVelocity = Vector3(600, 0, 0);
				Shape = GeometryFactory::CreateSphere(Vector3(-5, 0, 0), 0.5f);
			}
			RigidBodyDynamic* Body = World.CreateRigidBody(Shape, Moving)->CastDynamic();
			World.Simulate(TimeStep);
			EXPECT((World.GetLastCCDHitCount() > 0) == Allowed);
			EXPECT(Body->HadCCDContact == Allowed);
			if (!Angular) EXPECT(Allowed ? Body->X.x < 0 : Body->X.x > 1);
			if (Angular && !Allowed) EXPECT_NEAR(Body->W.z, 120.0f, Tolerance);
		}

		for (ResolutionPhaseSolver Solver : Solvers)
		for (bool Continuous : { false, true })
		for (bool Allowed : { false, true })
		{
			int Triggers = 0;
			PhysicsWorldParam Param;
			Param.resolutionSolver = Solver;
			Param.collisionFilter = LayerPolicy();
			Param.onTrigger = [&](const PhysicsTriggerInfo& Info) { ++Triggers; EXPECT(Info.IsCCD == Continuous); };
			PhysicsWorld World(Param);
			Geometry* Sensor = AddBox(World, Vector3::Zero(), false, SolidLayer, Allowed ? UINT32_MAX : 0)->mGeometries[0];
			Sensor->SetTriggerEnabled(true);
			Sensor->SetSimulationEnabled(false);
			Sensor->SetTriggerCCDEnabled(true);
			RigidBodyParam Moving;
			Moving.rigidType = RigidType::Dynamic;
			Moving.motionType = Continuous ? MorionType::Continuous : MorionType::Discrete;
			Moving.linearVelocity = Continuous ? Vector3(600, 0, 0) : Vector3::Zero();
			World.CreateRigidBody(GeometryFactory::CreateSphere(Continuous ? Vector3(-5, 0, 0) : Vector3::Zero(), 0.25f), Moving);
			World.Simulate(TimeStep);
			EXPECT((Triggers > 0) == Allowed);
		}
	}

	void TestCompoundCCDAndTransformedSweep()
	{
		PhysicsWorldParam Param;
		Param.collisionFilter = LayerPolicy();
		PhysicsWorld World(Param);
		RigidBodyParam Static;
		Static.rigidType = RigidType::Static;
		Geometry* Wall = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 4, 4));
		World.CreateRigidBody(Wall, Static);
		Geometry* Front = GeometryFactory::CreateSphere(Vector3(-4, 0, 0), 0.25f);
		Geometry* Back = GeometryFactory::CreateSphere(Vector3(-6, 0, 0), 0.25f);
		Front->SetFilterData(CollisionData::ForLayer(IgnoredLayer, 0));
		RigidBodyParam Moving;
		Moving.rigidType = RigidType::Dynamic;
		Moving.linearVelocity = Vector3(600, 0, 0);
		Moving.motionType = MorionType::Continuous;
		RigidBodyDynamic* Body = World.CreateRigidBody({ Front, Back }, Moving, Transform(Vector3(-5, 0, 0)))->CastDynamic();
		World.Simulate(TimeStep);
		EXPECT(World.GetLastCCDHitCount() == 1);
		EXPECT(Body->GetGeometryTransform(Back).pos.x < -0.25f);
		EXPECT(Body->GetGeometryTransform(Front).pos.x > 1.0f);

		Quaternion Rotation;
		Rotation.FromRotationAxis(Vector3::UnitY(), 0.4f);
		const Vector3 Origin(3, 4, 5);
		std::unique_ptr<Geometry> Cast(GeometryFactory::CreateOBB(Origin, Vector3(0.25f), Rotation));
		std::unique_ptr<Geometry> Target(GeometryFactory::CreateOBB(Origin + Rotation * Vector3(4, 0, 0), Vector3(0.5f), Rotation));
		Vector3 Position, Normal;
		float Time;
		EXPECT(Cast->Sweep(Vector3::UnitX(), Target.get(), &Position, &Normal, &Time));
		EXPECT_NEAR(Time, 3.25f, Tolerance);
		EXPECT_NEAR(Position.x, 3.5f, Tolerance);
		EXPECT(Normal.Dot(-Vector3::UnitX()) > 0.99f);
		EXPECT(!Cast->Sweep(-Vector3::UnitX(), Target.get(), &Position, &Normal, &Time));
	}

	void TestCallbackAndCompound()
	{
		bool Enabled = false;
		int Contacts = 0;
		Geometry* Accepted = nullptr;
		PhysicsWorldParam Param;
		Param.collisionFilter.reset(CollisionFilter::CreateCallback([&](const CollisionData& A, const CollisionData& B)
		{
			return Enabled && A.v2 != IgnoredLayer && B.v2 != IgnoredLayer;
		}));
		Param.onContact = [&](const PhysicsContactInfo& Info)
		{
			++Contacts;
			EXPECT(Info.GeomA == Accepted || Info.GeomB == Accepted);
		};
		PhysicsWorld World(Param);
		RigidBodyParam Static;
		Static.rigidType = RigidType::Static;
		World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), Static);
		Geometry* Ignored = GeometryFactory::CreateSphere(Vector3(-1, 0.2f, 0), 0.3f);
		CollisionData Data; Data.v2 = IgnoredLayer;
		Ignored->SetFilterData(Data);
		Accepted = GeometryFactory::CreateSphere(Vector3(1, 0.2f, 0), 0.3f);
		RigidBodyParam Dynamic;
		Dynamic.rigidType = RigidType::Dynamic;
		Dynamic.sleepThreshold = 0.0f;
		RigidBodyDynamic* Body = World.CreateRigidBody({ Ignored, Accepted }, Dynamic, Transform(Vector3(0, 0.2f, 0)))->CastDynamic();
		World.Simulate(TimeStep);
		EXPECT(Contacts == 0);
		Body->Sleep();
		Enabled = true;
		World.RefreshCollisionFiltering();
		EXPECT(!Body->Sleeping);
		World.Simulate(TimeStep);
		EXPECT(Contacts > 0);
	}

}

void TestCollisionFiltering()
{
	printf("Running shared collision filter policies, queries, contacts, CCD, triggers and demo\n");
	TestPolicyContracts();
	TestQueryParity();
	TestDiscreteFiltering();
	TestCCDAndTriggers();
	TestCompoundCCDAndTransformedSweep();
	TestCallbackAndCompound();

}
}

void TestCollisionFiltering() { Test_CollisionFiltering_Cases::TestCollisionFiltering(); }

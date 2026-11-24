#include "Test.h"
#include <memory>
#include <stdexcept>
#include <limits>
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"

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
	void TestConfigurationAndDemo()
	{
		std::string Error;
		for (const char* Invalid : { R"("contactOffset":-1)", R"("contactOffset":0.1,"restOffset":0.2)", R"("restOffset":"zero")" })
		{
			SceneWorld Scene;
			EXPECT(!Scene.LoadFromText(std::string(R"({"objects":[{"type":"box",)") + Invalid + "}]}", "invalid offset", &Error));
			EXPECT(Error.find("contactOffset") != std::string::npos);
		}
		for (const char* Invalid : { R"("lengthScale":0)", R"("minimumTolerance":-1)", R"("speculativeContacts":1)",
			R"("distanceIterations":1.5)", R"("distanceIterations":0)", R"("relativePenetrationSlop":-0.1)" })
		{
			SceneWorld Scene;
			EXPECT(!Scene.LoadFromText(std::string(R"({"physics":{"contactGeneration":{)") + Invalid + "}}}", "invalid settings", &Error));
			EXPECT(Error.find("contactGeneration") != std::string::npos);
		}
		PhysicsWorld World{PhysicsWorldParam()};
		ContactGenerationSettings Invalid; Invalid.LengthScale = std::numeric_limits<float>::infinity();
		bool Rejected = false;
		try { World.SetContactGenerationSettings(Invalid); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected && World.GetContactGenerationSettings().LengthScale == 1.0f);
		for (ResolutionPhaseSolver Solver : Solvers)
		{
			SceneWorld Scene;
			EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/01_stack_boxes.json", &Error));
			PhysicsWorld* Simulation = Scene.GetSimulation();
			if (!Simulation) { EXPECT(false); continue; }
			EXPECT(Simulation->SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			ContactGenerationSettings Settings = Simulation->GetContactGenerationSettings();
			EXPECT(!Settings.UseShapeOffsets && !Settings.SpeculativeContacts);
			Settings.UseShapeOffsets = true; Settings.SpeculativeContacts = true;
			Simulation->SetContactGenerationSettings(Settings);
			for (int Step = 0; Step < 240; ++Step) Scene.Step(TimeStep);
			std::vector<Geometry*> Shapes; Simulation->GetAllGeometries(&Shapes);
			int Count = 0;
			for (Geometry* Shape : Shapes)
				if (RigidBodyDynamic* Body = Shape->GetParent<RigidBody>()->CastDynamic())
				{
					++Count;
					EXPECT_NEAR(Shape->GetContactOffset(), 0.03f, Tolerance);
					EXPECT_NEAR(Shape->GetRestOffset(), 0.01f, Tolerance);
					EXPECT(std::isfinite(Body->X.y) && Body->X.y > -4.1f && Body->X.y < 4.3f);
				}
			EXPECT(Count == 5);
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
	TestConfigurationAndDemo();
}

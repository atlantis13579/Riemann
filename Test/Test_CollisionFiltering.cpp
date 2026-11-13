#include "Test.h"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Viewer/SceneWorld.h"

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

	void TestExistingStackDemo()
	{
		SceneWorld Scene;
		std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/02_stack_boxes.json", &Error));
		PhysicsWorld* World = Scene.GetSimulation();
		EXPECT(World && !World->GetCollisionFilteringEnabled());
		if (!World) return;
		std::vector<Geometry*> Shapes;
		World->GetAllGeometries(&Shapes);
		int Boxes = 0;
		for (Geometry* Shape : Shapes)
			if (Shape->GetParent<RigidBody>()->CastDynamic())
			{
				++Boxes;
				EXPECT(Shape->GetFilterData().v0 == IgnoredLayer);
				EXPECT((Shape->GetFilterData().v1 & (1u << IgnoredLayer)) == 0);
			}
		EXPECT(Boxes == 125);
		World->SetCollisionFilteringEnabled(true);
		for (int Step = 0; Step < 150; ++Step) Scene.Step(TimeStep);
		for (Geometry* Shape : Shapes)
			if (RigidBodyDynamic* Body = Shape->GetParent<RigidBody>()->CastDynamic())
				EXPECT(Body->X.y > -4.2f && Body->X.y < -3.9f);

		for (const char* Invalid : { "-1", "32", "1.5", "4294967296", "\"one\"" })
		{
			SceneWorld InvalidScene;
			const std::string Text = std::string("{\"objects\":[{\"type\":\"box\",\"collisionLayer\":") + Invalid + "}]}";
			EXPECT(!InvalidScene.LoadFromText(Text, "invalid layer", &Error));
			EXPECT(Error.find("collisionLayer") != std::string::npos);
		}
		SceneWorld FractureScene;
		EXPECT(FractureScene.LoadFromText(R"({"physics":{"collisionFiltering":true},"objects":[
			{"type":"destructionBox","pieces":[2,1,1],"collisionLayer":2,"collisionMask":0}
		]})", "filtered fracture", &Error));
		Shapes.clear();
		FractureScene.GetSimulation()->GetAllGeometries(&Shapes);
		EXPECT(!Shapes.empty());
		DestructionSet* Set = nullptr;
		for (Geometry* Shape : Shapes)
		{
			EXPECT(Shape->GetFilterData().v0 == SolidLayer && Shape->GetFilterData().v1 == 0);
			RigidBodyDynamic* Body = Shape->GetParent<RigidBody>()->CastDynamic();
			if (Body) Set = static_cast<DestructionSet*>(Body->Parent);
		}
		EXPECT(Set != nullptr);
		if (Set)
		{
			// A cluster rebuild replaces its proxy; the replacement inherits the same policy.
			Set->RebuildClustersFromGraph();
			Shapes.clear();
			FractureScene.GetSimulation()->GetAllGeometries(&Shapes);
			for (Geometry* Replacement : Shapes)
				EXPECT(Replacement->GetFilterData().v0 == SolidLayer && Replacement->GetFilterData().v1 == 0);
		}
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
	TestExistingStackDemo();
}

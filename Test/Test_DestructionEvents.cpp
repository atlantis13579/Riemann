#include "Test.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Destruction/StrainSolver.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	void Author(DestructionSet& Set, int Count = 3, bool Support = false, float Strength = 1)
	{
		Set.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < Count; ++I)
		{
			const Vector3 Center((float)I,0,0); Set.AddChunk(Box3(Center-Vector3(0.5f),Center+Vector3(0.5f)),(float)(I+1),Support && I == 0);
			if (I) EXPECT(Set.AddBond(I-1,I,DestructionBondType::Connect,Strength,10+I));
		}
		Set.RebuildClustersFromGraph();
	}
	void Enable(DestructionSet& Set)
	{ DestructionEventSettings S; S.Enabled = true; Set.ConfigureEvents(S); }
	const DestructionEvent* Find(const std::vector<DestructionEvent>& Events, DestructionEventKind Kind)
	{ for (const auto& E : Events) if (E.Kind == Kind) return &E; return nullptr; }
	void IdentityAndKinematics()
	{
		DestructionSet Source; Author(Source); const auto Asset = DestructionAsset::Capture(Source);
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Set.InitializeAsset(Asset); Enable(Set);
		const auto Id = Set.GetInstanceId(), Generation = Set.GetGeneration();
		auto* Cluster = Set.FindClusterContaining(0); const auto ClusterId = Cluster->GetID(); auto* Body = Cluster->GetRigidBody();
		Body->X = Vector3(5,3,-2); Body->Q.FromRotationAxis(Vector3(1,2,3).Unit(),0.7f);
		Body->SetLinearVelocity(Vector3(1,2,3)); Body->SetAngularVelocity(Vector3(0.1f,0.2f,0.3f));
		Transform Pose; EXPECT(Cluster->GetWorldTransform(0,Pose)); const Vector3 ExpectedV = Body->V + Body->W.Cross(Pose.pos-Body->X);
		const Vector3 ExpectedPoint = Pose.LocalToWorld(Set.GetGraph().FindBond(0,1)->frame.Origin-Set.GetChunks()[0].Centroid);
		Set.DoFractures({{11,11,-1,999}}); auto Events = Set.TakeEvents(); EXPECT(Events.size() == 2);
		const auto* Broken = Find(Events,DestructionEventKind::BondBroken); const auto* Changed = Find(Events,DestructionEventKind::ClusterChanged);
		EXPECT(Broken && Changed);
		if (Broken)
		{
			EXPECT(Broken->BondIndex == 11 && Broken->NodeA == 0 && Broken->NodeB == 1);
			EXPECT(Broken->Cause == DestructionEventCause::Command && Broken->BondType == DestructionBondType::Connect);
			EXPECT_NEAR(Broken->BondStrength,1,0); EXPECT(Broken->Chunks.size() == 2);
			EXPECT_NEAR((Broken->Position-ExpectedPoint).Length(),0,1e-5);
			if (!Broken->Chunks.empty())
			{
				const auto& C = Broken->Chunks[0]; EXPECT(C.ChunkIndex == 0 && C.ClusterId == ClusterId && C.HasRigidBody);
				EXPECT_NEAR((C.Pose.pos-Pose.pos).Length(),0,1e-5); EXPECT_NEAR((C.LinearVelocity-ExpectedV).Length(),0,1e-5);
				EXPECT_NEAR(C.Mass,1,0);
			}
		}
		if (Changed)
		{
			EXPECT(Changed->PreviousClusterId == ClusterId && Changed->Chunks.size() == 3 && Changed->Fragments.size() == 2);
			std::set<int> Members; for (const auto& F : Changed->Fragments) for (int I : F.ChunkIndices) Members.insert(I);
			EXPECT(Members == std::set<int>({0,1,2}));
		}
		uint64_t Sequence = 0;
		for (const auto& E : Events)
		{
			EXPECT(E.InstanceId == Id && E.Generation == Generation && E.Sequence > Sequence);
			EXPECT(E.AssetFingerprint == Asset.GetStats().Fingerprint); Sequence = E.Sequence;
		}
		Set.RebuildClustersFromGraph(); Set.DoFractures({{11}}); EXPECT(Set.TakeEvents().empty());
		Set.Clear(); EXPECT(Set.GetInstanceId() == Id && Set.GetGeneration() > Generation && Set.TakeEvents().empty());
		World.Reset(); EXPECT(Events.size() == 2 && Events.front().InstanceId == Id);
		EXPECT(Source.GetInstanceId() != Id);
	}
	void CapacityAndFiltering()
	{
		DestructionSet Set; Author(Set); DestructionEventSettings S; S.Enabled = true;
		S.KindMask = 1u << (uint32_t)DestructionEventKind::BondBroken; S.MaximumQueuedEvents = 1; Set.ConfigureEvents(S);
		Set.DoFractures({{11,12}}); const auto Stats = Set.GetEventStats();
		EXPECT(Stats.Generated == 3 && Stats.Filtered == 1 && Stats.Dropped == 1 && Stats.Invalid == 0 && Stats.Pending == 1);
		auto Bad = S; Bad.MaximumQueuedEvents = 0; Reject([&] { Set.ConfigureEvents(Bad); });
		Bad = S; Bad.KindMask = ~DestructionEventSettings::AllKinds; Reject([&] { Set.ConfigureEvents(Bad); });
		Bad = S; Bad.MinimumContactImpulse = std::numeric_limits<float>::quiet_NaN(); Reject([&] { Set.ConfigureEvents(Bad); });
		auto Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].BondIndex == 11);
		EXPECT(Set.GetEventStats().Pending == 0); Set.RebuildClustersFromGraph(); EXPECT(Set.TakeEvents().empty());
		Set.Clear(); Author(Set); EXPECT(Set.GetEventStats().Generated == 0);
		Set.DoFractures({{11}}); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].Sequence > 3);
		S.Enabled = false; Set.ConfigureEvents(S); Set.DoFractures({{12}}); EXPECT(Set.TakeEvents().empty());
	}
	void DirectedSupportBond()
	{
		DestructionSet Set; Set.AddChunk(Box3(Vector3(-0.5f),Vector3(0.5f)),1,true);
		Set.AddChunk(Box3(Vector3(0.5f,-0.5f,-0.5f),Vector3(1.5f,0.5f,0.5f)),2);
		EXPECT(Set.GetGraph().AddDirectedBond(1,0,DestructionBondType::Support,3,77)); Set.RebuildClustersFromGraph(); Enable(Set);
		Set.DoFractures({{77}}); const auto Events = Set.TakeEvents(); const auto* B = Find(Events,DestructionEventKind::BondBroken); EXPECT(B);
		if (B)
		{
			EXPECT(B->NodeA == 1 && B->NodeB == 0 && B->BondType == DestructionBondType::Support && B->Chunks.size() == 2);
			EXPECT_NEAR(B->BondStrength,3,0); EXPECT(B->Chunks[0].ChunkIndex == 1 && B->Chunks[1].ChunkIndex == 0);
		}
	}
	void ReleaseAndExternalGraphEdits()
	{
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Author(Set); Enable(Set);
		Set.GetGraph().BreakBond(0,1); Set.RebuildClustersFromGraph(); auto Events = Set.TakeEvents();
		const auto* Bond = Find(Events,DestructionEventKind::BondBroken); EXPECT(Bond && Bond->Cause == DestructionEventCause::GraphEdit);
		auto* Cluster = Set.FindClusterContaining(1); auto* Body = Cluster->GetRigidBody(); Body->X += Vector3(3,4,5);
		Transform Pose; EXPECT(Cluster->GetWorldTransform(1,Pose));
		Set.AddDestructionInfo(DestructionEventType::Destruct,{1}); Set.ProcessDestructionInfo(); Events = Set.TakeEvents();
		const auto* Released = Find(Events,DestructionEventKind::ChunkReleased); const auto* Smashed = Find(Events,DestructionEventKind::ChunkSmashed);
		EXPECT(Released && Smashed);
		if (Released)
		{
			EXPECT(Released->Cause == DestructionEventCause::LegacyCommand && Released->Chunks.size() == 1);
			EXPECT_NEAR((Released->Position-Pose.pos).Length(),0,1e-5);
			EXPECT(!Released->Chunks[0].IsFree && Released->Chunks[0].HasRigidBody);
		}
		Set.Update(1.0f/60); EXPECT(Set.TakeEvents().empty());
		DestructionSet Unsupported; Author(Unsupported); Enable(Unsupported); EXPECT(Unsupported.PruneUnsupported().size() == 3);
		Events = Unsupported.TakeEvents(); Released = Find(Events,DestructionEventKind::ChunkReleased);
		EXPECT(Released && Released->Cause == DestructionEventCause::Unsupported);
	}
	void StressAndPartitionCauses()
	{
		DestructionSet Set; Author(Set,2,true,0.01f); StrainSolverSettings Stress; Stress.Hardness = 1; Set.EnableStressSolver(Stress); Enable(Set);
		EXPECT(Set.AddStressImpulse(1,Set.GetChunks()[1].Centroid,Vector3(100,0,0))); Set.Update(1.0f/60);
		auto Events = Set.TakeEvents(); const auto* Broken = Find(Events,DestructionEventKind::BondBroken);
		EXPECT(Broken && Broken->Cause == DestructionEventCause::Stress && Broken->UpdateIndex == 1);
		if (Broken) EXPECT_NEAR(Broken->DeltaTime,1.0f/60,0);
		DestructionSet Partition; Author(Partition,8); auto& C = Partition.GetDestructionConstants();
		for (auto& L : C.Levels) { L.BreakThreshold = 0; L.PartitionSize = 2; }
		Enable(Partition); EXPECT(Partition.ApplyMomentum(3,Vector3(100,0,0),Vector3(3,0,0),Vector3::UnitX()));
		Events = Partition.TakeEvents(); Broken = Find(Events,DestructionEventKind::BondBroken);
		EXPECT(Broken && Broken->Cause == DestructionEventCause::Momentum);
		bool Queued = false;
		for (int Frame = 0; Frame < 8; ++Frame)
		{
			Partition.Update(1.0f/60); for (const auto& E : Partition.TakeEvents()) Queued |= E.Cause == DestructionEventCause::Partition;
		}
		EXPECT(Queued);
	}
	void HierarchyNotifications()
	{
		DynamicMesh Mesh;
		for (const auto& P : {Vector3(-1,-1,-1),Vector3(1,-1,-1),Vector3(1,1,-1),Vector3(-1,1,-1),
			Vector3(-1,-1,1),Vector3(1,-1,1),Vector3(1,1,1),Vector3(-1,1,1)}) Mesh.AppendVertex(P);
		for (const auto& T : {Index3(0,2,1),Index3(0,3,2),Index3(4,5,6),Index3(4,6,7),Index3(0,4,7),Index3(0,7,3),
			Index3(1,2,6),Index3(1,6,5),Index3(0,1,5),Index3(0,5,4),Index3(3,7,6),Index3(3,6,2)}) Mesh.AppendTriangle(T.a,T.b,T.c);
		Mesh.BuildBounds(); DestructionSet Set; DestructionFractureSettings S; S.MaxChunkCount = 8; S.CutRetryCount = 8; S.MinChunkVolume = 0.001f;
		DestructionFractureLevel Level; Level.MinPieceCount = Level.MaxPieceCount = 2; S.Levels = {Level,Level};
		EXPECT(Set.BuildHierarchicalFracture(Mesh,S)); Enable(Set); const auto* B = Set.GetGraph().GetBondLists()[0].Bonds.data();
		EXPECT(!Set.GetGraph()[0].Bonds.empty()); if (Set.GetGraph()[0].Bonds.empty()) return;
		Set.DoFractures({{B->bond_index}}); auto Events = Set.TakeEvents();
		EXPECT(Find(Events,DestructionEventKind::HierarchyBroken));
		std::set<int> Hierarchy; std::set<std::pair<int,int>> Bonds;
		for (const auto& E : Events)
		{
			if (E.Kind == DestructionEventKind::HierarchyBroken) EXPECT(Hierarchy.insert(E.HierarchyNode).second);
			if (E.Kind == DestructionEventKind::BondBroken) EXPECT(Bonds.insert({E.NodeA,E.NodeB}).second);
		}
		Set.RebuildClustersFromGraph(); EXPECT(Set.TakeEvents().empty());
	}
	void ContactPayloadAndFilters()
	{
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Author(Set); DestructionEventSettings S; S.Enabled = true;
		S.KindMask = 1u << (uint32_t)DestructionEventKind::Contact; S.MinimumContactImpulse = 1; Set.ConfigureEvents(S);
		auto* C = Set.FindClusterContaining(1); PhysicsContactPoint Point{}; Point.Position = Vector3(1,0.5f,0); Point.Normal = Vector3::UnitY(); Point.Impulse = Vector3(0,2,0);
		PhysicsContactInfo Info{}; Info.BodyB = C->GetRigidBody(); Info.GeomB = C->GetGeometryByChunkIndex(1); Info.ContactPoints = &Point; Info.NumContactPoints = 1;
		Set.OnContact(Info); Point.Position = Vector3(100); auto Events = Set.TakeEvents(); EXPECT(Events.size() == 1);
		if (!Events.empty())
		{
			const auto& E = Events[0]; EXPECT(E.Cause == DestructionEventCause::Contact && E.Chunks[0].ChunkIndex == 1);
			EXPECT_NEAR((E.Position-Vector3(1,0.5f,0)).Length(),0,0); EXPECT_NEAR((E.Impulse-Vector3(0,2,0)).Length(),0,0); EXPECT(!E.ImpulseEstimated);
		}
		Point.Impulse = Vector3(0,0.5f,0); Set.OnContact(Info); EXPECT(Set.TakeEvents().empty() && Set.GetEventStats().Filtered == 1);
		Point.Impulse = Vector3(0,2,0); Point.Position.x = std::numeric_limits<float>::quiet_NaN(); Set.OnContact(Info);
		EXPECT(Set.TakeEvents().empty() && Set.GetEventStats().Invalid == 1);
		Point.Position = Vector3(1,0.5f,0); Point.Impulse.SetZero(); C->GetRigidBody()->SetLinearVelocity(Vector3(0,-2,0));
		Set.OnContact(Info); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].ImpulseEstimated);
		S.MinimumContactImpulse = 0; Set.ConfigureEvents(S); C->GetRigidBody()->SetLinearVelocity(Vector3::Zero());
		Set.OnContact(Info); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].Impulse == Vector3::Zero());
	}
	void AddFloor(PhysicsWorld& World)
	{
		RigidBodyParam P; P.rigidType = RigidType::Static;
		World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-1.5f,0),Vector3(8,0.5f,8)),P);
	}
	void RealContactsAndNoninterference()
	{
		DestructionSet Source; Author(Source); Source.GetDestructionConstants().PartitionAlgorithm = DestructionPartitionAlgorithm::None;
		Source.GetDestructionConstants().DamagePropagationStrength = 0; const auto Asset = DestructionAsset::Capture(Source);
		DestructionSet* Active = nullptr; bool CallbackRan = false, PollRejected = false, ConfigureRejected = false;
		PhysicsWorldParam P; P.gravityAcc = Vector3(0,-9.8f,0); P.narrowphase = NarrowPhaseSolver::PCM;
		PhysicsWorld Reference(P); P.onContact = [&](const PhysicsContactInfo& Info)
		{
			DefaultOnContact(Info); CallbackRan = true;
			try { Active->TakeEvents(); } catch (const std::logic_error&) { PollRejected = true; }
			try { Active->ConfigureEvents(Active->GetEventSettings()); } catch (const std::logic_error&) { ConfigureRejected = true; }
		};
		PhysicsWorld Observed(P); DestructionSet A(Reference), B(Observed); Active = &B;
		AddFloor(Reference); AddFloor(Observed); A.InitializeAsset(Asset); B.InitializeAsset(Asset); Enable(B);
		bool SawContact = false;
		for (int Frame = 0; Frame < 70; ++Frame)
		{
			Reference.Simulate(1.0f/60); Observed.Simulate(1.0f/60);
			for (const auto& E : B.TakeEvents())
			{
				if (E.Kind == DestructionEventKind::Contact) { SawContact = true; EXPECT(E.WorldTick == Observed.GetTick() && E.DeltaTime > 0); }
			}
			for (int I = 0; I < 3; ++I)
			{
				Transform PA, PB; EXPECT(A.FindClusterContaining(I)->GetWorldTransform(I,PA)); EXPECT(B.FindClusterContaining(I)->GetWorldTransform(I,PB));
				EXPECT_NEAR((PA.pos-PB.pos).Length(),0,1e-6);
			}
		}
		EXPECT(CallbackRan && PollRejected && ConfigureRejected && SawContact); EXPECT(A.TakeEvents().empty());
		EXPECT(B.GetEventStats().Dropped == 0 && B.GetEventStats().Invalid == 0);
	}
	void ExistingScenes()
	{
		for (const char* Name : {"10_destruction.json","11_glass_fracture.json"})
		for (bool Authored : {false,true})
		{
			SceneWorld Scene; Scene.SetAuthoredDestructionDemo(Authored); Scene.SetDestructionEventsDemo(true); std::string Error;
			EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error));
			for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I)
			{
				auto* Set = Scene.GetDestructionSet(I); EXPECT(Set->GetEventSettings().Enabled);
				for (const auto& N : Set->GetGraph().GetBondLists()) if (!N.Bonds.empty()) { Set->DoFractures({{N.Bonds[0].bond_index}}); break; }
			}
			Scene.Step(1.0f/60); EXPECT(Scene.GetDestructionEventCount() > 0 && !Scene.GetLastDestructionEvents().empty());
			EXPECT(Scene.GetDestructionDroppedEventCount() == 0);
			for (const auto& E : Scene.GetLastDestructionEvents()) EXPECT(E.InstanceId && (E.AssetFingerprint != 0) == Authored && E.Generation && E.Sequence);
			Scene.SetAuthoredDestructionDemo(false); Scene.SetDestructionEventsDemo(false);
			EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error)); Scene.Step(1.0f/60);
			EXPECT(Scene.GetDestructionEventCount() == 0 && Scene.GetLastDestructionEvents().empty());
			printf("Destruction event demo %s authored=%d passed\n",Name,(int)Authored);
		}
	}
}
void TestDestructionEventStream()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction events: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction event test exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("IdentityAndKinematics",IdentityAndKinematics); Run("CapacityAndFiltering",CapacityAndFiltering);
	Run("DirectedSupportBond",DirectedSupportBond);
	Run("ReleaseAndExternalGraphEdits",ReleaseAndExternalGraphEdits); Run("StressAndPartitionCauses",StressAndPartitionCauses);
	Run("HierarchyNotifications",HierarchyNotifications); Run("ContactPayloadAndFilters",ContactPayloadAndFilters);
	Run("RealContactsAndNoninterference",RealContactsAndNoninterference); Run("ExistingScenes",ExistingScenes);
}

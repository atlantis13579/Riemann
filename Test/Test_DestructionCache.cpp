#include "Test.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	DestructionAsset Asset()
	{
		DestructionSet S; S.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 P((float)I,2,0); S.AddChunk(Box3(P-Vector3(0.5f),P+Vector3(0.5f)),(float)(I+1));
			if (I) EXPECT(S.AddBond(I-1,I,DestructionBondType::Connect,2,10+I));
		}
		for (size_t I = 0; I < S.GetGraph().Size(); ++I) for (auto& B : S.GetGraph()[I].Bonds)
			B.frame.Origin = (S.GetChunks()[B.v0].Centroid+S.GetChunks()[B.v1].Centroid)*0.5f;
		S.GetGraph().RefreshBondCache(); S.RebuildClustersFromGraph(); return DestructionAsset::Capture(S);
	}
	void Enable(DestructionSet& S, size_t Capacity = 2048)
	{ DestructionEventSettings E; E.Enabled = true; E.MaximumQueuedEvents = Capacity; S.ConfigureEvents(E); }
	Transform Placement()
	{
		Quaternion Q; Q.FromRotationAxis(Vector3(0.3f,0.8f,-0.4f).Unit(),1.07f); return Transform(Vector3(4,-2,3),Q);
	}
	void SamplingAndRelocation()
	{
		const auto A = Asset(); PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); const auto P = Placement(); S.InitializeAsset(A,P); Enable(S);
		DestructionCacheRecorder Recorder(S); auto* Cluster = S.FindClusterContaining(0); auto* Body = Cluster->GetRigidBody();
		Quaternion Delta; Delta.FromRotationAxis(Vector3(1,-1,2).Unit(),1.3f); Body->Q = Delta; Body->X += Vector3(2,3,-1);
		Body->V = Vector3(1,2,3); Body->W = Vector3(0.2f,-0.3f,0.5f); Recorder.Record(1,S);
		const auto Cache = Recorder.Finish(); const auto Copy = Cache;
		EXPECT(Cache.GetStats().FrameCount == 2 && Cache.GetStats().TopologyCount == 1 && Cache.GetStats().EventCount == 0);
		EXPECT(Cache.GetFrame(0).Topology == Cache.GetFrame(1).Topology); EXPECT(&Cache.GetFrame(0) == &Copy.GetFrame(0));
		EXPECT(Cache.GetSourceInstanceId() == S.GetInstanceId() && Cache.GetSourceGeneration() == S.GetGeneration());
		const auto End = Cache.Sample(1,P), Middle = Cache.Sample(0.5), Start = Cache.Sample(0);
		EXPECT(End.LowerFrame == 1 && End.Interpolation == 0 && Middle.LowerFrame == 0 && Middle.UpperFrame == 1);
		EXPECT_NEAR(Middle.Interpolation,0.5f,0);
		for (int I = 0; I < 3; ++I)
		{
			Transform Actual; EXPECT(Cluster->GetWorldTransform(I,Actual));
			EXPECT_NEAR((End.Chunks[I].Pose.pos-Actual.pos).Length(),0,2e-5);
			const Vector3 Velocity = Body->V+Body->W.Cross(Actual.pos-Body->X);
			EXPECT_NEAR((End.Chunks[I].LinearVelocity-Velocity).Length(),0,2e-5);
			EXPECT_NEAR((Middle.Chunks[I].Pose.pos-Middle.Chunks[0].Pose.pos).Length(),(Start.Chunks[I].Pose.pos-Start.Chunks[0].Pose.pos).Length(),2e-5);
		}
		Quaternion Q; Q.FromRotationAxis(Vector3(-1,0.4f,0.7f).Unit(),-0.9f); const Transform Destination(Vector3(-3,6,2),Q);
		DestructionSet Relocated; Relocated.InitializeAsset(A,Destination); const auto Sample = Cache.Sample(1,Destination);
		for (int I = 0; I < 3; ++I)
		{
			Transform Original; Cluster->GetWorldTransform(I,Original); const auto& Hull = *S.GetChunks()[I].CollisionConvex; const auto& Target = *Relocated.GetChunks()[I].CollisionConvex;
			for (size_t J = 0; J < Hull.NumVertices; ++J)
			{
				const Vector3 Expected = Destination.LocalToWorld(P.WorldToLocal(Original.LocalToWorld(Hull.Vertices[J])));
				EXPECT_NEAR((Sample.Chunks[I].Pose.LocalToWorld(Target.Vertices[J])-Expected).Length(),0,3e-5);
			}
		}
		EXPECT(Cache.Sample(-2).Time == 0 && Cache.Sample(2).Time == 1);
		S.Clear(); W.Reset(); EXPECT(Cache.Sample(1).Chunks.size() == 3 && Cache.GetAsset().GetStats().Fingerprint == A.GetStats().Fingerprint);
		Reject([&] { Recorder.Record(2,S); }); Reject([&] { Recorder.Finish(); }); Reject([&] { Cache.GetFrame(2); });
		Reject([&] { Cache.Sample(std::numeric_limits<double>::quiet_NaN()); });
		Transform Bad; Bad.quat.x = 2; Reject([&] { Cache.Sample(0,Bad); });
	}
	void FractureRemovalAndEventIntervals()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset(),Placement()); Enable(S); DestructionCacheRecorder Recorder(S);
		S.DoFractures({{11}}); auto Events = S.TakeEvents(); EXPECT(Events.size() == 2);
		S.FindClusterContaining(0)->GetRigidBody()->X += Vector3(2,0,0); S.FindClusterContaining(1)->GetRigidBody()->X -= Vector3(1,0,0);
		Recorder.Record(1,S,Events); S.AddDestructionInfo(DestructionEventType::Destruct,{0}); S.ProcessDestructionInfo();
		auto Removed = S.TakeEvents(); EXPECT(!Removed.empty()); Recorder.Record(2,S,Removed); const auto Cache = Recorder.Finish();
		const auto Before = Cache.Sample(0.999), Broken = Cache.Sample(1), BeforeRemoval = Cache.Sample(1.999), AfterRemoval = Cache.Sample(2);
		EXPECT(Before.Topology->Clusters.size() == 1 && Before.Topology->Graph.HasBond(0,1));
		EXPECT(Broken.Topology->Clusters.size() == 2 && !Broken.Topology->Graph.HasBond(0,1));
		EXPECT_NEAR((Before.Chunks[2].Pose.pos-Before.Chunks[0].Pose.pos).Length(),2,2e-5);
		EXPECT(!BeforeRemoval.Chunks[0].IsFree && AfterRemoval.Chunks[0].IsFree && AfterRemoval.Chunks[0].IsSmashed);
		EXPECT_NEAR((BeforeRemoval.Chunks[0].Pose.pos-Broken.Chunks[0].Pose.pos).Length(),0,1e-6);
		EXPECT(!AfterRemoval.Chunks[0].HasRigidBody && AfterRemoval.Chunks[0].ClusterId == 0);
		EXPECT(Cache.GetStats().TopologyCount == 3 && Cache.GetStats().EventCount == Events.size()+Removed.size());
		EXPECT(Cache.GetEvents(0,0.999).empty()); EXPECT(Cache.GetEvents(0.999,1).size() == Events.size());
		EXPECT(Cache.GetEvents(1,2).size() == Removed.size()); EXPECT(Cache.GetEvents(1,1).empty());
		const auto WorldEvents = Cache.GetEvents(0,1,S.GetAssetPlacement()); EXPECT(WorldEvents.size() == Events.size());
		for (size_t I = 0; I < Events.size(); ++I)
		{
			EXPECT(WorldEvents[I].Time == 1 && WorldEvents[I].Event.Sequence == Events[I].Sequence && WorldEvents[I].Event.InstanceId == S.GetInstanceId());
			EXPECT_NEAR((WorldEvents[I].Event.Position-Events[I].Position).Length(),0,2e-5);
			for (size_t J = 0; J < Events[I].Chunks.size(); ++J) EXPECT_NEAR((WorldEvents[I].Event.Chunks[J].Pose.pos-Events[I].Chunks[J].Pose.pos).Length(),0,2e-5);
		}
		Reject([&] { Cache.GetEvents(2,1); });
	}
	void MissingDuplicateAndReconfiguredEvents()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset()); Enable(S); DestructionCacheRecorder R(S);
		S.DoFractures({{11}}); Reject([&] { R.Record(1,S); }); auto Events = S.TakeEvents();
		Reject([&] { R.Record(1,S); }); auto Bad = Events; Bad[1] = Bad[0]; Reject([&] { R.Record(1,S,Bad); });
		Bad = Events; ++Bad[0].InstanceId; Reject([&] { R.Record(1,S,Bad); });
		Bad = Events; Bad[0].Position.x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { R.Record(1,S,Bad); });
		EXPECT(R.GetStats().FrameCount == 1); R.Record(1,S,Events); EXPECT(R.GetStats().FrameCount == 2);
		Reject([&] { R.Record(2,S,Events); }); const auto Revision = S.GetEventRevision(); S.ConfigureEvents(S.GetEventSettings());
		EXPECT(S.GetEventRevision() > Revision); Reject([&] { R.Record(2,S); }); EXPECT(R.GetStats().FrameCount == 2);
		DestructionSet Overflow(W); Overflow.InitializeAsset(Asset()); Enable(Overflow,1); DestructionCacheRecorder Lost(Overflow);
		Overflow.DoFractures({{11}}); auto Batch = Overflow.TakeEvents(); EXPECT(Overflow.GetEventStats().Dropped > 0);
		Reject([&] { Lost.Record(1,Overflow,Batch); }); EXPECT(Lost.GetStats().FrameCount == 1);
	}
	void LimitsAndAuthoringValidation()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); const auto A = Asset(); S.InitializeAsset(A);
		Reject([&] { DestructionCacheRecorder MissingStream(S); }); DestructionCacheSettings C; C.RecordEvents = false;
		S.GetDestructionConstants().ContactOffset = -1; EXPECT(!S.GetDestructionConstants().IsValid());
		Reject([&] { DestructionCacheRecorder BadPolicy(S,C); }); S.GetDestructionConstants().ContactOffset = 0;
		DestructionCacheRecorder Measure(S,C); const auto InitialBytes = Measure.GetStats().PayloadBytes;
		C.MaximumPayloadBytes = InitialBytes; DestructionCacheRecorder Budget(S,C); Reject([&] { Budget.Record(1,S); });
		EXPECT(Budget.GetStats().PayloadBytes == InitialBytes && Budget.GetStats().FrameCount == 1);
		C.MaximumPayloadBytes = 1; Reject([&] { DestructionCacheRecorder TooSmall(S,C); });
		C = DestructionCacheSettings(); C.RecordEvents = false; C.MaximumFrames = 2; DestructionCacheRecorder Frames(S,C);
		Frames.Record(1,S); Reject([&] { Frames.Record(2,S); }); EXPECT(Frames.GetStats().FrameCount == 2);
		Reject([&] { Measure.Record(0,S); }); Reject([&] { Measure.Record(-1,S); });
		auto& Chunk = S.GetChunks()[0]; const Vector3 V = Chunk.CollisionConvex->Vertices[0]; Chunk.CollisionConvex->Vertices[0].x += 0.1f;
		Reject([&] { Measure.Record(1,S); }); Chunk.CollisionConvex->Vertices[0] = V;
		const float D = Chunk.CollisionConvex->Faces[0].plane.D; Chunk.CollisionConvex->Faces[0].plane.D = std::numeric_limits<float>::quiet_NaN();
		Reject([&] { Measure.Record(1,S); }); Chunk.CollisionConvex->Faces[0].plane.D = D;
		Chunk.Mass *= 2; Reject([&] { Measure.Record(1,S); }); Chunk.Mass /= 2;
		auto* Bond = S.GetGraph().FindBond(0,1); const auto Origin = Bond->frame.Origin; Bond->frame.Origin.x += 0.1f;
		Reject([&] { Measure.Record(1,S); }); Bond->frame.Origin = Origin;
		const auto Constants = S.GetDestructionConstants(); S.GetDestructionConstants().ContactOffset += 0.1f;
		Reject([&] { Measure.Record(1,S); }); S.GetDestructionConstants() = Constants;
		auto* Body = S.FindClusterContaining(0)->GetRigidBody(); const auto Q = Body->Q; Body->Q = Quaternion(0,0,0,0);
		Reject([&] { Measure.Record(1,S); }); Body->Q = Q;
		Measure.Record(1,S); EXPECT(Measure.GetStats().FrameCount == 2);
		DestructionSet Other(W); Other.InitializeAsset(A); Reject([&] { Measure.Record(2,Other); });
		W.Reset(); Reject([&] { Measure.Record(2,S); }); const auto Cache = Measure.Finish(); EXPECT(Cache.GetStats().FrameCount == 2);
		DestructionSet Detached; Detached.InitializeAsset(A); Reject([&] { DestructionCacheRecorder NoWorld(Detached,C); });
		DestructionCache Empty; Reject([&] { Empty.Sample(0); }); Reject([&] { Empty.GetStats(); });
	}
	void WorldBoundaryAndRetiredSource()
	{
		DestructionSet* Active = nullptr; DestructionCacheRecorder* Recording = nullptr; bool Rejected = false;
		PhysicsWorldParam P; P.gravityAcc = Vector3(0,-9.8f,0); P.onContact = [&](const PhysicsContactInfo& Info)
		{
			DefaultOnContact(Info); try { Recording->Record(1,*Active); } catch (const std::logic_error&) { Rejected = true; }
		};
		PhysicsWorld W(P); RigidBodyParam Floor; Floor.rigidType = RigidType::Static;
		W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-0.5f,0),Vector3(8,0.5f,8)),Floor);
		DestructionSet S(W); S.InitializeAsset(Asset()); Active = &S; DestructionCacheSettings C; C.RecordEvents = false;
		DestructionCacheRecorder R(S,C); Recording = &R;
		for (int I = 0; I < 60; ++I) { W.Simulate(1.0f/60); R.Record((I+1)/60.0,S); }
		EXPECT(Rejected); const auto Cache = R.Finish(); S.Clear(); EXPECT(Cache.GetStats().FrameCount == 61 && Cache.Sample(1).Chunks.size() == 3);
	}
	void ExistingScenes()
	{
		for (const char* Name : {"10_destruction.json","11_glass_fracture.json"})
		{
			SceneWorld Scene; Scene.SetAuthoredDestructionDemo(true); Scene.SetDestructionEventsDemo(true); std::string Error;
			EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error));
			std::vector<std::unique_ptr<DestructionCacheRecorder>> Recorders;
			for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I) Recorders.emplace_back(new DestructionCacheRecorder(*Scene.GetDestructionSet(I)));
			for (int Frame = 0; Frame < 25; ++Frame)
			{
				if (Frame == 5) for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I)
				{
					auto* S = Scene.GetDestructionSet(I); for (const auto& N : S->GetGraph().GetBondLists()) if (!N.Bonds.empty()) { S->DoFractures({{N.Bonds[0].bond_index}}); break; }
				}
				Scene.Step(1.0f/60);
				for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I)
				{
					auto* S = Scene.GetDestructionSet(I); std::vector<DestructionEvent> Batch;
					for (const auto& E : Scene.GetLastDestructionEvents()) if (E.InstanceId == S->GetInstanceId()) Batch.push_back(E);
					Recorders[I]->Record((Frame+1)/60.0,*S,Batch);
				}
			}
			for (size_t I = 0; I < Recorders.size(); ++I)
			{
				const auto Cache = Recorders[I]->Finish(); const auto Sample = Cache.Sample(25/60.0,Scene.GetDestructionSet(I)->GetAssetPlacement());
				EXPECT(Cache.GetStats().FrameCount == 26 && Cache.GetStats().TopologyCount > 1 && Cache.GetStats().EventCount > 0);
				for (const auto& C : Sample.Chunks) if (!C.IsFree)
				{
					Transform Actual; EXPECT(Scene.GetDestructionSet(I)->FindClusterContaining(C.ChunkIndex)->GetWorldTransform(C.ChunkIndex,Actual));
					EXPECT_NEAR((C.Pose.pos-Actual.pos).Length(),0,3e-5);
				}
				printf("Destruction cache scene %s actor=%zu frames=%zu topologies=%zu events=%zu\n",Name,I,Cache.GetStats().FrameCount,Cache.GetStats().TopologyCount,Cache.GetStats().EventCount);
			}
		}
	}
}
void TestDestructionCache()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction cache: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction cache exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("SamplingAndRelocation",SamplingAndRelocation); Run("FractureRemovalAndEventIntervals",FractureRemovalAndEventIntervals);
	Run("MissingDuplicateAndReconfiguredEvents",MissingDuplicateAndReconfiguredEvents); Run("LimitsAndAuthoringValidation",LimitsAndAuthoringValidation);
	Run("WorldBoundaryAndRetiredSource",WorldBoundaryAndRetiredSource); Run("ExistingScenes",ExistingScenes);
}

#include "Test.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	Transform Placement()
	{
		Quaternion Q; Q.FromRotationAxis(Vector3(1,2,-1).Unit(),0.8f); return Transform(Vector3(6,3,-4),Q);
	}
	DestructionCache Clip()
	{
		DestructionSet Author; Author.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 P((float)I,2,0); Author.AddChunk(Box3(P-Vector3(0.5f),P+Vector3(0.5f)),(float)(I+1));
			if (I) EXPECT(Author.AddBond(I-1,I,DestructionBondType::Connect,2,10+I));
		}
		Author.RebuildClustersFromGraph(); const auto Asset = DestructionAsset::Capture(Author);
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset,Placement());
		DestructionEventSettings E; E.Enabled = true; S.ConfigureEvents(E); DestructionCacheRecorder R(S);
		auto* Body = S.FindClusterContaining(0)->GetRigidBody(); Body->X += Vector3(1,2,-1);
		Body->Q.FromRotationAxis(Vector3(1,-1,2).Unit(),0.7f); Body->V = Vector3(1,0,0); Body->W = Vector3(0,0.2f,0);
		R.Record(1,S); S.DoFractures({{11}}); R.Record(2,S,S.TakeEvents());
		S.AddDestructionInfo(DestructionEventType::Destruct,{0}); S.ProcessDestructionInfo(); R.Record(3,S,S.TakeEvents());
		return R.Finish();
	}
	void RegisteredKinematics()
	{
		for (auto Solver : {ResolutionPhaseSolver::SequentialImpulse,ResolutionPhaseSolver::LCPGlobal,ResolutionPhaseSolver::PBD,ResolutionPhaseSolver::XPBD})
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld W(P);
			RigidBodyParam K; K.rigidType = RigidType::Kinematic; K.linearVelocity = Vector3(1,0,0);
			auto* A = W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(),Vector3(0.5f)),K);
			RigidBodyParam D; D.rigidType = RigidType::Dynamic; D.linearVelocity = Vector3(-1,0,0);
			auto* B = W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0.9f,0,0),Vector3(0.5f)),D);
			EXPECT(A->CastKinematic() && W.GetKinematicBodyCount() == 1 && A->InvMass == 0 && A->CM == Vector3::Zero());
			W.Simulate(1.0f/60); EXPECT_NEAR(A->X.Length(),0,1e-6); EXPECT_NEAR(A->V.x,1,1e-6); EXPECT(B->V.x > -0.5f);
			std::vector<Geometry*> Shapes; W.GetAllGeometries(&Shapes); EXPECT(Shapes.size() == 2);
			EXPECT(W.RemoveRigidBody(A)); EXPECT(!W.RemoveRigidBody(A)); A->ReleaseGeometries(); delete A;
			EXPECT(W.GetKinematicBodyCount() == 0); W.Reset();
		}
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam Bad; Bad.rigidType = (RigidType)255;
		Reject([&] { W.CreateRigidBody(Bad,Transform::Identity()); }); Reject([&] { W.CreateRigidBody(nullptr,Bad); });
	}
	void SamplesQueriesAndTopology()
	{
		const auto Cache = Clip(); PhysicsWorld W{PhysicsWorldParam()}; const auto P = Placement(); DestructionPlayback Player(W,Cache,P);
		EXPECT(Player.IsAttached() && !Player.IsPlaying() && Player.GetBodyCount() == 1 && W.GetKinematicBodyCount() == 1);
		Geometry* Initial = Player.GetChunkGeometry(0); const auto Revision = Player.GetRevision();
		for (double Time : {0.5,1.0,1.99,2.0,2.5,3.0,0.0})
		{
			Player.Seek(Time); const auto Sample = Cache.Sample(Time,P);
			EXPECT(Player.GetBodyCount() == Sample.Topology->Clusters.size());
			for (const auto& C : Sample.Chunks)
			{
				auto* Shape = Player.GetChunkGeometry(C.ChunkIndex);
				EXPECT((Shape != nullptr) == C.HasRigidBody);
				if (!Shape) continue;
				auto* Body = Shape->GetParent<RigidBody>(); const auto Pose = Body->GetGeometryTransform(Shape);
				EXPECT(Body->InvMass == 0 && Body->CM == Vector3::Zero() && Body->V == Vector3::Zero());
				EXPECT_NEAR((Pose.pos-C.Pose.pos).Length(),0,3e-5);
				const auto* Hull = Shape->GetShapeObj<ConvexMesh>(); const auto Bounds = Body->GetGeometryBounds(Shape);
				for (size_t I = 0; I < Hull->NumVertices; ++I)
					EXPECT_NEAR((Pose.LocalToWorld(Hull->Vertices[I])-C.Pose.LocalToWorld(Hull->Vertices[I])).Length(),0,3e-5);
				EXPECT((Bounds.GetCenter()-C.Pose.pos).Length() < 1e-4f);
			}
			if (Time < 2 && Player.GetRevision() == Revision) EXPECT(Player.GetChunkGeometry(0) == Initial);
			if (Time < 3)
			{
				RayCastOption O; O.MaxDist = 20; RayCastResult R;
				const Vector3 Center = Sample.Chunks[0].Pose.pos;
				EXPECT(W.GetGeometryQuery()->RayCastQuery(Center+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
			}
		}
		EXPECT(Player.GetRevision() > Revision); EXPECT(Player.GetChunkGeometry(-1) == nullptr && Player.GetChunkGeometry(99) == nullptr);
		Player.Detach(); EXPECT(!Player.IsAttached() && W.GetKinematicBodyCount() == 0); Player.Detach();
	}
	void TimeEventsAndEpochs()
	{
		const auto Cache = Clip(); PhysicsWorldParam P; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P); DestructionPlayback Player(W,Cache); const auto Epoch = Player.GetEpoch();
		W.Simulate(0.5f); EXPECT(Player.GetTime() == 0); Player.SetPlaying(true); W.Simulate(0.5f);
		EXPECT_NEAR(Player.GetTime(),0.5,1e-8); EXPECT(Player.TakeEvents().empty());
		const auto Sample = Cache.Sample(0.5); EXPECT_NEAR((Player.GetBody(0)->V-Sample.Clusters[0].LinearVelocity).Length(),0,1e-6);
		Player.SetPlaying(false); W.Simulate(1); EXPECT(Player.GetTime() == 0.5 && Player.GetBody(0)->V == Vector3::Zero());
		Player.SetPlaying(true); W.Simulate(1.5f); const auto Events = Player.TakeEvents(); const auto Source = Cache.GetEvents(0,2);
		EXPECT(Events.size() == Source.size() && !Events.empty());
		for (size_t I = 0; I < Events.size(); ++I)
		{
			EXPECT(Events[I].PlaybackId == Player.GetInstanceId() && Events[I].Epoch == Epoch);
			EXPECT(Events[I].Source.Event.InstanceId == Cache.GetSourceInstanceId());
			EXPECT(Events[I].Source.Event.Sequence == Source[I].Event.Sequence && Events[I].Source.Time == Source[I].Time);
		}
		W.Simulate(0.5f); EXPECT(Player.TakeEvents().empty()); Player.Seek(0); EXPECT(Player.GetEpoch() == Epoch+1 && !Player.IsPlaying());
		Player.SetPlaying(true); W.Simulate(2); EXPECT(Player.TakeEvents().size() == Events.size());
		W.Simulate(2); EXPECT(Player.GetTime() == 3 && !Player.IsPlaying() && Player.GetBody(0)->V == Vector3::Zero());
		EXPECT(!Player.TakeEvents().empty()); EXPECT(Player.TakeEvents().empty());
		DestructionPlaybackSettings S; S.Rate = 2; S.MaximumQueuedEvents = 1; DestructionPlayback Fast(W,Cache,Transform::Identity(),S);
		Fast.SetPlaying(true); W.Simulate(1); EXPECT(Fast.GetTime() == 2 && Fast.TakeEvents().size() == 1);
		EXPECT(Fast.GetDroppedEventCount() == Source.size()-1);
	}
	void DistantConvexRays()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionPlayback Player(W,Clip());
		const auto* Shape = Player.GetChunkGeometry(0); const auto* Hull = Shape->GetShapeObj<ConvexMesh>();
		const Vector3 Center = Shape->GetShapeBounds().GetCenter(); float Near = -1, Scaled = -1, Inside = -1;
		EXPECT(Hull->IntersectRay(Center+Vector3(0,0,10),Vector3(0,0,-1),&Near)); EXPECT(Near > 1);
		EXPECT(Hull->IntersectRay(Center+Vector3(0,0,10),Vector3(0,0,-0.5f),&Scaled)); EXPECT_NEAR(Scaled,Near*2,1e-5);
		EXPECT(Hull->IntersectRay(Center,Vector3::UnitX(),&Inside)); EXPECT(Inside == 0);
		EXPECT(!Hull->IntersectRay(Center,Vector3::Zero(),&Inside));
		EXPECT(!Hull->IntersectRay(Shape->GetShapeBounds().Max+Vector3(1),Vector3::UnitY(),&Inside));
		RayCastOption O; O.MaxDist = 1; RayCastResult R; const auto WorldCenter = Player.GetSample().Chunks[0].Pose.pos;
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(WorldCenter+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
	}
	void CoupledWorldAndCallbackBoundary()
	{
		for (bool Coupled : {false,true})
		{
			DestructionPlayback* Active = nullptr; PhysicsWorld* World = nullptr; int Rejected = 0;
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
			P.onContact = [&](const PhysicsContactInfo& Info)
			{
				DefaultOnContact(Info);
				try { Active->Seek(0); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->SetPlaying(false); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->TakeEvents(); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->Detach(); } catch (const std::logic_error&) { ++Rejected; }
				try { World->RemoveRigidBody(Active->GetBody(0)); } catch (const std::logic_error&) { ++Rejected; }
			};
			PhysicsWorld W(P); World = &W; DestructionPlayback Player(W,Clip()); Active = &Player;
			TetrahedralAssetInput Input; Input.Positions = {Vector3(50,0,0),Vector3(51,0,0),Vector3(50,1,0),Vector3(50,0,1)};
			Input.Elements.push_back({{0,1,2,3},0}); DeformableBodySettings Settings;
			Settings.Collision.RigidCollision = true; Settings.Gravity = Vector3::Zero(); Settings.GroundEnabled = false;
			if (Coupled) W.CreateDeformableBody(TetrahedralAsset::Prepare(Input),Settings);
			RigidBodyParam Floor; W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-0.5f,0),Vector3(6,0.5f,6)),Floor);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic;
			W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0,0.4f,0),0.5f),Dynamic);
			Player.SetPlaying(true); W.Simulate(2);
			EXPECT(Rejected >= 5 && Player.IsAttached()); EXPECT(Player.GetTime() == 2 && Player.GetBodyCount() == 2);
			EXPECT(!Player.TakeEvents().empty()); W.Reset(); EXPECT(!Player.IsAttached());
		}
	}

	void LiveCCDAndPlaybackPolicy()
	{
		for (bool Continue : {false,true})
		{
			PhysicsWorldParam P; P.ccdContinuation.Enabled = Continue; PhysicsWorld W(P); DestructionPlayback Player(W,Clip());
			RigidBodyParam D; D.rigidType = RigidType::Dynamic; D.motionType = MorionType::Continuous; D.linearVelocity = Vector3(60,0,0);
			auto* Bullet = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-3,2,0),0.25f),D);
			W.Simulate(0.1f); EXPECT(Bullet->X.x < -0.6f); EXPECT(W.GetLastCCDHitCount() > 0); EXPECT(Player.GetTime() == 0);
		}
		PhysicsWorld W{PhysicsWorldParam()}; DestructionPlaybackSettings S; S.QueryEnabled = false; S.CollisionEnabled = false;
		DestructionPlayback Hidden(W,Clip(),Transform::Identity(),S); RayCastOption O; RayCastResult R;
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(Vector3(0,2,10),Vector3(0,0,-1),O,&R));
		Hidden.SetPlaying(true); W.Simulate(2); EXPECT(Hidden.GetTime() == 2 && Hidden.GetBodyCount() == 2);
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(Hidden.GetSample().Chunks[0].Pose.pos+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
	}
	void WorldLifetimeAndEdits()
	{
		const auto Cache = Clip(); auto W = std::make_unique<PhysicsWorld>(PhysicsWorldParam()); auto Player = std::make_unique<DestructionPlayback>(*W,Cache);
		auto* Body = Player->GetBody(0); EXPECT(W->RemoveRigidBody(Body)); Body->ReleaseGeometries(); delete Body;
		EXPECT(!Player->IsPlaying() && Player->GetChunkGeometry(0) == nullptr && Player->GetBody(0) == nullptr);
		Player->Seek(1); EXPECT(Player->GetChunkGeometry(0) != nullptr && W->GetKinematicBodyCount() == 1);
		W->Reset(); EXPECT(!Player->IsAttached() && Player->GetBodyCount() == 0 && Player->GetChunkGeometry(0) == nullptr);
		Reject([&] { Player->Seek(0); }); Reject([&] { Player->SetPlaying(true); });
		Player.reset(new DestructionPlayback(*W,Cache)); W.reset(); EXPECT(!Player->IsAttached()); Player.reset();
		PhysicsWorld World{PhysicsWorldParam()}; DestructionPlayback Live(World,Cache); const auto Time = Live.GetTime();
		Reject([&] { Live.Seek(std::numeric_limits<double>::quiet_NaN()); }); EXPECT(Live.GetTime() == Time);
		DestructionPlaybackSettings Bad; Bad.Rate = 0; Reject([&] { DestructionPlayback Invalid(World,Cache,Transform::Identity(),Bad); });
		DestructionCache Empty; Reject([&] { DestructionPlayback Invalid(World,Empty); });
		EXPECT(World.GetKinematicBodyCount() == 1);
	}
	void ExistingSceneControls()
	{
		for (const char* Name : {"10_destruction.json","11_glass_fracture.json"})
		{
			SceneWorld Scene; Scene.SetDestructionCacheDemo(true); std::string Error;
			EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error)); EXPECT(Scene.IsDestructionCacheRecording());
			EXPECT(!Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::Replay));
			for (int Frame = 0; Frame < 12; ++Frame)
			{
				if (Frame == 4) for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I)
				{
					auto* S = Scene.GetDestructionSet(I); for (const auto& N : S->GetGraph().GetBondLists()) if (!N.Bonds.empty()) { S->DoFractures({{N.Bonds[0].bond_index}}); break; }
				}
				Scene.Step(1.0f/60);
			}
			const double Duration = Scene.GetDestructionCacheDuration(); EXPECT(Duration > 0);
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::Replay) && !Scene.IsDestructionCacheRecording() && Scene.IsDestructionCachePlaying());
			EXPECT(Scene.GetDestructionPlayback(0) && Scene.GetDestructionPlayback(0)->GetCache().GetStats().EventCount > 0);
			for (int I = 0; I < 16; ++I) Scene.Step(1.0f/60);
			EXPECT(!Scene.IsDestructionCachePlaying()); EXPECT_NEAR(Scene.GetDestructionCacheTime(),Duration,1e-8);
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::Seek,0.5f)); EXPECT_NEAR(Scene.GetDestructionCacheTime(),Duration*0.5,1e-8);
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::TogglePause) && Scene.IsDestructionCachePlaying());
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::TogglePause) && !Scene.IsDestructionCachePlaying());
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::Replay) && Scene.GetDestructionCacheTime() == 0); Scene.Step(1.0f/60);
			EXPECT(Scene.ApplyDestructionCacheDemo(DestructionPlaybackCommand::Seek,0)); EXPECT(!Scene.ApplyDestructionCacheDemo((DestructionPlaybackCommand)99));
			printf("Destruction playback scene %s actors=%zu duration=%g\n",Name,Scene.GetDestructionSetCount(),Duration);
			Scene.Reset(); EXPECT(Scene.GetDestructionPlayback(0) == nullptr && Scene.GetSimulation()->GetKinematicBodyCount() == 0);
		}
	}
}
void TestDestructionPlayback()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction playback: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction playback exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("RegisteredKinematics",RegisteredKinematics); Run("SamplesQueriesAndTopology",SamplesQueriesAndTopology);
	Run("TimeEventsAndEpochs",TimeEventsAndEpochs); Run("WorldLifetimeAndEdits",WorldLifetimeAndEdits); Run("ExistingSceneControls",ExistingSceneControls);
	Run("DistantConvexRays",DistantConvexRays); Run("CoupledWorldAndCallbackBoundary",CoupledWorldAndCallbackBoundary);
	Run("LiveCCDAndPlaybackPolicy",LiveCCDAndPlaybackPolicy);
}

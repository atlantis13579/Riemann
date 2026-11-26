#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsField.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Destruction/DestructionSet.h"
#include <limits>
#include <cmath>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	RigidBodyDynamic* Body(PhysicsWorld& W, const Vector3& P = Vector3::Zero(), float Mass = 2, uint32_t Layer = 0)
	{
		RigidBodyParam D; D.rigidType = RigidType::Dynamic; D.linearDamping = D.angularDamping = 1; D.sleepThreshold = 0;
		auto* G = GeometryFactory::CreateSphere(P,0.2f); G->SetFilterData(CollisionData::ForLayer(Layer));
		auto* B = W.CreateRigidBody(G,D)->CastDynamic(); B->InvMass = 1/Mass; B->InvInertia = Matrix3(1/Mass,1/Mass,1/Mass); return B;
	}
	void ExpressionsAndBudgets()
	{
		const auto R = ScalarField::Radial(Vector3::Zero(),1,3,FieldFalloff::Linear);
		EXPECT(R.Evaluate(Vector3::Zero()) == 1 && R.Evaluate(Vector3(4,0,0)) == 0); EXPECT_NEAR(R.Evaluate(Vector3(2,0,0)),0.5,1e-12);
		Quaternion Q; Q.FromRotateZ((float)(std::acos(-1.0)*0.5)); const Transform P(Vector3(3,4,5),Q);
		const auto Box = ScalarField::Box(P,Vector3(0.5f,0.25f,0.5f),Vector3(1,0.5f,1),FieldFalloff::Linear);
		EXPECT_NEAR(Box.Evaluate(P.LocalToWorld(Vector3(0.75f,0,0))),0.5,1e-5);
		const auto Hole = ScalarField::Clamp(ScalarField::Add(ScalarField::Constant(1),ScalarField::Multiply(R,ScalarField::Constant(-1))));
		EXPECT(Hole.Evaluate(Vector3::Zero()) == 0 && Hole.Evaluate(Vector3(4,0,0)) == 1);
		EXPECT_NEAR(ScalarField::Maximum(R,Hole).Evaluate(Vector3(2,0,0)),0.5,1e-12);
		EXPECT(ScalarField::Minimum(R,Hole).Evaluate(Vector3::Zero()) == 0);
		EXPECT_NEAR(ScalarField::Wave(1,2,1).Evaluate(Vector3::Zero(),0.25),3,1e-12);
		EXPECT(VectorField::Radial(Vector3::Zero()).Evaluate(Vector3::Zero()) == Vector3::Zero());
		EXPECT_NEAR((VectorField::Radial(Vector3::Zero()).Evaluate(Vector3(3,4,0))-Vector3(0.6f,0.8f,0)).Length(),0,1e-6);
		EXPECT(VectorField::Cross(VectorField::Constant(Vector3::UnitY()),VectorField::Constant(Vector3::UnitX())).Evaluate(Vector3::Zero()) == -Vector3::UnitZ());
		EXPECT(VectorField::Normalize(VectorField::Constant(Vector3::Zero())).Evaluate(Vector3::Zero()) == Vector3::Zero());
		PhysicsWorld W{PhysicsWorldParam()}; Body(W); ScalarField Shared = ScalarField::Constant(1);
		for (int I = 0; I < 20; ++I) Shared = ScalarField::Add(Shared,Shared);
		EXPECT(Shared.Evaluate(Vector3::Zero()) == 1048576);
		PhysicsFieldCommand C; C.Vector = VectorField::Scale(VectorField::Constant(Vector3::UnitX()),Shared); W.AddPhysicsField(C); W.Simulate(0.001f);
		EXPECT(W.GetPhysicsFieldStats().EvaluatedNodes < 30);
		Reject([] { ScalarField::Radial(Vector3::Zero(),1,1); }); Reject([] { ScalarField::Clamp(ScalarField(),2,1); });
		Reject([] { ScalarField::Constant(std::numeric_limits<double>::infinity()); });
		Reject([&] { R.Evaluate(Vector3(std::numeric_limits<float>::quiet_NaN(),0,0)); });
		Reject([] { ScalarField F; for (int I = 0; I < 80; ++I) F = ScalarField::Add(F,F); });
		ScalarField Moved; const auto Kept = std::move(Moved); Reject([&] { ScalarField::Add(Moved,Kept); });
	}
	void UnitsAndSubsteps()
	{
		for (auto Solver : {ResolutionPhaseSolver::SequentialImpulse,ResolutionPhaseSolver::LCPGlobal,ResolutionPhaseSolver::PBD,ResolutionPhaseSolver::XPBD}) for (int Count : {1,4,8})
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Count; PhysicsWorld W(P);
			auto* A = Body(W); auto* B = Body(W,Vector3(10,0,0),4,1);
			PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Force; C.LayerMask = 1; C.Vector = VectorField::Constant(Vector3(8,0,0));
			const auto Id = W.AddPhysicsField(C); W.Simulate(0.25f); EXPECT_NEAR(A->V.x,1,1e-6); EXPECT_NEAR(B->V.x,0,1e-6);
			EXPECT(W.GetPhysicsFieldCount() == 0 && !W.RemovePhysicsField(Id)); W.Simulate(0.25f); EXPECT_NEAR(A->V.x,1,1e-6);
			C.Target = PhysicsFieldTarget::Impulse; C.Vector = VectorField::Constant(Vector3(2,0,0)); W.AddPhysicsField(C); W.Simulate(0.25f);
			EXPECT_NEAR(A->V.x,2,1e-6);
			C.Target = PhysicsFieldTarget::Acceleration; C.LayerMask = UINT32_MAX; C.Persistent = true; C.Vector = VectorField::Constant(Vector3(0,4,0));
			const auto Persistent = W.AddPhysicsField(C); W.Simulate(0.25f); EXPECT_NEAR(A->V.y,1,1e-6); EXPECT_NEAR(B->V.y,1,1e-6);
			EXPECT(W.RemovePhysicsField(Persistent)); C.Persistent = false; C.Target = PhysicsFieldTarget::Torque; C.Vector = VectorField::Constant(Vector3(0,0,4));
			W.AddPhysicsField(C); W.Simulate(0.25f); EXPECT_NEAR(A->W.z,0.5,1e-6); EXPECT_NEAR(B->W.z,0.25,1e-6);
			C.Persistent = true; C.Target = PhysicsFieldTarget::Impulse; Reject([&] { W.AddPhysicsField(C); });
		}
	}
	void ActivationVelocityAndMaterial()
	{
		for (auto Solver : {ResolutionPhaseSolver::SequentialImpulse,ResolutionPhaseSolver::LCPGlobal,ResolutionPhaseSolver::PBD,ResolutionPhaseSolver::XPBD})
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld W(P); auto* A = Body(W);
			const auto Position = A->X; const float Mass = A->InvMass;
			PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Activation; C.Activation = PhysicsFieldActivation::Freeze; W.AddPhysicsField(C); W.Simulate(0.01f);
			EXPECT(A->Freezing && A->GetInverseMass() == 0 && A->InvMass == Mass); A->ApplyForce(Vector3(100,0,0)); W.Simulate(0.1f);
			EXPECT_NEAR((A->X-Position).Length(),0,1e-6); EXPECT_NEAR(A->V.Length(),0,1e-6);
			auto* Other = Body(W,Vector3(0.3f,0,0),1,1); Other->V = Vector3(-1,0,0); W.Simulate(0.01f);
			EXPECT_NEAR((A->X-Position).Length(),0,1e-6); EXPECT_NEAR(A->V.Length(),0,1e-6);
			EXPECT(W.RemoveRigidBody(Other)); Other->ReleaseGeometries(); delete Other;
			C.Activation = PhysicsFieldActivation::Unfreeze; C.LayerMask = 1; W.AddPhysicsField(C); W.Simulate(0.01f);
			EXPECT(!A->Freezing && A->GetInverseMass() == Mass);
			C.Target = PhysicsFieldTarget::LinearVelocity; C.Vector = VectorField::Constant(Vector3(0,3,0)); W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT_NEAR(A->V.y,3,1e-4);
			C.Target = PhysicsFieldTarget::AngularVelocity; C.Vector = VectorField::Constant(Vector3(0,0,2)); W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT_NEAR(A->W.z,2,1e-4);
			C.Target = PhysicsFieldTarget::Material; C.Persistent = true; C.StaticFriction = ScalarField::Constant(0.8); C.DynamicFriction = ScalarField::Constant(0.2); C.Restitution = ScalarField::Constant(0.4);
			W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT_NEAR(A->GetFrictionStatic(),0.8,1e-6); EXPECT_NEAR(A->GetFrictionDynamic(),0.2,1e-6); EXPECT_NEAR(A->GetRestitution(),0.4,1e-6);
			A->Sleep(); W.Simulate(0.01f); EXPECT(A->Sleeping);
		}
	}
	void SleepingIslandsAndForceRetry()
	{
		PhysicsWorldParam P; P.islandSleeping.Enabled = true; P.gravityAcc = Vector3(0,-4,0);
		PhysicsWorld W(P); auto* A = Body(W,Vector3(-1,2,0)); auto* B = Body(W,Vector3(1,2,0),2,1);
		EXPECT(W.CreateFixedConstraint(A,B)); A->Sleep(); B->Sleep();
		PhysicsFieldCommand C; C.LayerMask = 1; C.Vector = VectorField::Constant(Vector3(8,0,0)); W.AddPhysicsField(C); W.Simulate(0.1f);
		EXPECT(!A->Sleeping && !B->Sleeping); EXPECT_NEAR(A->V.y,-0.4,2e-3); EXPECT_NEAR(B->V.y,-0.4,2e-3);
		EXPECT_NEAR(A->V.x/A->InvMass+B->V.x/B->InvMass,0.8,2e-3);
		P = {}; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4; PhysicsWorld Retry(P); auto* R = Body(Retry);
		C = {}; C.Vector = VectorField::Constant(Vector3(8,0,0)); Retry.AddPhysicsField(C);
		C.Vector = VectorField::Scale(VectorField::Constant(Vector3::UnitX()),ScalarField::Constant(std::numeric_limits<double>::max()));
		const auto Bad = Retry.AddPhysicsField(C); Reject([&] { Retry.Simulate(0.25f); });
		EXPECT(R->ExtForce == Vector3::Zero() && R->V == Vector3::Zero()); EXPECT(Retry.RemovePhysicsField(Bad));
		Retry.Simulate(0.25f); EXPECT_NEAR(R->V.x,1,1e-6); EXPECT(Retry.GetPhysicsFieldCount() == 0);
	}
	void StaticAndKinematicMaterials()
	{
		PhysicsWorld W{PhysicsWorldParam()}; auto* Dynamic = Body(W); const float Original = Dynamic->GetFrictionStatic();
		RigidBodyParam P; auto* S = GeometryFactory::CreateSphere(Vector3(10,0,0),0.2f); S->SetFilterData(CollisionData::ForLayer(1));
		auto* Static = W.CreateRigidBody(S,P); P.rigidType = RigidType::Kinematic;
		auto* K = GeometryFactory::CreateSphere(Vector3(20,0,0),0.2f); K->SetFilterData(CollisionData::ForLayer(1)); auto* Kinematic = W.CreateRigidBody(K,P);
		PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Material; C.LayerMask = 1u << 1;
		C.StaticFriction = ScalarField::Constant(0.9); C.DynamicFriction = ScalarField::Constant(0.7); C.Restitution = ScalarField::Constant(0.2);
		W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT_NEAR(Static->GetFrictionStatic(),0.9,1e-6); EXPECT_NEAR(Kinematic->GetFrictionDynamic(),0.7,1e-6);
		EXPECT_NEAR(Kinematic->GetRestitution(),0.2,1e-6); EXPECT(Dynamic->GetFrictionStatic() == Original);
		C.Mask = ScalarField::Radial(Static->X,0,1); C.StaticFriction = ScalarField::Constant(1);
		W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT(Static->GetFrictionStatic() == 1); EXPECT_NEAR(Kinematic->GetFrictionStatic(),0.9,1e-6);
	}
	void StrainFramesEventsAndCache()
	{
		DestructionSet Author; Author.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 P((float)I*2,0,0); Author.AddChunk(Box3(P-Vector3(0.5f),P+Vector3(0.5f)));
			if (I) EXPECT(Author.AddBond(I-1,I,DestructionBondType::Connect,1,10+I));
		}
		for (size_t I = 0; I < Author.GetGraph().Size(); ++I) for (auto& B : Author.GetGraph()[I].Bonds)
			B.frame.Origin = (Author.GetChunks()[B.v0].Centroid+Author.GetChunks()[B.v1].Centroid)*0.5f;
		Author.GetGraph().RefreshBondCache(); Author.RebuildClustersFromGraph();
		Quaternion Q; Q.FromRotateY(0.8f); const Transform Placement(Vector3(5,3,2),Q);
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(DestructionAsset::Capture(Author),Placement);
		auto* Cluster = S.FindClusterContaining(0); Cluster->GetRigidBody()->X += Vector3(4,2,-1); Cluster->GetRigidBody()->Q.FromRotateZ(0.7f);
		Transform Pose; EXPECT(Cluster->GetWorldTransform(0,Pose));
		const auto Point = Pose.LocalToWorld(S.GetGraph().FindBond(0,1)->frame.Origin-S.GetChunks()[0].Centroid);
		DestructionEventSettings Events; Events.Enabled = true; S.ConfigureEvents(Events); DestructionCacheRecorder Recorder(S);
		PhysicsFieldCommand Freeze; Freeze.Target = PhysicsFieldTarget::Activation; Freeze.Activation = PhysicsFieldActivation::Freeze; W.AddPhysicsField(Freeze);
		PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Strain; C.Mask = ScalarField::Radial(Point,0,0.3); C.Scalar = ScalarField::Constant(2); W.AddPhysicsField(C);
		W.Simulate(0.01f); EXPECT(!S.GetGraph().HasBond(0,1) && S.GetGraph().HasBond(1,2)); EXPECT(W.GetPhysicsFieldStats().BrokenBonds == 1);
		for (auto* Fragment : S.GetClusters()) EXPECT(Fragment->GetRigidBody()->Freezing);
		const auto Batch = S.TakeEvents(); bool FieldCause = false; for (const auto& E : Batch) FieldCause |= E.Cause == DestructionEventCause::Field;
		EXPECT(FieldCause); Recorder.Record(0.01,S,Batch); const auto Cache = Recorder.Finish(); EXPECT(!Cache.GetEvents(0,0.01).empty());
	}
	void ValidationAndLifetime()
	{
		PhysicsWorld W{PhysicsWorldParam()}; auto* A = Body(W); Body(W,Vector3(10,0,0)); A->V = Vector3::UnitY();
		PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::LinearVelocity;
		C.Vector = VectorField::Scale(VectorField::Radial(Vector3::Zero()),ScalarField::Constant(std::numeric_limits<double>::max()));
		const auto Id = W.AddPhysicsField(C); Reject([&] { W.Simulate(0.1f); }); EXPECT(A->V == Vector3::UnitY()); EXPECT(W.RemovePhysicsField(Id));
		C.Mask = ScalarField::Radial(Vector3::Zero(),0,0.2); W.AddPhysicsField(C); W.Simulate(0.01f); EXPECT(A->V == Vector3::Zero());
		C = {}; C.Target = PhysicsFieldTarget::Material; C.StaticFriction = ScalarField::Constant(0.1); C.DynamicFriction = ScalarField::Constant(0.8);
		const auto Material = W.AddPhysicsField(C); const float Old = A->GetFrictionStatic(); Reject([&] { W.Simulate(0.1f); }); EXPECT(A->GetFrictionStatic() == Old); W.RemovePhysicsField(Material);
		C = {}; C.Persistent = true; const auto OldId = W.AddPhysicsField(C); W.Simulate(0.1f); W.Reset(); EXPECT(W.GetPhysicsFieldCount() == 0 && W.GetPhysicsFieldStats().Time == 0);
		const auto NewId = W.AddPhysicsField(C); EXPECT(NewId > OldId && !W.RemovePhysicsField(OldId)); W.ClearPhysicsFields(); EXPECT(W.GetPhysicsFieldCount() == 0);
		C.MinimumMask = std::numeric_limits<double>::quiet_NaN(); Reject([&] { W.AddPhysicsField(C); });
		int Rejected = 0; PhysicsWorld* Active = nullptr;
		PhysicsWorldParam P; P.onContact = [&](const PhysicsContactInfo&)
		{
			try { Active->AddPhysicsField(PhysicsFieldCommand()); } catch (const std::logic_error&) { ++Rejected; }
			try { Active->ClearPhysicsFields(); } catch (const std::logic_error&) { ++Rejected; }
			try { Active->Reset(); } catch (const std::logic_error&) { ++Rejected; }
		};
		PhysicsWorld Callback(P); Active = &Callback; C = {}; C.Persistent = true; Callback.AddPhysicsField(C);
		Body(Callback); Body(Callback,Vector3(0.3f,0,0)); Callback.Simulate(0.01f); EXPECT(Rejected >= 3);
	}
	void ReducedRagdollLoads()
	{
		PhysicsWorld W{PhysicsWorldParam()}; auto* B = Body(W); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1); S.Links[0].Joint = RagdollReducedJointType::Floating; S.Links[0].Mass = 2;
		auto* A = W.CreateRagdoll(S,{B}); PhysicsFieldCommand C; C.Vector = VectorField::Constant(Vector3(8,0,0)); W.AddPhysicsField(C); W.Simulate(0.25f);
		EXPECT_NEAR(B->V.x,1,1e-6); C.Target = PhysicsFieldTarget::Impulse; C.Vector = VectorField::Constant(Vector3(2,0,0)); W.AddPhysicsField(C); W.Simulate(0.25f); EXPECT_NEAR(B->V.x,2,1e-6);
		C.Target = PhysicsFieldTarget::LinearVelocity; const auto Id = W.AddPhysicsField(C); Reject([&] { W.Simulate(0.01f); }); EXPECT(W.RemovePhysicsField(Id)); EXPECT_NEAR(B->V.x,2,1e-6);
		EXPECT(A->GetBodies().size() == 1);
	}
}
void TestPhysicsField()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Physics fields: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Physics field exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("ExpressionsAndBudgets",ExpressionsAndBudgets); Run("UnitsAndSubsteps",UnitsAndSubsteps);
	Run("ActivationVelocityAndMaterial",ActivationVelocityAndMaterial); Run("StrainFramesEventsAndCache",StrainFramesEventsAndCache);
	Run("SleepingIslandsAndForceRetry",SleepingIslandsAndForceRetry);
	Run("StaticAndKinematicMaterials",StaticAndKinematicMaterials);
	Run("ValidationAndLifetime",ValidationAndLifetime); Run("ReducedRagdollLoads",ReducedRagdollLoads);
}

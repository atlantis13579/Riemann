#include "Test.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/SurfaceCollision.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include "../Viewer/SolverPanel.h"
#include <limits>
#include <memory>

using namespace Riemann;
namespace
{
	using V=StrandVector;
	V Double(const Vector3& P) { return V(P.x,P.y,P.z); }
	double Length(const V& P) { return std::hypot(std::hypot(P.x,P.y),P.z); }
	template<class F> void Reject(F Function) { bool Caught=false; try { Function(); } catch (const std::exception&) { Caught=true; } EXPECT(Caught); }
	StrandSettings Settings(bool Continuous=false)
	{
		StrandSettings S; S.Substeps=1; S.Gravity=Vector3::Zero(); S.LinearDamping=S.AngularDamping=0;
		S.Collision.Enabled=true; S.Collision.Continuous=Continuous; S.Collision.Material.StaticFriction=S.Collision.Material.DynamicFriction=0; return S;
	}
	StrandGuide Line(const Vector3& A,const Vector3& B)
	{
		StrandGuide G; G.Positions={A,B}; G.PinRoot=false; G.Material.Density=1; G.Material.RadiusX=G.Material.RadiusY=0.05; return G;
	}
	std::unique_ptr<Geometry> Floor()
	{
		std::unique_ptr<Geometry> Shape(GeometryFactory::CreateTriangleMesh()); auto* Mesh=Shape->GetShapeObj<TriangleMesh>();
		for (const auto& P : {Vector3(-5,0,-5),Vector3(-5,0,5),Vector3(5,0,5),Vector3(5,0,-5)}) Mesh->AddVertex(P);
		Mesh->AddTriangle(0,1,2); Mesh->AddTriangle(0,2,3); Mesh->Compact(); Shape->SetShapeBounds(Mesh->BoundingVolume); return Shape;
	}
	double PairDistance(const StrandSystem& S,size_t A,int EA,size_t B,int EB)
	{
		const Vector3 PA[]={S.GetParticle(A,EA).Position,S.GetParticle(A,EA+1).Position}, PB[]={S.GetParticle(B,EB).Position,S.GetParticle(B,EB+1).Position};
		return SurfaceCollision::Closest(PA,2,PB,2).Distance;
	}
	V Center(const StrandSystem& S)
	{
		V P(0); double M=0; for (size_t G=0; G<S.GetGuideCount(); ++G) for (size_t I=0; I<S.GetParticleCount(G); ++I)
		{ const auto& A=S.GetParticle(G,I); P+=Double(A.Position)/A.InverseMass; M+=1/A.InverseMass; } return P/M;
	}
	void SegmentInteriorAndSnapshots()
	{
		for (auto Algorithm : {StrandAlgorithm::DistanceConstraints,StrandAlgorithm::CosseratRods}) for (bool Capsule : {false,true})
		{
			auto Policy=Settings(); Policy.Algorithm=Algorithm;
			StrandSystem S({Line(Vector3(-2,0,0),Vector3(2,0,0))},Policy);
			std::unique_ptr<Geometry> Shape(Capsule ? GeometryFactory::CreateCapsule(Vector3(0,0.03f,-1),Vector3(0,0.03f,1),0.3f) : GeometryFactory::CreateSphere(Vector3(0,0.03f,0),0.3f));
			EXPECT(S.AddCollider(Shape.get())); Shape.reset(); S.Simulate(0.01f);
			const Vector3 A[]={S.GetParticle(0,0).Position,S.GetParticle(0,1).Position}, B[]={Vector3(0,0.03f,-1),Vector3(0,0.03f,1)};
			const Vector3 Sphere=Vector3(0,0.03f,0); const double Distance=SurfaceCollision::Closest(A,2,Capsule ? B : &Sphere,Capsule ? 2 : 1).Distance;
			EXPECT(Distance>=0.35-1e-5 && S.GetCollisionStats().RigidContacts>0);
		}
	}
	void MeshAndConvexRecovery()
	{
		for (bool Convex : {false,true})
		{
			std::unique_ptr<Geometry> Shape(Convex ? GeometryFactory::CreateConvexMesh() : GeometryFactory::CreateOBB(Vector3::Zero(),Vector3(0.5f)));
			if (Convex)
			{
				std::vector<Vector3> Points; for (int I=0; I<8; ++I) Points.emplace_back(I&1 ? 0.5f : -0.5f,I&2 ? 0.5f : -0.5f,I&4 ? 0.5f : -0.5f);
				auto* Mesh=Shape->GetShapeObj<ConvexMesh>(); EXPECT(Mesh->BuildFromPoints(Points.data(),(int)Points.size())); Shape->SetShapeBounds(Mesh->Bounds);
			}
			StrandSystem S({Line(Vector3(-2,0,0),Vector3(2,0,0))},Settings()); EXPECT(S.AddCollider(Shape.get())); S.Simulate(0.01f);
			EXPECT(std::abs(S.GetParticle(0,0).Position.y)>=0.55-1e-5 || std::abs(S.GetParticle(0,0).Position.z)>=0.55-1e-5);
			EXPECT(S.GetCollisionStats().RigidContacts>0);
		}
		auto Shape=Floor(); StrandSystem S({Line(Vector3(-1,0.04f,0),Vector3(1,0.04f,0))},Settings()); EXPECT(S.AddCollider(Shape.get())); S.Simulate(0.01f);
		EXPECT(S.GetParticle(0,0).Position.y>=0.05-1e-5 && S.GetParticle(0,1).Position.y>=0.05-1e-5);
	}
	void StrandPairsAndSelf()
	{
		auto P=Settings(); P.Collision.InterGuideCollision=true;
		StrandSystem S({Line(Vector3(-1,0,0),Vector3(1,0,0)),Line(Vector3(0,0.04f,-1),Vector3(0,0.04f,1))},P); const auto Before=Center(S); S.Simulate(0.01f);
		EXPECT(PairDistance(S,0,0,1,0)>=0.1-1e-5); EXPECT_NEAR(Length(Center(S)-Before),0,1e-6); EXPECT(S.GetCollisionStats().InterGuideContacts>0);
		StrandGuide Fold=Line(Vector3::Zero(),Vector3::UnitZ()); Fold.Positions={Vector3(-1,0,0),Vector3(1,0,0),Vector3(1,0,1),Vector3(0,0.04f,1),Vector3(0,0.04f,-1)};
		P.Collision.InterGuideCollision=false; P.Collision.SelfCollision=true; StrandSystem Self({Fold},P); const auto SelfCenter=Center(Self); Self.Simulate(0.01f);
		EXPECT(PairDistance(Self,0,0,0,3)>=0.1-1e-5); EXPECT_NEAR(Length(Center(Self)-SelfCenter),0,2e-6); EXPECT(Self.GetCollisionStats().SelfContacts>0);
		Fold.Positions.clear(); for (int I=0; I<12; ++I) Fold.Positions.emplace_back(0,0,(float)I*0.01f); StrandSystem Neighbors({Fold},P); Neighbors.Simulate(0.01f); EXPECT(Neighbors.GetCollisionStats().SelfContacts==0);
	}
	void ContinuousCrossing()
	{
		for (bool Continuous : {false,true})
		{
			StrandSystem S({Line(Vector3(-1,0.5f,0),Vector3(1,0.5f,0))},Settings(Continuous)); auto Shape=Floor(); EXPECT(S.AddCollider(Shape.get()));
			S.SetParticleVelocity(0,0,Vector3(0,-20,0)); S.SetParticleVelocity(0,1,Vector3(0,-20,0)); S.Simulate(0.1f);
			if (Continuous) { EXPECT(S.GetParticle(0,0).Position.y>=0.05-1e-5 && S.GetParticle(0,1).Position.y>=0.05-1e-5); EXPECT(S.GetCollisionStats().ContinuousContacts>0); }
			else EXPECT(S.GetParticle(0,0).Position.y<0);
		}
		auto P=Settings(true); P.Collision.InterGuideCollision=true;
		StrandSystem S({Line(Vector3(-1,0,0),Vector3(1,0,0)),Line(Vector3(0,0.5f,-1),Vector3(0,0.5f,1))},P);
		S.SetParticleVelocity(1,0,Vector3(0,-20,0)); S.SetParticleVelocity(1,1,Vector3(0,-20,0)); S.Simulate(0.05f);
		EXPECT(PairDistance(S,0,0,1,0)>=0.1-1e-5); EXPECT(S.GetParticle(1,0).Position.y>S.GetParticle(0,0).Position.y); EXPECT_NEAR(Center(S).y,-0.25,1e-5);
	}
	void FrictionAndMotion()
	{
		for (bool Friction : {false,true})
		{
			PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam B; B.rigidType=RigidType::Static;
			auto* Shape=GeometryFactory::CreateOBB(Vector3(0,-0.5f,0),Vector3(5,0.5f,5)); auto* Body=W.CreateRigidBody(Shape,B);
			auto P=Settings(); if (Friction) P.Collision.Material.StaticFriction=P.Collision.Material.DynamicFriction=100;
			StrandSystem S({Line(Vector3(-1,0.03f,0),Vector3(1,0.03f,0))},P); EXPECT(S.AddCollider(Shape)); Body->X.x+=0.1f; S.Simulate(0.1f);
			const double X=(S.GetParticle(0,0).Position.x+S.GetParticle(0,1).Position.x)*0.5;
			if (Friction) EXPECT(X>0.05); else EXPECT_NEAR(X,0,1e-6);
		}
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam B; B.rigidType=RigidType::Static; auto* Shape=GeometryFactory::CreateSphere(Vector3::Zero(),0.5f); auto* Body=W.CreateRigidBody(Shape,B);
		auto P=Settings(); P.Collision.Material.StaticFriction=P.Collision.Material.DynamicFriction=100;
		StrandSystem S({Line(Vector3(-0.4f,0.53f,0),Vector3(0.4f,0.53f,0))},P); EXPECT(S.AddCollider(Shape)); Body->Q.FromRotateZ(0.2f); S.Simulate(0.1f);
		EXPECT((S.GetParticle(0,0).Position.x+S.GetParticle(0,1).Position.x)*0.5f<-0.05f);
	}
	void RefinementAndLifetime()
	{
		auto P=Settings(true); P.Collision.SweepIterations=1; P.Collision.SweepRefinements=0; P.Collision.MaximumRefinements=0;
		StrandSystem S({Line(Vector3(-1,1,0),Vector3(1,1,0))},P); auto Shape=Floor(); EXPECT(S.AddCollider(Shape.get()));
		for (int I=0; I<2; ++I) { S.SetParticleVelocity(0,I,Vector3(0,-20,0)); S.AddForce(0,I,Vector3(0.001f,0,0)); }
		const auto Before=S.GetParticle(0,0); Reject([&] { S.Simulate(0.1f); }); EXPECT(S.GetParticle(0,0).Position==Before.Position && S.GetParticle(0,0).Velocity==Before.Velocity);
		EXPECT(S.GetStats().Time==0 && S.GetStats().FailedSteps==1 && S.GetCollisionStats().UnresolvedSweeps>0);
		P.Collision.SweepIterations=64; P.Collision.SweepRefinements=64; P.Collision.MaximumRefinements=64; S.SetCollisionSettings(P.Collision); S.Simulate(0.1f); EXPECT(S.GetParticle(0,0).Position.y>=0.05-1e-5 && S.GetParticle(0,0).Velocity.x>0);
		PhysicsWorld W{PhysicsWorldParam()}, Other{PhysicsWorldParam()}; RigidBodyParam B; B.rigidType=RigidType::Static;
		auto* Bound=W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.2f),B); auto* Foreign=Other.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.2f),B);
		auto* Owned=W.CreateStrands({Line(Vector3(-1,1,0),Vector3(1,1,0))},Settings()); EXPECT(Owned->AddCollider(Bound->Geometries()[0])); Reject([&] { Owned->AddCollider(Foreign->Geometries()[0]); });
		EXPECT(W.RemoveRigidBody(Bound)); Bound->ReleaseGeometries(); delete Bound; EXPECT(Owned->GetColliderCount()==0); W.Simulate(0.01f);
		P=Settings(true); P.Collision.SweepIterations=1; P.Collision.SweepRefinements=0; P.Collision.MaximumRefinementDepth=24;
		StrandSystem Refined({Line(Vector3(-1,1,0),Vector3(1,1,0))},P); EXPECT(Refined.AddCollider(Shape.get()));
		for (int I=0; I<2; ++I) Refined.SetParticleVelocity(0,I,Vector3(0,-20,0)); Refined.Simulate(0.1f);
		EXPECT(Refined.GetCollisionStats().Refinements>0 && Refined.GetStats().Substeps>1); EXPECT(Refined.GetParticle(0,0).Position.y>=0.05-1e-5);
	}
	void DisabledHistoryAndAngularMotion()
	{
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam B; B.rigidType=RigidType::Static;
		auto* Shape=GeometryFactory::CreateSphere(Vector3(-2,0,0),0.2f); auto* Body=W.CreateRigidBody(Shape,B);
		auto P=Settings(true); P.Collision.Enabled=false; StrandSystem S({Line(Vector3(0,0,-0.5f),Vector3(0,0,0.5f))},P); EXPECT(S.AddCollider(Shape));
		Body->X.x+=4; S.Simulate(0.01f); P.Collision.Enabled=true; P.Collision.SweepIterations=1; P.Collision.SweepRefinements=P.Collision.MaximumRefinements=0;
		S.SetCollisionSettings(P.Collision); S.Simulate(0.01f); EXPECT(S.GetCollisionStats().Candidates==0);
		EXPECT(!S.RemoveCollider(nullptr)); EXPECT(S.AddCapsuleCollider(nullptr,0.1f,Vector3(5,0,0),Vector3(5,0,1))); S.DetachBody(nullptr); EXPECT(S.GetColliderCount()==2); S.ClearColliders(); EXPECT(S.GetColliderCount()==0);
		P=Settings(true); P.Collision.MaximumColliderAngle=0.1; StrandSystem Rotating({Line(Vector3(-0.2f,0.6f,0),Vector3(0.2f,0.6f,0))},P);
		auto* Capsule=GeometryFactory::CreateCapsule(Vector3(-1,0,0),Vector3(1,0,0),0.1f); auto* Rotor=W.CreateRigidBody(Capsule,B); EXPECT(Rotating.AddCollider(Capsule));
		Rotor->Q.FromRotateZ(0.2f); Rotating.Simulate(0.01f); EXPECT(Rotating.GetStats().Substeps>=2);
		// A half-turn returns this capsule to the same occupied volume. Endpoint
		// overlap alone cannot detect the intervening arc through the guide.
		Rotor->Q=Quaternion::One(); StrandSystem Arc({Line(Vector3(0,0.7f,-0.2f),Vector3(0,0.7f,0.2f))},P); EXPECT(Arc.AddCollider(Capsule));
		Rotor->Q=Quaternion(0,0,1,0); Arc.Simulate(0.1f); EXPECT(Arc.GetCollisionStats().ContinuousContacts>0);
		EXPECT(Length(Center(Arc)-V(0,0.7,0))>0.05);
		for (bool Continuous : {false,true})
		{
			Body->X=Vector3(-2,0,0); auto Policy=Settings(Continuous); StrandSystem Sweep({Line(Vector3(0,0,-1),Vector3(0,0,1))},Policy); EXPECT(Sweep.AddCollider(Shape));
			Body->X.x+=4; Sweep.Simulate(0.1f);
			if (Continuous) EXPECT(Center(Sweep).x>=2.25-1e-5); else EXPECT_NEAR(Center(Sweep).x,0,1e-6);
		}
	}
	void ExistingSceneContacts()
	{
		SceneWorld Scene; Scene.SetStrandDemo(true); std::string Error; EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error)); auto* S=Scene.GetDemoStrands(); EXPECT(S && S->GetGroom()); if (!S) return;
		EXPECT(S->GetColliderCount()==2); const float RootHeight=0.04f, Shift=RootHeight-S->GetRootPose(0).pos.y;
		// Position the existing procedural groom just above the existing ground.
		// Its own roots are edited independently through the attachment API.
		for (size_t G=0; G<S->GetGuideCount(); ++G) { auto Pose=S->GetRootPose(G); Pose.pos.y+=Shift; S->SetRootPose(G,Pose); }
		for (size_t C=0; C<S->GetGroom()->GetStrandCount(); ++C) { auto P=S->GetGroomRootPosition(C); P.y+=Shift; S->SetGroomRootPosition(C,P); }
		S->Reset(); EXPECT(Scene.ApplyStrandCollisionDemo(true,true,true,false)); const float Dt=1.0f/120; uint64_t Contacts=0;
		for (size_t Level : {size_t(0),size_t(1),size_t(2),size_t(0)})
		{
			S->SetGroomLod(Level);
			for (int Frame=0; Frame<32; ++Frame)
			{
				Scene.Step(Dt); Contacts+=S->GetCollisionStats().RigidContacts;
				for (size_t G=0; G<S->GetGuideCount(); ++G) if (S->IsGuideSimulated(G)) for (size_t I=0; I<S->GetParticleCount(G); ++I) EXPECT(S->GetParticle(G,I).Position.y>=0.035-2e-5);
			}
			SolverPanelState Panel; RefreshSolverPanelTelemetry(Panel,Scene); EXPECT(Panel.StrandContactRows==S->GetCollisionStats().Contacts && Panel.StrandContactCandidates==S->GetCollisionStats().Candidates);
			std::printf("Strand contacts scene LOD=%zu rows=%llu refinements=%llu gap=%g\n",Level,(unsigned long long)Panel.StrandContactRows,(unsigned long long)Panel.StrandContactRefinements,Panel.StrandContactPenetration);
		}
		EXPECT(Contacts>0); EXPECT(Scene.ApplyStrandCollisionDemo(false,false,false,true)); Scene.Step(Dt); EXPECT(S->GetCollisionStats().Contacts==0);
	}
	void SpatialIndexAndValidation()
	{
		auto P=Settings(); P.Collision.InterGuideCollision=true; P.SolverIterations=1; std::vector<StrandGuide> Guides;
		for (int I=0; I<128; ++I) Guides.push_back(Line(Vector3((float)I*5,0,0),Vector3((float)I*5,0,1))); StrandSystem S(Guides,P); S.Simulate(0.01f); EXPECT(S.GetCollisionStats().Candidates==0 && S.GetCollisionStats().Contacts==0);
		P.Collision.RadiusScale=0; Reject([&] { S.SetCollisionSettings(P.Collision); }); EXPECT(!S.AddCollider(nullptr));
		SurfaceCollisionMesh Index(4,{},{{0,1},{2,3}}); Index.Update({Vector3(-1,0,0),Vector3(1,0,0),Vector3(0,0,-1),Vector3(0,0,1)},{Vector3(-1,0,0),Vector3(1,0,0),Vector3(0,0,-1),Vector3(0,0,1)},0.1f);
		std::vector<SurfaceCandidate> Candidates; Index.FindSelfCandidates(Candidates); EXPECT(Candidates.size()==1 && Candidates[0].Type==SurfacePairType::EdgeEdge);
		Reject([] { SurfaceCollisionMesh Bad(2,{},{{0,2}}); });
	}
}
void TestStrandCollision()
{
	SegmentInteriorAndSnapshots(); MeshAndConvexRecovery(); StrandPairsAndSelf(); ContinuousCrossing(); FrictionAndMotion(); RefinementAndLifetime(); DisabledHistoryAndAngularMotion(); ExistingSceneContacts(); SpatialIndexAndValidation();
}

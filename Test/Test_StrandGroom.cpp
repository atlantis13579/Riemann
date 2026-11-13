#include "Test.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Viewer/SceneWorld.h"
#include "../Viewer/SolverPanel.h"
#include <limits>
#include <stdexcept>

using namespace Riemann;
namespace
{
	using V=StrandVector;
	V Double(const Vector3& P) { return V(P.x,P.y,P.z); }
	double Length(const V& P) { return std::hypot(std::hypot(P.x,P.y),P.z); }
	void Near(const Vector3& A,const Vector3& B,double Tolerance) { EXPECT_NEAR(Length(Double(A)-Double(B)),0,Tolerance); }
	void Near(const Quaternion& A,const Quaternion& B,double Tolerance)
	{
		const double Error=Length(StrandElasticity::Log(A*B.Conjugate()));
		if (Error>Tolerance) std::printf("Groom orientation residual %g: (%g,%g,%g,%g), (%g,%g,%g,%g)\n",Error,A.x,A.y,A.z,A.w,B.x,B.y,B.z,B.w);
		EXPECT_NEAR(Error,0,Tolerance);
	}
	template<class F> void Reject(F Function) { bool Caught=false; try { Function(); } catch (const std::exception&) { Caught=true; } EXPECT(Caught); }
	StrandSettings Quiet() { StrandSettings S; S.Gravity=Vector3::Zero(); S.LinearDamping=S.AngularDamping=0; return S; }
	std::vector<StrandGuide> Guides(size_t Count=4)
	{
		std::vector<StrandGuide> Result;
		for (size_t I=0; I<Count; ++I)
		{
			StrandGuide G; const float X=(float)I*0.2f; G.Positions={Vector3(X,0,0),Vector3(X,0,0.3f),Vector3(X,0,1)};
			G.Material.RadiusX=G.Material.RadiusY=0.1; G.Material.Density=1; Result.push_back(G);
		}
		return Result;
	}
	StrandGroomDefinition Definition(size_t Count=4)
	{
		StrandGroomDefinition D;
		GroomLod Full; for (size_t I=0; I<Count; ++I) Full.Guides.push_back(I); D.Lods.push_back(Full);
		if (Count>2) { GroomLod Medium; Medium.Guides={0,2}; Medium.Distance=10; D.Lods.push_back(Medium); }
		if (Count>1) { GroomLod Low; Low.Guides={0}; Low.Distance=25; D.Lods.push_back(Low); }
		for (size_t I=0; I<Count*4; ++I)
		{
			GroomStrand R; const float X=(float)I*(Count-1)*0.2f/(Count*4-1);
			R.Positions={Vector3(X,0.02f,0),Vector3(X+0.01f,0.03f,0.15f),Vector3(X-0.01f,0.01f,0.6f),Vector3(X,0.02f,1)}; D.Strands.push_back(R);
		}
		return D;
	}
	void RestAndRigidMotion()
	{
		const auto G=Guides(); const auto D=Definition(); StrandSystem S(G,Quiet()); S.ConfigureGroom(D);
		EXPECT(S.GetGroom()->GetStrandCount()==D.Strands.size());
		for (size_t C=0; C<D.Strands.size(); ++C) for (size_t I=0; I<D.Strands[C].Positions.size(); ++I) Near(S.GetGroomStrand(C).Positions[I],D.Strands[C].Positions[I],1e-6);
		const auto Q=StrandElasticity::Increment(V(0.4,-0.7,0.2)); const Transform T(Vector3(2,-3,4),Q);
		for (size_t I=0; I<G.size(); ++I) S.SetRootPose(I,Transform(T.LocalToWorld(G[I].Positions[0]),Q)); S.Reset();
		for (size_t C=0; C<D.Strands.size(); ++C)
		{
			const auto& Rest=S.GetGroom()->GetStrand(C);
			for (size_t I=0; I<Rest.Positions.size(); ++I) Near(S.GetGroomStrand(C).Positions[I],T.LocalToWorld(Rest.Positions[I]),2e-6);
			for (size_t I=0; I<Rest.Directors.size(); ++I) Near(S.GetGroomStrand(C).Directors[I],Q*Rest.Directors[I],2e-6);
		}
		S.SetGroomLod(2); S.Simulate(0.01f);
		for (size_t C=0; C<D.Strands.size(); ++C) for (size_t I=0; I<D.Strands[C].Positions.size(); ++I) Near(S.GetGroomStrand(C).Positions[I],T.LocalToWorld(D.Strands[C].Positions[I]),2e-5);
	}
	void ArcLengthAndTwist()
	{
		auto G=Guides(1); StrandSystem S(G,Quiet()); auto D=Definition(1); D.Strands.clear(); GroomStrand R;
		R.Positions={Vector3(0.05f,0,0),Vector3(0.05f,0,0.25f),Vector3(0.05f,0,0.9f),Vector3(0.05f,0,1)};
		R.Influences={{0,7}}; D.Strands.push_back(R); S.ConfigureGroom(D); EXPECT_NEAR(S.GetGroom()->GetStrand(0).Influences[0].Weight,1,1e-12);
		for (size_t I=0; I<G[0].Positions.size(); ++I) { auto P=G[0].Positions[I]; P.z*=2; S.SetParticlePosition(0,I,P); } S.RefreshGroom();
		for (size_t I=0; I<R.Positions.size(); ++I) { auto Expected=R.Positions[I]; Expected.z*=2; Near(S.GetGroomStrand(0).Positions[I],Expected,1e-6); }
		S.Reset(); const auto Q=StrandElasticity::Increment(V(0,0,0.7));
		for (size_t I=0; I<S.GetParticleCount(0); ++I) S.PinParticle(0,I,G[0].Positions[I]);
		for (size_t I=0; I<S.GetDirectorCount(0); ++I) S.PinDirector(0,I,Q); S.Simulate(0.01f);
		for (size_t I=0; I<R.Positions.size(); ++I) Near(S.GetGroomStrand(0).Positions[I],Q*R.Positions[I],1e-6);
		for (const auto& Frame : S.GetGroomStrand(0).Directors) Near(Frame,Q,1e-6);
	}
	void QuaternionGaugeInvariance()
	{
		auto Evaluate=[](bool NegativeRest, const Quaternion& Global)
		{
			auto G=Guides(2); if (NegativeRest) G[1].Directors.assign(2,Quaternion(0,0,0,-1));
			StrandSystem S(G,Quiet()); auto D=Definition(2); D.Strands.resize(1);
			D.Strands[0].Positions={Vector3(0.1f,0,0),Vector3(0.1f,0,1)}; D.Strands[0].Influences={{0,1},{1,1}}; S.ConfigureGroom(D);
			for (size_t Curve=0; Curve<2; ++Curve)
			{
				for (size_t I=0; I<S.GetParticleCount(Curve); ++I) S.PinParticle(Curve,I,Global*G[Curve].Positions[I]);
				for (size_t I=0; I<S.GetDirectorCount(Curve); ++I) S.PinDirector(Curve,I,Global*(Curve ? Quaternion(0,0,1,0) : Quaternion::One()));
			}
			S.Simulate(0.01f); return S.GetGroomStrand(0).Directors[0];
		};
		// Quaternion signs are a representation choice, including at a half turn.
		const auto Base=Evaluate(false,Quaternion::One()), Global=Quaternion(1,0,0,0);
		Near(Base,Evaluate(true,Quaternion::One()),1e-6); Near(Global*Base,Evaluate(false,Global),1e-6);
		auto TwistFrame=[](bool Negative)
		{
			auto G=Guides(1); StrandSystem S(G,Quiet()); auto D=Definition(1); D.Strands.resize(1);
			D.Strands[0].Positions={Vector3(0.05f,0,0),Vector3(0.05f,0,0.2f),Vector3(0.05f,0,0.6f),Vector3(0.05f,0,1)}; S.ConfigureGroom(D);
			for (size_t I=1; I<S.GetParticleCount(0); ++I) S.PinParticle(0,I,G[0].Positions[I]);
			S.PinDirector(0,1,Quaternion(0,0,Negative ? -1.0f : 1.0f,0)); S.Simulate(0.01f); return S.GetGroomStrand(0).Directors[1];
		};
		Near(TwistFrame(false),TwistFrame(true),1e-6);
	}
	void LevelsAndTransfer()
	{
		auto Settings=Quiet(); Settings.LinearDamping=Settings.AngularDamping=2; StrandSystem S(Guides(),Settings); const auto D=Definition(); S.ConfigureGroom(D); const float Dt=1.0f/120;
		for (int Frame=0; Frame<20; ++Frame) { S.AddForce(1,2,Vector3(0.2f,0.1f,0)); S.Simulate(Dt); }
		const auto Position=S.GetParticle(1,2).Position, Velocity=S.GetParticle(1,2).Velocity; const auto Q=S.GetDirector(1,1).Orientation;
		const auto Render=S.GetGroomStrand(3).Positions; const auto FullWork=S.GetStats().ConstraintSolves;
		S.SetGroomLod(1); Near(S.GetParticle(1,2).Position,Position,0); Near(S.GetParticle(1,2).Velocity,Velocity,0); Near(S.GetDirector(1,1).Orientation,Q,1e-6); EXPECT(S.GetGroomStrand(3).Positions==Render);
		double Weight=0; for (size_t G=0; G<S.GetGuideCount(); ++G) Weight+=S.GetGroom()->GetSimulationWeight(1,G); EXPECT_NEAR(Weight,4,1e-12);
		const float TinyStep=0.0001f; S.Simulate(TinyStep);
		Near(S.GetParticle(1,2).Position,Position+Velocity*TinyStep,2e-6);
		EXPECT(S.GetStats().SimulatedGuides==2 && S.GetStats().InterpolatedGuides==2 && S.GetStats().ConstraintSolves*2==FullWork);
		Reject([&] { S.PinParticle(1,1,Vector3::Zero()); }); Reject([&] { S.UnpinParticle(0,0); }); Reject([&] { S.SetParticleVelocity(1,1,Vector3::Zero()); });
		S.SetRootPose(3,Transform(Vector3(0.6f,0.2f,0),StrandElasticity::Increment(V(0,0,0.3))));
		for (int I=0; I<40; ++I) { S.Simulate(Dt); Near(S.GetParticle(3,0).Position,S.GetRootPose(3).pos,1e-6); Near(S.GetDirector(3,0).Orientation,S.GetRootPose(3).quat,1e-6); }
		const auto P=S.GetParticle(1,2); const auto F=S.GetDirector(1,1); const auto Omega=S.GetAngularVelocity(1,1); S.SetGroomLod(0);
		Near(S.GetParticle(1,2).Position,P.Position,0); Near(S.GetParticle(1,2).Velocity,P.Velocity,0); Near(S.GetDirector(1,1).Orientation,F.Orientation,1e-6); EXPECT_NEAR(Length(S.GetAngularVelocity(1,1)-Omega),0,1e-8);
		S.Simulate(Dt); EXPECT(std::isfinite(S.GetStats().ElasticEnergy)); S.PinParticle(1,1,S.GetParticle(1,1).Position); Reject([&] { S.SetGroomLod(1); }); EXPECT(S.GetGroomLod()==0); S.UnpinParticle(1,1);
		S.UpdateGroomDistance(10.4); EXPECT(S.GetGroomLod()==0); S.UpdateGroomDistance(10.6); EXPECT(S.GetGroomLod()==1);
		S.UpdateGroomDistance(9.8); EXPECT(S.GetGroomLod()==1); S.UpdateGroomDistance(9.4); EXPECT(S.GetGroomLod()==0);
		S.UpdateGroomDistance(30); EXPECT(S.GetGroomLod()==2); S.UpdateGroomDistance(0); EXPECT(S.GetGroomLod()==0);
		S.SetGroomLod(2); S.ClearGroom(); EXPECT(!S.GetGroom() && S.GetStats().SimulatedGuides==4); S.UnpinParticle(1,0);
	}
	void RoutedLoads()
	{
		for (int Count : {1,4,8})
		{
			PhysicsWorldParam P; P.temporalSubsteps.Enabled=true; P.temporalSubsteps.Substeps=Count; PhysicsWorld W(P);
			auto G=Guides(2); for (auto& Curve : G) Curve.Positions.erase(Curve.Positions.begin()+1); G[1].Positions[0].x=G[1].Positions[1].x=1;
			auto Settings=Quiet(); Settings.LinearDamping=Settings.AngularDamping=20; auto* S=W.CreateStrands(G,Settings); auto D=Definition(2); D.Strands.clear(); D.Lods[1].TransitionTime=0; S->ConfigureGroom(D);
			const float Force=0.01f; S->AddForce(1,1,Vector3(0,Force,0)); S->SetGroomLod(1); W.Simulate(0.02f);
			EXPECT_NEAR(S->GetRootTorque(0).z,Force,1e-7); EXPECT_NEAR(Length(S->GetRootForce(1)),0,1e-12);
			for (int Frame=0; Frame<40; ++Frame) { S->AddForce(1,1,Vector3(0,Force,0)); W.Simulate(0.01f); }
			EXPECT_NEAR(S->GetRootForce(0).y,Force,1e-5); EXPECT_NEAR(S->GetRootTorque(0).x,-Force,1e-5); EXPECT_NEAR(S->GetRootTorque(0).z,Force,1e-7);
			W.Simulate(0.01f); EXPECT_NEAR(S->GetRootTorque(0).z,0,1e-7); S->Reset(); S->AddTorque(1,0,Vector3(0,0,0.02f)); W.Simulate(0.01f); EXPECT_NEAR(S->GetRootTorque(0).z,0.02,1e-7);
		}
	}
	void Validation()
	{
		StrandSystem S(Guides(),Quiet()); auto D=Definition(); S.ConfigureGroom(D); const auto* Original=S.GetGroom();
		auto Bad=D; Bad.Strands[0].Positions[1]=Bad.Strands[0].Positions[0]; Reject([&] { S.ConfigureGroom(Bad); }); EXPECT(S.GetGroom()==Original);
		Bad=D; Bad.Lods[1].Guides={1,1}; Reject([&] { S.ConfigureGroom(Bad); });
		Bad=D; Bad.Lods[2].Guides={3}; Reject([&] { S.ConfigureGroom(Bad); });
		Bad=D; Bad.GuideGroups={0,0,1,1}; Bad.Lods[1].Guides={0}; Reject([&] { S.ConfigureGroom(Bad); });
		Bad=D; Bad.Strands[0].Influences={{0,std::numeric_limits<double>::infinity()}}; Reject([&] { S.ConfigureGroom(Bad); });
		Bad=D; Bad.Strands[0].Influences={{0,1},{0,1}}; Reject([&] { S.ConfigureGroom(Bad); });
		Reject([&] { S.SetGroomLod(99); }); Reject([&] { S.UpdateGroomDistance(-1); });
		auto Free=Guides(1); Free[0].PinRoot=false; StrandSystem Floating(Free,Quiet()); Reject([&] { Floating.ConfigureGroom(Definition(1)); });
		D.Interpolation.LodHysteresis=100; S.ConfigureGroom(D); S.UpdateGroomDistance(16); EXPECT(S.GetGroomLod()==1); S.UpdateGroomDistance(0); EXPECT(S.GetGroomLod()==0);
	}
	void RenderRootBindings()
	{
		PhysicsWorld W{PhysicsWorldParam()}, Other{PhysicsWorldParam()}; RigidBodyParam P; P.rigidType=RigidType::Static;
		auto* Root=W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.1f),P);
		auto* Foreign=Other.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.1f),P);
		const auto G=Guides(1); auto* S=W.CreateStrands(G,Quiet()); const auto D=Definition(1); S->ConfigureGroom(D); const auto RestRoot=D.Strands[0].Positions[0];
		S->AttachGroomRoot(0,Root,RestRoot); Reject([&] { S->AttachGroomRoot(0,Foreign,RestRoot); });
		const auto Twist=StrandElasticity::Increment(V(0,0,0.7));
		for (size_t I=0; I<S->GetParticleCount(0); ++I) S->PinParticle(0,I,G[0].Positions[I]);
		for (size_t I=0; I<S->GetDirectorCount(0); ++I) S->PinDirector(0,I,Twist);
		W.Simulate(0.01f); Near(S->GetGroomStrand(0).Positions.front(),RestRoot,1e-6); Near(S->GetGroomStrand(0).Positions.back(),Twist*D.Strands[0].Positions.back(),1e-6);
		Root->X=Vector3(0.3f,0.1f,0.2f); Root->Q=StrandElasticity::Increment(V(0,0,0.1)); W.Simulate(0.01f);
		const auto Bound=Root->BodyLocalToWorld(RestRoot); Near(S->GetGroomStrand(0).Positions.front(),Bound,1e-6);
		EXPECT(W.RemoveRigidBody(Root)); Root->ReleaseGeometries(); delete Root; W.Simulate(0.01f); Near(S->GetGroomStrand(0).Positions.front(),Bound,1e-6);
		S->UnpinGroomRoot(0); S->RefreshGroom(); Near(S->GetGroomStrand(0).Positions.front(),Twist*RestRoot,1e-6);
	}
	void FailedRenderRestoresStep()
	{
		StrandSystem S(Guides(1),Quiet()); auto D=Definition(1); D.Strands.resize(1);
		// Finite authored coordinates and a finite root target can still produce
		// an unrepresentable skinned point. Fail after dynamics, before publication.
		D.Strands[0].Positions={Vector3::Zero(),Vector3(3e38f,0,0.5f),Vector3::UnitZ()}; S.ConfigureGroom(D);
		const auto Before=S.GetGroomStrand(0).Positions; const auto Particle=S.GetParticle(0,2);
		S.SetGroomRootPosition(0,Vector3(std::numeric_limits<float>::max(),0,0)); S.AddForce(0,2,Vector3(0,0.001f,0));
		Reject([&] { S.Simulate(0.01f); }); EXPECT(S.GetGroomStrand(0).Positions==Before); Near(S.GetParticle(0,2).Position,Particle.Position,0);
		EXPECT(S.GetStats().FailedSteps==1 && S.GetStats().Time==0);
		S.SetGroomRootPosition(0,Vector3::Zero()); S.Simulate(0.01f); EXPECT(S.GetParticle(0,2).Velocity.y>0);
	}
	Geometry* Object(SceneWorld& S,const char* Id) { for (const auto& O : S.GetObjects()) if (O.Id==Id) return O.GeometryPtr; return nullptr; }
	void ExistingSceneLods()
	{
		SceneWorld Scene; Scene.SetStrandDemo(true); std::string Error; EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error));
		auto* S=Scene.GetDemoStrands(); EXPECT(S && S->GetGroom()); if (!S || !S->GetGroom()) return;
		auto* Handle=Object(Scene,"handle"); auto* Surface=Object(Scene,"groom_strand_0"); EXPECT(Handle && Surface); if (!Handle || !Surface) return;
		auto* Mesh=Surface->GetShapeObj<TriangleMesh>(); EXPECT(Mesh && Mesh->NumVertices>0); if (!Mesh) return;
		EXPECT(Scene.SelectMouseControlGeometry(Handle)); const auto Origin=Handle->GetParent<RigidBody>()->X; const float Dt=1.0f/120; uint64_t FullWork=0;
		for (size_t Level : {size_t(0),size_t(1),size_t(2),size_t(0)})
		{
			const auto Before=S->GetGroomStrand(0).Positions; EXPECT(Scene.ApplyStrandDemo(0.4f,true,Level)); EXPECT(S->GetGroomStrand(0).Positions==Before);
			for (int Frame=0; Frame<24; ++Frame)
			{
				const float T=(float)S->GetStats().Time+Dt; EXPECT(Scene.SetMouseControlPosition(Origin+Vector3(0.05f*std::sin(T),0.05f*T,0),Dt)); Scene.Step(Dt);
				for (size_t C=0; C<S->GetGroom()->GetStrandCount(); ++C)
				{
					const auto& Curve=S->GetGroomStrand(C); Near(Curve.Positions.front(),S->GetGroomRootPosition(C),1e-6);
					for (const auto& Position : Curve.Positions) EXPECT(std::isfinite(Position.x) && std::isfinite(Position.y) && std::isfinite(Position.z));
				}
			}
			SolverPanelState Panel; RefreshSolverPanelTelemetry(Panel,Scene); const size_t Active=S->GetGroom()->GetLod(Level).Guides.size();
			EXPECT(Panel.StrandActiveGuides==Active && Panel.StrandRenderCurves==32 && Panel.StrandGuides==8);
			if (Level==0) FullWork=S->GetStats().ConstraintSolves; EXPECT(S->GetStats().ConstraintSolves*8==FullWork*Active);
			std::printf("Groom scene LOD=%zu guides=%zu render=%llu constraint blocks=%llu strain=%g\n",Level,Active,(unsigned long long)Panel.StrandRenderCurves,(unsigned long long)S->GetStats().ConstraintSolves,S->GetStats().MaximumStretchShear);
			const int Sides=8; Vector3 Center=Vector3::Zero(); for (int I=0; I<Sides; ++I) Center+=Mesh->Vertices[I]; Near(Center/(float)Sides,S->GetGroomStrand(0).Positions[0],1e-6);
		}
		S->ClearGroom(); Scene.Step(Dt); EXPECT(Mesh->NumVertices==0); SolverPanelState Panel; RefreshSolverPanelTelemetry(Panel,Scene); EXPECT(Panel.StrandRenderCurves==0);
		EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error)); EXPECT(Scene.GetDemoStrands()->GetGroom()->GetStrandCount()==32);
	}
}
void TestStrandGroom()
{
	RestAndRigidMotion(); ArcLengthAndTwist(); QuaternionGaugeInvariance(); LevelsAndTransfer(); RoutedLoads(); Validation(); RenderRootBindings(); FailedRenderRestoresStep(); ExistingSceneLods();
}

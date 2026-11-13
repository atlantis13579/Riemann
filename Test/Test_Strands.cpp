#include "Test.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Viewer/SceneWorld.h"
#include "../Viewer/SolverPanel.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

using namespace Riemann;
namespace
{
	using V = StrandVector;
	using M = StrandMatrix;
	template<class F> void Reject(F Function)
	{ bool Caught=false; try { Function(); } catch (const std::exception&) { Caught=true; } EXPECT(Caught); }
	void Near(const V& A, const V& B, double Tolerance)
	{
		if ((A-B).Length()>Tolerance) std::printf("Strand vector residual %.12g (limit %.12g): actual (%g,%g,%g), expected (%g,%g,%g)\n",(A-B).Length(),Tolerance,A.x,A.y,A.z,B.x,B.y,B.z);
		EXPECT_NEAR((A-B).Length(),0,Tolerance);
	}
	V Double(const Vector3& A) { return V(A.x,A.y,A.z); }
	Quaternion Turn(const V& A) { return StrandElasticity::Increment(A); }
	StrandGuide Straight(int Segments=2)
	{
		StrandGuide G; for (int I=0; I<=Segments; ++I) G.Positions.push_back(Vector3(0,0,(float)I));
		G.Material.RadiusX=0.1; G.Material.RadiusY=0.07; G.Material.Density=1; return G;
	}
	StrandSettings Quiet(int Substeps=4)
	{
		StrandSettings S; S.Substeps=Substeps; S.Gravity=Vector3::Zero(); S.LinearDamping=S.AngularDamping=0; return S;
	}
	void MaterialAndLinearAlgebra()
	{
		StrandMaterial S; S.RadiusX=0.2; S.RadiusY=0.1; S.YoungModulus=200; S.Density=3;
		const double Area=std::acos(-1.0)*0.02, Length=2, Shear=200/(2*(1+S.PoissonRatio));
		Near(S.StretchShearStiffness(Length),V(Shear*Area*Length,Shear*Area*Length,200*Area*Length),1e-12);
		Near(S.BendTwistStiffness(Length),V(200*Area*0.01/4/Length,200*Area*0.04/4/Length,Shear*Area*0.04*0.01/0.05/Length),1e-12);
		Near(S.DirectorInertia(Length),V(3*Area*Length*0.01/4,3*Area*Length*0.04/4,3*Area*Length*0.05/4),1e-12);
		M A=M::Diagonal(V(1e-20,1,1e20)); V X(0); EXPECT(A.Solve(V(2e-20,3,4e20),X,1e-14)); Near(X,V(2,3,4),1e-12);
		M Singular; EXPECT(!Singular.Solve(V(1),X,1e-14)); A=M::Identity(); A[1][0]=A[0][1]=2; EXPECT(!A.Solve(V(1),X,1e-14));
		S.PoissonRatio=0.5; EXPECT(!S.IsValid()); S.PoissonRatio=0.3; S.RadiusX=std::numeric_limits<double>::min(); EXPECT(!S.IsValid());
	}
	void DerivativesAndCovariance()
	{
		// Public coordinates/quaternions are floats. Central differences use a
		// resolvable perturbation; analytic strain/energy evaluation uses doubles.
		const double Epsilon=0.001, DerivativeTolerance=0.002, EnergyTolerance=0.003;
		StrandMaterial Material; Material.RadiusX=0.12; Material.RadiusY=0.08;
		const Vector3 A(0.2f,-0.3f,0.1f), B(0.4f,0.1f,1.3f); const double Length=1.2;
		const auto Q=Turn(V(0.3,-0.2,0.15));
		const auto R=StrandElasticity::StretchShear(A,B,Q,Length,Material);
		for (int Variable=0; Variable<3; ++Variable) for (int Axis=0; Axis<3; ++Axis)
		{
			Vector3 D=Vector3::Zero(); D[Axis]=(float)Epsilon; V Angle(0); Angle[Axis]=Epsilon;
			const auto Plus=StrandElasticity::StretchShear(A+(Variable==0 ? D : Vector3::Zero()),B+(Variable==1 ? D : Vector3::Zero()),Variable==2 ? Turn(Angle)*Q : Q,Length,Material);
			const auto Minus=StrandElasticity::StretchShear(A-(Variable==0 ? D : Vector3::Zero()),B-(Variable==1 ? D : Vector3::Zero()),Variable==2 ? Turn(-Angle)*Q : Q,Length,Material);
			const M J=Variable==0 ? R.PositionA : Variable==1 ? R.PositionB : R.RotationA; V Unit(0); Unit[Axis]=1;
			Near((Plus.Value-Minus.Value)/(2*Epsilon),J*Unit,DerivativeTolerance);
			const double Analytic=(J.Transpose()*V(R.Stiffness.x*R.Value.x,R.Stiffness.y*R.Value.y,R.Stiffness.z*R.Value.z))[Axis];
			EXPECT_NEAR((Plus.Energy-Minus.Energy)/(2*Epsilon),Analytic,std::max(1.0,std::abs(Analytic))*EnergyTolerance);
		}
		const auto Global=Turn(V(-0.7,0.5,0.3)); const Transform Pose(Vector3(2,-4,3),Global);
		const auto Changed=StrandElasticity::StretchShear(Pose.LocalToWorld(A),Pose.LocalToWorld(B),Global*Q,Length,Material);
		Near(Changed.Value,R.Value,1e-6); EXPECT_NEAR(Changed.Energy,R.Energy,0.002);
		const auto Rest=Turn(V(0.2,0.3,-0.1));
		for (const V& Strain : {V(0),V(0.4,-0.2,0.6),V(0,0,std::acos(-1.0)-0.02)})
		{
			const auto QB=Q*Rest*Turn(Strain); const auto Bend=StrandElasticity::BendTwist(Q,QB,Rest,Length,Material);
			Near(Bend.Value,Strain,1e-6);
			for (int Body=0; Body<2; ++Body) for (int Axis=0; Axis<3; ++Axis)
			{
				V D(0); D[Axis]=Epsilon; const auto DP=Turn(D), DM=Turn(-D);
				const auto Plus=StrandElasticity::BendTwist(Body==0 ? DP*Q : Q,Body==1 ? DP*QB : QB,Rest,Length,Material);
				const auto Minus=StrandElasticity::BendTwist(Body==0 ? DM*Q : Q,Body==1 ? DM*QB : QB,Rest,Length,Material);
				const auto J=Body==0 ? Bend.RotationA : Bend.RotationB; V Unit(0); Unit[Axis]=1;
				Near((Plus.Value-Minus.Value)/(2*Epsilon),J*Unit,DerivativeTolerance);
				const double Analytic=(J.Transpose()*V(Bend.Stiffness.x*Bend.Value.x,Bend.Stiffness.y*Bend.Value.y,Bend.Stiffness.z*Bend.Value.z))[Axis];
				EXPECT_NEAR((Plus.Energy-Minus.Energy)/(2*Epsilon),Analytic,std::max(1.0,std::abs(Analytic))*EnergyTolerance);
			}
			const auto Equivalent=StrandElasticity::BendTwist(Global*Q,Global*QB,Rest,Length,Material);
			Near(Equivalent.Value,Bend.Value,1e-6);
			const auto Opposite=Quaternion(-QB.x,-QB.y,-QB.z,-QB.w);
			Near(StrandElasticity::BendTwist(Q,Opposite,Rest,Length,Material).Value,Bend.Value,1e-12);
		}
		// The stretch torque cancels the endpoint orbital torque, including shear.
		const auto Stress=V(R.Stiffness.x*R.Value.x,R.Stiffness.y*R.Value.y,R.Stiffness.z*R.Value.z);
		Near(Double(A).Cross(R.PositionA.Transpose()*Stress)+Double(B).Cross(R.PositionB.Transpose()*Stress)+R.RotationA.Transpose()*Stress,V(0),1e-10);
	}
	void StaticReactions()
	{
		for (auto Solver : {StrandSolver::SequentialBlocks,StrandSolver::CoupledBanded}) for (int Count : {1,4,8})
		{
			auto Settings=Quiet(Count); Settings.Solver=Solver;
			const auto G=Straight(1); StrandSystem S({G},Settings); const Vector3 Tip(0,0,1.02f); S.PinParticle(0,1,Tip);
			S.Simulate(0.02f); S.Simulate(0.02f);
			const double Strain=(double)Tip.z-1, Force=G.Material.YoungModulus*G.Material.Area()*Strain;
			EXPECT_NEAR(S.GetRootForce(0).z,Force,Force*1e-6); EXPECT_NEAR(S.GetRootForce(0).x,0,1e-10);
			EXPECT_NEAR(S.GetStats().ElasticEnergy,Force*Strain*0.5,1e-9);
			const auto T=Straight(); StrandSystem Twist({T},Settings); Twist.PinParticle(0,1,T.Positions[1]); Twist.PinParticle(0,2,T.Positions[2]);
			const double Angle=0.2; Twist.PinDirector(0,1,Turn(V(0,0,Angle))); Twist.Simulate(0.02f); Twist.Simulate(0.02f);
			const double Torque=T.Material.BendTwistStiffness(1).z*Angle;
			EXPECT_NEAR(Twist.GetRootTorque(0).z,Torque,Torque*1e-6); EXPECT_NEAR(Twist.GetStats().ElasticEnergy,Torque*Angle*0.5,Torque*1e-6);
		}
	}
	void AlgorithmSelection()
	{
		auto Settings=Quiet(); Settings.Algorithm=StrandAlgorithm::DistanceConstraints;
		StrandSystem S({Straight()},Settings);
		S.SetParticlePosition(0,1,Vector3(0.2f,0,1));
		S.Simulate(0.01f);
		EXPECT(S.GetSettings().Algorithm==StrandAlgorithm::DistanceConstraints);
		EXPECT(S.GetStats().ConstraintSolves>0);
		EXPECT(S.GetStats().MaximumStretchShear<1e-4);
		EXPECT_NEAR(S.GetStats().ElasticEnergy,0,0);
		S.SetAlgorithm(StrandAlgorithm::CosseratRods); S.Simulate(0.01f);
		EXPECT(S.GetSettings().Algorithm==StrandAlgorithm::CosseratRods);
		Reject([&] { S.SetAlgorithm((StrandAlgorithm)-1); });
	}
	void CantileverCalibration()
	{
		// Linear discrete Timoshenko rod: fixed first director, N equal segments.
		// Summing the bending moments F*h*(N-i) gives tip displacement
		// F*h^3/EI * sum(i^2, i=1..N-1) + F*L/(kGA). This independent equilibrium
		// check includes free particle/director dynamics, not prescribed strains.
		const int Segments=12, Frames=180; const double Length=1, Force=0.01, H=Length/Segments;
		auto G=Straight(Segments); for (auto& P : G.Positions) P.z/=(float)Segments;
		G.Material.RadiusX=G.Material.RadiusY=0.05; auto Settings=Quiet(); Settings.LinearDamping=Settings.AngularDamping=20; Settings.SolverIterations=8;
		StrandSystem S({G},Settings); const float Dt=1.0f/120;
		for (int Frame=0; Frame<Frames; ++Frame) { S.AddForce(0,Segments,Vector3((float)Force,0,0)); S.Simulate(Dt); }
		const double SumSquares=(Segments-1)*Segments*(2*Segments-1)/6.0;
		const double Expected=Force*H*H*H*SumSquares/G.Material.BendTwistStiffness(1).y+Force*Length/G.Material.StretchShearStiffness(1).x;
		std::printf("Strand cantilever: tip=%g m, linear reference=%g m, root force=%g N, root moment=%g Nm\n",S.GetParticle(0,Segments).Position.x,Expected,S.GetRootForce(0).x,S.GetRootTorque(0).y);
		EXPECT_NEAR(S.GetParticle(0,Segments).Position.x,Expected,Expected*0.05);
		EXPECT_NEAR(S.GetRootForce(0).x,Force,Force*0.03); EXPECT_NEAR(S.GetRootTorque(0).y,Force*Length,Force*Length*0.03);
		EXPECT(S.GetParticle(0,Segments).Velocity.Length()<0.001);
	}
	void MomentumAndLoads()
	{
		for (int Count : {1,4,8})
		{
			auto G=Straight(1); G.PinRoot=false; G.Material.RadiusY=G.Material.RadiusX; StrandSystem S({G},Quiet(Count));
			const Vector3 Velocity(1,2,3); S.SetParticleVelocity(0,0,Velocity); S.SetParticleVelocity(0,1,Velocity);
			S.AddTorque(0,0,Vector3(0,0,0.0001f)); const float Dt=0.01f; S.Simulate(Dt);
			EXPECT_NEAR(S.GetDirector(0,0).AngularMomentum.z,0.0001*Dt,1e-12);
			const double VelocityTolerance=2*std::numeric_limits<float>::epsilon()*Count/Dt;
			Near(Double(S.GetParticle(0,0).Velocity),Double(Velocity),VelocityTolerance);
			Near((Double(S.GetParticle(0,0).Position)+Double(S.GetParticle(0,1).Position))*0.5,V(0,0,0.5)+Double(Velocity)*Dt,1e-6);
			const auto L=S.GetDirector(0,0).AngularMomentum; S.Simulate(Dt); Near(S.GetDirector(0,0).AngularMomentum,L,1e-12);
			S.AddForce(0,0,Vector3(0.01f,0,0)); S.AddForce(0,1,Vector3(0.01f,0,0)); S.Simulate(Dt);
			const double Mass=G.Material.Density*G.Material.Area();
			const auto Momentum=Double(S.GetParticle(0,0).Velocity)/S.GetParticle(0,0).InverseMass+Double(S.GetParticle(0,1).Velocity)/S.GetParticle(0,1).InverseMass;
			Near(Momentum,Double(Velocity)*Mass+V(0.02*Dt,0,0),2e-6);
		}
	}
	void AttachmentsAndFailure()
	{
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam P; P.rigidType=RigidType::Static;
		auto* Body=W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.1f),P);
		StrandSystem S({Straight()},Quiet()); const Transform Initial(Vector3(0,1,0),Quaternion::One()); S.AttachRoot(0,Body,Initial); S.Reset();
		const auto Q=Turn(V(0.2,0.1,0.3)); Body->X=Vector3(1,2,3); Body->Q=Q; S.Simulate(0.01f);
		const auto Target=Transform(Body->X,Q).LocalToWorld(Initial.pos);
		Near(Double(S.GetParticle(0,0).Position),Double(Target),1e-6); Near(StrandElasticity::Log(S.GetDirector(0,0).Orientation*Q.Conjugate()),V(0),1e-6);
		S.Reset(); EXPECT_NEAR(S.GetStats().ElasticEnergy,0,1e-8); S.DetachBody(Body); EXPECT(W.RemoveRigidBody(Body)); Body->ReleaseGeometries(); delete Body;
		S.Simulate(0.01f); Near(Double(S.GetParticle(0,0).Position),Double(Target),1e-6);
		auto Settings=Quiet(1); Settings.MaximumSubsteps=1; Settings.MaximumRootAngle=0.01;
		StrandSystem Failed({Straight()},Settings); const auto Before=Failed.GetParticle(0,1).Position;
		Failed.AddForce(0,1,Vector3(1,0,0)); Failed.SetRootPose(0,Transform(Vector3::Zero(),Turn(V(0,0,0.2))));
		Reject([&] { Failed.Simulate(0.01f); }); EXPECT(Failed.GetStats().FailedSteps==1 && Failed.GetStats().Time==0); EXPECT(Failed.GetParticle(0,1).Position==Before);
		Failed.SetRootPose(0,Transform(Vector3::Zero(),Quaternion::One())); Failed.Simulate(0.01f); EXPECT(Failed.GetParticle(0,1).Position.x>0);
		Reject([&] { Failed.Simulate(0); }); Reject([&] { Failed.UnpinDirector(0,0); }); Reject([&] { Failed.AttachRoot(0,nullptr,Initial); });
		auto Bad=Straight(); Bad.Positions[1]=Bad.Positions[0]; Reject([&] { StrandSystem Invalid({Bad}); });
		Bad=Straight(); Bad.Directors={Quaternion::One()}; Reject([&] { StrandSystem Invalid({Bad}); });
		Settings.MaximumRootAngle=std::numeric_limits<double>::min(); StrandSystem Tiny({Straight()},Settings); Tiny.SetRootPose(0,Transform(Vector3::Zero(),Q)); Reject([&] { Tiny.Simulate(0.01f); });
		Reject([&] { S.SetSolver((StrandSolver)-1); });
		Reject([&] { Failed.AddForce(0,1,Vector3(std::numeric_limits<float>::infinity(),0,0)); });
		const Quaternion Pi(1,0,0,0), OppositePi(-1,0,0,-0.0f); Near(StrandElasticity::Log(Pi),StrandElasticity::Log(OppositePi),0);
	}
	void WorldFramesAndLifetime()
	{
		for (auto Solver : {ResolutionPhaseSolver::SequentialImpulse,ResolutionPhaseSolver::LCPGlobal,ResolutionPhaseSolver::PBD,ResolutionPhaseSolver::XPBD}) for (int Count : {1,4,8})
		{
			PhysicsWorldParam P; P.resolutionSolver=Solver; P.temporalSubsteps.Enabled=true; P.temporalSubsteps.Substeps=Count; PhysicsWorld W(P);
			auto G=Straight(1); G.PinRoot=false; G.Material.RadiusY=G.Material.RadiusX;
			auto* S=W.CreateStrands({G},Quiet()); EXPECT(W.GetStrandSystemCount()==1 && W.GetStrandSystem(0)==S && !W.GetStrandSystem(1));
			const double Mass=G.Material.Density*G.Material.Area(); const float Dt=0.1f, Acceleration=3, Torque=0.0001f;
			for (int I=0; I<2; ++I) S->AddForce(0,I,Vector3((float)(Mass*Acceleration*0.5),0,0));
			S->AddTorque(0,0,Vector3(0,0,Torque)); W.Simulate(Dt);
			EXPECT_NEAR(S->GetParticle(0,0).Velocity.x,Acceleration*Dt,2e-5);
			EXPECT_NEAR(S->GetDirector(0,0).AngularMomentum.z,(double)Torque*Dt,1e-12);
			EXPECT_NEAR(S->GetStats().Time,Dt,1e-8); EXPECT(S->GetStats().Substeps==Count*S->GetSettings().Substeps);
			EXPECT(S->GetStats().ConstraintSolves==(uint64_t)(Count*S->GetSettings().Substeps*S->GetSettings().SolverIterations));
			W.Simulate(Dt); EXPECT_NEAR(S->GetParticle(0,0).Velocity.x,Acceleration*Dt,3e-5);
			EXPECT_NEAR(S->GetDirector(0,0).AngularMomentum.z,(double)Torque*Dt,1e-12);
			Reject([&] { S->Simulate(Dt); }); EXPECT(W.RemoveStrands(S)); EXPECT(!W.RemoveStrands(S));
		}
		PhysicsWorld W{PhysicsWorldParam()}, Other{PhysicsWorldParam()}; RigidBodyParam P; P.rigidType=RigidType::Static;
		auto* Body=W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.1f),P);
		auto* Foreign=Other.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(),0.1f),P);
		auto* S=W.CreateStrands({Straight()},Quiet()); S->AttachRoot(0,Body,S->GetRootPose(0));
		Reject([&] { S->AttachRoot(0,Foreign,S->GetRootPose(0)); });
		Body->X=Vector3(1,2,3); const auto Pose=S->GetRootPose(0); EXPECT(W.RemoveRigidBody(Body)); Body->ReleaseGeometries(); delete Body;
		W.Simulate(0.01f); Near(Double(S->GetParticle(0,0).Position),Double(Pose.pos),1e-6);
		W.Reset(); EXPECT(W.GetStrandSystemCount()==0);
		auto* Metrics=W.CreateStrands({Straight(1)},Quiet()); Metrics->PinParticle(0,1,Vector3(0,0,1.02f)); W.Simulate(0.01f);
		EXPECT(Metrics->GetStats().MaximumStretchShear>0.01 && Metrics->GetStats().ElasticEnergy>0);
		Metrics->PinParticle(0,1,Vector3::UnitZ()); W.Simulate(0.01f);
		EXPECT(Metrics->GetStats().MaximumStretchShear<1e-6 && Metrics->GetStats().ElasticEnergy<1e-12);
	}
	class StrandEditField : public ForceField
	{
	public:
		PhysicsWorld* World=nullptr; StrandSystem* Strands=nullptr; RigidBody* Root=nullptr; bool Called=false;
		bool ApplyForce(RigidBodyDynamic*) override
		{
			Called=true; Reject([&] { World->RemoveStrands(Strands); }); Reject([&] { World->Reset(); }); Reject([&] { World->RemoveRigidBody(Root); });
			Reject([&] { Strands->Reset(); }); Reject([&] { Strands->DetachBody(Root); }); Reject([&] { Strands->AddForce(0,1,Vector3::UnitX()); }); return false;
		}
	};
	void EditBoundaries()
	{
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam P; P.rigidType=RigidType::Dynamic;
		auto* Root=W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0,10,0),0.1f),P);
		auto* S=W.CreateStrands({Straight()},Quiet()); S->AttachRoot(0,Root,S->GetRootPose(0));
		auto* Field=new StrandEditField; Field->World=&W; Field->Strands=S; Field->Root=Root; W.AddForceField(Field); W.Simulate(0.01f); EXPECT(Field->Called);
	}
	void DestructionRootRemoval()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet Set(W); Set.GetDestructionConstants().MinClusterVolume=0;
		for (int I=0; I<3; ++I)
		{
			const Vector3 Center((float)I*2,0,0); Set.AddChunk(Box3(Center-Vector3(0.5f),Center+Vector3(0.5f)));
			if (I) EXPECT(Set.AddBond(I-1,I));
		}
		Set.RebuildClustersFromGraph(); auto* Root=Set.FindClusterContaining(0)->GetRigidBody(); EXPECT(Root); if (!Root) return;
		auto* S=W.CreateStrands({Straight()},Quiet()); S->AttachRoot(0,Root,S->GetRootPose(0));
		Set.AddDestructionInfo(DestructionEventType::SelfCollapse,{1}); W.Simulate(0.01f); EXPECT(Set.GetClusters().size()>1);
		const auto Detached=S->GetRootPose(0);
		for (auto* Cluster : Set.GetClusters()) if (auto* Fragment=Cluster->GetRigidBody()) Fragment->X+=Vector3(5,2,1);
		W.Simulate(0.01f); Near(Double(S->GetParticle(0,0).Position),Double(Detached.pos),1e-6);
	}
	Geometry* Object(SceneWorld& Scene, const char* Id)
	{ for (const auto& O : Scene.GetObjects()) if (O.Id==Id) return O.GeometryPtr; return nullptr; }
	void ExistingScenePreview()
	{
		SceneWorld Scene; Scene.SetStrandDemo(true); std::string Error;
		const bool Loaded=Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error); if (!Loaded) std::printf("Strand scene: %s\n",Error.c_str()); EXPECT(Loaded); if (!Loaded) return;
		auto* S=Scene.GetDemoStrands(); EXPECT(S && S->GetGuideCount()==8); if (!S) return;
		auto* Surface=Object(Scene,"cosserat_guide_0"); auto* Handle=Object(Scene,"handle"); EXPECT(Surface && Handle && Object(Scene,"tow_rope_surface")); if (!Surface || !Handle) return;
		auto* Mesh=Surface->GetShapeObj<TriangleMesh>(); EXPECT(Mesh && Mesh->NumVertices>0); if (!Mesh || !Mesh->NumVertices) return;
		const auto FirstVertex=Mesh->Vertices[0]; const auto FirstRoot=S->GetParticle(0,0).Position;
		// Lock centerlines to isolate visible material twist from positional motion.
		for (size_t G=0; G<S->GetGuideCount(); ++G)
		{
			for (size_t I=1; I<S->GetParticleCount(G); ++I) S->PinParticle(G,I,S->GetParticle(G,I).Position);
			for (size_t I=1; I<S->GetDirectorCount(G); ++I) S->PinDirector(G,I,S->GetDirector(G,I).Orientation);
		}
		EXPECT(Scene.ApplyStrandDemo(0.7f)); const float Dt=1.0f/60; const auto Revision=Scene.GetRenderRevision(); Scene.Step(Dt);
		EXPECT(S->GetParticle(0,0).Position==FirstRoot); EXPECT((Mesh->Vertices[0]-FirstVertex).Length()>0.01f); EXPECT(Scene.GetRenderRevision()>Revision);
		SolverPanelState Panel; RefreshSolverPanelTelemetry(Panel,Scene); EXPECT(Panel.HasStrands && Panel.StrandGuides==8 && Panel.StrandEnergy>0 && Panel.StrandConstraintSolves>0);
		for (size_t G=0; G<S->GetGuideCount(); ++G)
		{
			for (size_t I=1; I<S->GetParticleCount(G); ++I) S->UnpinParticle(G,I);
			for (size_t I=1; I<S->GetDirectorCount(G); ++I) S->UnpinDirector(G,I);
		}
		S->Reset(); EXPECT(Scene.SelectMouseControlGeometry(Handle)); const auto Origin=Handle->GetParent<RigidBody>()->X;
		for (int Frame=0; Frame<48; ++Frame)
		{
			const float T=(Frame+1)*Dt; EXPECT(Scene.SetMouseControlPosition(Origin+Vector3(0.1f*std::sin(T),0.1f*T,0),Dt)); Scene.Step(Dt);
			for (size_t G=0; G<S->GetGuideCount(); ++G)
			{
				Near(Double(S->GetParticle(G,0).Position),Double(S->GetRootPose(G).pos),1e-6);
				for (size_t I=0; I<S->GetParticleCount(G); ++I)
				{
					const auto& P=S->GetParticle(G,I); EXPECT(std::isfinite(P.Position.x) && std::isfinite(P.Position.y) && std::isfinite(P.Position.z));
				}
				for (size_t I=0; I<S->GetDirectorCount(G); ++I) EXPECT_NEAR(S->GetDirector(G,I).Orientation.SquareLength(),1,1e-5);
			}
		}
		std::printf("Strand preview: energy=%g J, stretch/shear=%g, bend/twist=%g\n",S->GetStats().ElasticEnergy,S->GetStats().MaximumStretchShear,S->GetStats().MaximumBendTwist);
		EXPECT(S->GetStats().ElasticEnergy>0 && S->GetStats().MaximumStretchShear<0.1); RefreshSolverPanelTelemetry(Panel,Scene); EXPECT_NEAR(Panel.StrandEnergy,S->GetStats().ElasticEnergy,1e-12);
		const auto RemovedId=S->GetInstanceId(); EXPECT(Scene.GetSimulation()->RemoveStrands(S));
		// Replace before rendering; allocator address reuse must not bind the old
		// eight-guide preview to an unrelated new one-guide system.
		auto* Replacement=Scene.GetSimulation()->CreateStrands({Straight()},Quiet()); EXPECT(Replacement->GetInstanceId()!=RemovedId);
		EXPECT(!Scene.GetDemoStrands()); Scene.Step(Dt); EXPECT(Mesh->NumVertices==0);
		RefreshSolverPanelTelemetry(Panel,Scene); EXPECT(!Panel.HasStrands && Panel.StrandGuides==0 && Panel.StrandEnergy==0);
		EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error)); EXPECT(Scene.GetDemoStrands()!=nullptr);
		Scene.SetStrandDemo(false); EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/18_rope_towing.json",&Error)); EXPECT(!Scene.GetDemoStrands());
	}
}
void TestStrands()
{
	MaterialAndLinearAlgebra(); DerivativesAndCovariance(); StaticReactions(); AlgorithmSelection(); CantileverCalibration(); MomentumAndLoads(); AttachmentsAndFailure();
	WorldFramesAndLifetime(); EditBoundaries(); DestructionRootRemoval(); ExistingScenePreview();
}

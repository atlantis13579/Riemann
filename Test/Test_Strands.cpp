#include "Test.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Destruction/DestructionSet.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include "../Src/Collision/SurfaceCollision.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include <memory>

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
}
void TestStrands()
{
	MaterialAndLinearAlgebra(); DerivativesAndCovariance(); StaticReactions(); AlgorithmSelection(); CantileverCalibration(); MomentumAndLoads(); AttachmentsAndFailure();
	WorldFramesAndLifetime(); EditBoundaries(); DestructionRootRemoval();
}

namespace Test_StrandGroom_Cases
{
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
}
void TestStrandGroom()
{
	RestAndRigidMotion(); ArcLengthAndTwist(); QuaternionGaugeInvariance(); LevelsAndTransfer(); RoutedLoads(); Validation(); RenderRootBindings(); FailedRenderRestoresStep();
}
}

void TestStrandGroom() { Test_StrandGroom_Cases::TestStrandGroom(); }

namespace Test_StrandCollision_Cases
{
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
	SegmentInteriorAndSnapshots(); MeshAndConvexRecovery(); StrandPairsAndSelf(); ContinuousCrossing(); FrictionAndMotion(); RefinementAndLifetime(); DisabledHistoryAndAngularMotion();  SpatialIndexAndValidation();
}
}

void TestStrandCollision() { Test_StrandCollision_Cases::TestStrandCollision(); }

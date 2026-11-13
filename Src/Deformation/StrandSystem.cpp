#include "StrandSystem.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include "../RigidBodyDynamics/PhysicsWorld.h"
#include "../Collision/GeometryObject.h"
#include <functional>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <stdexcept>

namespace Riemann
{
	/* Guide dynamics and attachment boundary
	   --------------------------------------
	   Material frames are rotational degrees of freedom, not display tangents.
	   Coupled 3x3 XPBD blocks solve material stretch/shear and log bend/twist, using
	   physical vertex mass and cross-section director inertia. Multipliers persist
	   within one physical step and reset at the next step. Damped corrections scale
	   the whole multiplier increment, preserving the block's force direction.
	   Numerical correction limits and pivot tolerances are explicit settings.

	   Thin rods have very small physical director inertia. Sequential blocks can
	   transfer strain repeatedly between stretch/shear and bend/twist instead of
	   converging. The coupled route orders rows S0,B0,S1,B1,...; each row shares a
	   degree of freedom only with its two neighboring rows on either side. Hence
	   J M^-1 J^T + compliance has a constant scalar bandwidth. Diagonally scaled
	   band Cholesky solves the coupled multiplier increment in O(segment count).
	   Both routes use the SAME physical mass, energy and compliance. This avoids
	   artificial director inertia or stiffness reduction as a convergence fix.
	   Nonlinear geometry/inertia is rebuilt at each iteration; a common correction
	   scale bounds the entire update without changing its force direction.
	   Positions/radii are meters, moduli are Pa, density is kg/m^3, damping rates
	   are inverse seconds, loads are N/Nm and reported elastic energy is joules.

	   Free directors integrate WORLD angular momentum, which remains constant under
	   torque-free prediction. Constraint torques add J^T dLambda / h; particles
	   reconstruct velocity from their projected positions. Forces/torques are held
	   across every internal substep of one public call, then consumed once.
	   Drag scales existing velocity/momentum before external impulses are added;
	   damping an already applied load would change static force balance with h.

	   Root poses are prescribed kinematics. Interpolate position and shortest-arc
	   rotation across substeps and expose integrated reaction force/torque, including
	   prescribed root inertia. Standalone callers must detach borrowed bodies before
	   deleting them. World ownership automates notification and holds queued loads
	   across outer temporal substeps. Root changes
	   must be sampled frequently enough to disambiguate turns greater than pi.
	   A failed local step restores guide state and pending loads; this is not a
	   checkpoint/rollback of a rigid world that the caller may already have advanced.

	   Groom LODs skip inactive guide solves. Inactive states interpolate from the
	   active subset, corrected to their own root attachments. A root-local source
	   trajectory retains relative linear/angular velocity; quintic blending ends
	   with zero blend derivatives. Reactivation starts from that displayed state
	   and its reconstructed velocity, rather than a stale last-simulated pose.
	   Loads on inactive samples transfer to active sample brackets; an additional
	   director torque preserves the application point's moment. Pending loads are
	   transferred when changing LOD. Non-root prescribed pins require active guides.
	   Representative weights scale mass, director inertia and energy together.
	   Render roots may have independent positional skin attachments. Their root
	   displacement correction fades linearly in rest arc length to zero at the tip;
	   material rotations still come from guides. This keeps roots on the scalp when
	   guide twist rotates a radial offset. Root constraints are not collision solves.
	   Active guides predict together, then alternate material and segment-contact
	   projection. Contact refinement restores a rejected interval before bisecting
	   its physical duration, with explicit depth/count/total-substep budgets. Collider
	   history commits only with the public step; a failure also restores pending
	   loads and keeps the last published groom output.
	   TODO(asset): external groom/scalp import and skeletal skinning require authored
	   resources. Existing procedural rope/cloth scenes supply runtime coverage first.
	*/
	namespace
	{
		using V = StrandVector;
		using M = StrandMatrix;
		V Double(const Vector3& P) { return V(P.x,P.y,P.z); }
		double Magnitude(const V& P) { return std::hypot(std::hypot(P.x,P.y),P.z); }
		bool Finite(const V& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		void Require(bool Valid, const char* Message) { if (!Valid) throw std::invalid_argument(Message); }
		uint64_t AllocateInstanceId()
		{
			static std::atomic<uint64_t> Next{1}; auto Id=Next.load(std::memory_order_relaxed);
			for (;;)
			{
				Require(Id<std::numeric_limits<uint64_t>::max(),"Strand instance identities exhausted");
				if (Next.compare_exchange_weak(Id,Id+1,std::memory_order_relaxed)) return Id;
			}
		}
		Vector3 Single(const V& P)
		{
			const double Limit=std::numeric_limits<float>::max();
			Require(Finite(P) && std::abs(P.x)<=Limit && std::abs(P.y)<=Limit && std::abs(P.z)<=Limit,"Strand position or velocity overflow");
			return Vector3((float)P.x,(float)P.y,(float)P.z);
		}
		Quaternion ValidRotation(const Quaternion& Q, const StrandSettings& S)
		{
			const double Norm=(double)Q.x*Q.x+(double)Q.y*Q.y+(double)Q.z*Q.z+(double)Q.w*Q.w;
			Require(std::isfinite(Norm) && std::abs(Norm-1)<=S.Numerics.QuaternionTolerance,"Invalid strand director"); return Q.Unit();
		}
		M WorldInertia(const StrandDirector& D, bool Inverse)
		{
			const M R=M::Rotation(D.Orientation); V Diagonal=D.InverseInertia;
			if (!Inverse) for (int I=0; I<3; ++I) Diagonal[I]=1/Diagonal[I];
			return R*M::Diagonal(Diagonal)*R.Transpose();
		}
		M EffectiveInertia(const StrandDirector& D) { return D.Pinned ? M() : WorldInertia(D,true); }
		class BandSystem
		{
		public:
			BandSystem(size_t Size, size_t Bandwidth) : m_Size(Size), m_Bandwidth(Bandwidth), m_Values(Size*(Bandwidth+1),0) {}
			double& At(size_t I, size_t J) { return m_Values[I*(m_Bandwidth+1)+I-J]; }
			std::vector<double> Solve(const std::vector<double>& Right, double Tolerance)
			{
				std::vector<double> Scale(m_Size), Y(m_Size), Z(m_Size), X(m_Size);
				for (size_t I=0; I<m_Size; ++I)
				{ Require(At(I,I)>0 && std::isfinite(At(I,I)),"Invalid strand band diagonal"); Scale[I]=std::sqrt(At(I,I)); }
				for (size_t I=0; I<m_Size; ++I)
				{
					const size_t First=I>m_Bandwidth ? I-m_Bandwidth : 0;
					for (size_t J=First; J<=I; ++J)
					{
						double Value=(At(I,J)/Scale[I])/Scale[J];
						for (size_t K=First; K<J; ++K) Value-=At(I,K)*At(J,K);
						if (!std::isfinite(Value) || (I==J && Value<=Tolerance)) throw std::runtime_error("Strand band factor is ill-conditioned");
						At(I,J)=I==J ? std::sqrt(Value) : Value/At(J,J);
					}
					Y[I]=Right[I]/Scale[I]; for (size_t J=First; J<I; ++J) Y[I]-=At(I,J)*Y[J]; Y[I]/=At(I,I);
				}
				for (size_t I=m_Size; I-->0;)
				{
					Z[I]=Y[I]; for (size_t J=I+1; J<std::min(m_Size,I+m_Bandwidth+1); ++J) Z[I]-=At(J,I)*Z[J];
					Z[I]/=At(I,I); X[I]=Z[I]/Scale[I]; Require(std::isfinite(X[I]),"Strand band solution overflow");
				}
				return X;
			}
		private:
			size_t m_Size, m_Bandwidth;
			std::vector<double> m_Values;
		};
	}
	bool StrandSettings::IsValid() const
	{
		return (Algorithm==StrandAlgorithm::DistanceConstraints || Algorithm==StrandAlgorithm::CosseratRods)
			&& (Solver==StrandSolver::SequentialBlocks || Solver==StrandSolver::CoupledBanded) && Substeps>0 && SolverIterations>0 && MaximumSubsteps>=Substeps && std::isfinite(LinearDamping) && LinearDamping>=0
			&& std::isfinite(AngularDamping) && AngularDamping>=0 && std::isfinite(MaximumRootAngle) && MaximumRootAngle>0
			&& std::isfinite(RestTangentTolerance) && RestTangentTolerance>0 && RestTangentTolerance<1 && Finite(Double(Gravity)) && Numerics.IsValid() && Collision.IsValid();
	}
	struct StrandSystem::Storage
	{
		struct Guide
		{
			StrandGuide Rest;
			std::vector<StrandParticle> Points;
			std::vector<StrandDirector> Frames;
			std::vector<Vector3> Pins;
			std::vector<Quaternion> FramePins, RestRelative;
			std::vector<V> Forces, Torques, StretchLambda, BendLambda;
			std::vector<double> Lengths;
			std::vector<double> BaseInverseMass;
			double Weight=1, BlendElapsed=0, BlendDuration=0;
			std::vector<V> BlendPositions, BlendVelocities, BlendAngularVelocities;
			std::vector<Quaternion> BlendFrames;
			Transform Root, PreviousRoot, LocalRoot;
			RigidBody* Body=nullptr;
			bool RootEnabled=false;
			V RootImpulse=V(0), RootMoment=V(0), RootForce=V(0), RootTorque=V(0);
			Transform Target() const { return Body ? Transform(Body->BodyLocalToWorld(LocalRoot.pos),Body->Q*LocalRoot.quat) : Root; }
		};
		StrandSettings Settings;
		uint64_t InstanceId=AllocateInstanceId();
		StrandStats Stats;
		std::unique_ptr<StrandCollision> Collision;
		StrandCollisionStats CollisionStats, FrameCollisionStats;
		std::vector<Guide> Guides;
		std::shared_ptr<StrandGroom> Groom;
		std::vector<GroomCurveState> Render, RenderScratch;
		struct RenderRoot
		{
			RigidBody* Body=nullptr; Vector3 Position=Vector3::Zero(), Local=Vector3::Zero(); bool Enabled=false;
			Vector3 Target() const { return Body ? Body->BodyLocalToWorld(Local) : Position; }
		};
		std::vector<RenderRoot> RenderRoots;
		size_t Lod=0;
		PhysicsWorld* World=nullptr;
		StrandStats FrameStats;
		std::vector<V> FrameImpulses, FrameMoments;
		double FrameDt=0, FrameElapsed=0;
		bool Stepping=false;
		void Idle() const { if (Stepping || (World && World->IsSimulating())) throw std::logic_error("Edit strands between world steps"); }
		bool Active(size_t I) const { return !Groom || Groom->IsSimulated(Lod,I); }
		StrandResponse StretchResponse(const Guide& G, size_t I) const
		{
			auto R=StrandElasticity::StretchShear(G.Points[I].Position,G.Points[I+1].Position,G.Frames[I].Orientation,G.Lengths[I],G.Rest.Material,Settings.Numerics);
			R.Stiffness*=G.Weight; R.Energy*=G.Weight; return R;
		}
		StrandResponse BendResponse(const Guide& G, size_t I) const
		{
			auto R=StrandElasticity::BendTwist(G.Frames[I].Orientation,G.Frames[I+1].Orientation,G.RestRelative[I],(G.Lengths[I]+G.Lengths[I+1])*0.5,G.Rest.Material,Settings.Numerics);
			R.Stiffness*=G.Weight; R.Energy*=G.Weight; return R;
		}
		void WeightGuide(Guide& G, double Weight)
		{
			Require(std::isfinite(Weight) && Weight>0,"Invalid groom representative weight");
			for (size_t I=0; I<G.Points.size(); ++I)
			{ const double Inverse=G.BaseInverseMass[I]/Weight; Require(std::isfinite(Inverse) && Inverse>0,"Groom representative mass overflow"); G.Points[I].InverseMass=Inverse; }
			for (size_t I=0; I<G.Frames.size(); ++I)
			{
				auto& F=G.Frames[I]; const auto Inertia=G.Rest.Material.DirectorInertia(G.Lengths[I]);
				for (int Axis=0; Axis<3; ++Axis)
				{ const double Inverse=1/(Inertia[Axis]*Weight); Require(std::isfinite(Inverse) && Inverse>0,"Groom representative inertia overflow"); F.InverseInertia[Axis]=Inverse; }
				F.AngularMomentum*=Weight/G.Weight; Require(Finite(F.AngularMomentum),"Groom representative momentum overflow");
			}
			G.Weight=Weight;
		}
		void RenderGroom(const StrandSystem& System)
		{
			if (!Groom) { Render.clear(); RenderScratch.clear(); return; } RenderScratch.resize(Groom->GetStrandCount());
			for (size_t I=0; I<RenderScratch.size(); ++I)
			{
				auto& Curve=RenderScratch[I]; Groom->EvaluateStrand(I,System,Curve);
				if (RenderRoots[I].Enabled)
				{
					const V Correction=Double(RenderRoots[I].Target())-Double(Curve.Positions[0]);
					for (size_t P=0; P<Curve.Positions.size(); ++P) Curve.Positions[P]=Single(Double(Curve.Positions[P])+Correction*(1-Groom->GetStrandParameter(I,P)));
				}
			}
			Render.swap(RenderScratch);
		}
		void CaptureBlend(Guide& G, double Duration)
		{
			const V Origin=Double(G.Points[0].Position), RootVelocity=Double(G.Points[0].Velocity), RootOmega=WorldInertia(G.Frames[0],true)*G.Frames[0].AngularMomentum;
			const auto Inverse=StrandMatrix::Rotation(G.Frames[0].Orientation).Transpose(); const auto InverseQ=G.Frames[0].Orientation.Conjugate();
			G.BlendElapsed=0; G.BlendDuration=Duration; G.BlendPositions.clear(); G.BlendVelocities.clear(); G.BlendFrames.clear(); G.BlendAngularVelocities.clear();
			for (const auto& P : G.Points)
			{
				G.BlendPositions.push_back(Inverse*(Double(P.Position)-Origin));
				G.BlendVelocities.push_back(Inverse*(Double(P.Velocity)-RootVelocity-RootOmega.Cross(Double(P.Position)-Origin)));
			}
			for (const auto& F : G.Frames)
			{
				G.BlendFrames.push_back((InverseQ*F.Orientation).Unit()); G.BlendAngularVelocities.push_back(Inverse*(WorldInertia(F,true)*F.AngularMomentum-RootOmega));
			}
		}
		void InterpolateGuide(size_t Index, const Transform& Root, double H, const StrandSystem& System)
		{
			auto& G=Guides[Index]; GroomCurveState Target; Groom->EvaluateGuide(Lod,Index,System,Target);
			const Transform RawRoot(Target.Positions[0],Target.Directors[0]); const auto Delta=(Root.quat*RawRoot.quat.Conjugate()).Unit();
			G.BlendElapsed=std::min(G.BlendDuration,G.BlendElapsed+H);
			const double U=G.BlendDuration>0 ? G.BlendElapsed/G.BlendDuration : 1;
			const double Blend=U*U*U*(U*(U*6-15)+10); const auto Rotation=StrandMatrix::Rotation(Root.quat);
			for (size_t I=0; I<G.Points.size(); ++I)
			{
				auto& P=G.Points[I]; const V Destination=Double(Root.pos)+StrandMatrix::Rotation(Delta)*(Double(Target.Positions[I])-Double(RawRoot.pos));
				const V Source=Blend<1 ? Double(Root.pos)+Rotation*(G.BlendPositions[I]+G.BlendVelocities[I]*G.BlendElapsed) : Destination;
				const auto Position=Single(Source*(1-Blend)+Destination*Blend); P.Velocity=Single((Double(Position)-Double(P.Position))/H); P.Position=Position;
			}
			for (size_t I=0; I<G.Frames.size(); ++I)
			{
				auto& F=G.Frames[I]; const auto Destination=(Delta*Target.Directors[I]).Unit();
				const auto Source=Blend<1 ? (Root.quat*StrandElasticity::Increment(G.BlendAngularVelocities[I]*G.BlendElapsed)*G.BlendFrames[I]).Unit() : Destination;
				const auto Q=StrandElasticity::Interpolate(Source,Destination,Blend); const auto Omega=StrandElasticity::Log(Q*F.Orientation.Conjugate())/H;
				F.Orientation=Q; F.AngularMomentum=WorldInertia(F,false)*Omega;
			}
			G.Pins[0]=Root.pos; G.FramePins[0]=Root.quat;
		}
		void RouteLoad(size_t GuideIndex, size_t Index, bool Torque, const V& Load)
		{
			if (Load==V(0)) return;
			if (Active(GuideIndex))
			{ auto& Target=(Torque ? Guides[GuideIndex].Torques : Guides[GuideIndex].Forces).at(Index); const auto Sum=Target+Load; Require(Finite(Sum),"Strand load overflow"); Target=Sum; return; }
			using Key=std::tuple<size_t,bool,size_t>; std::map<Key,V> Pending;
			auto Add=[&](size_t G,size_t I,bool T,const V& Value) { Pending.emplace(Key(G,T,I),V(0)).first->second+=Value; };
			const auto& Samples=Groom->GetGuideSamples(Lod,GuideIndex,Index,Torque);
			const V Point=Torque ? V(0) : Double(Guides[GuideIndex].Points[Index].Position);
			for (const auto& Sample : Samples)
			{
				const auto& G=Guides[Sample.Guide];
				if (Torque)
				{ Add(Sample.Guide,Sample.FrameA,true,Load*(Sample.Weight*(1-Sample.FrameBlend))); Add(Sample.Guide,Sample.FrameB,true,Load*(Sample.Weight*Sample.FrameBlend)); }
				else
				{
					const V F=Load*Sample.Weight; const V Center=Double(G.Points[Sample.Particle].Position)*(1-Sample.PositionBlend)+Double(G.Points[Sample.Particle+1].Position)*Sample.PositionBlend;
					Add(Sample.Guide,Sample.Particle,false,F*(1-Sample.PositionBlend)); Add(Sample.Guide,Sample.Particle+1,false,F*Sample.PositionBlend);
					const V Moment=(Point-Center).Cross(F); Add(Sample.Guide,Sample.FrameA,true,Moment*(1-Sample.FrameBlend)); Add(Sample.Guide,Sample.FrameB,true,Moment*Sample.FrameBlend);
				}
			}
			auto Target=[&](const Key& K)->V& { auto& G=Guides[std::get<0>(K)]; return (std::get<1>(K) ? G.Torques : G.Forces)[std::get<2>(K)]; };
			for (const auto& P : Pending) Require(Finite(Target(P.first)+P.second),"Groom load transfer overflow");
			for (const auto& P : Pending) Target(P.first)+=P.second;
		}
		void Project(Guide& G, size_t I, bool Bend, double H)
		{
			auto& A=G.Points[I]; auto& B=G.Points[I+1]; auto& Q=G.Frames[I];
			const auto Response=Bend ? BendResponse(G,I) : StretchResponse(G,I);
			const double WA=Bend || A.Pinned ? 0 : A.InverseMass, WB=Bend || B.Pinned ? 0 : B.InverseMass;
			const M IA=EffectiveInertia(Q), IB=Bend ? EffectiveInertia(G.Frames[I+1]) : M();
			V Alpha(0); for (int J=0; J<3; ++J) Alpha[J]=1/(Response.Stiffness[J]*H*H);
			const M Compliance=M::Diagonal(Alpha);
			const M System=(Response.PositionA*Response.PositionA.Transpose())*WA+(Response.PositionB*Response.PositionB.Transpose())*WB
				+Response.RotationA*IA*Response.RotationA.Transpose()+Response.RotationB*IB*Response.RotationB.Transpose()+Compliance;
			auto& Lambda=Bend ? G.BendLambda[I] : G.StretchLambda[I]; V Delta(0);
			if (!System.Solve((Response.Value+Compliance*Lambda)*(-1),Delta,Settings.Numerics.RelativePivotTolerance)) throw std::runtime_error("Strand XPBD block is ill-conditioned");
			const V DXA=(Response.PositionA.Transpose()*Delta)*WA, DXB=(Response.PositionB.Transpose()*Delta)*WB;
			const V TA=Response.RotationA.Transpose()*Delta, TB=Response.RotationB.Transpose()*Delta;
			const V DQA=IA*TA, DQB=IB*TB;
			double Scale=1; const double PositionLimit=G.Lengths[I]*Settings.Numerics.MaximumPositionCorrectionRatio;
			for (const auto& D : {DXA,DXB}) if (Magnitude(D)>PositionLimit) Scale=std::min(Scale,PositionLimit/Magnitude(D));
			for (const auto& D : {DQA,DQB}) if (Magnitude(D)>Settings.Numerics.MaximumAngularCorrection) Scale=std::min(Scale,Settings.Numerics.MaximumAngularCorrection/Magnitude(D));
			Delta*=Scale; Lambda+=Delta;
			if (!Bend) { A.Position=Single(Double(A.Position)+DXA*Scale); B.Position=Single(Double(B.Position)+DXB*Scale); }
			if (!Q.Pinned) { Q.Orientation=(StrandElasticity::Increment(DQA*Scale)*Q.Orientation).Unit(); Q.AngularMomentum+=TA*(Scale/H); }
			if (Bend && !G.Frames[I+1].Pinned)
			{
				auto& Other=G.Frames[I+1]; Other.Orientation=(StrandElasticity::Increment(DQB*Scale)*Other.Orientation).Unit(); Other.AngularMomentum+=TB*(Scale/H);
			}
			if (G.RootEnabled && I==0)
			{
				if (!Bend) G.RootImpulse+=(Response.PositionA.Transpose()*Delta)/H;
				G.RootMoment+=(Response.RotationA.Transpose()*Delta)/H;
			}
			++Stats.ConstraintSolves;
		}
		void ProjectCoupled(Guide& G, double H)
		{
			struct Term { size_t Dof=0; M Jacobian; };
			struct Row { StrandResponse Response; std::array<Term,3> Terms; size_t Count=0; V* Lambda=nullptr; };
			const size_t PointCount=G.Points.size(), Dofs=PointCount+G.Frames.size(), Axes=3, NeighborRows=2;
			std::vector<M> Weights; Weights.reserve(Dofs);
			for (const auto& P : G.Points) Weights.push_back(M::Diagonal(V(P.Pinned ? 0 : P.InverseMass)));
			for (const auto& F : G.Frames) Weights.push_back(EffectiveInertia(F));
			std::vector<Row> Rows; Rows.reserve(G.Lengths.size()+G.RestRelative.size());
			for (size_t I=0; I<G.Lengths.size(); ++I)
			{
				Row S; S.Response=StretchResponse(G,I);
				S.Terms[0]={I,S.Response.PositionA}; S.Terms[1]={I+1,S.Response.PositionB}; S.Terms[2]={PointCount+I,S.Response.RotationA}; S.Count=3; S.Lambda=&G.StretchLambda[I]; Rows.push_back(S);
				if (I<G.RestRelative.size())
				{
					Row B; B.Response=BendResponse(G,I);
					B.Terms[0]={PointCount+I,B.Response.RotationA}; B.Terms[1]={PointCount+I+1,B.Response.RotationB}; B.Count=2; B.Lambda=&G.BendLambda[I]; Rows.push_back(B);
				}
			}
			BandSystem System(Rows.size()*Axes,NeighborRows*Axes+Axes-1); std::vector<double> Right(Rows.size()*Axes);
			for (size_t I=0; I<Rows.size(); ++I)
			{
				const auto& A=Rows[I];
				for (size_t J=I>NeighborRows ? I-NeighborRows : 0; J<=I; ++J)
				{
					const auto& B=Rows[J]; M Block;
					for (size_t TA=0; TA<A.Count; ++TA) for (size_t TB=0; TB<B.Count; ++TB)
						if (A.Terms[TA].Dof==B.Terms[TB].Dof) Block=Block+A.Terms[TA].Jacobian*Weights[A.Terms[TA].Dof]*B.Terms[TB].Jacobian.Transpose();
					for (size_t AxisA=0; AxisA<Axes; ++AxisA) for (size_t AxisB=0; AxisB<(I==J ? AxisA+1 : Axes); ++AxisB)
						System.At(I*Axes+AxisA,J*Axes+AxisB)=Block[(int)AxisA][(int)AxisB];
				}
				for (size_t Axis=0; Axis<Axes; ++Axis)
				{
					const double Alpha=1/(A.Response.Stiffness[(int)Axis]*H*H); const size_t Index=I*Axes+Axis;
					System.At(Index,Index)+=Alpha; Right[Index]=-A.Response.Value[(int)Axis]-Alpha*(*A.Lambda)[(int)Axis];
				}
			}
			const auto Delta=System.Solve(Right,Settings.Numerics.RelativePivotTolerance); std::vector<V> Impulses(Dofs,V(0)), Correction(Dofs,V(0));
			for (size_t I=0; I<Rows.size(); ++I)
			{
				const V DL(Delta[I*Axes],Delta[I*Axes+1],Delta[I*Axes+2]); const auto& R=Rows[I];
				for (size_t T=0; T<R.Count; ++T) Impulses[R.Terms[T].Dof]+=R.Terms[T].Jacobian.Transpose()*DL;
			}
			double Scale=1;
			for (size_t I=0; I<Dofs; ++I)
			{
				Correction[I]=Weights[I]*Impulses[I]; const double Length=Magnitude(Correction[I]); Require(std::isfinite(Length),"Strand coupled correction overflow");
				const double Limit=I<PointCount ? std::min(G.Lengths[I>0 ? I-1 : 0],G.Lengths[std::min(I,G.Lengths.size()-1)])*Settings.Numerics.MaximumPositionCorrectionRatio : Settings.Numerics.MaximumAngularCorrection;
				if (Length>Limit) Scale=std::min(Scale,Limit/Length);
			}
			for (size_t I=0; I<Rows.size(); ++I) *Rows[I].Lambda+=V(Delta[I*Axes],Delta[I*Axes+1],Delta[I*Axes+2])*Scale;
			for (size_t I=0; I<PointCount; ++I) if (!G.Points[I].Pinned) G.Points[I].Position=Single(Double(G.Points[I].Position)+Correction[I]*Scale);
			for (size_t I=0; I<G.Frames.size(); ++I) if (!G.Frames[I].Pinned)
			{
				auto& F=G.Frames[I]; F.Orientation=(StrandElasticity::Increment(Correction[PointCount+I]*Scale)*F.Orientation).Unit(); F.AngularMomentum+=Impulses[PointCount+I]*(Scale/H);
			}
			if (G.RootEnabled) { G.RootImpulse+=Impulses[0]*(Scale/H); G.RootMoment+=Impulses[PointCount]*(Scale/H); }
			Stats.ConstraintSolves+=Rows.size();
		}
		void Predict(Guide& G, const Transform& Root, double H)
		{
			if (G.RootEnabled) { G.Pins[0]=Root.pos; G.FramePins[0]=Root.quat; }
			for (size_t I=0; I<G.Points.size(); ++I)
			{
				auto& P=G.Points[I];
				if (P.Pinned)
				{
					const V Velocity=(Double(G.Pins[I])-Double(P.Position))/H;
					if (G.RootEnabled && I==0) G.RootImpulse+=(Double(P.Velocity)-Velocity)/P.InverseMass+(Double(Settings.Gravity)/P.InverseMass+G.Forces[I])*H;
					P.Position=G.Pins[I];
				}
				else
				{
					P.Velocity=Single(Double(P.Velocity)*std::exp(-Settings.LinearDamping*H)+(Double(Settings.Gravity)+G.Forces[I]*P.InverseMass)*H);
					P.Position=Single(Double(P.Position)+Double(P.Velocity)*H);
				}
			}
			for (size_t I=0; I<G.Frames.size(); ++I)
			{
				auto& D=G.Frames[I];
				if (D.Pinned)
				{
					const auto Old=D.Orientation; const V Momentum=D.AngularMomentum; D.Orientation=G.FramePins[I];
					D.AngularMomentum=WorldInertia(D,false)*(StrandElasticity::Log(D.Orientation*Old.Conjugate())/H);
					if (G.RootEnabled && I==0) G.RootMoment+=Momentum-D.AngularMomentum+G.Torques[I]*H;
				}
				else
				{
					D.AngularMomentum=D.AngularMomentum*std::exp(-Settings.AngularDamping*H)+G.Torques[I]*H;
					D.Orientation=(StrandElasticity::Increment((WorldInertia(D,true)*D.AngularMomentum)*H)*D.Orientation).Unit();
				}
			}
			std::fill(G.StretchLambda.begin(),G.StretchLambda.end(),V(0)); std::fill(G.BendLambda.begin(),G.BendLambda.end(),V(0));
		}
		void ProjectMaterial(Guide& G,double H)
		{
			if (Settings.Algorithm==StrandAlgorithm::DistanceConstraints)
			{
				for (size_t I=0; I<G.Lengths.size(); ++I)
				{
					auto& A=G.Points[I]; auto& B=G.Points[I+1];
					const V Delta=Double(B.Position)-Double(A.Position); const double Length=Magnitude(Delta);
					if (Length<=Settings.Numerics.MinimumLength) continue;
					const double WA=A.Pinned ? 0 : A.InverseMass, WB=B.Pinned ? 0 : B.InverseMass;
					if (WA+WB==0) continue;
					const V Correction=Delta*((Length-G.Lengths[I])/(Length*(WA+WB)));
					if (WA>0) A.Position=Single(Double(A.Position)+Correction*WA);
					if (WB>0) B.Position=Single(Double(B.Position)-Correction*WB);
					if (G.RootEnabled && I==0) G.RootImpulse+=Correction/H;
					++Stats.ConstraintSolves;
				}
				return;
			}
			if (Settings.Solver==StrandSolver::CoupledBanded) ProjectCoupled(G,H);
			else
			{
				for (size_t I=0; I<G.Lengths.size(); ++I) Project(G,I,false,H);
				for (size_t I=0; I<G.RestRelative.size(); ++I) Project(G,I,true,H);
			}
		}
		bool HasCollision() const { return Settings.Collision.Enabled && (Collision->GetColliderCount() || Settings.Collision.SelfCollision || Settings.Collision.InterGuideCollision); }
		std::vector<StrandContactGuide> ContactViews()
		{
			std::vector<StrandContactGuide> Views;
			for (size_t I=0; I<Guides.size(); ++I) if (Active(I))
			{
				// Contacts borrow live positions only until EndSubstep. Restoring a
				// rejected interval rebuilds these views before another projection.
				auto& G=Guides[I]; StrandContactGuide View; View.Id=I; View.Rooted=G.RootEnabled; View.RootImpulse=G.RootEnabled ? &G.RootImpulse : nullptr;
				View.Radius=std::max(G.Rest.Material.RadiusX,G.Rest.Material.RadiusY); View.RestArc.push_back(0);
				for (const auto L : G.Lengths) View.RestArc.push_back(View.RestArc.back()+L);
				for (auto& P : G.Points) { View.Particles.push_back({&P.Position,P.Pinned ? 0 : P.InverseMass}); View.Start.push_back(P.Position); }
				Views.push_back(std::move(View));
			}
			return Views;
		}
		void Measure()
		{
			Stats.ElasticEnergy=Stats.MaximumStretchShear=Stats.MaximumBendTwist=0;
			Stats.SimulatedGuides=Stats.InterpolatedGuides=0;
			for (size_t GuideIndex=0; GuideIndex<Guides.size(); ++GuideIndex)
			{
				if (!Active(GuideIndex)) { ++Stats.InterpolatedGuides; continue; } ++Stats.SimulatedGuides; const auto& G=Guides[GuideIndex];
				if (Settings.Algorithm==StrandAlgorithm::DistanceConstraints)
				{
					for (size_t I=0; I<G.Lengths.size(); ++I)
						Stats.MaximumStretchShear=std::max(Stats.MaximumStretchShear,std::abs(Magnitude(Double(G.Points[I+1].Position)-Double(G.Points[I].Position))-G.Lengths[I])/G.Lengths[I]);
					continue;
				}
				for (size_t I=0; I<G.Lengths.size(); ++I)
				{
					const auto R=StretchResponse(G,I);
					Stats.ElasticEnergy+=R.Energy; Stats.MaximumStretchShear=std::max(Stats.MaximumStretchShear,Magnitude(R.Value));
				}
				for (size_t I=0; I<G.RestRelative.size(); ++I)
				{
					const auto R=BendResponse(G,I);
					Stats.ElasticEnergy+=R.Energy; Stats.MaximumBendTwist=std::max(Stats.MaximumBendTwist,Magnitude(R.Value));
				}
			}
			Require(std::isfinite(Stats.ElasticEnergy),"Strand system energy overflow");
		}
	};
	StrandSystem::StrandSystem(const std::vector<StrandGuide>& Input, const StrandSettings& S) : m_Data(new Storage)
	{
		Require(S.IsValid() && !Input.empty(),"Invalid strand system settings or guides"); m_Data->Settings=S; m_Data->Collision.reset(new StrandCollision(S.Collision));
		for (const auto& Rest : Input)
		{
			Require(Rest.Positions.size()>=2 && Rest.Material.IsValid() && (Rest.Directors.empty() || Rest.Directors.size()+1==Rest.Positions.size()),"Invalid strand guide");
			for (const auto& P : Rest.Positions) Require(Finite(Double(P)),"Invalid strand rest position");
			Storage::Guide G; G.Rest=Rest; G.Rest.Directors=StrandElasticity::PrepareDirectors(Rest.Positions,Rest.Directors,Rest.ReferenceNormal,S.RestTangentTolerance,S.Numerics); G.RootEnabled=Rest.PinRoot;
			G.Root=G.PreviousRoot=Transform(Rest.Positions[0],G.Rest.Directors[0]);
			G.Points.resize(Rest.Positions.size()); G.Pins=Rest.Positions; G.Forces.resize(Rest.Positions.size(),V(0));
			std::vector<double> Mass(Rest.Positions.size(),0);
			for (size_t I=0; I<G.Rest.Directors.size(); ++I)
			{
				const double Length=Magnitude(Double(Rest.Positions[I+1])-Double(Rest.Positions[I])); G.Lengths.push_back(Length);
				const double ElementMass=Rest.Material.Density*Rest.Material.Area()*Length;
				Require(std::isfinite(ElementMass) && ElementMass>0,"Invalid strand mass"); Mass[I]+=ElementMass*0.5; Mass[I+1]+=ElementMass*0.5;
				StrandDirector D; D.Orientation=G.Rest.Directors[I]; const V Inertia=Rest.Material.DirectorInertia(Length);
				for (int J=0; J<3; ++J) { D.InverseInertia[J]=1/Inertia[J]; Require(std::isfinite(D.InverseInertia[J]) && D.InverseInertia[J]>0,"Invalid strand director inertia"); }
				D.Pinned=Rest.PinRoot && I==0; G.Frames.push_back(D);
				if (I) G.RestRelative.push_back((G.Rest.Directors[I-1].Conjugate()*D.Orientation).Unit());
			}
			for (size_t I=0; I<G.Points.size(); ++I)
			{
				Require(std::isfinite(Mass[I]) && Mass[I]>0 && std::isfinite(1/Mass[I]),"Invalid strand particle mass");
				G.Points[I].Position=Rest.Positions[I]; G.Points[I].InverseMass=1/Mass[I]; G.BaseInverseMass.push_back(1/Mass[I]); G.Points[I].Pinned=Rest.PinRoot && I==0;
			}
			G.FramePins=G.Rest.Directors; G.Torques.resize(G.Frames.size(),V(0)); G.StretchLambda.resize(G.Lengths.size(),V(0)); G.BendLambda.resize(G.RestRelative.size(),V(0)); m_Data->Guides.push_back(std::move(G));
		}
		m_Data->Measure();
	}
	StrandSystem::~StrandSystem() = default;
	size_t StrandSystem::GetGuideCount() const { return m_Data->Guides.size(); }
	uint64_t StrandSystem::GetInstanceId() const { return m_Data->InstanceId; }
	size_t StrandSystem::GetParticleCount(size_t G) const { return m_Data->Guides.at(G).Points.size(); }
	size_t StrandSystem::GetDirectorCount(size_t G) const { return m_Data->Guides.at(G).Frames.size(); }
	const StrandParticle& StrandSystem::GetParticle(size_t G, size_t P) const { return m_Data->Guides.at(G).Points.at(P); }
	const StrandDirector& StrandSystem::GetDirector(size_t G, size_t D) const { return m_Data->Guides.at(G).Frames.at(D); }
	StrandVector StrandSystem::GetAngularVelocity(size_t G, size_t D) const { const auto& F=GetDirector(G,D); return WorldInertia(F,true)*F.AngularMomentum; }
	const StrandStats& StrandSystem::GetStats() const { return m_Data->Stats; }
	const StrandSettings& StrandSystem::GetSettings() const { return m_Data->Settings; }
	void StrandSystem::SetSolver(StrandSolver Solver)
	{ m_Data->Idle(); auto S=m_Data->Settings; S.Solver=Solver; Require(S.IsValid(),"Invalid strand solver"); m_Data->Settings=S; }
	void StrandSystem::SetAlgorithm(StrandAlgorithm Algorithm)
	{ m_Data->Idle(); auto S=m_Data->Settings; S.Algorithm=Algorithm; Require(S.IsValid(),"Invalid strand algorithm"); m_Data->Settings=S; }
	void StrandSystem::SetCollisionSettings(const StrandCollisionSettings& Settings)
	{ m_Data->Idle(); Require(Settings.IsValid(),"Invalid strand collision settings"); m_Data->Collision->SetSettings(Settings); m_Data->Settings.Collision=Settings; }
	bool StrandSystem::AddCollider(Geometry* Shape)
	{
		m_Data->Idle(); if (!Shape) return false; auto* Body=Shape->GetParent<RigidBody>();
		if (auto* W=m_Data->World) if (Body) Require(std::find(W->m_StaticBodies.begin(),W->m_StaticBodies.end(),Body)!=W->m_StaticBodies.end()
			|| std::find(W->m_DynamicBodies.begin(),W->m_DynamicBodies.end(),Body)!=W->m_DynamicBodies.end()
			|| std::find(W->m_KinematicBodies.begin(),W->m_KinematicBodies.end(),Body)!=W->m_KinematicBodies.end(),"Register strand colliders from their world");
		return m_Data->Collision->AddCollider(Shape);
	}
	bool StrandSystem::AddCapsuleCollider(RigidBody* Body,float Radius,const Vector3& A,const Vector3& B)
	{
		m_Data->Idle();
		if (auto* W=m_Data->World) if (Body) Require(std::find(W->m_StaticBodies.begin(),W->m_StaticBodies.end(),Body)!=W->m_StaticBodies.end()
			|| std::find(W->m_DynamicBodies.begin(),W->m_DynamicBodies.end(),Body)!=W->m_DynamicBodies.end()
			|| std::find(W->m_KinematicBodies.begin(),W->m_KinematicBodies.end(),Body)!=W->m_KinematicBodies.end(),"Register strand colliders from their world");
		return m_Data->Collision->AddCapsule(Body,Radius,A,B);
	}
	bool StrandSystem::RemoveCollider(const Geometry* Shape) { m_Data->Idle(); return m_Data->Collision->RemoveCollider(Shape); }
	void StrandSystem::ClearColliders() { m_Data->Idle(); m_Data->Collision->ClearColliders(); }
	size_t StrandSystem::GetColliderCount() const { return m_Data->Collision->GetColliderCount(); }
	const StrandCollisionStats& StrandSystem::GetCollisionStats() const { return m_Data->CollisionStats; }
	const StrandGroom* StrandSystem::GetGroom() const { return m_Data->Groom.get(); }
	const GroomCurveState& StrandSystem::GetGroomStrand(size_t Index) const { return m_Data->Render.at(Index); }
	size_t StrandSystem::GetGroomLod() const { return m_Data->Lod; }
	bool StrandSystem::IsGuideSimulated(size_t Index) const { (void)m_Data->Guides.at(Index); return m_Data->Active(Index); }
	void StrandSystem::RefreshGroom() { m_Data->Idle(); m_Data->RenderGroom(*this); }
	void StrandSystem::SetGroomRootPosition(size_t Strand, const Vector3& Position)
	{ m_Data->Idle(); Require(Finite(Double(Position)),"Invalid groom root position"); auto& Root=m_Data->RenderRoots.at(Strand); Root.Position=Position; Root.Body=nullptr; Root.Enabled=true; }
	void StrandSystem::AttachGroomRoot(size_t Strand, RigidBody* Body, const Vector3& Position)
	{
		m_Data->Idle(); Require(Body!=nullptr && Finite(Double(Position)) && Finite(Double(Body->X)),"Invalid groom root attachment"); ValidRotation(Body->Q,m_Data->Settings);
		if (auto* W=m_Data->World) Require(std::find(W->m_StaticBodies.begin(),W->m_StaticBodies.end(),Body)!=W->m_StaticBodies.end()
			|| std::find(W->m_DynamicBodies.begin(),W->m_DynamicBodies.end(),Body)!=W->m_DynamicBodies.end()
			|| std::find(W->m_KinematicBodies.begin(),W->m_KinematicBodies.end(),Body)!=W->m_KinematicBodies.end(),"Attach groom roots to a body in their world");
		const auto Local=Body->WorldToBodyLocal(Position); Require(Finite(Double(Local)),"Invalid local groom root");
		auto& Root=m_Data->RenderRoots.at(Strand); Root.Body=Body; Root.Position=Position; Root.Local=Local; Root.Enabled=true;
	}
	Vector3 StrandSystem::GetGroomRootPosition(size_t Strand) const
	{ const auto& Root=m_Data->RenderRoots.at(Strand); return Root.Enabled ? Root.Target() : m_Data->Render.at(Strand).Positions[0]; }
	void StrandSystem::UnpinGroomRoot(size_t Strand)
	{ m_Data->Idle(); auto& Root=m_Data->RenderRoots.at(Strand); Root.Enabled=false; Root.Body=nullptr; }
	void StrandSystem::ConfigureGroom(const StrandGroomDefinition& Definition)
	{
		m_Data->Idle(); std::vector<StrandGuide> Rest;
		for (const auto& G : m_Data->Guides) { Require(G.RootEnabled,"A groom requires rooted guides"); Rest.push_back(G.Rest); }
		auto Groom=std::make_shared<StrandGroom>(Rest,Definition,m_Data->Settings); std::vector<GroomCurveState> Render(Groom->GetStrandCount());
		for (size_t I=0; I<Render.size(); ++I) Groom->EvaluateStrand(I,*this,Render[I]);
		std::vector<Storage::RenderRoot> Roots(Render.size());
		ClearGroom(); m_Data->Groom=std::move(Groom); m_Data->Render=std::move(Render); m_Data->RenderRoots=std::move(Roots); m_Data->Measure();
	}
	void StrandSystem::ClearGroom()
	{
		m_Data->Idle(); if (m_Data->Groom && m_Data->Lod) SetGroomLod(0);
		m_Data->Groom.reset(); m_Data->Render.clear(); m_Data->RenderScratch.clear(); m_Data->RenderRoots.clear(); m_Data->Lod=0;
		for (auto& G : m_Data->Guides) { m_Data->WeightGuide(G,1); G.BlendDuration=G.BlendElapsed=0; } m_Data->Measure();
	}
	void StrandSystem::SetGroomLod(size_t Level)
	{
		m_Data->Idle(); Require(m_Data->Groom!=nullptr,"Configure a groom before selecting LOD"); const auto& L=m_Data->Groom->GetLod(Level); if (Level==m_Data->Lod) return;
		for (size_t I=0; I<GetGuideCount(); ++I) if (!m_Data->Groom->IsSimulated(Level,I))
		{
			const auto& G=m_Data->Guides[I];
			for (size_t P=1; P<G.Points.size(); ++P) Require(!G.Points[P].Pinned,"Groom LOD cannot remove a guide with prescribed particles");
			for (size_t F=1; F<G.Frames.size(); ++F) Require(!G.Frames[F].Pinned,"Groom LOD cannot remove a guide with prescribed directors");
		}
		const auto Saved=m_Data->Guides; const auto OldLod=m_Data->Lod; const auto OldStats=m_Data->Stats;
		try
		{
			m_Data->Lod=Level;
			for (size_t I=0; I<GetGuideCount(); ++I)
			{
				auto& G=m_Data->Guides[I]; m_Data->WeightGuide(G,m_Data->Active(I) ? m_Data->Groom->GetSimulationWeight(Level,I) : 1);
				if (!m_Data->Active(I)) m_Data->CaptureBlend(G,L.TransitionTime);
				std::fill(G.Forces.begin(),G.Forces.end(),V(0)); std::fill(G.Torques.begin(),G.Torques.end(),V(0));
				G.RootForce=G.RootTorque=G.RootImpulse=G.RootMoment=V(0);
			}
			for (size_t I=0; I<GetGuideCount(); ++I)
			{
				for (size_t P=0; P<Saved[I].Forces.size(); ++P) m_Data->RouteLoad(I,P,false,Saved[I].Forces[P]);
				for (size_t F=0; F<Saved[I].Torques.size(); ++F) m_Data->RouteLoad(I,F,true,Saved[I].Torques[F]);
			}
			for (const auto& G : m_Data->Guides) { for (const auto& F : G.Forces) Require(Finite(F),"Groom force transfer overflow"); for (const auto& T : G.Torques) Require(Finite(T),"Groom torque transfer overflow"); }
			m_Data->Measure();
		}
		catch (...) { m_Data->Guides=Saved; m_Data->Lod=OldLod; m_Data->Stats=OldStats; throw; }
	}
	void StrandSystem::UpdateGroomDistance(double Distance)
	{
		m_Data->Idle(); Require(m_Data->Groom && std::isfinite(Distance) && Distance>=0,"Invalid groom LOD distance"); size_t L=m_Data->Lod;
		const auto& Groom=*m_Data->Groom;
		auto Hysteresis=[&](size_t Level) { return std::min(Groom.GetInterpolation().LodHysteresis,(Groom.GetLod(Level).Distance-Groom.GetLod(Level-1).Distance)*0.5); };
		while (L+1<Groom.GetLodCount() && Distance>=Groom.GetLod(L+1).Distance+Hysteresis(L+1)) ++L;
		while (L>0 && Distance<Groom.GetLod(L).Distance-Hysteresis(L)) --L;
		SetGroomLod(L);
	}
	Transform StrandSystem::GetRootPose(size_t G) const { return m_Data->Guides.at(G).Target(); }
	StrandVector StrandSystem::GetRootForce(size_t G) const { return m_Data->Guides.at(G).RootForce; }
	StrandVector StrandSystem::GetRootTorque(size_t G) const { return m_Data->Guides.at(G).RootTorque; }
	void StrandSystem::SetParticlePosition(size_t G, size_t P, const Vector3& Position, bool ResetVelocity)
	{ m_Data->Idle(); Require(m_Data->Active(G),"Edit a simulated guide"); Require(Finite(Double(Position)),"Invalid strand position"); auto& Point=m_Data->Guides.at(G).Points.at(P); Point.Position=Position; if (ResetVelocity) Point.Velocity=Vector3::Zero(); }
	void StrandSystem::SetParticleVelocity(size_t G, size_t P, const Vector3& Velocity)
	{ m_Data->Idle(); Require(m_Data->Active(G),"Edit a simulated guide"); Require(Finite(Double(Velocity)),"Invalid strand velocity"); m_Data->Guides.at(G).Points.at(P).Velocity=Velocity; }
	void StrandSystem::PinParticle(size_t G, size_t P, const Vector3& Position)
	{
		m_Data->Idle(); Require(P==0 || m_Data->Active(G),"Pin a simulated guide"); Require(Finite(Double(Position)),"Invalid strand pin"); auto& Guide=m_Data->Guides.at(G); Guide.Points.at(P).Pinned=true; Guide.Pins.at(P)=Position;
		if (P==0) { auto Pose=Guide.Target(); Pose.pos=Position; SetRootPose(G,Pose); }
	}
	void StrandSystem::UnpinParticle(size_t G, size_t P)
	{ m_Data->Idle(); Require(P!=0 || !m_Data->Groom,"Clear the groom before unpinning its root"); auto& Guide=m_Data->Guides.at(G); Guide.Points.at(P).Pinned=false; if (P==0) { Guide.RootEnabled=false; Guide.Body=nullptr; Guide.Frames[0].Pinned=false; } }
	void StrandSystem::PinDirector(size_t G, size_t D, const Quaternion& Q)
	{
		m_Data->Idle(); Require(D==0 || m_Data->Active(G),"Pin a simulated guide"); const auto Rotation=ValidRotation(Q,m_Data->Settings); auto& Guide=m_Data->Guides.at(G); Guide.Frames.at(D).Pinned=true; Guide.FramePins.at(D)=Rotation;
		if (D==0 && Guide.RootEnabled) { auto Pose=Guide.Target(); Pose.quat=Rotation; SetRootPose(G,Pose); }
	}
	void StrandSystem::UnpinDirector(size_t G, size_t D)
	{ m_Data->Idle(); auto& Guide=m_Data->Guides.at(G); Require(D!=0 || !Guide.RootEnabled,"A groom root owns its first director"); Guide.Frames.at(D).Pinned=false; }
	void StrandSystem::AddForce(size_t G, size_t P, const Vector3& Force)
	{ m_Data->Idle(); (void)m_Data->Guides.at(G).Forces.at(P); Require(Finite(Double(Force)),"Invalid strand force"); m_Data->RouteLoad(G,P,false,Double(Force)); }
	void StrandSystem::AddTorque(size_t G, size_t D, const Vector3& Torque)
	{ m_Data->Idle(); (void)m_Data->Guides.at(G).Torques.at(D); Require(Finite(Double(Torque)),"Invalid strand torque"); m_Data->RouteLoad(G,D,true,Double(Torque)); }
	void StrandSystem::SetRootPose(size_t Index, const Transform& Pose)
	{
		m_Data->Idle(); Require(Finite(Double(Pose.pos)),"Invalid strand root position"); const auto Q=ValidRotation(Pose.quat,m_Data->Settings); auto& G=m_Data->Guides.at(Index);
		G.Body=nullptr; G.Root=Transform(Pose.pos,Q); G.RootEnabled=true; G.Points[0].Pinned=G.Frames[0].Pinned=true;
	}
	void StrandSystem::AttachRoot(size_t Index, RigidBody* Body, const Transform& Pose)
	{
		m_Data->Idle(); Require(Body!=nullptr,"Strand attachment requires a body");
		if (auto* W=m_Data->World)
			Require(std::find(W->m_StaticBodies.begin(),W->m_StaticBodies.end(),Body)!=W->m_StaticBodies.end()
				|| std::find(W->m_DynamicBodies.begin(),W->m_DynamicBodies.end(),Body)!=W->m_DynamicBodies.end()
				|| std::find(W->m_KinematicBodies.begin(),W->m_KinematicBodies.end(),Body)!=W->m_KinematicBodies.end(),"Attach strands to a body in their world");
		Require(Finite(Double(Body->X)) && Finite(Double(Pose.pos)),"Invalid attached body or root position");
		ValidRotation(Body->Q,m_Data->Settings); ValidRotation(Pose.quat,m_Data->Settings);
		const Transform Local(Body->WorldToBodyLocal(Pose.pos),(Body->Q.Conjugate()*Pose.quat).Unit());
		Require(Finite(Double(Local.pos)),"Invalid local strand root"); SetRootPose(Index,Pose); auto& G=m_Data->Guides.at(Index);
		G.LocalRoot=Local; G.Body=Body;
	}
	void StrandSystem::DetachBody(RigidBody* Body)
	{ m_Data->Idle(); OnBodyRemoved(Body); }
	void StrandSystem::OnBodyRemoved(RigidBody* Body)
	{
		for (auto& G : m_Data->Guides) if (G.Body==Body) { G.Root=G.Target(); G.Body=nullptr; }
		for (auto& R : m_Data->RenderRoots) if (R.Body==Body) { R.Position=R.Target(); R.Body=nullptr; }
		m_Data->Collision->RemoveBody(Body);
	}
	void StrandSystem::Reset()
	{
		m_Data->Idle(); m_Data->Stats={};
		for (auto& G : m_Data->Guides)
		{
			const auto Target=G.Target(); const Transform RestRoot(G.Rest.Positions[0],G.Rest.Directors[0]);
			for (size_t I=0; I<G.Points.size(); ++I) { G.Points[I].Position=Target.LocalToWorld(RestRoot.WorldToLocal(G.Rest.Positions[I])); G.Points[I].Velocity=Vector3::Zero(); G.Pins[I]=G.Points[I].Position; G.Forces[I]=V(0); }
			for (size_t I=0; I<G.Frames.size(); ++I) { G.Frames[I].Orientation=(Target.quat*RestRoot.quat.Conjugate()*G.Rest.Directors[I]).Unit(); G.FramePins[I]=G.Frames[I].Orientation; G.Frames[I].AngularMomentum=G.Torques[I]=V(0); }
			G.PreviousRoot=Target; G.BlendElapsed=G.BlendDuration=0; G.RootImpulse=G.RootMoment=G.RootForce=G.RootTorque=V(0);
		}
		m_Data->Measure(); m_Data->RenderGroom(*this); m_Data->Collision->BeginFrame(); m_Data->Collision->EndFrame(true); m_Data->CollisionStats={};
	}
	void StrandSystem::Simulate(float Dt)
	{
		m_Data->Idle(); Require(!m_Data->World,"Step owned strands through their physics world"); Step(Dt,true);
	}
	void StrandSystem::SetWorld(PhysicsWorld* World) { m_Data->World=World; }
	void StrandSystem::BeginWorldFrame(float Dt)
	{
		m_Data->FrameDt=Dt; m_Data->FrameElapsed=0; m_Data->FrameStats={};
		m_Data->FrameCollisionStats={};
		m_Data->FrameStats.ElasticEnergy=m_Data->Stats.ElasticEnergy;
		m_Data->FrameImpulses.assign(GetGuideCount(),V(0)); m_Data->FrameMoments.assign(GetGuideCount(),V(0));
	}
	void StrandSystem::SimulateWorldStep(float Dt)
	{
		try { Step(Dt,false); } catch (...) { m_Data->FrameCollisionStats.Accumulate(m_Data->CollisionStats); throw; }
		m_Data->FrameCollisionStats.Accumulate(m_Data->CollisionStats); auto& F=m_Data->FrameStats; const auto& S=m_Data->Stats;
		F.Substeps+=S.Substeps; F.ConstraintSolves+=S.ConstraintSolves; F.ElasticEnergy=S.ElasticEnergy;
		F.SimulatedGuides=S.SimulatedGuides; F.InterpolatedGuides=S.InterpolatedGuides;
		F.MaximumStretchShear=std::max(F.MaximumStretchShear,S.MaximumStretchShear); F.MaximumBendTwist=std::max(F.MaximumBendTwist,S.MaximumBendTwist);
		m_Data->FrameElapsed+=Dt;
		for (size_t I=0; I<GetGuideCount(); ++I) { m_Data->FrameImpulses[I]+=m_Data->Guides[I].RootImpulse; m_Data->FrameMoments[I]+=m_Data->Guides[I].RootMoment; }
	}
	void StrandSystem::EndWorldFrame(bool Success)
	{
		m_Data->CollisionStats=m_Data->FrameCollisionStats;
		const auto Previous=m_Data->Stats; m_Data->Stats=m_Data->FrameStats; m_Data->Stats.Time=Previous.Time; m_Data->Stats.FailedSteps=Previous.FailedSteps;
		if (!Success && m_Data->FrameElapsed==0)
		{ m_Data->Stats.MaximumStretchShear=Previous.MaximumStretchShear; m_Data->Stats.MaximumBendTwist=Previous.MaximumBendTwist; }
		const double Duration=Success ? m_Data->FrameDt : m_Data->FrameElapsed;
		for (size_t I=0; I<GetGuideCount(); ++I)
		{
			auto& G=m_Data->Guides[I]; G.RootForce=Duration>0 ? m_Data->FrameImpulses[I]/Duration : V(0); G.RootTorque=Duration>0 ? m_Data->FrameMoments[I]/Duration : V(0);
			if (Success) { std::fill(G.Forces.begin(),G.Forces.end(),V(0)); std::fill(G.Torques.begin(),G.Torques.end(),V(0)); }
		}
	}
	void StrandSystem::Step(float Dt, bool ConsumeLoads)
	{
		Require(std::isfinite(Dt) && Dt>0,"Invalid strand step"); const auto Saved=m_Data->Guides; const auto OldStats=m_Data->Stats;
		m_Data->Stepping=true;
		try
		{
			m_Data->Collision->BeginFrame();
			int Count=m_Data->Settings.Substeps; std::vector<Transform> Targets;
			for (const auto& G : m_Data->Guides)
			{
				const auto Target=G.Target(); Require(Finite(Double(Target.pos)),"Invalid attached strand root"); ValidRotation(Target.quat,m_Data->Settings); Targets.push_back(Target);
				if (G.RootEnabled)
				{
					const double Required=std::ceil(Magnitude(StrandElasticity::Log(Target.quat*G.PreviousRoot.quat.Conjugate()))/m_Data->Settings.MaximumRootAngle);
					Require(Required<=m_Data->Settings.MaximumSubsteps,"Strand root motion exceeds its substep budget"); Count=std::max(Count,(int)Required);
				}
			}
			Count=std::max(Count,m_Data->Collision->RequiredSubsteps(m_Data->Settings.MaximumSubsteps));
			Require(Count<=m_Data->Settings.MaximumSubsteps,"Strand root motion exceeds its substep budget");
			m_Data->Stats={}; m_Data->Stats.Time=OldStats.Time; m_Data->Stats.FailedSteps=OldStats.FailedSteps;
			for (auto& G : m_Data->Guides) G.RootImpulse=G.RootMoment=V(0);
			int Refinements=0; const bool Contacts=m_Data->HasCollision(); std::function<void(double,double,int)> Advance;
			Advance=[&](double Begin,double End,int Depth)
			{
				const auto Checkpoint=Contacts ? m_Data->Guides : std::vector<Storage::Guide>(); const double H=(double)Dt*(End-Begin);
				std::vector<std::vector<Vector3>> Start;
				for (const auto& G : m_Data->Guides) { Start.emplace_back(); for (const auto& P : G.Points) Start.back().push_back(P.Position); }
				try
				{
					if (Contacts) m_Data->Collision->BeginSubstep((float)Begin,(float)End,m_Data->ContactViews()); std::vector<Transform> Roots;
					for (size_t I=0; I<m_Data->Guides.size(); ++I)
					{
						auto& G=m_Data->Guides[I]; const auto Root=Transform(Single(Double(G.PreviousRoot.pos)+(Double(Targets[I].pos)-Double(G.PreviousRoot.pos))*End),StrandElasticity::Interpolate(G.PreviousRoot.quat,Targets[I].quat,End));
						Roots.push_back(Root); if (m_Data->Active(I)) m_Data->Predict(G,Root,H);
					}
					for (int Iteration=0; Iteration<m_Data->Settings.SolverIterations; ++Iteration)
					{
						for (size_t I=0; I<m_Data->Guides.size(); ++I) if (m_Data->Active(I)) m_Data->ProjectMaterial(m_Data->Guides[I],H);
						if (Contacts) m_Data->Collision->Solve(H);
					}
					if (Contacts && !m_Data->Collision->Finish(H)) throw StrandCollision::RefinementNeeded();
					for (size_t I=0; I<m_Data->Guides.size(); ++I)
					{
						auto& G=m_Data->Guides[I]; if (m_Data->Active(I)) for (size_t P=0; P<G.Points.size(); ++P) G.Points[P].Velocity=Single((Double(G.Points[P].Position)-Double(Start[I][P]))/H);
						else m_Data->InterpolateGuide(I,Roots[I],H,*this);
					}
					m_Data->Collision->EndSubstep(); ++m_Data->Stats.Substeps;
				}
				catch (const StrandCollision::RefinementNeeded&)
				{
					m_Data->Collision->EndSubstep(); m_Data->Guides=Checkpoint; const auto& Policy=m_Data->Settings.Collision;
					if (Depth>=Policy.MaximumRefinementDepth || Refinements>=Policy.MaximumRefinements || Count+Refinements>=m_Data->Settings.MaximumSubsteps) throw std::runtime_error("Strand contact refinement budget exhausted");
					++Refinements; m_Data->Collision->RecordRefinement(); const double Middle=(Begin+End)*0.5;
					Advance(Begin,Middle,Depth+1); Advance(Middle,End,Depth+1);
				}
			};
			for (int Step=0; Step<Count; ++Step) Advance((double)Step/Count,(double)(Step+1)/Count,0);
			m_Data->Measure();
			for (size_t I=0; I<m_Data->Guides.size(); ++I)
			{
				auto& G=m_Data->Guides[I]; G.PreviousRoot=Targets[I]; G.RootForce=G.RootImpulse/Dt; G.RootTorque=G.RootMoment/Dt;
				Require(Finite(G.RootForce) && Finite(G.RootTorque),"Strand root reaction overflow");
				if (ConsumeLoads) { std::fill(G.Forces.begin(),G.Forces.end(),V(0)); std::fill(G.Torques.begin(),G.Torques.end(),V(0)); }
			}
			m_Data->Stats.Time+=Dt; m_Data->RenderGroom(*this); m_Data->Collision->EndFrame(true); m_Data->CollisionStats=m_Data->Collision->GetStats(); m_Data->Stepping=false;
		}
		catch (...) { m_Data->Collision->EndFrame(false); m_Data->CollisionStats=m_Data->Collision->GetStats(); m_Data->Guides=Saved; m_Data->Stats=OldStats; ++m_Data->Stats.FailedSteps; m_Data->Stepping=false; throw; }
	}
}

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* Material-frame Cosserat strain
	   -----------------------------
	   A strand has a centerline and independent material directors. Elliptical
	   cross sections give axial EA, shear kGA, bending EIx/EIy and torsion GJ.
	   Material-coordinate stretch/shear is C = R^T (xB-xA)/L - e3. Its rotational
	   Jacobian is R^T [ (xB-xA)/L ]x. This derivative includes material-frame rotation,
	   and balances the orbital torque of the equal/opposite endpoint forces.

	   Bending/twist uses the SO(3) logarithm of RestRelative^-1 A^-1 B. The local
	   energy is 0.5 C^T diag(EIx,EIy,GJ)/DualLength C. Left world perturbations give
	   JA = -Jleft^-1(C) RestRelative^T RA^T and JB = -JA. Unlike a componentwise
	   quaternion difference, the shortest log is invariant to quaternion sign.
	   Its inverse Jacobian uses the cotangent half-angle form, including at pi;
	   the small-angle branch uses the mathematical Taylor coefficients.

	   Directors carry cross-section inertia rho*Ix/Iy/Jpolar*L. A rigid cylinder's
	   additional mL^2/12 would double count motion already represented by the
	   centerline's distributed translational mass. Torsion stiffness uses the
	   Saint-Venant constant pi*a^3*b^3/(a^2+b^2), while inertia uses the polar moment.

	   References: Kugelstadt and Schoemer, Position and Orientation Based Cosserat
	   Rods (SCA 2016), https://animation.rwth-aachen.de/media/papers/2016-SCA-Cosserat-Rods.pdf ; Macklin, Mueller and
	   Chentanez, XPBD (MIG 2016), https://mmacklin.com/xpbd.pdf . This implementation
	   uses tangent-space rotations and log strain, rather than copying quaternion
	   component projection from the rod paper. Finite-difference tests cover both
	   Jacobians and the corresponding energy gradients.
	*/
	namespace
	{
		bool ModelFinite(const StrandVector& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		void ModelRequire(bool Valid, const char* Message) { if (!Valid) throw std::invalid_argument(Message); }
		Quaternion Unit(const Quaternion& Q)
		{
			const double Norm = std::hypot(std::hypot((double)Q.x,(double)Q.y),std::hypot((double)Q.z,(double)Q.w));
			ModelRequire(std::isfinite(Norm) && Norm > 0,"Invalid strand rotation");
			return Quaternion((float)(Q.x/Norm),(float)(Q.y/Norm),(float)(Q.z/Norm),(float)(Q.w/Norm));
		}
		void Validate(const Quaternion& Q, const StrandNumerics& N)
		{
			const double Norm = (double)Q.x*Q.x+(double)Q.y*Q.y+(double)Q.z*Q.z+(double)Q.w*Q.w;
			ModelRequire(std::isfinite(Norm) && std::abs(Norm-1) <= N.QuaternionTolerance,"Strand director must be a unit quaternion");
		}
		double Energy(const StrandVector& C, const StrandVector& K)
		{ const double E = (K.x*C.x*C.x+K.y*C.y*C.y+K.z*C.z*C.z)*0.5; ModelRequire(std::isfinite(E),"Strand energy overflow"); return E; }
	}
	StrandMatrix StrandMatrix::Identity() { return Diagonal(StrandVector(1)); }
	StrandMatrix StrandMatrix::Diagonal(const StrandVector& V)
	{ StrandMatrix M; for (int I = 0; I < 3; ++I) M[I][I] = V[I]; return M; }
	StrandMatrix StrandMatrix::Rotation(const Quaternion& Input)
	{
		const auto Q = Unit(Input); const double X = Q.x, Y = Q.y, Z = Q.z, W = Q.w; StrandMatrix M;
		M[0][0]=1-2*(Y*Y+Z*Z); M[0][1]=2*(X*Y-W*Z); M[0][2]=2*(X*Z+W*Y);
		M[1][0]=2*(X*Y+W*Z); M[1][1]=1-2*(X*X+Z*Z); M[1][2]=2*(Y*Z-W*X);
		M[2][0]=2*(X*Z-W*Y); M[2][1]=2*(Y*Z+W*X); M[2][2]=1-2*(X*X+Y*Y); return M;
	}
	StrandMatrix StrandMatrix::Cross(const StrandVector& V)
	{
		StrandMatrix M; M[0][1]=-V.z; M[0][2]=V.y; M[1][0]=V.z; M[1][2]=-V.x; M[2][0]=-V.y; M[2][1]=V.x; return M;
	}
	StrandMatrix StrandMatrix::Transpose() const
	{ StrandMatrix M; for (int I=0; I<3; ++I) for (int J=0; J<3; ++J) M[I][J]=Values[J][I]; return M; }
	StrandMatrix StrandMatrix::operator+(const StrandMatrix& B) const
	{ StrandMatrix M; for (int I=0; I<3; ++I) for (int J=0; J<3; ++J) M[I][J]=Values[I][J]+B[I][J]; return M; }
	StrandMatrix StrandMatrix::operator*(const StrandMatrix& B) const
	{ StrandMatrix M; for (int I=0; I<3; ++I) for (int J=0; J<3; ++J) for (int K=0; K<3; ++K) M[I][J]+=Values[I][K]*B[K][J]; return M; }
	StrandMatrix StrandMatrix::operator*(double S) const
	{ StrandMatrix M; for (int I=0; I<3; ++I) for (int J=0; J<3; ++J) M[I][J]=Values[I][J]*S; return M; }
	StrandVector StrandMatrix::operator*(const StrandVector& V) const
	{ StrandVector R(0); for (int I=0; I<3; ++I) for (int J=0; J<3; ++J) R[I]+=Values[I][J]*V[J]; return R; }
	bool StrandMatrix::Solve(const StrandVector& B, StrandVector& X, double Tolerance) const
	{
		// Symmetric diagonal scaling avoids comparing unrelated translational and
		// angular inverse masses against a fixed determinant epsilon.
		StrandVector Scale(0), Y(0), Z(0); StrandMatrix L;
		for (int I=0; I<3; ++I) { if (!(Values[I][I] > 0) || !std::isfinite(Values[I][I])) return false; Scale[I]=std::sqrt(Values[I][I]); }
		for (int I=0; I<3; ++I) for (int J=0; J<=I; ++J)
		{
			double Value=(Values[I][J]/Scale[I])/Scale[J];
			for (int K=0; K<J; ++K) Value-=L[I][K]*L[J][K];
			if (!std::isfinite(Value) || (I==J && Value<=Tolerance)) return false;
			L[I][J]=I==J ? std::sqrt(Value) : Value/L[J][J];
		}
		for (int I=0; I<3; ++I) { Y[I]=B[I]/Scale[I]; for (int J=0; J<I; ++J) Y[I]-=L[I][J]*Y[J]; Y[I]/=L[I][I]; }
		for (int I=2; I>=0; --I) { Z[I]=Y[I]; for (int J=I+1; J<3; ++J) Z[I]-=L[J][I]*Z[J]; Z[I]/=L[I][I]; X[I]=Z[I]/Scale[I]; }
		return ModelFinite(X);
	}
	double StrandMaterial::Area() const { return std::acos(-1.0)*RadiusX*RadiusY; }
	StrandVector StrandMaterial::StretchShearStiffness(double L) const
	{ const double EA=YoungModulus*Area()*L, GA=EA*ShearAreaFactor/(2*(1+PoissonRatio)); return StrandVector(GA,GA,EA); }
	StrandVector StrandMaterial::BendTwistStiffness(double L) const
	{
		const double Ix=Area()*RadiusY*RadiusY/4, Iy=Area()*RadiusX*RadiusX/4;
		const double J=Area()*RadiusX*RadiusX*RadiusY*RadiusY/(RadiusX*RadiusX+RadiusY*RadiusY);
		return StrandVector(YoungModulus*Ix/L,YoungModulus*Iy/L,YoungModulus*J/(2*(1+PoissonRatio)*L));
	}
	StrandVector StrandMaterial::DirectorInertia(double L) const
	{ const double Mass=Density*Area()*L; return StrandVector(Mass*RadiusY*RadiusY/4,Mass*RadiusX*RadiusX/4,Mass*(RadiusX*RadiusX+RadiusY*RadiusY)/4); }
	bool StrandMaterial::IsValid() const
	{
		if (!std::isfinite(YoungModulus) || YoungModulus<=0 || !std::isfinite(PoissonRatio) || PoissonRatio<=-1 || PoissonRatio>=0.5
			|| !std::isfinite(Density) || Density<=0 || !std::isfinite(RadiusX) || RadiusX<=0 || !std::isfinite(RadiusY) || RadiusY<=0
			|| !std::isfinite(ShearAreaFactor) || ShearAreaFactor<=0 || ShearAreaFactor>1) return false;
		const auto S=StretchShearStiffness(1), B=BendTwistStiffness(1), I=DirectorInertia(1);
		return ModelFinite(S) && ModelFinite(B) && ModelFinite(I) && S.x>0 && S.z>0 && B.x>0 && B.y>0 && B.z>0 && I.x>0 && I.y>0 && I.z>0;
	}
	bool StrandNumerics::IsValid() const
	{
		return std::isfinite(MinimumLength) && MinimumLength>0 && std::isfinite(QuaternionTolerance) && QuaternionTolerance>0 && QuaternionTolerance<1
			&& std::isfinite(RotationSeriesAngle) && RotationSeriesAngle>0 && RotationSeriesAngle<1 && std::isfinite(RelativePivotTolerance) && RelativePivotTolerance>0 && RelativePivotTolerance<1
			&& std::isfinite(MaximumPositionCorrectionRatio) && MaximumPositionCorrectionRatio>0 && std::isfinite(MaximumAngularCorrection) && MaximumAngularCorrection>0;
	}
	Quaternion StrandElasticity::Increment(const StrandVector& V)
	{
		ModelRequire(ModelFinite(V),"Invalid strand angular increment"); const double Angle=std::hypot(std::hypot(V.x,V.y),V.z);
		ModelRequire(std::isfinite(Angle),"Strand angular increment overflow"); if (Angle==0) return Quaternion::One();
		const double S=std::sin(Angle*0.5)/Angle;
		return Unit(Quaternion((float)(V.x*S),(float)(V.y*S),(float)(V.z*S),(float)std::cos(Angle*0.5)));
	}
	Quaternion StrandElasticity::Interpolate(const Quaternion& A, const Quaternion& B, double Fraction)
	{
		ModelRequire(std::isfinite(Fraction) && Fraction>=0 && Fraction<=1,"Invalid strand rotation interpolation"); const auto Start=Unit(A);
		if (Fraction==0) return Start;
		if (Fraction==1) return Unit(B);
		// The relative log lives in A's material frame. Its canonical pi axis is
		// unchanged by a common world rotation or either quaternion's sign.
		return Unit(Start*Increment(Log(Start.Conjugate()*Unit(B))*Fraction));
	}
	std::vector<Quaternion> StrandElasticity::PrepareDirectors(const std::vector<Vector3>& Positions, const std::vector<Quaternion>& Directors,
		const Vector3& ReferenceNormal, double TangentTolerance, const StrandNumerics& N)
	{
		ModelRequire(N.IsValid() && Positions.size()>=2 && (Directors.empty() || Directors.size()+1==Positions.size())
			&& std::isfinite(TangentTolerance) && TangentTolerance>0 && TangentTolerance<1,"Invalid strand frame preparation");
		std::vector<Quaternion> Frames; Vector3 Normal=ReferenceNormal;
		ModelRequire(ModelFinite(StrandVector(Normal.x,Normal.y,Normal.z)) && Normal.SquareLength()>0,"Invalid strand reference normal");
		for (const auto& P : Positions) ModelRequire(ModelFinite(StrandVector(P.x,P.y,P.z)),"Invalid strand rest position");
		for (size_t I=0; I+1<Positions.size(); ++I)
		{
			const StrandVector Edge((double)Positions[I+1].x-Positions[I].x,(double)Positions[I+1].y-Positions[I].y,(double)Positions[I+1].z-Positions[I].z);
			const double Length=std::hypot(std::hypot(Edge.x,Edge.y),Edge.z);
			ModelRequire(std::isfinite(Length) && Length>=N.MinimumLength,"Degenerate strand segment");
			const Vector3 Tangent((float)(Edge.x/Length),(float)(Edge.y/Length),(float)(Edge.z/Length));
			if (!Directors.empty())
			{
				Validate(Directors[I],N); const auto Q=Unit(Directors[I]);
				ModelRequire((Q*Vector3::UnitZ()-Tangent).Length()<=TangentTolerance,"Rest strand director does not follow its tangent"); Frames.push_back(Q); continue;
			}
			Normal-=Tangent*Normal.Dot(Tangent);
			if (Normal.SquareLength()<=TangentTolerance*TangentTolerance)
			{
				Vector3 Basis=Vector3::Zero(); int Axis=0;
				for (int J=1; J<3; ++J) if (std::abs(Tangent[J])<std::abs(Tangent[Axis])) Axis=J;
				Basis[Axis]=1; Normal=Basis-Tangent*Basis.Dot(Tangent);
			}
			Normal=Normal.Unit(); const Vector3 Binormal=Tangent.Cross(Normal).Unit(); Normal=Binormal.Cross(Tangent).Unit();
			const auto Q=Quaternion(Matrix3(Normal,Binormal,Tangent,false)).Unit(); Validate(Q,N); Frames.push_back(Q);
		}
		return Frames;
	}
	StrandVector StrandElasticity::Log(const Quaternion& Input)
	{
		auto Q=Unit(Input); bool Reverse=Q.w<0;
		// At the pi cut there is no unique differentiable logarithm. Choose the
		// same axis for both quaternion signs, including signed zero in w.
		if (Q.w==0)
		{
			const double Axis[3]={Q.x,Q.y,Q.z}; int Largest=0;
			for (int I=1; I<3; ++I) if (std::abs(Axis[I])>std::abs(Axis[Largest])) Largest=I;
			Reverse=Axis[Largest]<0;
		}
		if (Reverse) Q=Quaternion(-Q.x,-Q.y,-Q.z,-Q.w);
		const double S=std::hypot(std::hypot((double)Q.x,(double)Q.y),(double)Q.z);
		return S>0 ? StrandVector(Q.x,Q.y,Q.z)*(2*std::atan2(S,(double)Q.w)/S) : StrandVector(0);
	}
	StrandResponse StrandElasticity::StretchShear(const Vector3& A, const Vector3& B, const Quaternion& Q, double L, const StrandMaterial& M, const StrandNumerics& N)
	{
		ModelRequire(M.IsValid() && N.IsValid() && std::isfinite(L) && L>=N.MinimumLength,"Invalid strand element"); Validate(Q,N);
		const StrandVector Edge=StrandVector((double)B.x-A.x,(double)B.y-A.y,(double)B.z-A.z)/L; ModelRequire(ModelFinite(Edge),"Invalid strand centerline");
		const auto Inverse=StrandMatrix::Rotation(Q).Transpose(); StrandResponse R;
		R.Value=Inverse*Edge-StrandVector(0,0,1); R.Stiffness=M.StretchShearStiffness(L);
		R.PositionA=Inverse*(-1/L); R.PositionB=Inverse*(1/L); R.RotationA=Inverse*StrandMatrix::Cross(Edge); R.Energy=Energy(R.Value,R.Stiffness); return R;
	}
	StrandResponse StrandElasticity::BendTwist(const Quaternion& A, const Quaternion& B, const Quaternion& Rest, double L, const StrandMaterial& M, const StrandNumerics& N)
	{
		ModelRequire(M.IsValid() && N.IsValid() && std::isfinite(L) && L>=N.MinimumLength,"Invalid strand joint"); Validate(A,N); Validate(B,N); Validate(Rest,N);
		StrandResponse R; R.Value=Log(Unit(Rest).Conjugate()*Unit(A).Conjugate()*Unit(B)); R.Stiffness=M.BendTwistStiffness(L);
		const double Angle=std::hypot(std::hypot(R.Value.x,R.Value.y),R.Value.z), Square=Angle*Angle; double Coefficient;
		if (Angle<N.RotationSeriesAngle) Coefficient=1.0/12+Square/720+Square*Square/30240;
		else Coefficient=(1-Angle*0.5/std::tan(Angle*0.5))/Square;
		const auto Cross=StrandMatrix::Cross(R.Value);
		const auto InverseLeft=StrandMatrix::Identity()+Cross*(-0.5)+(Cross*Cross)*Coefficient;
		R.RotationB=InverseLeft*StrandMatrix::Rotation(Rest).Transpose()*StrandMatrix::Rotation(A).Transpose();
		R.RotationA=R.RotationB*(-1); R.Energy=Energy(R.Value,R.Stiffness); return R;
	}
}

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace Riemann
{
	/* Prepared guide interpolation
	   ----------------------------
	   Each rest curve is parameterized by normalized arc length. Root neighborhoods
	   are fixed at preparation time, within an authored group, with either explicit
	   weights or normalized inverse-distance weights. Point/rotation sample brackets
	   are cached, so evaluation does not search curves or rebuild neighborhoods.

	   A material-frame influence maps p0 to p + R R0^T (p0 - guideRestPoint).
	   Blend these positions and hemisphere-aligned material rotations. Thus rest
	   curves reproduce exactly, a common rigid motion is reproduced exactly, and
	   guide twist rotates radial detail instead of disappearing in tangent-only
	   interpolation. Different guide motions may shrink a blend: this is local
	   linear skinning, not an RBF/global elasticity or collision correction method.
	   At the non-unique half-turn cut, choose a canonical relative axis so quaternion
	   sign changes cannot select the other equally short rotational mean.

	   LODs contain nested guide subsets; every group must retain a representative.
	   Inactive guides use the same prepared interpolation, and runtime root-frame
	   correction preserves each guide's own attachment. Render bindings always use
	   the full guide set, whose inactive states remain continuous during transitions.
	   Representative weights sum the full-guide neighborhood weights. Scaling mass
	   AND elastic energy by a representative's weight preserves its E/rho response.
	   This is a guide-density approximation; heterogeneous shapes/materials should
	   use separately authored groups. LOD changes are not conservative remeshing.

	   The material-frame mapping here is explicit local skinning, independently
	   implemented; no engine asset format or solver dependency is introduced.
	   TODO(asset): Alembic groom import and skeletal/scalp skin weights need authored
	   resources. Existing procedural scene curves exercise preparation and runtime.
	*/
	namespace
	{
		using GroomVector=StrandVector;
		GroomVector GroomDouble(const Vector3& P) { return GroomVector(P.x,P.y,P.z); }
		double Length(const GroomVector& P) { return std::hypot(std::hypot(P.x,P.y),P.z); }
		void GroomRequire(bool Value, const char* Message) { if (!Value) throw std::invalid_argument(Message); }
		Vector3 GroomSingle(const GroomVector& P)
		{
			const double Limit=std::numeric_limits<float>::max();
			GroomRequire(std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z) && std::abs(P.x)<=Limit && std::abs(P.y)<=Limit && std::abs(P.z)<=Limit,"Groom position overflow");
			return Vector3((float)P.x,(float)P.y,(float)P.z);
		}
		std::vector<double> Parameters(const std::vector<Vector3>& Positions)
		{
			std::vector<double> P(Positions.size(),0);
			for (size_t I=1; I<P.size(); ++I) P[I]=P[I-1]+Length(GroomDouble(Positions[I])-GroomDouble(Positions[I-1]));
			GroomRequire(std::isfinite(P.back()) && P.back()>0,"Invalid groom arc length"); const double Total=P.back(); for (auto& S : P) S/=Total; return P;
		}
		std::pair<size_t,double> Bracket(const std::vector<double>& Parameters, double S)
		{
			if (S<=Parameters.front()) return {0,0};
			if (S>=Parameters.back()) return {Parameters.size()-2,1};
			const size_t I=(size_t)(std::upper_bound(Parameters.begin(),Parameters.end(),S)-Parameters.begin())-1;
			return {I,(S-Parameters[I])/(Parameters[I+1]-Parameters[I])};
		}
		Quaternion Blend(const std::vector<std::pair<Quaternion,double>>& Values)
		{
			const auto Anchor=std::max_element(Values.begin(),Values.end(),[](const auto& A,const auto& B) { return A.second<B.second; })->first;
			double X=0,Y=0,Z=0,W=0;
			for (const auto& Entry : Values)
			{
				const auto& Q=Entry.first; const double Dot=(double)Q.x*Anchor.x+(double)Q.y*Anchor.y+(double)Q.z*Anchor.z+(double)Q.w*Anchor.w;
				double Sign=Dot<0 ? -1 : 1;
				if (Dot==0)
				{
					const auto Relative=Anchor.Conjugate()*Q; const double Axis[3]={Relative.x,Relative.y,Relative.z}; int Largest=0;
					for (int I=1; I<3; ++I) if (std::abs(Axis[I])>std::abs(Axis[Largest])) Largest=I;
					if (Axis[Largest]<0) Sign=-1;
				}
				const double Weight=Sign*Entry.second; X+=Q.x*Weight; Y+=Q.y*Weight; Z+=Q.z*Weight; W+=Q.w*Weight;
			}
			const double Norm=std::hypot(std::hypot(X,Y),std::hypot(Z,W)); GroomRequire(std::isfinite(Norm) && Norm>0,"Invalid groom rotation blend");
			return Quaternion((float)(X/Norm),(float)(Y/Norm),(float)(Z/Norm),(float)(W/Norm));
		}
		struct Binding { Vector3 RestPosition; Quaternion RestDirector; double Parameter=0; std::vector<GroomSample> Samples; };
		Transform Evaluate(const Binding& B, const StrandSystem& System)
		{
			GroomVector Position(0); std::vector<std::pair<Quaternion,double>> Rotations; Rotations.reserve(B.Samples.size());
			for (const auto& S : B.Samples)
			{
				const auto& A=System.GetParticle(S.Guide,S.Particle).Position; const auto& C=System.GetParticle(S.Guide,S.Particle+1).Position;
				const GroomVector P=GroomDouble(A)+(GroomDouble(C)-GroomDouble(A))*S.PositionBlend;
				const auto Q=StrandElasticity::Interpolate(System.GetDirector(S.Guide,S.FrameA).Orientation,System.GetDirector(S.Guide,S.FrameB).Orientation,S.FrameBlend);
				const auto Rotation=(Q*S.RestDirector.Conjugate()).Unit();
				Position+=(P+StrandMatrix::Rotation(Rotation)*(GroomDouble(B.RestPosition)-GroomDouble(S.RestPosition)))*S.Weight;
				Rotations.emplace_back((Rotation*B.RestDirector).Unit(),S.Weight);
			}
			return Transform(GroomSingle(Position),Blend(Rotations));
		}
		struct Curve { std::vector<Binding> Points, Frames; };
		void EvaluateCurve(const Curve& Curve, const StrandSystem& System, GroomCurveState& Output)
		{
			Output.Positions.resize(Curve.Points.size()); Output.Directors.resize(Curve.Frames.size());
			for (size_t I=0; I<Curve.Points.size(); ++I) Output.Positions[I]=Evaluate(Curve.Points[I],System).pos;
			for (size_t I=0; I<Curve.Frames.size(); ++I) Output.Directors[I]=Evaluate(Curve.Frames[I],System).quat;
		}
	}
	bool GroomInterpolation::IsValid() const
	{
		return GuidesPerStrand>0 && std::isfinite(RootDistancePower) && RootDistancePower>0 && std::isfinite(CoincidentRootTolerance) && CoincidentRootTolerance>0
			&& std::isfinite(LodHysteresis) && LodHysteresis>=0;
	}
	struct StrandGroom::Storage
	{
		std::vector<StrandGuide> Guides;
		StrandGroomDefinition Definition;
		std::vector<std::vector<double>> Parameters, FrameParameters;
		std::vector<Curve> Render;
		std::vector<std::vector<Curve>> LodCurves;
		std::vector<std::vector<double>> Weights;
		std::vector<GroomInfluence> Neighborhood(const Vector3& Root, uint32_t Group, const std::vector<size_t>& Candidates) const
		{
			std::vector<std::pair<double,size_t>> Distances;
			for (const auto I : Candidates) if (Definition.GuideGroups[I]==Group) Distances.emplace_back(Length(GroomDouble(Root)-GroomDouble(Guides[I].Positions[0])),I);
			GroomRequire(!Distances.empty(),"Groom LOD removes an entire guide group"); std::sort(Distances.begin(),Distances.end());
			Distances.resize(std::min(Distances.size(),Definition.Interpolation.GuidesPerStrand));
			std::vector<GroomInfluence> Result; double Sum=0; const double Minimum=Distances[0].first;
			for (const auto& D : Distances)
			{
				const double Weight=Minimum<=Definition.Interpolation.CoincidentRootTolerance ? (D.first<=Definition.Interpolation.CoincidentRootTolerance ? 1 : 0) : std::pow(Minimum/D.first,Definition.Interpolation.RootDistancePower);
				if (Weight>0) { Result.push_back({D.second,Weight}); Sum+=Weight; }
			}
			for (auto& R : Result) R.Weight/=Sum; return Result;
		}
		Binding Bind(const Vector3& Position, const Quaternion& Frame, double S, const std::vector<GroomInfluence>& Influences) const
		{
			Binding B; B.RestPosition=Position; B.RestDirector=Frame; B.Parameter=S;
			for (const auto& Influence : Influences)
			{
				const auto G=Influence.Guide; const auto P=Bracket(Parameters[G],S); GroomSample Sample; Sample.Guide=G; Sample.Weight=Influence.Weight;
				Sample.Particle=P.first; Sample.PositionBlend=P.second;
				Sample.RestPosition=GroomSingle(GroomDouble(Guides[G].Positions[P.first])+(GroomDouble(Guides[G].Positions[P.first+1])-GroomDouble(Guides[G].Positions[P.first]))*P.second);
				if (Guides[G].Directors.size()==1) Sample.FrameA=Sample.FrameB=0;
				else { const auto F=Bracket(FrameParameters[G],S); Sample.FrameA=F.first; Sample.FrameB=F.first+1; Sample.FrameBlend=F.second; }
				Sample.RestDirector=StrandElasticity::Interpolate(Guides[G].Directors[Sample.FrameA],Guides[G].Directors[Sample.FrameB],Sample.FrameBlend); B.Samples.push_back(Sample);
			}
			return B;
		}
		Curve Prepare(const std::vector<Vector3>& Positions, const std::vector<Quaternion>& Frames, const std::vector<GroomInfluence>& Influences) const
		{
			Curve Result; const auto P=Riemann::Parameters(Positions);
			for (size_t I=0; I<Positions.size(); ++I)
				Result.Points.push_back(Bind(Positions[I],Frames[std::min(I,Frames.size()-1)],P[I],Influences));
			for (size_t I=0; I<Frames.size(); ++I)
				Result.Frames.push_back(Bind(GroomSingle((GroomDouble(Positions[I])+GroomDouble(Positions[I+1]))*0.5),Frames[I],I ? (P[I]+P[I+1])*0.5 : 0,Influences));
			return Result;
		}
	};
	StrandGroom::StrandGroom(const std::vector<StrandGuide>& Input, const StrandGroomDefinition& Definition, const StrandSettings& Settings)
	{
		GroomRequire(!Input.empty() && Settings.IsValid() && Definition.Interpolation.IsValid(),"Invalid groom definition"); auto Data=std::make_shared<Storage>(); Data->Guides=Input; Data->Definition=Definition;
		auto& D=Data->Definition; if (D.GuideGroups.empty()) D.GuideGroups.assign(Input.size(),0); GroomRequire(D.GuideGroups.size()==Input.size(),"Invalid groom guide groups");
		std::vector<size_t> All(Input.size()); std::iota(All.begin(),All.end(),0);
		for (auto& G : Data->Guides)
		{
			G.Directors=StrandElasticity::PrepareDirectors(G.Positions,G.Directors,G.ReferenceNormal,Settings.RestTangentTolerance,Settings.Numerics);
			Data->Parameters.push_back(Parameters(G.Positions)); std::vector<double> Centers;
			for (size_t I=0; I<G.Directors.size(); ++I) Centers.push_back((Data->Parameters.back()[I]+Data->Parameters.back()[I+1])*0.5);
			Data->FrameParameters.push_back(std::move(Centers));
		}
		for (auto& Strand : D.Strands)
		{
			GroomRequire(std::isfinite(Strand.RadiusX) && Strand.RadiusX>0 && std::isfinite(Strand.RadiusY) && Strand.RadiusY>0,"Invalid groom strand radius");
			Strand.Directors=StrandElasticity::PrepareDirectors(Strand.Positions,Strand.Directors,Strand.ReferenceNormal,Settings.RestTangentTolerance,Settings.Numerics);
			auto Influences=Strand.Influences.empty() ? Data->Neighborhood(Strand.Positions[0],Strand.Group,All) : Strand.Influences;
			double Sum=0; std::vector<size_t> Used;
			for (const auto& I : Influences)
			{
				GroomRequire(I.Guide<Input.size() && D.GuideGroups[I.Guide]==Strand.Group && std::isfinite(I.Weight) && I.Weight>0 && std::find(Used.begin(),Used.end(),I.Guide)==Used.end(),"Invalid groom influence"); Used.push_back(I.Guide); Sum+=I.Weight;
			}
			GroomRequire(std::isfinite(Sum) && Sum>0,"Invalid groom influence sum"); for (auto& I : Influences) I.Weight/=Sum; Strand.Influences=Influences;
			Data->Render.push_back(Data->Prepare(Strand.Positions,Strand.Directors,Influences));
		}
		if (D.Lods.empty()) { GroomLod L; L.Guides=All; D.Lods.push_back(L); }
		std::vector<size_t> Previous=All; double PreviousDistance=-1;
		for (size_t Level=0; Level<D.Lods.size(); ++Level)
		{
			auto& L=D.Lods[Level]; std::sort(L.Guides.begin(),L.Guides.end());
			GroomRequire(!L.Guides.empty() && std::adjacent_find(L.Guides.begin(),L.Guides.end())==L.Guides.end() && std::includes(Previous.begin(),Previous.end(),L.Guides.begin(),L.Guides.end())
				&& (Level>0 || L.Guides==All) && std::isfinite(L.TransitionTime) && L.TransitionTime>=0 && std::isfinite(L.Distance) && L.Distance>=0 && L.Distance>PreviousDistance,"Invalid groom LOD hierarchy");
			std::vector<Curve> Curves(Input.size()); std::vector<double> Weights(Input.size(),0);
			for (size_t I=0; I<Input.size(); ++I)
			{
				const bool Active=std::binary_search(L.Guides.begin(),L.Guides.end(),I);
				const auto Influences=Active ? std::vector<GroomInfluence>{{I,1}} : Data->Neighborhood(Input[I].Positions[0],D.GuideGroups[I],L.Guides);
				for (const auto& Influence : Influences) Weights[Influence.Guide]+=Influence.Weight;
				Curves[I]=Data->Prepare(Data->Guides[I].Positions,Data->Guides[I].Directors,Influences);
			}
			Data->LodCurves.push_back(std::move(Curves)); Data->Weights.push_back(std::move(Weights)); Previous=L.Guides; PreviousDistance=L.Distance;
		}
		m_Data=std::move(Data);
	}
	StrandGroom::~StrandGroom() = default;
	size_t StrandGroom::GetStrandCount() const { return m_Data->Render.size(); }
	size_t StrandGroom::GetLodCount() const { return m_Data->Definition.Lods.size(); }
	const GroomStrand& StrandGroom::GetStrand(size_t I) const { return m_Data->Definition.Strands.at(I); }
	double StrandGroom::GetStrandParameter(size_t S, size_t P) const { return m_Data->Render.at(S).Points.at(P).Parameter; }
	const GroomLod& StrandGroom::GetLod(size_t I) const { return m_Data->Definition.Lods.at(I); }
	const GroomInterpolation& StrandGroom::GetInterpolation() const { return m_Data->Definition.Interpolation; }
	bool StrandGroom::IsSimulated(size_t L, size_t G) const { return m_Data->Weights.at(L).at(G)>0; }
	double StrandGroom::GetSimulationWeight(size_t L, size_t G) const { return m_Data->Weights.at(L).at(G); }
	const std::vector<GroomSample>& StrandGroom::GetGuideSamples(size_t L, size_t G, size_t I, bool Director) const
	{ const auto& C=m_Data->LodCurves.at(L).at(G); return (Director ? C.Frames : C.Points).at(I).Samples; }
	void StrandGroom::EvaluateGuide(size_t L, size_t G, const StrandSystem& System, GroomCurveState& Output) const { EvaluateCurve(m_Data->LodCurves.at(L).at(G),System,Output); }
	void StrandGroom::EvaluateStrand(size_t S, const StrandSystem& System, GroomCurveState& Output) const { EvaluateCurve(m_Data->Render.at(S),System,Output); }
}

#include "../Collision/SurfaceCollision.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>

namespace Riemann
{
	/* Segment contact pipeline
	   ------------------------
	   Physical guides are chains of circular collision envelopes. Elliptical
	   material sections use their circumscribed radius; render followers remain
	   guide-driven approximations. Explicit collider registration is a whitelist.
	   Rigid snapshots are prescribed obstacles, as in the cloth collision route.

	   The shared dynamic edge BVH selects segment pairs and segment/triangle
	   candidates. Closest witnesses include segment interiors and both endpoints,
	   avoiding particle-only gaps and a single-witness parallel contact interval.
	   Same-guide material neighbors are excluded by rest arc length. Different
	   guides receive equal/opposite mass-weighted corrections, once per pair.

	   SurfaceCollision supplies discrete or swept witnesses; SurfaceContactConstraint
	   supplies unilateral XPBD and Coulomb-disk friction. Rows retain multipliers
	   only within a physical substep and refresh geometry/reference slip each
	   iteration. The reference begins at impact, including a rounded obstacle's
	   rotational surface motion. Unseen finite features retire instead of becoming
	   infinite planes. This centerline model has no rolling/spinning friction DOF.

	   Closed convexes recover initially embedded segments with a common outward
	   plane and an interior barycentric witness. A valid incoming CCD path uses
	   its entry face instead. A strictly embedded start may exit freely. Pinned
	   witnesses cannot respond; they are reported and excluded from feasibility.
	   RootExclusionSegments can omit authored scalp attachment neighborhoods.

	   Continuous barriers are hard. Final separation passes resweep corrected paths
	   and check both constraint and endpoint geometric gaps. Uncertain sweeps or
	   exhausted projection request a smaller physical interval from StrandSystem,
	   which restores the attempted state before retrying. No distant conservative
	   result is projected as penetration. Statistics include rejected attempts.
	   These are iterated finite-feature impact planes, not a rigid impact-zone solve.
	   The shared kernels document XPBD and Bridson et al.'s contact formulation.
	*/
	namespace
	{
		using CollisionVector=StrandVector;
		CollisionVector CollisionDouble(const Vector3& P) { return CollisionVector(P.x,P.y,P.z); }
		void CollisionRequire(bool Value,const char* Message) { if (!Value) throw std::invalid_argument(Message); }
		float CollisionSingle(double X) { CollisionRequire(std::isfinite(X) && X>=0 && X<=std::numeric_limits<float>::max(),"Strand collision bound overflow"); return (float)X; }
		Vector3 Weighted(const SurfaceMotion& M,const Vector3& Weights,double Time)
		{
			CollisionVector P(0); for (int I=0; I<M.Count; ++I) P+=(CollisionDouble(M.Start[I])+(CollisionDouble(M.End[I])-CollisionDouble(M.Start[I]))*Time)*(double)Weights[I];
			return Vector3((float)P.x,(float)P.y,(float)P.z);
		}
		struct Feature { int Count=1, Index[3]={0,0,0}; };
		Feature Point(int I) { Feature F; F.Index[0]=I; return F; }
		Feature Edge(int A,int B) { Feature F; F.Count=2; F.Index[0]=A; F.Index[1]=B; return F; }
		Feature Triangle(const Index3& T) { Feature F; F.Count=3; for (int I=0; I<3; ++I) F.Index[I]=T[I]; return F; }
		using Key=std::tuple<int,size_t,size_t,int,int>;
		struct RootTerm { CollisionVector* Impulse=nullptr; double Weight=0; };
		struct Row
		{
			Vector3 Fixed=Vector3::Zero(), Normal=Vector3::Zero();
			std::unique_ptr<SurfaceContactConstraint> Constraint;
			std::vector<RootTerm> Roots;
			double Gap=0;
			bool Seen=false;
			void Solve(double Dt,bool Separation)
			{
				const double Before=Constraint->GetNormalMultiplier(); const auto Tangent=Constraint->GetTangentMultiplier();
				if (Separation) Constraint->ProjectSeparation(); else Constraint->Solve(Dt);
				const CollisionVector Delta=CollisionDouble(Normal)*(Constraint->GetNormalMultiplier()-Before)+Constraint->GetTangentMultiplier()-Tangent;
				for (const auto& R : Roots) *R.Impulse+=Delta*(R.Weight/Dt);
			}
		};
	}
	bool StrandCollisionSettings::IsValid() const
	{
		return std::isfinite(RadiusScale) && RadiusScale>0 && std::isfinite(Margin) && Margin>=0 && std::isfinite(ContactTolerance) && ContactTolerance>0
			&& std::isfinite(MaximumColliderAngle) && MaximumColliderAngle>0 && MaximumColliderAngle<=std::acos(-1.0) && RootExclusionSegments>=0
			&& SweepIterations>0 && SweepRefinements>=0 && SeparationIterations>0 && MaximumRefinements>=0 && MaximumRefinementDepth>0 && Material.IsValid();
	}
	void StrandCollisionStats::Accumulate(const StrandCollisionStats& S)
	{
		Candidates+=S.Candidates; Contacts+=S.Contacts; RigidContacts+=S.RigidContacts; SelfContacts+=S.SelfContacts; InterGuideContacts+=S.InterGuideContacts;
		ContinuousContacts+=S.ContinuousContacts; PinnedContacts+=S.PinnedContacts; UnresolvedSweeps+=S.UnresolvedSweeps; Substeps+=S.Substeps; Refinements+=S.Refinements;
		MaximumPenetration=std::max(MaximumPenetration,S.MaximumPenetration); MaximumConstraintViolation=std::max(MaximumConstraintViolation,S.MaximumConstraintViolation); ProjectionLimitReached|=S.ProjectionLimitReached;
	}
	struct StrandCollision::Storage
	{
		StrandCollisionSettings Settings;
		StrandCollisionStats Stats;
		std::vector<std::unique_ptr<SurfaceCollider>> Colliders;
		std::vector<StrandContactGuide> Guides;
		struct Segment { size_t Guide=0; int Local=0,A=0,B=0; double Radius=0; };
		std::vector<Segment> Segments;
		std::vector<std::pair<size_t,int>> Nodes;
		std::vector<Vector3> Start,Positions;
		std::unique_ptr<SurfaceCollisionMesh> Index;
		std::map<Key,std::unique_ptr<Row>> Rows;
		double Radius(size_t G) const { return Guides[G].Radius*Settings.RadiusScale+Settings.Margin; }
		SurfaceMotion Motion(size_t G,const Feature& F) const
		{
			SurfaceMotion M; M.Count=F.Count;
			for (int I=0; I<F.Count; ++I) { M.Start[I]=Guides[G].Start[F.Index[I]]; M.End[I]=*Guides[G].Particles[F.Index[I]].Position; } return M;
		}
		SurfaceMotion Motion(const SurfaceCollider& C,const Feature& F) const
		{
			SurfaceMotion M; M.Count=F.Count; for (int I=0; I<F.Count; ++I) { M.Start[I]=C.Start[F.Index[I]]; M.End[I]=C.End[F.Index[I]]; } return M;
		}
		bool Query(const SurfaceMotion& A,const SurfaceMotion& B,double Thickness,bool Continuous,SurfaceContact& Contact)
		{
			++Stats.Candidates; SurfaceCollisionSettings Q; Q.Thickness=Thickness; Q.DistanceTolerance=Settings.ContactTolerance; Q.RelativeTolerance=0; Q.MaxIterations=Settings.SweepIterations;
			if (Continuous)
			{
				const auto Old=SurfaceCollision::Closest(A.Start,A.Count,B.Start,B.Count);
				if (!SurfaceCollision::SweepRefined(A,B,Q,Settings.SweepRefinements,Contact,Old.Normal)) return false;
				if (Contact.Conservative) { ++Stats.UnresolvedSweeps; throw RefinementNeeded(); }
				if (Contact.Time==0)
				{
					const auto Normal=Contact.Normal; Contact=SurfaceCollision::Closest(A.End,A.Count,B.End,B.Count);
					if (Contact.Normal.IsZero()) Contact.Normal=Normal; else if (Contact.Normal.Dot(Normal)<0) Contact.Normal=-Contact.Normal;
					Contact.Time=0; Contact.InitialOverlap=true;
				}
				return true;
			}
			Contact=SurfaceCollision::Closest(A.End,A.Count,B.End,B.Count); if (Contact.Distance>Thickness+Settings.ContactTolerance) return false;
			if (Contact.Normal.IsZero())
			{
				SurfaceMotion X=A,Y=B; for (int I=0; I<A.Count; ++I) X.Start[I]=X.End[I]; for (int I=0; I<B.Count; ++I) Y.Start[I]=Y.End[I];
				const auto Old=SurfaceCollision::Closest(A.Start,A.Count,B.Start,B.Count); if (!SurfaceCollision::Sweep(X,Y,Q,Contact,Old.Normal)) return false;
			}
			return true;
		}
		void Roots(Row& R,size_t G,const Feature& F,const Vector3& W,double Sign)
		{ for (int I=0; I<F.Count; ++I) if (F.Index[I]==0 && Guides[G].RootImpulse) R.Roots.push_back({Guides[G].RootImpulse,Sign*W[I]}); }
		void Store(const Key& K,const SurfaceContactParticle* A,int CountA,const SurfaceContactParticle* B,int CountB,SurfaceContact Contact,double Thickness,double Gap,bool Continuous,
			size_t GuideA,const Feature& FA,const size_t* GuideB=nullptr,const Feature* FB=nullptr)
		{
			double Inverse=0; for (int I=0; I<CountA; ++I) Inverse+=A[I].InverseMass*Contact.WeightsA[I]*Contact.WeightsA[I];
			for (int I=0; I<CountB; ++I) Inverse+=B[I].InverseMass*Contact.WeightsB[I]*Contact.WeightsB[I];
			if (!(Inverse>0)) { ++Stats.PinnedContacts; return; }
			auto& Entry=Rows[K]; const bool New=!Entry || !Entry->Constraint; if (!Entry) Entry.reset(new Row); auto& R=*Entry; R.Seen=true; R.Gap=Gap; R.Normal=Contact.Normal; R.Roots.clear();
			auto Material=Settings.Material; if (Settings.Continuous) Material.Compliance=0;
			if (R.Constraint) R.Constraint->Refresh(A,CountA,B,CountB,Contact,Thickness); else R.Constraint.reset(new SurfaceContactConstraint(A,CountA,B,CountB,Contact,Thickness,Material));
			Roots(R,GuideA,FA,Contact.WeightsA,1); if (GuideB) Roots(R,*GuideB,*FB,Contact.WeightsB,-1);
			if (New) { ++Stats.Contacts; if (Continuous) ++Stats.ContinuousContacts; if (GuideB) { if (GuideA==*GuideB) ++Stats.SelfContacts; else ++Stats.InterGuideContacts; } else ++Stats.RigidContacts; }
		}
		void Pair(const Key& K,size_t GA,const Feature& A,size_t GB,const Feature& B)
		{
			const auto MA=Motion(GA,A), MB=Motion(GB,B); SurfaceContact Contact; const double Thickness=Radius(GA)+Radius(GB);
			if (!Query(MA,MB,Thickness,Settings.Continuous,Contact)) return;
			const auto End=SurfaceCollision::Closest(MA.End,MA.Count,MB.End,MB.Count); const double Time=Settings.Continuous ? Contact.Time : 0;
			Contact.PointA=Weighted(MA,Contact.WeightsA,Time); Contact.PointB=Weighted(MB,Contact.WeightsB,Time);
			SurfaceContactParticle PA[3],PB[3]; for (int I=0; I<A.Count; ++I) PA[I]=Guides[GA].Particles[A.Index[I]]; for (int I=0; I<B.Count; ++I) PB[I]=Guides[GB].Particles[B.Index[I]];
			Store(K,PA,A.Count,PB,B.Count,Contact,Thickness,End.Distance-Thickness,Settings.Continuous,GA,A,&GB,&B);
		}
		void Rigid(const Key& K,size_t G,const Feature& A,size_t CIndex,const Feature& B,bool Continuous)
		{
			auto& C=*Colliders[CIndex]; const auto MA=Motion(G,A), MB=Motion(C,B); SurfaceContact Contact; const double Thickness=Radius(G)+C.Radius+C.ArcBound;
			if (!Query(MA,MB,Thickness,Continuous,Contact)) return; const auto Endpoint=SurfaceCollision::Closest(MA.End,MA.Count,MB.End,MB.Count);
			const float Time=Continuous ? (float)Contact.Time : 0;
			const Transform Reference(C.StepStart.pos+(C.StepEnd.pos-C.StepStart.pos)*Time,Quaternion::Slerp(C.StepStart.quat,C.StepEnd.quat,Time));
			const auto RefNormal=Continuous ? Contact.Normal : C.StepStart.quat*(C.StepEnd.quat.Conjugate()*Contact.Normal);
			const auto EndNormal=Continuous ? C.StepEnd.quat*(Reference.quat.Conjugate()*Contact.Normal) : Contact.Normal;
			Contact.PointA=Weighted(MA,Contact.WeightsA,Time)+EndNormal*C.Radius; Contact.PointB=Weighted(MB,Contact.WeightsB,Time)+RefNormal*C.Radius;
			SurfaceContactParticle PA[3],PB[3]; for (int I=0; I<A.Count; ++I) PA[I]=Guides[G].Particles[A.Index[I]]; for (int I=0; I<B.Count; ++I) PB[I]={&C.End[B.Index[I]],0};
			Store(K,PA,A.Count,PB,B.Count,Contact,Thickness,Endpoint.Distance-Thickness,Continuous,G,A);
		}
		bool Clip(const SurfaceCollider& C,const SurfaceMotion& Motion,bool AtStart,double Margin,double& Parameter) const
		{
			const auto& Pose=AtStart ? C.StepStart : C.StepEnd; const auto A=CollisionDouble(Pose.WorldToLocal(AtStart ? Motion.Start[0] : Motion.End[0]));
			const auto B=CollisionDouble(Pose.WorldToLocal(AtStart ? Motion.Start[1] : Motion.End[1])); double Begin=0,End=1;
			for (const auto& Plane : C.Planes)
			{
				const double DA=CollisionDouble(Plane.Normal).Dot(A-CollisionDouble(Plane.Point))-Margin, DB=CollisionDouble(Plane.Normal).Dot(B-CollisionDouble(Plane.Point))-Margin;
				if (DA>0 && DB>0) return false; if (DA<=0 && DB<=0) continue;
				const double T=DA/(DA-DB); if (DA>0) Begin=std::max(Begin,T); else End=std::min(End,T); if (Begin>End) return false;
			}
			Parameter=(Begin+End)*0.5; return true;
		}
		bool Interior(size_t SegmentIndex,size_t CIndex,bool& DiscreteExit)
		{
			const auto& S=Segments[SegmentIndex]; auto& C=*Colliders[CIndex]; if (C.Planes.empty()) return false;
			const auto A=Edge(S.Local,S.Local+1); const auto MotionA=Motion(S.Guide,A); double StartT=0,EndT=0;
			const bool StartedInside=Clip(C,MotionA,true,-Settings.ContactTolerance,StartT);
			DiscreteExit=Settings.Continuous && StartedInside;
			if (Settings.Continuous && !StartedInside) return false;
			if (!Clip(C,MotionA,false,0,EndT)) return false;
			const auto LA=CollisionDouble(C.StepEnd.WorldToLocal(MotionA.End[0])), LB=CollisionDouble(C.StepEnd.WorldToLocal(MotionA.End[1]));
			double Best=-std::numeric_limits<double>::infinity(); size_t Face=0;
			for (size_t I=0; I<C.Planes.size(); ++I)
			{
				const auto& P=C.Planes[I]; const double Distance=std::min(CollisionDouble(P.Normal).Dot(LA-CollisionDouble(P.Point)),CollisionDouble(P.Normal).Dot(LB-CollisionDouble(P.Point)));
				if (Distance>Best) { Best=Distance; Face=I; }
			}
			const auto& Plane=C.Planes[Face]; const auto P=LA*(1-EndT)+LB*EndT; const double Distance=CollisionDouble(Plane.Normal).Dot(P-CollisionDouble(Plane.Point));
			const auto Projected=P-CollisionDouble(Plane.Normal)*Distance; const Vector3 Local((float)Projected.x,(float)Projected.y,(float)Projected.z);
			const Key K(2,SegmentIndex,CIndex,0,0); auto& Entry=Rows[K]; if (!Entry) Entry.reset(new Row); auto& R=*Entry;
			R.Fixed=C.StepEnd.LocalToWorld(Local); SurfaceContact Contact; Contact.Normal=C.StepEnd.quat*Plane.Normal; Contact.WeightsA=Vector3((float)(1-EndT),(float)EndT,0); Contact.WeightsB=Vector3::UnitX();
			Contact.PointA=Weighted(MotionA,Contact.WeightsA,0); Contact.PointB=C.StepStart.LocalToWorld(Local);
			const SurfaceContactParticle PA[2]={Guides[S.Guide].Particles[S.Local],Guides[S.Guide].Particles[S.Local+1]}, PB={&R.Fixed,0};
			Store(K,PA,2,&PB,1,Contact,S.Radius+C.ArcBound,Distance-S.Radius-C.ArcBound,Settings.Continuous,S.Guide,A); return true;
		}
		void Discover()
		{
			for (auto& R : Rows) R.second->Seen=false;
			for (size_t I=0; I<Nodes.size(); ++I) Positions[I]=*Guides[Nodes[I].first].Particles[Nodes[I].second].Position;
			double MaximumRadius=0; for (size_t G=0; G<Guides.size(); ++G) MaximumRadius=std::max(MaximumRadius,Radius(G));
			Index->Update(Settings.Continuous ? Start : Positions,Positions,CollisionSingle(MaximumRadius+Settings.ContactTolerance));
			std::vector<int> Hits;
			for (size_t CI=0; CI<Colliders.size(); ++CI)
			{
				auto& C=*Colliders[CI];
				if (C.Radius>0)
				{
					Box3 Bounds(C.End.data(),C.End.size()); if (Settings.Continuous) for (const auto& P : C.Start) Bounds.Encapsulate(P);
					const Vector3 Margin(CollisionSingle(C.Radius+C.ArcBound)); Bounds.Min-=Margin; Bounds.Max+=Margin; Index->QueryEdges(Bounds,Hits);
					for (const int SI : Hits)
					{
						const auto& S=Segments[SI]; const auto A=Edge(S.Local,S.Local+1),B=C.End.size()==1 ? Point(0) : Edge(0,1);
						Rigid(Key(0,SI,CI,0,0),S.Guide,A,CI,B,Settings.Continuous);
						for (int E=0; E<2; ++E) Rigid(Key(0,SI,CI,1+E,0),S.Guide,Point(S.Local+E),CI,B,Settings.Continuous);
						if (B.Count==2) for (int E=0; E<2; ++E) Rigid(Key(0,SI,CI,3+E,0),S.Guide,A,CI,Point(E),Settings.Continuous);
					}
				}
				else for (size_t SI=0; SI<Segments.size(); ++SI)
				{
					bool Exit=false; if (Interior(SI,CI,Exit)) continue; const auto& S=Segments[SI]; const auto A=Edge(S.Local,S.Local+1);
					Box3 Bounds(Settings.Continuous && !Exit ? Start[S.A] : Positions[S.A],Positions[S.A]); Bounds.Encapsulate(Positions[S.B]); if (Settings.Continuous && !Exit) Bounds.Encapsulate(Start[S.B]);
					const Vector3 Margin(CollisionSingle(S.Radius+Settings.ContactTolerance)); Bounds.Min-=Margin; Bounds.Max+=Margin; C.Index->QueryTriangles(Bounds,Hits);
					for (const int T : Hits)
					{
						const auto B=Triangle(C.Triangles[T]); Rigid(Key(1,SI,CI,T,0),S.Guide,A,CI,B,Settings.Continuous && !Exit);
						for (int E=0; E<2; ++E) Rigid(Key(1,SI,CI,T,1+E),S.Guide,Point(S.Local+E),CI,B,Settings.Continuous && !Exit);
					}
				}
			}
			if (Settings.SelfCollision || Settings.InterGuideCollision)
			{
				std::vector<SurfaceCandidate> Candidates; Index->FindSelfCandidates(Candidates,false);
				for (const auto& C : Candidates)
				{
					const auto& A=Segments[C.A]; const auto& B=Segments[C.B]; const bool Self=A.Guide==B.Guide;
					if ((Self && !Settings.SelfCollision) || (!Self && !Settings.InterGuideCollision)) continue;
					if (Self)
					{
						const auto& Arc=Guides[A.Guide].RestArc; const double Gap=Arc[std::max(A.Local,B.Local)]-Arc[std::min(A.Local,B.Local)+1];
						if (Gap<=A.Radius+B.Radius+Settings.ContactTolerance) continue;
					}
					Pair(Key(3,C.A,C.B,0,0),A.Guide,Edge(A.Local,A.Local+1),B.Guide,Edge(B.Local,B.Local+1));
					for (int E=0; E<2; ++E)
					{
						Pair(Key(3,C.A,C.B,1+E,0),A.Guide,Point(A.Local+E),B.Guide,Edge(B.Local,B.Local+1));
						Pair(Key(3,C.A,C.B,3+E,0),A.Guide,Edge(A.Local,A.Local+1),B.Guide,Point(B.Local+E));
					}
				}
			}
			for (auto It=Rows.begin(); It!=Rows.end();) if (!It->second->Seen) It=Rows.erase(It); else ++It;
		}
	};
	StrandCollision::StrandCollision(const StrandCollisionSettings& Settings) : m_Data(new Storage) { SetSettings(Settings); }
	StrandCollision::~StrandCollision() = default;
	void StrandCollision::SetSettings(const StrandCollisionSettings& Settings) { CollisionRequire(Settings.IsValid(),"Invalid strand collision settings"); m_Data->Rows.clear(); m_Data->Settings=Settings; }
	bool StrandCollision::AddCollider(Geometry* Shape)
	{ std::unique_ptr<SurfaceCollider> C(new SurfaceCollider); if (!C->Initialize(Shape)) return false; RemoveCollider(Shape); m_Data->Colliders.push_back(std::move(C)); return true; }
	bool StrandCollision::AddCapsule(RigidBody* Body,float Radius,const Vector3& A,const Vector3& B)
	{ std::unique_ptr<SurfaceCollider> C(new SurfaceCollider); if (!C->InitializeCapsule(Body,Radius,A,B)) return false; m_Data->Rows.clear(); m_Data->Colliders.push_back(std::move(C)); return true; }
	bool StrandCollision::RemoveCollider(const Geometry* Shape)
	{ if (!Shape) return false; auto& C=m_Data->Colliders; const auto N=C.size(); m_Data->Rows.clear(); C.erase(std::remove_if(C.begin(),C.end(),[&](const auto& P) { return P->Source==Shape; }),C.end()); return C.size()!=N; }
	void StrandCollision::ClearColliders() { m_Data->Rows.clear(); m_Data->Colliders.clear(); }
	void StrandCollision::RemoveBody(RigidBody* Body)
	{ if (!Body) return; auto& C=m_Data->Colliders; m_Data->Rows.clear(); C.erase(std::remove_if(C.begin(),C.end(),[&](const auto& P) { return P->Body==Body; }),C.end()); }
	size_t StrandCollision::GetColliderCount() const { return m_Data->Colliders.size(); }
	const StrandCollisionStats& StrandCollision::GetStats() const { return m_Data->Stats; }
	void StrandCollision::BeginFrame() { m_Data->Stats={}; for (auto& C : m_Data->Colliders) C->CaptureFrame(); }
	int StrandCollision::RequiredSubsteps(int Maximum) const
	{
		double Count=1; if (m_Data->Settings.Enabled && m_Data->Settings.Continuous) for (const auto& C : m_Data->Colliders) Count=std::max(Count,std::ceil(C->FrameAngle()/m_Data->Settings.MaximumColliderAngle));
		CollisionRequire(Count<=Maximum,"Strand collider motion exceeds its substep budget"); return (int)Count;
	}
	void StrandCollision::BeginSubstep(float Begin,float End,const std::vector<StrandContactGuide>& Guides)
	{
		auto& D=*m_Data; D.Rows.clear(); D.Guides=Guides; D.Nodes.clear(); D.Start.clear(); D.Positions.clear(); D.Segments.clear(); std::vector<SurfaceEdge> Edges;
		for (size_t G=0; G<Guides.size(); ++G)
		{
			const auto& View=Guides[G]; CollisionRequire(View.Particles.size()>=2 && View.Start.size()==View.Particles.size() && View.RestArc.size()==View.Particles.size() && std::isfinite(View.Radius) && View.Radius>0,"Invalid strand collision guide view");
			CollisionRequire(D.Nodes.size()+View.Particles.size()<=(size_t)std::numeric_limits<int>::max(),"Strand collision index overflow"); const int Offset=(int)D.Nodes.size();
			for (size_t I=0; I<View.Particles.size(); ++I)
			{ CollisionRequire(View.Particles[I].Position!=nullptr && std::isfinite(View.Particles[I].InverseMass) && View.Particles[I].InverseMass>=0,"Invalid strand collision particle"); D.Nodes.emplace_back(G,(int)I); D.Start.push_back(View.Start[I]); D.Positions.push_back(*View.Particles[I].Position); }
			for (size_t I=View.Rooted ? D.Settings.RootExclusionSegments : 0; I+1<View.Particles.size(); ++I) Edges.push_back({Offset+(int)I,Offset+(int)I+1});
		}
		CollisionRequire(!D.Nodes.empty(),"Strand contacts require guides"); D.Index.reset(new SurfaceCollisionMesh((int)D.Nodes.size(),{},Edges));
		for (const auto& E : D.Index->GetEdges()) { const auto& P=D.Nodes[E.A]; D.Segments.push_back({P.first,P.second,E.A,E.B,D.Radius(P.first)}); }
		for (auto& C : D.Colliders) C->Update(Begin,End,D.Settings.Continuous); ++D.Stats.Substeps;
	}
	void StrandCollision::Solve(double Dt,bool SeparationOnly) { m_Data->Discover(); for (auto& R : m_Data->Rows) R.second->Solve(Dt,SeparationOnly); }
	bool StrandCollision::Finish(double Dt)
	{
		auto& D=*m_Data;
		auto Valid=[&]()
		{
			D.Discover(); double Gap=0,Violation=0;
			for (const auto& R : D.Rows) { Gap=std::max(Gap,-R.second->Gap); Violation=std::max(Violation,-R.second->Constraint->GetNormalGap()); }
			D.Stats.MaximumPenetration=Gap; D.Stats.MaximumConstraintViolation=Violation; return std::max(Gap,Violation)<=D.Settings.ContactTolerance;
		};
		if (!D.Settings.Continuous) { Valid(); return true; }
		for (int I=0; I<D.Settings.SeparationIterations; ++I) { if (Valid()) return true; Solve(Dt,true); }
		if (Valid()) return true; D.Stats.ProjectionLimitReached=true; return false;
	}
	void StrandCollision::EndSubstep() { m_Data->Rows.clear(); m_Data->Guides.clear(); m_Data->Index.reset(); }
	void StrandCollision::RecordRefinement() { ++m_Data->Stats.Refinements; }
	void StrandCollision::EndFrame(bool Commit) { EndSubstep(); if (Commit) for (auto& C : m_Data->Colliders) C->CommitFrame(); }
}

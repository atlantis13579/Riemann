#include "PhysicsField.h"
#include "PhysicsWorld.h"
#include "Ragdoll.h"
#include "../Collision/GeometryObject.h"
#include "../Destruction/DestructionSet.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace Riemann
{
	/* Composable fields and world commands
	   -----------------------------------
	   Scalar/vector expressions are immutable DAGs, independent of body lifetime.
	   Composition builds spatial masks, unions/intersections, falloffs and vector
	   superposition. Rotated boxes evaluate in their own frames; radial directions
	   are zero at the center. Smooth falloff is cubic Hermite interpolation with
	   zero endpoint slopes. Wave frequency is cycles per simulated second.
	   Commands compile shared subexpressions into a bounded topological program;
	   each node runs once per sample, even for a repeatedly shared diamond DAG.

	   Masks are clamped to [0,1]. Forces, velocities and external strain use the mask
	   as a gain. Activation/material commands use it as a selection gate; compose
	   each material coefficient explicitly to obtain spatially varying properties.
	   Material fields include static and kinematic surfaces; motion/activation
	   commands address dynamic bodies. Samples use each body's COM/reference origin.
	   Bodies are sampled at their COM and selected by any attached shape's layer,
	   independently of query visibility. Strain samples the current bond material
	   point, not the original authoring position or an entire cluster's COM.

	   Boundary commands run before substep body/contact references are borrowed.
	   Topology commands precede body commands, so new fragments receive the same
	   boundary's activation, velocity and material fields. Registration order is
	   preserved within each phase.
	   Physical forces run after legacy input conversion, before velocity integration.
	   A transient force acts for one OUTER frame, across all temporal substeps; other
	   transient commands execute once. Persistent impulses are rejected: repeating
	   a fixed impulse each substep would change the result with the substep count.
	   External strain is an instantaneous bond demand, not accumulated time damage.
	   Material commands set absolute coefficients. Removing a field does not undo
	   already applied state. Direct Cartesian velocity assignment to reduced ragdoll
	   links is rejected; forces and impulses use generalized response instead.
	   Freeze changes effective solver mass and skips rigid integration without
	   overwriting physical mass/inertia; unfreeze needs no pointer-keyed restore map.

	   Registration is between steps. Each command stages and validates its selected
	   outputs before mutation. This is not a transaction across all commands or a
	   checkpoint rollback of a failed physics frame. Consumed transient commands
	   retire even when a later stage fails, preventing repeated impulses on retry.
	   The force phase stages all loads together; an invalid expression cannot leave
	   a partially added physical load for the legacy queue to rescale on retry.
	*/
	namespace
	{
		constexpr size_t MaximumFieldDepth = 64, MaximumFieldNodes = 4096;
		constexpr double QuaternionTolerance = 1e-4;
		enum class Operation { Scalar, Radial, Box, Wave, Add, Product, Minimum, Maximum, Clamp, Vector, Direction, VectorAdd, Scale, Cross, Normalize };
		enum class Slot : size_t { Mask, Scalar, Vector, StaticFriction, DynamicFriction, Restitution, Count };
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		void Require(bool Valid, const char* Message) { if (!Valid) throw std::invalid_argument(Message); }
		Vector3 CheckedVector(double X, double Y, double Z)
		{
			const double Limit = std::numeric_limits<float>::max();
			Require(std::isfinite(X) && std::isfinite(Y) && std::isfinite(Z) && std::abs(X) <= Limit && std::abs(Y) <= Limit && std::abs(Z) <= Limit,"Field vector is not representable");
			return Vector3((float)X,(float)Y,(float)Z);
		}
		Vector3 Scale(const Vector3& V, double S) { return CheckedVector(V.x*S,V.y*S,V.z*S); }
		double Falloff(double Weight, FieldFalloff Curve)
		{
			Weight = std::max(0.0,std::min(1.0,Weight));
			if (Curve == FieldFalloff::Constant) return Weight > 0 ? 1 : 0;
			if (Curve == FieldFalloff::Smooth) return Weight*Weight*(3-2*Weight);
			return Weight;
		}
		void ValidateCurve(FieldFalloff Curve)
		{ Require(Curve >= FieldFalloff::Linear && Curve <= FieldFalloff::Constant,"Invalid field falloff"); }
	}
	struct PhysicsFieldNode
	{
		explicit PhysicsFieldNode(Operation Op) : Op(Op) {}
		Operation Op;
		std::shared_ptr<const PhysicsFieldNode> Left, Right;
		double Value = 0, Upper = 0, Frequency = 0, Phase = 0;
		Vector3 Vector = Vector3::Zero(), Outer = Vector3::Zero();
		Transform Pose;
		FieldFalloff Curve = FieldFalloff::Linear;
		size_t Depth = 1;
	};
	namespace
	{
		using Node = std::shared_ptr<const PhysicsFieldNode>;
		std::shared_ptr<PhysicsFieldNode> Make(Operation Op, Node A = {}, Node B = {})
		{
			if (Op == Operation::Add || Op == Operation::Product || Op == Operation::Minimum || Op == Operation::Maximum || Op == Operation::VectorAdd || Op == Operation::Scale || Op == Operation::Cross)
				Require(A && B,"Missing field operand");
			if (Op == Operation::Clamp || Op == Operation::Normalize) Require(A != nullptr,"Missing field operand");
			auto N = std::make_shared<PhysicsFieldNode>(Op); N->Left = A; N->Right = B;
			N->Depth = 1+std::max(A ? A->Depth : 0,B ? B->Depth : 0);
			Require(N->Depth <= MaximumFieldDepth,"Field expression exceeds its depth budget"); return N;
		}
		struct Value { double Scalar = 0; Vector3 Vector = Vector3::Zero(); };
		using Values = std::array<Value,(size_t)Slot::Count>;
		class Program
		{
		public:
			explicit Program(const std::vector<std::pair<Slot,Node>>& Roots)
			{
				std::unordered_map<const PhysicsFieldNode*,size_t> Indices;
				for (const auto& Root : Roots)
				{
					Require(Root.second != nullptr,"Empty field expression");
					std::vector<std::pair<Node,bool>> Stack{{Root.second,false}};
					while (!Stack.empty())
					{
						const auto Item = Stack.back(); Stack.pop_back(); const auto& N = Item.first;
						if (Indices.count(N.get())) continue;
						if (!Item.second)
						{
							Stack.push_back({N,true}); if (N->Left) Stack.push_back({N->Left,false}); if (N->Right) Stack.push_back({N->Right,false});
							continue;
						}
						Require(m_Code.size() < MaximumFieldNodes,"Field expression exceeds its node budget");
						m_Code.push_back({N,N->Left ? Indices.at(N->Left.get()) : 0,N->Right ? Indices.at(N->Right.get()) : 0});
						Indices.emplace(N.get(),m_Code.size()-1);
					}
					m_Roots[(size_t)Root.first] = Indices.at(Root.second.get()); m_Present[(size_t)Root.first] = true;
				}
				m_Values.resize(m_Code.size());
			}
			size_t GetSize() const { return m_Code.size(); }
			size_t GetLastNodeCount() const { return m_LastNodeCount; }
			Values Evaluate(const Vector3& Position, double Time, double MinimumMask = -1)
			{
				m_LastNodeCount = 0;
				Require(Finite(Position) && std::isfinite(Time),"Invalid field sample coordinates");
				for (size_t I = 0; I < m_Code.size(); ++I)
				{
					const auto& C = m_Code[I]; const auto& N = *C.Expression; const auto A = m_Values[C.Left], B = m_Values[C.Right]; Value R;
					switch (N.Op)
					{
					case Operation::Scalar: R.Scalar = N.Value; break;
					case Operation::Radial:
					{
						const double Distance = std::hypot(std::hypot((double)Position.x-N.Vector.x,(double)Position.y-N.Vector.y),(double)Position.z-N.Vector.z);
						R.Scalar = Distance <= N.Value ? 1 : Falloff((N.Upper-Distance)/(N.Upper-N.Value),N.Curve); break;
					}
					case Operation::Box:
					{
						double Weight = 1;
						for (int Axis = 0; Axis < 3; ++Axis)
						{
							Vector3 Basis = Vector3::Zero(); Basis[Axis] = 1; Basis = N.Pose.quat*Basis;
							const double Coordinate = std::abs(((double)Position.x-N.Pose.pos.x)*Basis.x+((double)Position.y-N.Pose.pos.y)*Basis.y+((double)Position.z-N.Pose.pos.z)*Basis.z);
							if (Coordinate > N.Vector[Axis]) Weight = std::min(Weight,N.Outer[Axis] == N.Vector[Axis] ? 0.0 : (N.Outer[Axis]-Coordinate)/(N.Outer[Axis]-N.Vector[Axis]));
						}
						R.Scalar = Falloff(Weight,N.Curve); break;
					}
					case Operation::Wave: R.Scalar = N.Value+N.Upper*std::sin(2*std::acos(-1.0)*N.Frequency*Time+N.Phase); break;
					case Operation::Add: R.Scalar = A.Scalar+B.Scalar; break;
					case Operation::Product: R.Scalar = A.Scalar*B.Scalar; break;
					case Operation::Minimum: R.Scalar = std::min(A.Scalar,B.Scalar); break;
					case Operation::Maximum: R.Scalar = std::max(A.Scalar,B.Scalar); break;
					case Operation::Clamp: R.Scalar = std::max(N.Value,std::min(N.Upper,A.Scalar)); break;
					case Operation::Vector: R.Vector = N.Vector; break;
					case Operation::Direction:
					case Operation::Normalize:
					{
						const double X = N.Op == Operation::Direction ? (double)Position.x-N.Vector.x : A.Vector.x;
						const double Y = N.Op == Operation::Direction ? (double)Position.y-N.Vector.y : A.Vector.y;
						const double Z = N.Op == Operation::Direction ? (double)Position.z-N.Vector.z : A.Vector.z;
						const double Length = std::hypot(std::hypot(X,Y),Z); R.Vector = Length > 0 ? CheckedVector(X/Length,Y/Length,Z/Length) : Vector3::Zero(); break;
					}
					case Operation::VectorAdd: R.Vector = CheckedVector((double)A.Vector.x+B.Vector.x,(double)A.Vector.y+B.Vector.y,(double)A.Vector.z+B.Vector.z); break;
					case Operation::Scale: R.Vector = Scale(A.Vector,B.Scalar); break;
					case Operation::Cross: R.Vector = CheckedVector((double)A.Vector.y*B.Vector.z-(double)A.Vector.z*B.Vector.y,(double)A.Vector.z*B.Vector.x-(double)A.Vector.x*B.Vector.z,(double)A.Vector.x*B.Vector.y-(double)A.Vector.y*B.Vector.x); break;
					}
					Require(std::isfinite(R.Scalar),"Field scalar overflow"); m_Values[I] = R; m_LastNodeCount = I+1;
					if (m_Present[(size_t)Slot::Mask] && I == m_Roots[(size_t)Slot::Mask])
					{
						const double Weight = std::max(0.0,std::min(1.0,R.Scalar));
						if (Weight <= MinimumMask) { Values Result; Result[(size_t)Slot::Mask].Scalar = Weight; return Result; }
					}
				}
				Values Result; for (size_t I = 0; I < m_Roots.size(); ++I) if (m_Present[I]) Result[I] = m_Values[m_Roots[I]]; return Result;
			}
		private:
			struct Instruction { Node Expression; size_t Left, Right; };
			std::vector<Instruction> m_Code;
			std::vector<Value> m_Values;
			std::array<size_t,(size_t)Slot::Count> m_Roots{};
			std::array<bool,(size_t)Slot::Count> m_Present{};
			size_t m_LastNodeCount = 0;
		};
	}
	ScalarField::ScalarField() : m_Node(Make(Operation::Scalar)) {}
	ScalarField::ScalarField(Node N) : m_Node(std::move(N)) {}
	ScalarField ScalarField::Constant(double V)
	{ Require(std::isfinite(V),"Invalid scalar field constant"); auto N = Make(Operation::Scalar); N->Value = V; return ScalarField(N); }
	ScalarField ScalarField::Radial(const Vector3& Center, double Inner, double Outer, FieldFalloff Curve)
	{
		ValidateCurve(Curve); Require(Finite(Center) && std::isfinite(Inner) && std::isfinite(Outer) && Inner >= 0 && Outer > Inner,"Invalid radial field");
		auto N = Make(Operation::Radial); N->Vector = Center; N->Value = Inner; N->Upper = Outer; N->Curve = Curve; return ScalarField(N);
	}
	ScalarField ScalarField::Box(const Transform& Pose, const Vector3& Inner, const Vector3& Outer, FieldFalloff Curve)
	{
		const auto& Q = Pose.quat; const double Norm = (double)Q.x*Q.x+(double)Q.y*Q.y+(double)Q.z*Q.z+(double)Q.w*Q.w;
		ValidateCurve(Curve); Require(Finite(Pose.pos) && std::isfinite(Norm) && std::abs(Norm-1) <= QuaternionTolerance && Finite(Inner) && Finite(Outer),"Invalid box field frame");
		for (int I = 0; I < 3; ++I) Require(Inner[I] >= 0 && Outer[I] >= Inner[I],"Invalid box field extents");
		auto N = Make(Operation::Box); N->Pose = Pose; N->Pose.quat = Pose.quat.Unit(); N->Vector = Inner; N->Outer = Outer; N->Curve = Curve; return ScalarField(N);
	}
	ScalarField ScalarField::Wave(double Offset, double Amplitude, double Frequency, double Phase)
	{
		Require(std::isfinite(Offset) && std::isfinite(Amplitude) && std::isfinite(Frequency) && std::isfinite(Phase),"Invalid field wave");
		auto N = Make(Operation::Wave); N->Value = Offset; N->Upper = Amplitude; N->Frequency = Frequency; N->Phase = Phase; return ScalarField(N);
	}
	ScalarField ScalarField::Add(const ScalarField& A, const ScalarField& B) { return ScalarField(Make(Operation::Add,A.m_Node,B.m_Node)); }
	ScalarField ScalarField::Multiply(const ScalarField& A, const ScalarField& B) { return ScalarField(Make(Operation::Product,A.m_Node,B.m_Node)); }
	ScalarField ScalarField::Minimum(const ScalarField& A, const ScalarField& B) { return ScalarField(Make(Operation::Minimum,A.m_Node,B.m_Node)); }
	ScalarField ScalarField::Maximum(const ScalarField& A, const ScalarField& B) { return ScalarField(Make(Operation::Maximum,A.m_Node,B.m_Node)); }
	ScalarField ScalarField::Clamp(const ScalarField& A, double Lower, double Upper)
	{
		Require(std::isfinite(Lower) && std::isfinite(Upper) && Lower <= Upper,"Invalid field clamp");
		auto N = Make(Operation::Clamp,A.m_Node); N->Value = Lower; N->Upper = Upper; return ScalarField(N);
	}
	VectorField::VectorField() : m_Node(Make(Operation::Vector)) {}
	VectorField::VectorField(Node N) : m_Node(std::move(N)) {}
	VectorField VectorField::Constant(const Vector3& V) { Require(Finite(V),"Invalid vector field constant"); auto N = Make(Operation::Vector); N->Vector = V; return VectorField(N); }
	VectorField VectorField::Radial(const Vector3& Center) { Require(Finite(Center),"Invalid vector field center"); auto N = Make(Operation::Direction); N->Vector = Center; return VectorField(N); }
	VectorField VectorField::Add(const VectorField& A, const VectorField& B) { return VectorField(Make(Operation::VectorAdd,A.m_Node,B.m_Node)); }
	VectorField VectorField::Scale(const VectorField& A, const ScalarField& S) { return VectorField(Make(Operation::Scale,A.m_Node,S.m_Node)); }
	VectorField VectorField::Cross(const VectorField& A, const VectorField& B) { return VectorField(Make(Operation::Cross,A.m_Node,B.m_Node)); }
	VectorField VectorField::Normalize(const VectorField& A) { return VectorField(Make(Operation::Normalize,A.m_Node)); }
	double ScalarField::Evaluate(const Vector3& P, double T) const { Program Eval({{Slot::Scalar,m_Node}}); return Eval.Evaluate(P,T)[(size_t)Slot::Scalar].Scalar; }
	Vector3 VectorField::Evaluate(const Vector3& P, double T) const { Program Eval({{Slot::Vector,m_Node}}); return Eval.Evaluate(P,T)[(size_t)Slot::Vector].Vector; }
	namespace
	{
		bool ForceTarget(PhysicsFieldTarget Target)
		{ return Target == PhysicsFieldTarget::Force || Target == PhysicsFieldTarget::Acceleration || Target == PhysicsFieldTarget::Torque; }
		bool Selects(const RigidBody& Body, uint32_t Mask)
		{
			for (const auto* Shape : Body.Geometries())
				if (Shape->GetFilterData().v0 < CollisionData::LayerCount && (Mask & (1u << Shape->GetFilterData().v0))) return true;
			return false;
		}
		float CheckedScalar(double Value)
		{ Require(std::isfinite(Value) && std::abs(Value) <= std::numeric_limits<float>::max(),"Field coefficient is not representable"); return (float)Value; }
	}
	struct PhysicsFieldSystem::Storage
	{
		explicit Storage(PhysicsWorld& World) : World(World) {}
		struct Entry { uint64_t Id; PhysicsFieldCommand Command; std::unique_ptr<Program> Evaluation; bool Applied = false; };
		struct BodySample { RigidBody* Body; Values Fields; Vector3 Vector; };
		PhysicsWorld& World;
		std::vector<Entry> Entries;
		uint64_t NextId = 1;
		PhysicsFieldStats Stats;
		void RequireIdle() const { if (World.IsSimulating()) throw std::logic_error("Edit physics fields between steps"); }
		Values Sample(Entry& E, const Vector3& Position)
		{
			++Stats.Samples;
			auto V = E.Evaluation->Evaluate(Position,Stats.Time,E.Command.MinimumMask);
			Stats.EvaluatedNodes += E.Evaluation->GetLastNodeCount();
			V[(size_t)Slot::Mask].Scalar = std::max(0.0,std::min(1.0,V[(size_t)Slot::Mask].Scalar)); return V;
		}
		template<class T> std::vector<BodySample> Stage(Entry& E, const std::vector<T*>& Bodies)
		{
			std::vector<BodySample> Result; const auto& C = E.Command;
			for (auto* B : Bodies)
			{
				auto* Dynamic = B->CastDynamic(); if (C.Target != PhysicsFieldTarget::Material && !Dynamic) continue;
				if (!Selects(*B,C.LayerMask)) continue;
				if ((ForceTarget(C.Target) || C.Target == PhysicsFieldTarget::Impulse) && (Dynamic->Freezing || (C.Target != PhysicsFieldTarget::Torque && B->InvMass <= 0))) continue;
				if ((C.Target == PhysicsFieldTarget::LinearVelocity || C.Target == PhysicsFieldTarget::AngularVelocity) && Dynamic->Freezing) continue;
				auto V = Sample(E,B->X); const double Weight = V[(size_t)Slot::Mask].Scalar;
				if (Weight <= C.MinimumMask) continue;
				Vector3 Vector = Scale(V[(size_t)Slot::Vector].Vector,Weight);
				if (C.Target == PhysicsFieldTarget::Acceleration) Vector = Scale(Vector,1.0/B->InvMass);
				if (ForceTarget(C.Target))
				{
					const auto Existing = C.Target == PhysicsFieldTarget::Torque ? Dynamic->ExtTorque : Dynamic->ExtForce;
					CheckedVector((double)Existing.x+Vector.x,(double)Existing.y+Vector.y,(double)Existing.z+Vector.z);
				}
				if (C.Target == PhysicsFieldTarget::LinearVelocity || C.Target == PhysicsFieldTarget::AngularVelocity)
					Require(!B->ReducedRagdoll,"Use forces or impulses instead of assigning reduced ragdoll link velocities");
				if (C.Target == PhysicsFieldTarget::Impulse && !B->ReducedRagdoll)
					CheckedVector((double)B->V.x+Vector.x*B->InvMass,(double)B->V.y+Vector.y*B->InvMass,(double)B->V.z+Vector.z*B->InvMass);
				if (C.Target == PhysicsFieldTarget::Material)
				{
					const float S = CheckedScalar(V[(size_t)Slot::StaticFriction].Scalar), D = CheckedScalar(V[(size_t)Slot::DynamicFriction].Scalar), R = CheckedScalar(V[(size_t)Slot::Restitution].Scalar);
					Require(D >= 0 && S >= D && R >= 0 && R <= 1,"Invalid field material coefficients");
				}
				Result.push_back({B,V,Vector});
			}
			return Result;
		}
	};
	PhysicsFieldSystem::PhysicsFieldSystem(PhysicsWorld& World) : m_Data(new Storage(World)) {}
	PhysicsFieldSystem::~PhysicsFieldSystem() = default;
	uint64_t PhysicsFieldSystem::Add(const PhysicsFieldCommand& C)
	{
		m_Data->RequireIdle();
		Require(C.Target >= PhysicsFieldTarget::Force && C.Target <= PhysicsFieldTarget::Material && C.Activation >= PhysicsFieldActivation::Wake && C.Activation <= PhysicsFieldActivation::Unfreeze
			&& std::isfinite(C.MinimumMask) && C.MinimumMask >= 0 && C.MinimumMask <= 1,"Invalid physics field command");
		Require(!C.Persistent || C.Target != PhysicsFieldTarget::Impulse,"Persistent impulses are not substep invariant; use force or acceleration");
		std::vector<std::pair<Slot,Node>> Roots{{Slot::Mask,C.Mask.m_Node}};
		if (C.Target == PhysicsFieldTarget::Strain) Roots.push_back({Slot::Scalar,C.Scalar.m_Node});
		else if (C.Target == PhysicsFieldTarget::Material)
		{
			Roots.push_back({Slot::StaticFriction,C.StaticFriction.m_Node}); Roots.push_back({Slot::DynamicFriction,C.DynamicFriction.m_Node}); Roots.push_back({Slot::Restitution,C.Restitution.m_Node});
		}
		else if (C.Target != PhysicsFieldTarget::Activation) Roots.push_back({Slot::Vector,C.Vector.m_Node});
		std::unique_ptr<Program> Evaluation(new Program(Roots));
		if (m_Data->NextId == 0) throw std::overflow_error("Physics field identity exhausted");
		const uint64_t Id = m_Data->NextId++; m_Data->Entries.push_back({Id,C,std::move(Evaluation),false}); return Id;
	}
	bool PhysicsFieldSystem::Remove(uint64_t Id)
	{
		m_Data->RequireIdle(); auto& Entries = m_Data->Entries;
		const auto I = std::find_if(Entries.begin(),Entries.end(),[&](const Storage::Entry& E) { return E.Id == Id; });
		if (I == Entries.end()) return false; Entries.erase(I); return true;
	}
	void PhysicsFieldSystem::Clear() { m_Data->RequireIdle(); m_Data->Entries.clear(); }
	size_t PhysicsFieldSystem::GetCount() const { return m_Data->Entries.size(); }
	const PhysicsFieldStats& PhysicsFieldSystem::GetStats() const { return m_Data->Stats; }
	void PhysicsFieldSystem::BeginFrame() { const double Time = m_Data->Stats.Time; m_Data->Stats = {}; m_Data->Stats.Time = Time; }
	void PhysicsFieldSystem::Reset() { m_Data->Entries.clear(); m_Data->Stats = {}; }
	void PhysicsFieldSystem::FinishSubstep(float Dt) { m_Data->Stats.Time += Dt; }
	void PhysicsFieldSystem::EndFrame()
	{
		auto& Entries = m_Data->Entries;
		Entries.erase(std::remove_if(Entries.begin(),Entries.end(),[](const Storage::Entry& E) { return !E.Command.Persistent && E.Applied; }),Entries.end());
	}
	void PhysicsFieldSystem::ApplyForces()
	{
		struct Load { RigidBodyDynamic* Body; Vector3 Force = Vector3::Zero(), Torque = Vector3::Zero(); };
		std::vector<Load> Loads; std::unordered_map<RigidBodyDynamic*,size_t> Slots; uint64_t Applications = 0;
		for (auto& E : m_Data->Entries) if (ForceTarget(E.Command.Target))
		{
			const auto Samples = m_Data->Stage(E,m_Data->World.m_DynamicBodies);
			for (const auto& S : Samples)
			{
				auto* Body = S.Body->CastDynamic();
				const auto Slot = Slots.emplace(Body,Loads.size()); if (Slot.second) Loads.push_back({Body});
				auto& Load = Loads[Slot.first->second]; const bool Torque = E.Command.Target == PhysicsFieldTarget::Torque;
				auto& Value = Torque ? Load.Torque : Load.Force;
				Value = CheckedVector((double)Value.x+S.Vector.x,(double)Value.y+S.Vector.y,(double)Value.z+S.Vector.z);
				const auto Existing = Torque ? Body->ExtTorque : Body->ExtForce;
				CheckedVector((double)Existing.x+Value.x,(double)Existing.y+Value.y,(double)Existing.z+Value.z); ++Applications;
			}
		}
		for (const auto& Load : Loads)
		{
			Load.Body->ApplyForce(Load.Force); Load.Body->ApplyTorgue(Load.Torque);
			if ((Load.Force != Vector3::Zero() || Load.Torque != Vector3::Zero()) && m_Data->World.m_IslandSleeping.GetSettings().Enabled)
				m_Data->World.m_IslandSleeping.Activate(Load.Body);
		}
		m_Data->Stats.BodyApplications += Applications;
		for (auto& E : m_Data->Entries) if (ForceTarget(E.Command.Target)) E.Applied = true;
	}
	void PhysicsFieldSystem::ApplyBoundary()
	{
		for (bool StrainPass : {true,false}) for (auto& E : m_Data->Entries)
		{
			const auto& C = E.Command; if (ForceTarget(C.Target) || (!C.Persistent && E.Applied)) continue;
			if ((C.Target == PhysicsFieldTarget::Strain) != StrainPass) continue;
			if (C.Target == PhysicsFieldTarget::Strain)
			{
				struct SetSample { DestructionSet* Set; std::vector<DestructionBondStrain> Strains; };
				std::vector<SetSample> Samples;
				for (auto* Set : m_Data->World.m_DestructionSets)
				{
					std::vector<Transform> Poses(Set->GetChunks().size()); std::vector<bool> Active(Poses.size(),false);
					for (auto* Cluster : Set->GetClusters()) if (Cluster->GetRigidBody() && Selects(*Cluster->GetRigidBody(),C.LayerMask))
					{
						const auto Frame = Cluster->GetClusterWorldTransform(); const auto& Indices = Cluster->GetSourceIndices(); const auto& Local = Cluster->GetLocalTransforms();
						for (size_t I = 0; I < Indices.size(); ++I) { Poses[Indices[I]] = Frame*Local[I]; Active[Indices[I]] = true; }
					}
					Samples.push_back({Set,{}}); std::set<std::pair<int,int>> Visited;
					for (const auto& Row : Set->GetGraph().GetBondLists()) for (const auto& B : Row.Bonds)
					{
						Require(B.v0 >= 0 && B.v1 >= 0 && (size_t)B.v0 < Poses.size() && (size_t)B.v1 < Poses.size(),"Invalid strain field bond endpoints");
						if (!Active[B.v0] || !Active[B.v1] || !Visited.insert(std::minmax(B.v0,B.v1)).second) continue;
						const auto& Chunks = Set->GetChunks();
						const auto Point = (Poses[B.v0].LocalToWorld(B.frame.Origin-Chunks[B.v0].Centroid)+Poses[B.v1].LocalToWorld(B.frame.Origin-Chunks[B.v1].Centroid))*0.5f;
						const auto V = m_Data->Sample(E,Point); const double Weight = V[(size_t)Slot::Mask].Scalar;
						if (Weight <= C.MinimumMask) continue;
						const float Demand = CheckedScalar(V[(size_t)Slot::Scalar].Scalar*Weight); Require(Demand >= 0,"External strain must be nonnegative");
						Samples.back().Strains.push_back({B.v0,B.v1,Demand});
					}
				}
				for (const auto& S : Samples) m_Data->Stats.BrokenBonds += S.Set->ApplyBondStrain(S.Strains);
			}
			else
			{
				std::vector<Storage::BodySample> Samples;
				if (C.Target == PhysicsFieldTarget::Material)
				{
					std::vector<RigidBody*> Bodies(m_Data->World.m_StaticBodies.begin(),m_Data->World.m_StaticBodies.end());
					Bodies.insert(Bodies.end(),m_Data->World.m_DynamicBodies.begin(),m_Data->World.m_DynamicBodies.end());
					Bodies.insert(Bodies.end(),m_Data->World.m_KinematicBodies.begin(),m_Data->World.m_KinematicBodies.end());
					Samples = m_Data->Stage(E,Bodies);
				}
				else Samples = m_Data->Stage(E,m_Data->World.m_DynamicBodies);
				bool Invalidate = false;
				for (const auto& S : Samples)
				{
					auto* B = S.Body->CastDynamic();
					switch (C.Target)
					{
					case PhysicsFieldTarget::Impulse:
						if (B->ReducedRagdoll) B->ReducedRagdoll->ApplyImpulse(B->RagdollLink,B->X,S.Vector);
						else { B->V += S.Vector*B->InvMass; if (S.Vector != Vector3::Zero()) B->Wakeup(); } break;
					case PhysicsFieldTarget::LinearVelocity: B->V = S.Vector; B->Wakeup(); break;
					case PhysicsFieldTarget::AngularVelocity: B->W = S.Vector; B->Wakeup(); break;
					case PhysicsFieldTarget::Activation:
					{
						const bool Sleeping = B->Sleeping, Frozen = B->Freezing;
						switch (C.Activation)
						{
						case PhysicsFieldActivation::Wake: B->Wakeup(); break;
						case PhysicsFieldActivation::Sleep: B->Sleep(); B->V = B->W = Vector3::Zero(); break;
						case PhysicsFieldActivation::Freeze: B->Freeze(); B->V = B->W = B->ExtForce = B->ExtTorque = Vector3::Zero(); break;
						case PhysicsFieldActivation::Unfreeze: B->Defreeze(); B->Wakeup(); break;
						}
						Invalidate |= Sleeping != B->Sleeping || Frozen != B->Freezing; break;
					}
					case PhysicsFieldTarget::Material:
					{
						const float Static = (float)S.Fields[(size_t)Slot::StaticFriction].Scalar, Dynamic = (float)S.Fields[(size_t)Slot::DynamicFriction].Scalar, Restitution = (float)S.Fields[(size_t)Slot::Restitution].Scalar;
						const bool Changed = Static != S.Body->GetFrictionStatic() || Dynamic != S.Body->GetFrictionDynamic() || Restitution != S.Body->GetRestitution();
						Invalidate |= Changed; if (Changed) { S.Body->SetFriction(Dynamic,Static); S.Body->SetRestitution(Restitution); } break;
					}
					default: break;
					}
					++m_Data->Stats.BodyApplications;
				}
				if (Invalidate)
				{
					if (m_Data->World.m_Solver) m_Data->World.m_Solver->ResetState();
					if (m_Data->World.m_PositionSolver) m_Data->World.m_PositionSolver->ResetState();
				}
			}
			E.Applied = true;
		}
	}
}

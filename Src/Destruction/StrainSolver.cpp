#include "StrainSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace Riemann
{
	namespace
	{
		bool IsFinite(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		float PositiveSetting(float Value, float Fallback)
		{
			return std::isfinite(Value) && Value > 0.0f ? Value : Fallback;
		}

		float NonNegativeSetting(float Value, float Fallback)
		{
			return std::isfinite(Value) && Value >= 0.0f ? Value : Fallback;
		}

		double Dot(const std::vector<double>& A, const std::vector<double>& B)
		{
			double Value = 0.0;
			for (size_t i = 0; i < A.size(); ++i) Value += A[i] * B[i];
			return Value;
		}

		void ClearBondResults(StrainBondInfo& Info)
		{
			Info.ImpulseLinear = Info.ImpulseAngular = Vector3::Zero();
			Info.ForceLinear = Info.Torque = Vector3::Zero();
			Info.Stress = Info.Tension = Info.Compression = Info.Shear = Info.Bending = Info.Torsion = 0.0f;
		}
	}

	StrainSolver::StrainSolver(const ConnectionGraph& Graph, const StrainSolverSettings& Settings)
	{
		SetSettings(Settings);
		ResetGraph(Graph);
	}

	StrainSolver::StrainSolver(const DestructionSet& DestructSet, const StrainSolverSettings& Settings)
	{
		SetSettings(Settings);
		ResetGraph(DestructSet.GetGraph());
		SetAllNodesInfo(DestructSet);
	}

	void StrainSolver::SetSettings(const StrainSolverSettings& Settings)
	{
		mSettings = Settings;
		mSettings.Hardness = PositiveSetting(Settings.Hardness, 1000.0f);
		mSettings.StressLinearFactor = NonNegativeSetting(Settings.StressLinearFactor, 0.25f);
		mSettings.StressAngularFactor = NonNegativeSetting(Settings.StressAngularFactor, 0.75f);
		mSettings.RelativeTolerance = PositiveSetting(Settings.RelativeTolerance, 1.0e-6f);
		mSettings.AbsoluteTolerance = PositiveSetting(Settings.AbsoluteTolerance, 1.0e-7f);
	}

	void StrainSolver::ResetGraph(const ConnectionGraph& Graph)
	{
		mNodes.assign(Graph.Size(), RuntimeNode());
		mRuntimeBonds.clear();
		SyncGraph(Graph);
		Reset();
	}

	uint32_t StrainSolver::SyncGraph(const ConnectionGraph& Graph)
	{
		bool Changed = mNodes.size() != Graph.Size();
		mNodes.resize(Graph.Size());
		std::map<std::pair<int, int>, RuntimeBond> Previous;
		for (const RuntimeBond& Existing : mRuntimeBonds)
		{
			Previous[std::make_pair(Existing.Info.Node0, Existing.Info.Node1)] = Existing;
		}
		std::set<int> ReservedIndices;
		for (size_t Node = 0; Node < Graph.Size(); ++Node)
		{
			for (const Bond& Existing : Graph[Node].Bonds)
			{
				if (Existing.bond_index >= 0) ReservedIndices.insert(Existing.bond_index);
			}
		}
		std::set<int> AssignedIndices;
		std::set<std::pair<int, int>> Pairs;
		std::vector<RuntimeBond> Rebuilt;
		int GeneratedIndex = 0;
		for (size_t Node = 0; Node < Graph.Size(); ++Node)
		{
			for (const Bond& Existing : Graph[Node].Bonds)
			{
				const int Other = Existing.GetOther((int)Node);
				if (Other <= (int)Node || (size_t)Other >= Graph.Size() || !mNodes[Node].Enabled
					|| !mNodes[(size_t)Other].Enabled || !std::isfinite(Existing.strength) || Existing.strength <= 0.0f) continue;
				const std::pair<int, int> Pair((int)Node, Other);
				if (!Pairs.insert(Pair).second) continue;
				const auto Old = Previous.find(Pair);
				int BondIndex = Existing.bond_index;
				if (BondIndex < 0 && Old != Previous.end() && !ReservedIndices.count(Old->second.Info.BondIndex)
					&& !AssignedIndices.count(Old->second.Info.BondIndex)) BondIndex = Old->second.Info.BondIndex;
				if (BondIndex < 0 || AssignedIndices.count(BondIndex))
				{
					while (ReservedIndices.count(GeneratedIndex) || AssignedIndices.count(GeneratedIndex)) ++GeneratedIndex;
					BondIndex = GeneratedIndex++;
				}
				AssignedIndices.insert(BondIndex);
				RuntimeBond Current;
				if (Old != Previous.end())
				{
					Current = Old->second;
					Changed = Changed || Current.Info.BondIndex != BondIndex;
					Previous.erase(Old);
				}
				else Changed = true;
				Current.Info.Node0 = (int)Node;
				Current.Info.Node1 = Other;
				Current.Info.BondIndex = BondIndex;
				if (Current.GraphHealth != Existing.strength) Current.Info.Health = Existing.strength;
				Current.GraphHealth = Existing.strength;
				Rebuilt.push_back(Current);
			}
		}
		const uint32_t Removed = (uint32_t)Previous.size();
		Changed = Changed || Removed > 0;
		mRuntimeBonds.swap(Rebuilt);
		UpdateBondGeometry();
		if (Changed) ClearSolution();
		else PublishBonds();
		return Removed;
	}

	void StrainSolver::EnsureNodeCount(size_t Count)
	{
		if (mNodes.size() < Count) mNodes.resize(Count);
	}

	bool StrainSolver::IsActiveNode(int Index) const
	{
		return Index >= 0 && (size_t)Index < mNodes.size() && mNodes[(size_t)Index].Enabled;
	}

	void StrainSolver::SetNodeInfo(int Index, float Mass, float Volume, const Vector3& Position, bool Static)
	{
		if (Index < 0 || !std::isfinite(Mass) || Mass <= 0.0f || !std::isfinite(Volume) || Volume <= 0.0f || !IsFinite(Position)) return;
		EnsureNodeCount((size_t)Index + 1);
		RuntimeNode& Node = mNodes[(size_t)Index];
		const bool Changed = !Node.InfoInitialized || Node.Info.Mass != Mass || Node.Info.Volume != Volume || Node.Info.IsStatic != Static
			|| Node.Info.LocalPosition != Position || !Node.Enabled;
		if (!Changed) return;
		Node.Info.Mass = Mass;
		Node.Info.Volume = Volume;
		Node.Info.LocalPosition = Position;
		Node.Info.IsStatic = Static;
		Node.Enabled = true;
		Node.InfoInitialized = true;
		if (!Node.CustomInertia)
		{
			const float Radius = std::cbrt(Volume * 3.0f / (4.0f * 3.14159265358979323846f));
			const float Inertia = std::max(0.4f * Mass * Radius * Radius, 1.0e-12f);
			Node.Info.Inertia = Matrix3(Inertia, Inertia, Inertia);
			Node.InverseInertia = Matrix3(1.0f / Inertia, 1.0f / Inertia, 1.0f / Inertia);
		}
		UpdateBondGeometry();
		ClearSolution();
	}

	bool StrainSolver::SetNodeInertia(int Index, const Matrix3& Inertia)
	{
		if (!IsActiveNode(Index)) return false;
		float Scale = 0.0f;
		for (int Row = 0; Row < 3; ++Row)
		{
			for (int Column = 0; Column < 3; ++Column)
			{
				if (!std::isfinite(Inertia[Row][Column])) return false;
				Scale = std::max(Scale, std::fabs(Inertia[Row][Column]));
			}
		}
		if (Scale <= 0.0f) return false;
		for (int Row = 0; Row < 3; ++Row)
		{
			for (int Column = 0; Column < Row; ++Column)
			{
				if (std::fabs(Inertia[Row][Column] - Inertia[Column][Row]) > Scale * 1.0e-6f) return false;
			}
		}
		Matrix3 Normalized = Inertia;
		Normalized /= Scale;
		if (Normalized[0][0] <= 0.0f
			|| Normalized[0][0] * Normalized[1][1] - Normalized[0][1] * Normalized[1][0] <= 0.0f
			|| Normalized.Determinant() <= 1.0e-12f) return false;
		Matrix3 Inverse = Normalized.Inverse();
		for (int Row = 0; Row < 3; ++Row)
		{
			for (int Column = 0; Column < 3; ++Column)
			{
				const double Value = (double)Inverse[Row][Column] / Scale;
				if (!std::isfinite(Value) || std::fabs(Value) > std::numeric_limits<float>::max()) return false;
				Inverse[Row][Column] = (float)Value;
			}
		}
		RuntimeNode& Node = mNodes[(size_t)Index];
		Node.CustomInertia = true;
		bool Changed = false;
		for (int Row = 0; Row < 3; ++Row)
		{
			for (int Column = 0; Column < 3; ++Column) Changed = Changed || Node.Info.Inertia[Row][Column] != Inertia[Row][Column];
		}
		if (Changed)
		{
			Node.Info.Inertia = Inertia;
			Node.InverseInertia = Inverse;
			ClearSolution();
		}
		return true;
	}

	void StrainSolver::SetAllNodesInfo(const DestructionSet& DestructSet, float Density)
	{
		Density = PositiveSetting(Density, 1.0f);
		const std::vector<DestructionChunk>& Chunks = DestructSet.GetChunks();
		EnsureNodeCount(Chunks.size());
		bool Disabled = false;
		for (size_t Index = 0; Index < mNodes.size(); ++Index)
		{
			if (Index >= Chunks.size() || Chunks[Index].IsFree || Chunks[Index].IsSmashed)
			{
				Disabled = Disabled || mNodes[Index].Enabled;
				mNodes[Index].Enabled = false;
				mNodes[Index].Impulse = mNodes[Index].AngularImpulse = mNodes[Index].Force = mNodes[Index].Torque = Vector3::Zero();
				continue;
			}
			const DestructionChunk& Chunk = Chunks[Index];
			const float Volume = PositiveSetting(Chunk.Volume, 1.0e-3f);
			const float Mass = PositiveSetting(Chunk.Mass, Volume * Density);
			SetNodeInfo((int)Index, Mass, Volume, Chunk.Centroid, Chunk.IsStatic);
			const Vector3 Size = Chunk.Bounds.GetSize();
			if (!mNodes[Index].CustomInertia && IsFinite(Size) && Size.x > 0.0f && Size.y > 0.0f && Size.z > 0.0f)
			{
				const Vector3 Squared(Size.x * Size.x, Size.y * Size.y, Size.z * Size.z);
				SetNodeInertia((int)Index, Matrix3(Mass * (Squared.y + Squared.z) / 12.0f,
					Mass * (Squared.x + Squared.z) / 12.0f, Mass * (Squared.x + Squared.y) / 12.0f));
				mNodes[Index].CustomInertia = false;
			}
		}
		if (Disabled)
		{
			mRuntimeBonds.erase(std::remove_if(mRuntimeBonds.begin(), mRuntimeBonds.end(), [this](const RuntimeBond& Bond)
			{
				return !IsActiveNode(Bond.Info.Node0) || !IsActiveNode(Bond.Info.Node1);
			}), mRuntimeBonds.end());
			ClearSolution();
		}
	}

	void StrainSolver::UpdateBondGeometry()
	{
		for (RuntimeBond& Bond : mRuntimeBonds)
		{
			const Vector3 A = mNodes[(size_t)Bond.Info.Node0].Info.LocalPosition;
			const Vector3 B = mNodes[(size_t)Bond.Info.Node1].Info.LocalPosition;
			if (!Bond.CustomGeometry)
			{
				Bond.Info.Centroid = (A + B) * 0.5f;
				Bond.Info.Normal = (B - A).SquareLength() > 1.0e-12f ? (B - A).SafeUnit() : Vector3::UnitX();
			}
			Bond.Offset0 = Bond.Info.Centroid - A;
			Bond.Offset1 = Bond.Info.Centroid - B;
		}
	}

	bool StrainSolver::SetBondGeometry(int Index, const Vector3& Centroid, const Vector3& Normal, float Area)
	{
		if (!IsFinite(Centroid) || !IsFinite(Normal) || Normal.SquareLength() < 1.0e-12f || !std::isfinite(Area) || Area <= 0.0f) return false;
		for (RuntimeBond& Bond : mRuntimeBonds)
		{
			if (Bond.Info.BondIndex != Index) continue;
			Bond.Info.Centroid = Centroid;
			Bond.Info.Normal = Normal.SafeUnit();
			Bond.Info.Area = Area;
			Bond.CustomGeometry = true;
			UpdateBondGeometry();
			ClearSolution();
			return true;
		}
		return false;
	}

	bool StrainSolver::SetBondHealth(int Index, float Health)
	{
		if (!std::isfinite(Health) || Health < 0.0f) return false;
		if (Health == 0.0f) return RemoveBond(Index);
		for (RuntimeBond& Bond : mRuntimeBonds)
		{
			if (Bond.Info.BondIndex != Index) continue;
			Bond.Info.Health = Health;
			PublishBonds();
			return true;
		}
		return false;
	}

	bool StrainSolver::RemoveBond(int Index)
	{
		const auto Found = std::find_if(mRuntimeBonds.begin(), mRuntimeBonds.end(), [Index](const RuntimeBond& Bond) { return Bond.Info.BondIndex == Index; });
		if (Found == mRuntimeBonds.end()) return false;
		mRuntimeBonds.erase(Found);
		ClearSolution();
		return true;
	}

	void StrainSolver::AddForce(int Index, const Vector3& Force, StrainForceMode Mode)
	{
		if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic || !IsFinite(Force)) return;
		RuntimeNode& Node = mNodes[(size_t)Index];
		switch (Mode)
		{
		case StrainForceMode::Impulse: Node.Impulse += Force; break;
		case StrainForceMode::VelocityChange: Node.Impulse += Force * Node.Info.Mass; break;
		case StrainForceMode::Force: Node.Force += Force; break;
		case StrainForceMode::Acceleration: Node.Force += Force * Node.Info.Mass; break;
		}
	}

	bool StrainSolver::AddTorque(int Index, const Vector3& Torque, StrainForceMode Mode)
	{
		if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic || !IsFinite(Torque)) return false;
		RuntimeNode& Node = mNodes[(size_t)Index];
		switch (Mode)
		{
		case StrainForceMode::Impulse: Node.AngularImpulse += Torque; break;
		case StrainForceMode::VelocityChange: Node.AngularImpulse += Node.Info.Inertia * Torque; break;
		case StrainForceMode::Force: Node.Torque += Torque; break;
		case StrainForceMode::Acceleration: Node.Torque += Node.Info.Inertia * Torque; break;
		default: return false;
		}
		return true;
	}

	bool StrainSolver::AddForceAtPosition(int Index, const Vector3& Position, const Vector3& Force, StrainForceMode Mode)
	{
		if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic || !IsFinite(Position) || !IsFinite(Force)) return false;
		const bool MassScaled = Mode == StrainForceMode::VelocityChange || Mode == StrainForceMode::Acceleration;
		const bool Continuous = Mode == StrainForceMode::Force || Mode == StrainForceMode::Acceleration;
		if (Mode != StrainForceMode::Impulse && Mode != StrainForceMode::VelocityChange && !Continuous) return false;
		const RuntimeNode& Node = mNodes[(size_t)Index];
		const Vector3 PhysicalForce = MassScaled ? Force * Node.Info.Mass : Force;
		const Vector3 Torque = (Position - Node.Info.LocalPosition).Cross(PhysicalForce);
		AddForce(Index, PhysicalForce, Continuous ? StrainForceMode::Force : StrainForceMode::Impulse);
		return AddTorque(Index, Torque, Continuous ? StrainForceMode::Force : StrainForceMode::Impulse);
	}

	int StrainSolver::FindNearestClusterNode(const DestructionCluster& Cluster, const Vector3& Position) const
	{
		int Best = -1;
		float Distance = std::numeric_limits<float>::max();
		for (int Index : Cluster.GetSourceIndices())
		{
			if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic) continue;
			const float Current = (mNodes[(size_t)Index].Info.LocalPosition - Position).SquareLength();
			if (Current < Distance) { Best = Index; Distance = Current; }
		}
		return Best;
	}

	bool StrainSolver::AddForce(const DestructionCluster& Cluster, const Vector3& Position, const Vector3& Force, StrainForceMode Mode)
	{
		return AddForceAtPosition(FindNearestClusterNode(Cluster, Position), Position, Force, Mode);
	}

	bool StrainSolver::AddGravityForce(const DestructionCluster& Cluster, const Vector3& Gravity)
	{
		if (!IsFinite(Gravity)) return false;
		bool Applied = false;
		for (int Index : Cluster.GetSourceIndices())
		{
			if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic) continue;
			AddForce(Index, Gravity, StrainForceMode::Acceleration);
			Applied = true;
		}
		return Applied;
	}

	bool StrainSolver::AddAngularVelocity(const DestructionCluster& Cluster, const Vector3& Center, const Vector3& AngularVelocity)
	{
		if (!IsFinite(Center) || !IsFinite(AngularVelocity)) return false;
		bool Applied = false;
		for (int Index : Cluster.GetSourceIndices())
		{
			if (!IsActiveNode(Index) || mNodes[(size_t)Index].Info.IsStatic) continue;
			const Vector3 R = mNodes[(size_t)Index].Info.LocalPosition - Center;
			// Outward centrifugal loading must be balanced by internal tension.
			AddForce(Index, -AngularVelocity.Cross(AngularVelocity.Cross(R)), StrainForceMode::Acceleration);
			Applied = true;
		}
		return Applied;
	}

	void StrainSolver::ApplyImpulseResponse(const std::vector<double>& Bonds, std::vector<double>& Nodes) const
	{
		Nodes.assign(mNodes.size() * 6, 0.0);
		for (size_t i = 0; i < mRuntimeBonds.size(); ++i)
		{
			const RuntimeBond& Bond = mRuntimeBonds[i];
			const double* Impulse = &Bonds[i * 6];
			for (int End = 0; End < 2; ++End)
			{
				const size_t Index = (size_t)(End == 0 ? Bond.Info.Node0 : Bond.Info.Node1);
				const Vector3 R = End == 0 ? Bond.Offset0 : Bond.Offset1;
				const double Sign = End == 0 ? 1.0 : -1.0;
				double* Node = &Nodes[Index * 6];
				for (int Axis = 0; Axis < 3; ++Axis) Node[Axis] += Sign * Impulse[Axis];
				Node[3] += Sign * (Impulse[3] + R.y * Impulse[2] - R.z * Impulse[1]);
				Node[4] += Sign * (Impulse[4] + R.z * Impulse[0] - R.x * Impulse[2]);
				Node[5] += Sign * (Impulse[5] + R.x * Impulse[1] - R.y * Impulse[0]);
			}
		}
		for (size_t i = 0; i < mNodes.size(); ++i)
		{
			const RuntimeNode& Node = mNodes[i];
			double* Value = &Nodes[i * 6];
			if (Node.Info.IsStatic || !Node.Enabled)
			{
				std::fill(Value, Value + 6, 0.0);
				continue;
			}
			for (int Axis = 0; Axis < 3; ++Axis) Value[Axis] /= Node.Info.Mass;
			double Angular[3] = {};
			for (int Row = 0; Row < 3; ++Row)
			{
				for (int Column = 0; Column < 3; ++Column) Angular[Row] += Node.InverseInertia[Row][Column] * Value[Column + 3];
			}
			std::copy(Angular, Angular + 3, Value + 3);
		}
	}

	void StrainSolver::ApplyJacobian(const std::vector<double>& Nodes, std::vector<double>& Bonds) const
	{
		Bonds.resize(mRuntimeBonds.size() * 6);
		for (size_t i = 0; i < mRuntimeBonds.size(); ++i)
		{
			const RuntimeBond& Bond = mRuntimeBonds[i];
			const double* A = &Nodes[(size_t)Bond.Info.Node0 * 6];
			const double* B = &Nodes[(size_t)Bond.Info.Node1 * 6];
			double* Value = &Bonds[i * 6];
			Value[0] = A[0] - B[0] + A[4] * Bond.Offset0.z - A[5] * Bond.Offset0.y - B[4] * Bond.Offset1.z + B[5] * Bond.Offset1.y;
			Value[1] = A[1] - B[1] + A[5] * Bond.Offset0.x - A[3] * Bond.Offset0.z - B[5] * Bond.Offset1.x + B[3] * Bond.Offset1.z;
			Value[2] = A[2] - B[2] + A[3] * Bond.Offset0.y - A[4] * Bond.Offset0.x - B[3] * Bond.Offset1.y + B[4] * Bond.Offset1.x;
			for (int Axis = 3; Axis < 6; ++Axis) Value[Axis] = A[Axis] - B[Axis];
		}
	}

	void StrainSolver::ApplyMatrix(const std::vector<double>& Input, std::vector<double>& Output, std::vector<double>& NodeScratch) const
	{
		ApplyImpulseResponse(Input, NodeScratch);
		ApplyJacobian(NodeScratch, Output);
	}

	void StrainSolver::Solve(uint32_t IterationCount, bool WarmStart)
	{
		// Independently implemented from the equilibrium formulation documented by
		// NVIDIA Blast's stress_solver/stress.cpp: (J M^-1 J^T) lambda = -J v.
		// Each multiply visits nodes and bonds; no dense global matrix is stored.
		const size_t Dimension = mRuntimeBonds.size() * 6;
		std::vector<double> NodeVelocity(mNodes.size() * 6, 0.0);
		for (size_t i = 0; i < mNodes.size(); ++i)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				NodeVelocity[i * 6 + Axis] = mNodes[i].VelocityLinear[Axis];
				NodeVelocity[i * 6 + Axis + 3] = mNodes[i].VelocityAngular[Axis];
			}
		}
		std::vector<double> RHS;
		ApplyJacobian(NodeVelocity, RHS);
		for (double& Value : RHS) Value = -Value;
		const double RHSNorm = std::sqrt(Dot(RHS, RHS));
		const double Threshold = mSettings.AbsoluteTolerance + mSettings.RelativeTolerance * RHSNorm;
		std::vector<double> Solution(Dimension, 0.0), InverseDiagonal(Dimension, 1.0);
		std::vector<double> Residual(Dimension), Direction(Dimension), Preconditioned(Dimension), Product, Scratch;
		for (size_t i = 0; i < mRuntimeBonds.size(); ++i)
		{
			const RuntimeBond& Bond = mRuntimeBonds[i];
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				if (WarmStart && RHSNorm > mSettings.AbsoluteTolerance)
				{
					Solution[i * 6 + Axis] = Bond.Info.ImpulseLinear[Axis];
					Solution[i * 6 + Axis + 3] = Bond.Info.ImpulseAngular[Axis];
				}
				Vector3 Unit = Vector3::Zero();
				Unit[Axis] = 1.0f;
				double Linear = 0.0, Angular = 0.0;
				for (int End = 0; End < 2; ++End)
				{
					const RuntimeNode& Node = mNodes[(size_t)(End == 0 ? Bond.Info.Node0 : Bond.Info.Node1)];
					if (Node.Info.IsStatic || !Node.Enabled) continue;
					const Vector3 Cross = (End == 0 ? Bond.Offset0 : Bond.Offset1).Cross(Unit);
					Linear += 1.0 / Node.Info.Mass + Cross.Dot(Node.InverseInertia * Cross);
					Angular += Node.InverseInertia[Axis][Axis];
				}
				InverseDiagonal[i * 6 + Axis] = Linear > 0.0 ? 1.0 / Linear : 1.0;
				InverseDiagonal[i * 6 + Axis + 3] = Angular > 0.0 ? 1.0 / Angular : 1.0;
			}
		}
		ApplyMatrix(Solution, Product, Scratch);
		for (size_t i = 0; i < Dimension; ++i)
		{
			Residual[i] = RHS[i] - Product[i];
			Direction[i] = Preconditioned[i] = Residual[i] * InverseDiagonal[i];
		}
		double ResidualSquared = Dot(Residual, Residual);
		double RZ = Dot(Residual, Preconditioned);
		mLastIterations = 0;
		while (mLastIterations < IterationCount && ResidualSquared > Threshold * Threshold)
		{
			ApplyMatrix(Direction, Product, Scratch);
			const double Denominator = Dot(Direction, Product);
			if (!std::isfinite(Denominator) || Denominator <= 0.0 || RZ <= 0.0) break;
			const double Alpha = RZ / Denominator;
			for (size_t i = 0; i < Dimension; ++i)
			{
				Solution[i] += Alpha * Direction[i];
				Residual[i] -= Alpha * Product[i];
			}
			++mLastIterations;
			ResidualSquared = Dot(Residual, Residual);
			for (size_t i = 0; i < Dimension; ++i) Preconditioned[i] = Residual[i] * InverseDiagonal[i];
			const double NextRZ = Dot(Residual, Preconditioned);
			const double Beta = NextRZ / RZ;
			for (size_t i = 0; i < Dimension; ++i) Direction[i] = Preconditioned[i] + Beta * Direction[i];
			RZ = NextRZ;
		}
		ApplyMatrix(Solution, Product, Scratch);
		for (size_t i = 0; i < Dimension; ++i) Residual[i] = RHS[i] - Product[i];
		const double ResidualNorm = std::sqrt(Dot(Residual, Residual));
		mConverged = std::isfinite(ResidualNorm) && ResidualNorm <= Threshold;
		mRelativeResidual = (float)(ResidualNorm / std::max(RHSNorm, (double)mSettings.AbsoluteTolerance));
		for (size_t i = 0; i < mRuntimeBonds.size(); ++i)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				mRuntimeBonds[i].Info.ImpulseLinear[Axis] = (float)Solution[i * 6 + Axis];
				mRuntimeBonds[i].Info.ImpulseAngular[Axis] = (float)Solution[i * 6 + Axis + 3];
			}
		}
		// Scratch holds M^-1 J^T lambda from the final matrix multiply.
		for (size_t i = 0; i < mNodes.size(); ++i)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				mNodes[i].VelocityLinear[Axis] = (float)(NodeVelocity[i * 6 + Axis] + Scratch[i * 6 + Axis]);
				mNodes[i].VelocityAngular[Axis] = (float)(NodeVelocity[i * 6 + Axis + 3] + Scratch[i * 6 + Axis + 3]);
			}
		}
	}

	void StrainSolver::Update(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		for (RuntimeNode& Node : mNodes)
		{
			if (Node.Info.IsStatic || !Node.Enabled)
			{
				Node.VelocityLinear = Node.VelocityAngular = Vector3::Zero();
			}
			else
			{
				Node.VelocityLinear = (Node.Impulse + Node.Force * Dt) / Node.Info.Mass;
				Node.VelocityAngular = Node.InverseInertia * (Node.AngularImpulse + Node.Torque * Dt);
			}
		}
		Solve(StrainSolverSettings::GetIterationsPerFrame(mSettings, (uint32_t)mRuntimeBonds.size()), mSettings.WarmStart && !bResetRequested);
		bResetRequested = false;
		for (RuntimeNode& Node : mNodes) Node.Impulse = Node.AngularImpulse = Node.Force = Node.Torque = Vector3::Zero();
		for (RuntimeBond& Bond : mRuntimeBonds)
		{
			StrainBondInfo& Info = Bond.Info;
			Info.ForceLinear = Info.ImpulseLinear / Dt;
			Info.Torque = Info.ImpulseAngular / Dt;
			const float NormalLoad = Info.ForceLinear.Dot(Info.Normal);
			const float Twist = Info.Torque.Dot(Info.Normal);
			const Vector3 TangentialLoad = Info.ForceLinear - Info.Normal * NormalLoad;
			const Vector3 Bend = Info.Torque - Info.Normal * Twist;
			// Equivalent circular sections: I = A*r^2/4 and polar I = A*r^2/2.
			const float Radius = std::sqrt(Info.Area / 3.14159265358979323846f);
			Info.Bending = 4.0f * Bend.Length() / (Info.Area * Radius);
			Info.Torsion = 2.0f * std::fabs(Twist) / (Info.Area * Radius);
			Info.Tension = std::max(NormalLoad, 0.0f) / Info.Area + Info.Bending;
			Info.Compression = std::max(-NormalLoad, 0.0f) / Info.Area + Info.Bending;
			Info.Shear = TangentialLoad.Length() / Info.Area + Info.Torsion;
			Info.Stress = (mSettings.StressLinearFactor * Info.ForceLinear.Length() / Info.Area
				+ mSettings.StressAngularFactor * std::sqrt(Info.Bending * Info.Bending + Info.Torsion * Info.Torsion)) / mSettings.Hardness;
		}
		PublishBonds();
		CalculateError();
		++mFrameCount;
	}

	void StrainSolver::PublishBonds()
	{
		mBonds.clear();
		mOverstressedBondIndices.clear();
		for (const RuntimeBond& Bond : mRuntimeBonds)
		{
			mBonds.push_back(Bond.Info);
			if (Bond.Info.Stress > Bond.Info.Health) mOverstressedBondIndices.push_back(Bond.Info.BondIndex);
		}
	}

	void StrainSolver::ClearSolution()
	{
		for (RuntimeBond& Bond : mRuntimeBonds) ClearBondResults(Bond.Info);
		mErrorLinear = mErrorAngular = mRelativeResidual = 0.0f;
		mLastIterations = 0;
		mConverged = true;
		bResetRequested = true;
		PublishBonds();
	}

	void StrainSolver::Reset()
	{
		for (RuntimeNode& Node : mNodes)
		{
			Node.Impulse = Node.AngularImpulse = Node.Force = Node.Torque = Vector3::Zero();
			Node.VelocityLinear = Node.VelocityAngular = Vector3::Zero();
		}
		ClearSolution();
		mFrameCount = 0;
	}

	void StrainSolver::CalculateError()
	{
		mErrorLinear = mErrorAngular = 0.0f;
		for (const RuntimeBond& Bond : mRuntimeBonds)
		{
			const RuntimeNode& A = mNodes[(size_t)Bond.Info.Node0];
			const RuntimeNode& B = mNodes[(size_t)Bond.Info.Node1];
			const Vector3 Error = A.VelocityLinear + A.VelocityAngular.Cross(Bond.Offset0)
				- B.VelocityLinear - B.VelocityAngular.Cross(Bond.Offset1);
			mErrorLinear += Error.SquareLength();
			mErrorAngular += (A.VelocityAngular - B.VelocityAngular).SquareLength();
		}
		mErrorLinear = std::sqrt(mErrorLinear);
		mErrorAngular = std::sqrt(mErrorAngular);
	}

	void StrainSolver::GenerateFractureCommands(DestructionFractureBuffer& Commands) const
	{
		Commands.BondFractures = mOverstressedBondIndices;
	}

	const StrainBondInfo* StrainSolver::GetBondInfo(int Index) const
	{
		for (const StrainBondInfo& Bond : mBonds) if (Bond.BondIndex == Index) return &Bond;
		return nullptr;
	}

	float StrainSolver::GetBondStress(int Index) const
	{
		const StrainBondInfo* Bond = GetBondInfo(Index);
		return Bond ? Bond->Stress : 0.0f;
	}

	Vector3 StrainSolver::GetNodeLinearVelocity(int Index) const
	{
		return IsActiveNode(Index) ? mNodes[(size_t)Index].VelocityLinear : Vector3::Zero();
	}

	Vector3 StrainSolver::GetNodeAngularVelocity(int Index) const
	{
		return IsActiveNode(Index) ? mNodes[(size_t)Index].VelocityAngular : Vector3::Zero();
	}
}


#include "StrainSolver.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>

namespace Riemann
{
	namespace
	{
		float SafeInv(float Value)
		{
			return std::fabs(Value) > 1e-6f ? 1.0f / Value : 0.0f;
		}

		float EstimateInvInertia(float InvMass, float Volume)
		{
			if (InvMass <= 0.0f)
			{
				return 0.0f;
			}
			const float Radius = std::pow(std::max(Volume, 1e-6f) * 3.0f / (4.0f * 3.14159265358979323846f), 1.0f / 3.0f);
			return Radius > 1e-6f ? InvMass / (Radius * Radius * 0.4f) : 0.0f;
		}
	}

	StrainSolver::StrainSolver(const ConnectionGraph& Graph, const StrainSolverSettings& Settings)
		: mSettings(Settings)
	{
		ResetGraph(Graph);
	}

	StrainSolver::StrainSolver(const DestructionSet& DestructSet, const StrainSolverSettings& Settings)
		: mSettings(Settings)
	{
		ResetGraph(DestructSet.GetGraph());
		SetAllNodesInfo(DestructSet);
	}

	void StrainSolver::ResetGraph(const ConnectionGraph& Graph)
	{
		mRuntimeBonds.clear();
		mBonds.clear();
		mOverstressedBondIndices.clear();
		EnsureNodeCount(Graph.Size());

		int GeneratedBondIndex = 0;
		for (size_t Node = 0; Node < Graph.Size(); ++Node)
		{
			for (const Bond& ExistingBond : Graph[Node].Bonds)
			{
				const int Other = ExistingBond.GetOther((int)Node);
				if (Other < 0 || (int)Node >= Other)
				{
					continue;
				}

				RuntimeBond Runtime;
				Runtime.Info.Node0 = (int)Node;
				Runtime.Info.Node1 = Other;
				Runtime.Info.BondIndex = ExistingBond.bond_index >= 0 ? ExistingBond.bond_index : GeneratedBondIndex;
				Runtime.Info.Health = std::max(ExistingBond.strength, 1e-3f);
				Runtime.Info.Stress = 0.0f;
				mRuntimeBonds.push_back(Runtime);
				mBonds.push_back(Runtime.Info);
				++GeneratedBondIndex;
			}
		}

		Reset();
	}

	void StrainSolver::SetAllNodesInfo(const DestructionSet& DestructSet, float Density)
	{
		const std::vector<DestructionChunk>& Chunks = DestructSet.GetChunks();
		EnsureNodeCount(Chunks.size());
		for (size_t Index = 0; Index < Chunks.size(); ++Index)
		{
			const DestructionChunk& Chunk = Chunks[Index];
			const float Volume = std::max(Chunk.Volume, 1e-3f);
			const float Mass = Chunk.Mass > 0.0f ? Chunk.Mass : Volume * Density;
			SetNodeInfo((int)Index, Mass, Volume, Chunk.Centroid, Chunk.IsStatic);
		}
	}

	void StrainSolver::SetNodeInfo(int GraphNodeIndex, float Mass, float Volume, const Vector3& LocalPosition, bool bStatic)
	{
		if (GraphNodeIndex < 0)
		{
			return;
		}

		EnsureNodeCount((size_t)GraphNodeIndex + 1);
		RuntimeNode& Node = mNodes[(size_t)GraphNodeIndex];
		Node.Info.Mass = std::max(Mass, 1e-6f);
		Node.Info.Volume = std::max(Volume, 1e-6f);
		Node.Info.LocalPosition = LocalPosition;
		Node.Info.IsStatic = bStatic;

		for (RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
		{
			if (RuntimeBondInfo.Info.Node0 == GraphNodeIndex || RuntimeBondInfo.Info.Node1 == GraphNodeIndex)
			{
				const Vector3 P0 = mNodes[(size_t)RuntimeBondInfo.Info.Node0].Info.LocalPosition;
				const Vector3 P1 = mNodes[(size_t)RuntimeBondInfo.Info.Node1].Info.LocalPosition;
				RuntimeBondInfo.Offset = (P1 - P0) * 0.5f;
				RuntimeBondInfo.InvOffsetSqrLength = SafeInv(std::max(RuntimeBondInfo.Offset.SquareLength(), 1e-6f));
			}
		}
	}

	void StrainSolver::AddForce(int GraphNodeIndex, const Vector3& LocalForce, StrainForceMode Mode)
	{
		if (GraphNodeIndex < 0 || (size_t)GraphNodeIndex >= mNodes.size())
		{
			return;
		}

		RuntimeNode& Node = mNodes[(size_t)GraphNodeIndex];
		if (Node.Info.IsStatic)
		{
			return;
		}

		const Vector3 Impulse = (Mode == StrainForceMode::Impulse) ? LocalForce : LocalForce * Node.Info.Mass;
		Node.Impulse += Impulse;
	}

	bool StrainSolver::AddForce(const DestructionCluster& Cluster, const Vector3& LocalPosition, const Vector3& LocalForce, StrainForceMode Mode)
	{
		const int Node = FindNearestClusterNode(Cluster, LocalPosition);
		if (Node < 0)
		{
			return false;
		}

		AddForce(Node, LocalForce, Mode);
		return true;
	}

	bool StrainSolver::AddGravityForce(const DestructionCluster& Cluster, const Vector3& LocalGravity)
	{
		bool bApplied = false;
		for (int Node : Cluster.GetSourceIndices())
		{
			if (Node >= 0 && (size_t)Node < mNodes.size() && !mNodes[(size_t)Node].Info.IsStatic)
			{
				AddForce(Node, LocalGravity, StrainForceMode::VelocityChange);
				bApplied = true;
			}
		}
		return bApplied;
	}

	bool StrainSolver::AddAngularVelocity(const DestructionCluster& Cluster, const Vector3& LocalCenterMass, const Vector3& LocalAngularVelocity)
	{
		bool bApplied = false;
		for (int Node : Cluster.GetSourceIndices())
		{
			if (Node < 0 || (size_t)Node >= mNodes.size() || mNodes[(size_t)Node].Info.IsStatic)
			{
				continue;
			}

			const Vector3 R = mNodes[(size_t)Node].Info.LocalPosition - LocalCenterMass;
			const Vector3 Tangential = LocalAngularVelocity.Cross(R);
			const Vector3 Centrifugal = LocalAngularVelocity.Cross(Tangential);
			AddForce(Node, Centrifugal, StrainForceMode::VelocityChange);
			bApplied = true;
		}
		return bApplied;
	}

	void StrainSolver::Update()
	{
		for (RuntimeNode& Node : mNodes)
		{
			const float InvMass = Node.Info.IsStatic ? 0.0f : SafeInv(Node.Info.Mass);
			Node.VelocityLinear = Node.Impulse * InvMass;
			Node.VelocityAngular = Vector3::Zero();
		}

		const uint32_t IterationCount = StrainSolverSettings::GetIterationsPerFrame(mSettings, (uint32_t)mRuntimeBonds.size());
		Solve(IterationCount, !bResetRequested);
		bResetRequested = false;

		for (RuntimeNode& Node : mNodes)
		{
			Node.Impulse = Vector3::Zero();
		}

		mOverstressedBondIndices.clear();
		for (size_t BondIndex = 0; BondIndex < mRuntimeBonds.size(); ++BondIndex)
		{
			RuntimeBond& RuntimeBondInfo = mRuntimeBonds[BondIndex];
			const float RawStress = RuntimeBondInfo.ImpulseLinear.Length() * mSettings.StressLinearFactor +
				RuntimeBondInfo.ImpulseAngular.Length() * mSettings.StressAngularFactor;
			RuntimeBondInfo.Info.Stress = RawStress / std::max(mSettings.Hardness, 1e-6f);
			mBonds[BondIndex] = RuntimeBondInfo.Info;
			if (RuntimeBondInfo.Info.Stress > RuntimeBondInfo.Info.Health)
			{
				mOverstressedBondIndices.push_back(RuntimeBondInfo.Info.BondIndex);
			}
		}

		CalculateError();
		++mFrameCount;
	}

	void StrainSolver::GenerateFractureCommands(DestructionFractureBuffer& Commands) const
	{
		Commands.BondFractures = mOverstressedBondIndices;
	}

	void StrainSolver::Reset()
	{
		for (RuntimeNode& Node : mNodes)
		{
			Node.Impulse = Vector3::Zero();
			Node.VelocityLinear = Vector3::Zero();
			Node.VelocityAngular = Vector3::Zero();
		}
		for (RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
		{
			RuntimeBondInfo.ImpulseLinear = Vector3::Zero();
			RuntimeBondInfo.ImpulseAngular = Vector3::Zero();
			RuntimeBondInfo.Info.Stress = 0.0f;
		}
		mOverstressedBondIndices.clear();
		mFrameCount = 0;
		mErrorLinear = 0.0f;
		mErrorAngular = 0.0f;
		bResetRequested = true;
	}

	float StrainSolver::GetBondStress(int BondIndex) const
	{
		for (const StrainBondInfo& BondInfo : mBonds)
		{
			if (BondInfo.BondIndex == BondIndex)
			{
				return BondInfo.Stress;
			}
		}
		return 0.0f;
	}

	void StrainSolver::EnsureNodeCount(size_t NodeCount)
	{
		if (mNodes.size() < NodeCount)
		{
			mNodes.resize(NodeCount);
		}
	}

	int StrainSolver::FindNearestClusterNode(const DestructionCluster& Cluster, const Vector3& LocalPosition) const
	{
		int BestNode = -1;
		float BestDistanceSqr = FLT_MAX;
		for (int Node : Cluster.GetSourceIndices())
		{
			if (Node < 0 || (size_t)Node >= mNodes.size())
			{
				continue;
			}
			const float DistanceSqr = (mNodes[(size_t)Node].Info.LocalPosition - LocalPosition).SquareLength();
			if (DistanceSqr < BestDistanceSqr)
			{
				BestDistanceSqr = DistanceSqr;
				BestNode = Node;
			}
		}
		return BestNode;
	}

	void StrainSolver::Solve(uint32_t IterationCount, bool bWarmStart)
	{
		if (!bWarmStart)
		{
			for (RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
			{
				RuntimeBondInfo.ImpulseLinear = Vector3::Zero();
				RuntimeBondInfo.ImpulseAngular = Vector3::Zero();
			}
		}
		else
		{
			for (RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
			{
				RuntimeNode& Node0 = mNodes[(size_t)RuntimeBondInfo.Info.Node0];
				RuntimeNode& Node1 = mNodes[(size_t)RuntimeBondInfo.Info.Node1];
				const float InvMass0 = Node0.Info.IsStatic ? 0.0f : SafeInv(Node0.Info.Mass);
				const float InvMass1 = Node1.Info.IsStatic ? 0.0f : SafeInv(Node1.Info.Mass);
				const float InvI0 = EstimateInvInertia(InvMass0, Node0.Info.Volume);
				const float InvI1 = EstimateInvInertia(InvMass1, Node1.Info.Volume);

				const Vector3 VelocityLinearCorr0 = RuntimeBondInfo.ImpulseLinear * InvMass0;
				const Vector3 VelocityLinearCorr1 = RuntimeBondInfo.ImpulseLinear * InvMass1;
				const Vector3 VelocityAngularCorr0 = RuntimeBondInfo.ImpulseAngular * InvI0 - RuntimeBondInfo.Offset.Cross(VelocityLinearCorr0) * RuntimeBondInfo.InvOffsetSqrLength;
				const Vector3 VelocityAngularCorr1 = RuntimeBondInfo.ImpulseAngular * InvI1 + RuntimeBondInfo.Offset.Cross(VelocityLinearCorr1) * RuntimeBondInfo.InvOffsetSqrLength;

				Node0.VelocityLinear += VelocityLinearCorr0;
				Node1.VelocityLinear -= VelocityLinearCorr1;
				Node0.VelocityAngular += VelocityAngularCorr0;
				Node1.VelocityAngular -= VelocityAngularCorr1;
			}
		}

		for (uint32_t Iteration = 0; Iteration < IterationCount; ++Iteration)
		{
			for (RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
			{
				RuntimeNode& Node0 = mNodes[(size_t)RuntimeBondInfo.Info.Node0];
				RuntimeNode& Node1 = mNodes[(size_t)RuntimeBondInfo.Info.Node1];
				const float InvMass0 = Node0.Info.IsStatic ? 0.0f : SafeInv(Node0.Info.Mass);
				const float InvMass1 = Node1.Info.IsStatic ? 0.0f : SafeInv(Node1.Info.Mass);
				const float InvI0 = EstimateInvInertia(InvMass0, Node0.Info.Volume);
				const float InvI1 = EstimateInvInertia(InvMass1, Node1.Info.Volume);

				const Vector3 VA = Node0.VelocityLinear - Node0.VelocityAngular.Cross(RuntimeBondInfo.Offset);
				const Vector3 VB = Node1.VelocityLinear + Node1.VelocityAngular.Cross(RuntimeBondInfo.Offset);
				const Vector3 ErrorLinear = VA - VB;
				const Vector3 ErrorAngular = Node0.VelocityAngular - Node1.VelocityAngular;

				const float WeightedMass = SafeInv(InvMass0 + InvMass1);
				const float WeightedInertia = SafeInv(InvI0 + InvI1);
				if (WeightedMass <= 0.0f && WeightedInertia <= 0.0f)
				{
					continue;
				}

				const Vector3 OutImpulseLinear = -ErrorLinear * WeightedMass * 0.5f;
				const Vector3 OutImpulseAngular = -ErrorAngular * WeightedInertia * 0.5f;
				RuntimeBondInfo.ImpulseLinear += OutImpulseLinear;
				RuntimeBondInfo.ImpulseAngular += OutImpulseAngular;

				const Vector3 VelocityLinearCorr0 = OutImpulseLinear * InvMass0;
				const Vector3 VelocityLinearCorr1 = OutImpulseLinear * InvMass1;
				const Vector3 VelocityAngularCorr0 = OutImpulseAngular * InvI0 - RuntimeBondInfo.Offset.Cross(VelocityLinearCorr0) * RuntimeBondInfo.InvOffsetSqrLength;
				const Vector3 VelocityAngularCorr1 = OutImpulseAngular * InvI1 + RuntimeBondInfo.Offset.Cross(VelocityLinearCorr1) * RuntimeBondInfo.InvOffsetSqrLength;

				Node0.VelocityLinear += VelocityLinearCorr0;
				Node1.VelocityLinear -= VelocityLinearCorr1;
				Node0.VelocityAngular += VelocityAngularCorr0;
				Node1.VelocityAngular -= VelocityAngularCorr1;
			}
		}
	}

	void StrainSolver::CalculateError()
	{
		mErrorLinear = 0.0f;
		mErrorAngular = 0.0f;
		for (const RuntimeBond& RuntimeBondInfo : mRuntimeBonds)
		{
			const RuntimeNode& Node0 = mNodes[(size_t)RuntimeBondInfo.Info.Node0];
			const RuntimeNode& Node1 = mNodes[(size_t)RuntimeBondInfo.Info.Node1];
			const Vector3 VA = Node0.VelocityLinear - Node0.VelocityAngular.Cross(RuntimeBondInfo.Offset);
			const Vector3 VB = Node1.VelocityLinear + Node1.VelocityAngular.Cross(RuntimeBondInfo.Offset);
			mErrorLinear += (VA - VB).Length();
			mErrorAngular += (Node0.VelocityAngular - Node1.VelocityAngular).Length();
		}
	}

}	// namespace Riemann

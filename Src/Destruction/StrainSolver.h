#pragma once

#include <cstdint>
#include <vector>

#include "ConnectionGraph.h"
#include "DestructionCluster.h"
#include "DestructionSet.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	enum class StrainForceMode
	{
		Impulse,
		VelocityChange,
	};

	struct StrainSolverSettings
	{
		float Hardness = 1000.0f;
		float StressLinearFactor = 0.25f;
		float StressAngularFactor = 0.75f;
		uint32_t BondIterationsPerFrame = 18000;
		uint32_t GraphReductionLevel = 0;

		static uint32_t GetIterationsPerFrame(const StrainSolverSettings& Settings, uint32_t BondCount)
		{
			const uint32_t PerFrame = Settings.BondIterationsPerFrame / (BondCount + 1);
			return PerFrame > 0 ? PerFrame : 1;
		}
	};

	struct StrainNodeInfo
	{
		float Mass = 1.0f;
		float Volume = 1.0f;
		Vector3 LocalPosition = Vector3::Zero();
		bool IsStatic = false;
	};

	struct StrainBondInfo
	{
		int Node0 = -1;
		int Node1 = -1;
		int BondIndex = -1;
		float Health = 1.0f;
		float Stress = 0.0f;
	};

	class StrainSolver
	{
	public:
		StrainSolver() = default;
		explicit StrainSolver(const ConnectionGraph& Graph, const StrainSolverSettings& Settings = StrainSolverSettings());
		explicit StrainSolver(const DestructionSet& DestructSet, const StrainSolverSettings& Settings = StrainSolverSettings());

		void ResetGraph(const ConnectionGraph& Graph);
		void SetAllNodesInfo(const DestructionSet& DestructSet, float Density = 1.0f);
		void SetNodeInfo(int GraphNodeIndex, float Mass, float Volume, const Vector3& LocalPosition, bool bStatic);

		void SetSettings(const StrainSolverSettings& Settings) { mSettings = Settings; }
		const StrainSolverSettings& GetSettings() const { return mSettings; }

		void AddForce(int GraphNodeIndex, const Vector3& LocalForce, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddForce(const DestructionCluster& Cluster, const Vector3& LocalPosition, const Vector3& LocalForce, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddGravityForce(const DestructionCluster& Cluster, const Vector3& LocalGravity);
		bool AddAngularVelocity(const DestructionCluster& Cluster, const Vector3& LocalCenterMass, const Vector3& LocalAngularVelocity);

		void Update();
		void GenerateFractureCommands(DestructionFractureBuffer& Commands) const;
		void Reset();

		uint32_t GetOverstressedBondCount() const { return (uint32_t)mOverstressedBondIndices.size(); }
		uint32_t GetFrameCount() const { return mFrameCount; }
		uint32_t GetBondCount() const { return (uint32_t)mBonds.size(); }
		float GetStressErrorLinear() const { return mErrorLinear; }
		float GetStressErrorAngular() const { return mErrorAngular; }
		float GetBondStress(int BondIndex) const;

		const std::vector<StrainBondInfo>& GetBonds() const { return mBonds; }
		const std::vector<int>& GetOverstressedBondIndices() const { return mOverstressedBondIndices; }

	private:
		struct RuntimeNode
		{
			StrainNodeInfo Info;
			Vector3 Impulse = Vector3::Zero();
			Vector3 VelocityLinear = Vector3::Zero();
			Vector3 VelocityAngular = Vector3::Zero();
		};

		struct RuntimeBond
		{
			StrainBondInfo Info;
			Vector3 Offset = Vector3::Zero();
			Vector3 ImpulseLinear = Vector3::Zero();
			Vector3 ImpulseAngular = Vector3::Zero();
			float InvOffsetSqrLength = 1.0f;
		};

		void EnsureNodeCount(size_t NodeCount);
		int FindNearestClusterNode(const DestructionCluster& Cluster, const Vector3& LocalPosition) const;
		void Solve(uint32_t IterationCount, bool bWarmStart);
		void CalculateError();

	private:
		StrainSolverSettings mSettings;
		std::vector<RuntimeNode> mNodes;
		std::vector<RuntimeBond> mRuntimeBonds;
		std::vector<StrainBondInfo> mBonds;
		std::vector<int> mOverstressedBondIndices;
		uint32_t mFrameCount = 0;
		bool bResetRequested = false;
		float mErrorLinear = 0.0f;
		float mErrorAngular = 0.0f;
	};

}	// namespace Riemann

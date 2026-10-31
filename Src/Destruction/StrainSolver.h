#pragma once

#include <cstdint>
#include <vector>

#include "ConnectionGraph.h"
#include "DestructionCluster.h"
#include "DestructionSet.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
	enum class StrainForceMode
	{
		Impulse,
		VelocityChange,
		Force,
		Acceleration,
	};

	struct StrainSolverSettings
	{
		float Hardness = 1000.0f;
		float StressLinearFactor = 0.25f;
		float StressAngularFactor = 0.75f;
		uint32_t BondIterationsPerFrame = 18000;
		uint32_t GraphReductionLevel = 0; // Reserved for source compatibility; bonds are solved without reduction.
		uint32_t MaxIterations = 0;
		float RelativeTolerance = 1.0e-6f;
		float AbsoluteTolerance = 1.0e-7f;
		bool WarmStart = true;

		static uint32_t GetIterationsPerFrame(const StrainSolverSettings& Settings, uint32_t BondCount)
		{
			if (Settings.MaxIterations > 0) return Settings.MaxIterations;
			const uint32_t PerFrame = (uint32_t)((uint64_t)Settings.BondIterationsPerFrame / ((uint64_t)BondCount + 1));
			return PerFrame > 0 ? PerFrame : 1;
		}
	};

	struct StrainNodeInfo
	{
		float Mass = 1.0f;
		float Volume = 1.0f;
		Vector3 LocalPosition = Vector3::Zero();
		bool IsStatic = false;
		Matrix3 Inertia = Matrix3::Identity();
	};

	struct StrainBondInfo
	{
		int Node0 = -1;
		int Node1 = -1;
		int BondIndex = -1;
		float Health = 1.0f;
		float Stress = 0.0f;
		Vector3 Centroid = Vector3::Zero();
		Vector3 Normal = Vector3::UnitX();
		float Area = 1.0f;
		Vector3 ImpulseLinear = Vector3::Zero();
		Vector3 ImpulseAngular = Vector3::Zero();
		Vector3 ForceLinear = Vector3::Zero();
		Vector3 Torque = Vector3::Zero();
		float Tension = 0.0f;
		float Compression = 0.0f;
		float Shear = 0.0f;
		float Bending = 0.0f;
		float Torsion = 0.0f;
	};

	class StrainSolver
	{
	public:
		StrainSolver() = default;
		explicit StrainSolver(const ConnectionGraph& Graph, const StrainSolverSettings& Settings = StrainSolverSettings());
		explicit StrainSolver(const DestructionSet& DestructSet, const StrainSolverSettings& Settings = StrainSolverSettings());

		void ResetGraph(const ConnectionGraph& Graph);
		// Reconcile topology without discarding pending node loads or surviving bond geometry.
		uint32_t SyncGraph(const ConnectionGraph& Graph);
		void SetAllNodesInfo(const DestructionSet& DestructSet, float Density = 1.0f);
		void SetNodeInfo(int GraphNodeIndex, float Mass, float Volume, const Vector3& LocalPosition, bool bStatic);
		bool SetNodeInertia(int GraphNodeIndex, const Matrix3& Inertia);
		bool SetBondGeometry(int BondIndex, const Vector3& Centroid, const Vector3& Normal, float Area);
		bool SetBondHealth(int BondIndex, float Health);
		bool RemoveBond(int BondIndex);

		void SetSettings(const StrainSolverSettings& Settings);
		const StrainSolverSettings& GetSettings() const { return mSettings; }

		void AddForce(int GraphNodeIndex, const Vector3& LocalForce, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddTorque(int GraphNodeIndex, const Vector3& LocalTorque, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddForceAtPosition(int GraphNodeIndex, const Vector3& LocalPosition, const Vector3& LocalForce, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddForce(const DestructionCluster& Cluster, const Vector3& LocalPosition, const Vector3& LocalForce, StrainForceMode Mode = StrainForceMode::Impulse);
		bool AddGravityForce(const DestructionCluster& Cluster, const Vector3& LocalGravity);
		bool AddAngularVelocity(const DestructionCluster& Cluster, const Vector3& LocalCenterMass, const Vector3& LocalAngularVelocity);

		// Force/acceleration inputs are integrated over Dt; reported stresses are force per area.
		void Update(float Dt = 1.0f);
		void GenerateFractureCommands(DestructionFractureBuffer& Commands) const;
		void Reset();

		uint32_t GetOverstressedBondCount() const { return (uint32_t)mOverstressedBondIndices.size(); }
		uint32_t GetFrameCount() const { return mFrameCount; }
		uint32_t GetBondCount() const { return (uint32_t)mBonds.size(); }
		uint32_t GetNodeCount() const { return (uint32_t)mNodes.size(); }
		uint32_t GetLastIterationCount() const { return mLastIterations; }
		bool HasConverged() const { return mConverged; }
		float GetRelativeResidual() const { return mRelativeResidual; }
		float GetStressErrorLinear() const { return mErrorLinear; }
		float GetStressErrorAngular() const { return mErrorAngular; }
		float GetBondStress(int BondIndex) const;
		const StrainBondInfo* GetBondInfo(int BondIndex) const;
		Vector3 GetNodeLinearVelocity(int GraphNodeIndex) const;
		Vector3 GetNodeAngularVelocity(int GraphNodeIndex) const;

		const std::vector<StrainBondInfo>& GetBonds() const { return mBonds; }
		const std::vector<int>& GetOverstressedBondIndices() const { return mOverstressedBondIndices; }

	private:
		struct RuntimeNode
		{
			StrainNodeInfo Info;
			Vector3 Impulse = Vector3::Zero();
			Vector3 AngularImpulse = Vector3::Zero();
			Vector3 Force = Vector3::Zero();
			Vector3 Torque = Vector3::Zero();
			Vector3 VelocityLinear = Vector3::Zero();
			Vector3 VelocityAngular = Vector3::Zero();
			Matrix3 InverseInertia = Matrix3::Identity();
			bool Enabled = true;
			bool CustomInertia = false;
			bool InfoInitialized = false;
		};

		struct RuntimeBond
		{
			StrainBondInfo Info;
			Vector3 Offset0 = Vector3::Zero();
			Vector3 Offset1 = Vector3::Zero();
			bool CustomGeometry = false;
			float GraphHealth = 0.0f;
		};

		void EnsureNodeCount(size_t NodeCount);
		bool IsActiveNode(int GraphNodeIndex) const;
		void UpdateBondGeometry();
		void ClearSolution();
		void PublishBonds();
		int FindNearestClusterNode(const DestructionCluster& Cluster, const Vector3& LocalPosition) const;
		void Solve(uint32_t IterationCount, bool bWarmStart);
		void CalculateError();
		void ApplyJacobian(const std::vector<double>& Nodes, std::vector<double>& Bonds) const;
		void ApplyImpulseResponse(const std::vector<double>& Bonds, std::vector<double>& Nodes) const;
		void ApplyMatrix(const std::vector<double>& Input, std::vector<double>& Output, std::vector<double>& NodeScratch) const;

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
		uint32_t mLastIterations = 0;
		bool mConverged = true;
		float mRelativeResidual = 0.0f;
	};

}	// namespace Riemann

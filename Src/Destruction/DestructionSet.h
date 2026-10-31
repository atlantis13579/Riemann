#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ConnectionGraph.h"
#include "DestructionCluster.h"
#include "../Core/BitSet.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	class Geometry;
	class ConvexMesh;
	class DynamicMesh;
	class PhysicsWorld;
	struct PhysicsContactInfo;
	struct PhysicsContactPoint;
	class RigidBody;
	class RigidBodyDynamic;
	class StrainSolver;
	struct StrainSolverSettings;

	enum class DestructionEventType : int
	{
		Broken = -1,
		Other = 0,
		SelfContact = 1,
		SelfCollapse = 2,
		SelfBreak = 3,
		Destruct = 4,
		Break = 5,
		Count = 6,
	};

	enum class DestructionPartitionAlgorithm
	{
		None = 0,
		Metis = 1,
	};

	struct DestructionClusterLevelConstants
	{
		float BreakThreshold = 10.0f;
		int PartitionSize = 2;
	};

	struct DestructionConstants
	{
		float ImpulseCollapseStrength = 10.0f;
		float ImpulseBreakStrength = 0.5f;
		float ImpulseRadius = 10.0f;
		int ImpulseProcessPerTick = 1;
		int PartitionProcessPerTick = 4;                // Maximum queued cluster partitions processed in one tick.
		int PartitionProcessIntervalTicks = 1;          // Tick interval between partition batches.
		float PartitionSeparationAttenuation = 0.5f;    // Separation velocity multiplier applied at each deeper level.
		bool EnableClusterCollisionProxy = true;         // Use one aggregate collision shape until a cluster becomes a leaf.
		DestructionPartitionAlgorithm PartitionAlgorithm = DestructionPartitionAlgorithm::Metis;
		DestructionClusterLevelConstants Levels[8];
		float IdleTime = 30.0f;
		float DamagePropagationStrength = 0.1f;
		float DamagePropagationAttenuation = 0.1f;
		float MinClusterVolume = 10.0f;
		bool EnableSupportGroup = false;
		bool EnableHingeJoint = true;
		bool DebugLog = false;

		DestructionConstants();
	};

	struct DestructionMaterial
	{
		uint32_t TypeID = 0;
		std::string Name;
		float Density = 1.0f;
		float StaticFriction = 0.5f;
		float DynamicFriction = 0.5f;
		float Restitution = 0.0f;
		float LinearDamping = 0.5f;
		float AngularDamping = 0.5f;
		float Strength = 1.0f;
	};

	struct DestructionChunk
	{
		std::string Name;
		Box3 Bounds = Box3::Empty();
		Vector3 Centroid = Vector3::Zero();
		float Volume = 1.0f;
		float Mass = 1.0f;
		float Density = 1.0f;
		float CollapseStrength = 100000.0f;
		bool IsStatic = false;
		bool IsFree = false;
		bool IsSmashed = false;
		std::shared_ptr<ConvexMesh> CollisionConvex;
	};

	struct DestructionFractureLevel
	{
		int MinPieceCount = 2;
		int MaxPieceCount = 2;
		int PreserveLargestPieces = 0;          // Largest generated children kept as permanent leaves.
		float ChildFractureProbability = 1.0f;  // Chance that each remaining child is cut at the next level.
		float BreakThreshold = 1.0f;
		float StrengthVariation = 0.0f;
		bool UseClusteredVoronoi = true;
	};

	struct DestructionFractureSettings
	{
		int RandomSeed = 17;
		int MaxChunkCount = 256;
		int CutRetryCount = 3;
		float MinChunkVolume = 0.01f;
		float BondStrength = 1.0f;
		float DamageAttenuation = 0.6f;
		float NeighborDamageFraction = 0.15f;
		float Grout = 0.0f;
		std::vector<DestructionFractureLevel> Levels;
	};

	struct DestructionHierarchyNode
	{
		int Parent = -1;
		int Level = 0;
		std::vector<int> Children;
		std::vector<int> ChunkIndices;
		Box3 Bounds = Box3::Empty();
		float BreakThreshold = 1.0f;
		bool IsBreakable = false;
		bool IsBroken = false;
	};

	struct DestructionMomentum
	{
		int Index = -1;
		Vector3 Momentum = Vector3::Zero();
		Vector3 Position = Vector3::Zero();
		Vector3 Normal = Vector3::UnitY();
	};

	struct DestructionFractureBuffer
	{
		std::vector<int> BondFractures;
	};

	class DestructionSet : public GeometryAggregate
	{
	public:
		DestructionSet();
		explicit DestructionSet(PhysicsWorld* Simulation);
		explicit DestructionSet(PhysicsWorld& Simulation);
		DestructionSet(const DestructionSet&) = delete;
		DestructionSet& operator=(const DestructionSet&) = delete;
		DestructionSet(DestructionSet&&) = delete;
		DestructionSet& operator=(DestructionSet&&) = delete;
		virtual ~DestructionSet();

		struct Stats
		{
			uint32_t NormalCount = 0;
			uint32_t StaticCount = 0;
			uint32_t FreeCount = 0;
			uint32_t SmashedCount = 0;
		};

		int AddChunk(const Box3& Bounds, float Mass = 1.0f, bool bStatic = false, const std::string& Name = std::string());
		int AddChunk(const DynamicMesh& Mesh, float Mass = 1.0f, bool bStatic = false, const std::string& Name = std::string());
		bool BuildHierarchicalFracture(const DynamicMesh& SourceMesh, const DestructionFractureSettings& Settings,
			float TotalMass = 1.0f, bool bStatic = false, const std::string& Name = std::string());
		void SetChunks(const std::vector<DestructionChunk>& Chunks);
		void Clear();

		bool AddBond(int NodeA, int NodeB, DestructionBondType Type = DestructionBondType::Connect, float Strength = 1.0f, int BondIndex = -1);
		bool SetBondStrength(int NodeA, int NodeB, float Strength);
		void RemoveAllBonds();

		uint32_t AddCluster(int Level, const std::vector<int>& Indices);
		bool RemoveCluster(uint32_t ClusterID);
		void ClearClusters();
		void RebuildClustersFromGraph();

		DestructionCluster* FindClusterContaining(int ChunkIndex);
		const DestructionCluster* FindClusterContaining(int ChunkIndex) const;
		DestructionCluster* GetCluster(uint32_t ClusterID);
		const DestructionCluster* GetCluster(uint32_t ClusterID) const;
		std::vector<DestructionCluster*> GetClusters();
		uint32_t GetClusterRevision() const;

		void BreakBondEx(int Index);
		bool ApplyMomentum(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		void AddMomentumQueue(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		void ProcessMomentumQueue();

		void AddDestructionInfo(DestructionEventType Type, const std::vector<int>& Indices);
		uint32_t ProcessDestructionInfo();
		void DoFractures(const DestructionFractureBuffer& Commands);
		virtual void OnContact(const PhysicsContactInfo& ContactInfo) override;
		void Update(float DeltaTime);
		void EnableStressSolver(const StrainSolverSettings& Settings, const Vector3& WorldGravity = Vector3::Zero());
		void DisableStressSolver();
		StrainSolver* GetStressSolver() { return mStressSolver.get(); }
		bool AddStressImpulse(int Index, const Vector3& WorldPosition, const Vector3& WorldImpulse);

		BitSet FindUnsupported() const;
		std::vector<int> PruneUnsupported();
		std::vector<int> QueryChunksRadius(const Vector3& Center, float Radius) const;

		void RecalculateBounds();
		Stats GetStats() const;
		uint32_t AllocClusterID() { return mNextClusterID++; }
		PhysicsWorld* GetPhysicsWorld() const { return mPhysicsWorld; }

		std::vector<float> GetMasses() const;
		float GetVolume(int Index) const;

		ConnectionGraph& GetGraph() { return mGraph; }
		const ConnectionGraph& GetGraph() const { return mGraph; }
		const std::vector<DestructionChunk>& GetChunks() const { return mChunks; }
		std::vector<DestructionChunk>& GetChunks() { return mChunks; }
		const std::vector<DestructionHierarchyNode>& GetHierarchy() const { return mHierarchyNodes; }
		int GetHierarchyRoot() const { return mHierarchyRoot; }
		const Box3& GetBounds() const { return mBounds; }
		DestructionConstants& GetDestructionConstants() { return mConstants; }
		const DestructionConstants& GetDestructionConstants() const { return mConstants; }

	public:
		ConnectionGraph mGraph;
		Box3 mBounds = Box3::Empty();

	private:
		friend class PhysicsWorld;
		void SetPhysicsWorld(PhysicsWorld* Simulation) { mPhysicsWorld = Simulation; }
		void BuildClusterRigidBody(DestructionCluster& Cluster);
		void ReleaseClusterRigidBody(DestructionCluster& Cluster, bool bReleaseGeometries);
		void ReleaseClusterRigidBodies(bool bReleaseGeometries);
		bool PartitionClusterOnce(const std::vector<int>& Indices, int Level, float Strength, std::vector<std::vector<int>>& Components);
		void QueuePartitionChildren(const std::vector<std::vector<int>>& Components, int Level, float Strength, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		bool MergePendingPartition(const DestructionCluster& Cluster, float Strength, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		bool MergePendingHierarchyPartition(int HierarchyNode, int ImpactChunkIndex, float Strength,
			const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		void ProcessPartitionQueue();
		void ReconcileHierarchyWithGraph();
		void BreakHierarchyCrossChildBonds(int HierarchyNode);
		int FindActiveHierarchyNode(int ChunkIndex) const;
		bool BreakHierarchyNode(int HierarchyNode, float Strength);
		void QueueHierarchyChildren(int HierarchyNode, int ImpactChunkIndex, float Strength,
			const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		bool ApplyHierarchyMomentum(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal);
		std::vector<std::vector<int>> FindConnectedComponents(const std::vector<int>& Indices) const;
		void ApplyFractureSeparation(const std::vector<int>& Indices, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal, float SeparationScale);
		Box3 GetChunkWorldBounds(int Index) const;
		void MarkFree(int Index);
		bool BreakBondByIndex(int BondIndex);
		bool HasClusterRigidBodies() const;
		DestructionCluster* FindClusterByRigidBody(const RigidBody* Body);
		void QueueContactMomentum(RigidBody* Body, Geometry* Geom, const PhysicsContactPoint& Point, const Vector3& Momentum, const Vector3& Normal);

	private:
		struct PendingPartition
		{
			std::vector<int> Indices;
			int HierarchyNode = -1;
			int ImpactChunkIndex = -1;
			int Level = 0;
			float Strength = 0.0f;
			Vector3 Momentum = Vector3::Zero();
			Vector3 Position = Vector3::Zero();
			Vector3 Normal = Vector3::UnitY();
		};

		std::vector<DestructionChunk> mChunks;
		std::vector<DestructionHierarchyNode> mHierarchyNodes;
		std::vector<std::vector<std::pair<int, int>>> mHierarchyBoundaryBonds;
		std::vector<int> mChunkHierarchyLeaves;
		int mHierarchyRoot = -1;
		float mHierarchyDamageAttenuation = 0.6f;
		float mHierarchyNeighborDamageFraction = 0.15f;
		std::map<uint32_t, DestructionCluster> mClusters;
		std::vector<DestructionMomentum> mMomentumQueue;
		std::deque<PendingPartition> mPartitionQueue;
		int mPartitionTicksUntilProcess = 0;
		std::vector<int> mBrokenIndices;
		BitSet mDestructionInfo[(int)DestructionEventType::Count];
		bool bHasDestructionInfo = false;
		uint32_t mNextClusterID = 1;
		PhysicsWorld* mPhysicsWorld = nullptr;
		DestructionConstants mConstants;
		ConnectionGraph BuildStressGraph() const;
		void EnsureStressBondIndices();
		std::unique_ptr<StrainSolver> mStressSolver;
		std::vector<DestructionMomentum> mStressImpulses;
		Vector3 mStressGravity = Vector3::Zero();
	};

}	// namespace Riemann

#pragma once

#include <cstddef>
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
#include "../Collision/GeometryAggregate.h"
#include "../Collision/GeometryObject.h"
#include "../Maths/Box3.h"
#include "../Maths/Transform.h"
#include "../Maths/Vector3.h"
#include "../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	class DestructionSet;
	struct DestructionAssetSettings
	{
		float RelativeGeometryTolerance = 1e-5f;
		float MinimumLength = 1e-6f;
		float QuaternionTolerance = 1e-4f;
		bool IsValid() const;
	};
	struct DestructionAssetChunk
	{
		std::string Name;
		Box3 Bounds = Box3::Empty();
		Vector3 Centroid = Vector3::Zero();
		float Volume = 0, Mass = 0, Density = 0, CollapseStrength = 0;
		bool IsStatic = false;
	};
	struct DestructionAssetStats
	{
		size_t BondCount = 0, HierarchyNodes = 0, ReorientedFaces = 0;
		double TotalMass = 0, TotalVolume = 0;
		Box3 Bounds = Box3::Empty();
		uint64_t Fingerprint = 0;
	};
	class DestructionAsset
	{
	public:
		static DestructionAsset Capture(const DestructionSet& Source, const DestructionAssetSettings& Settings = {});
		bool IsValid() const { return m_Data != nullptr; }
		const std::vector<DestructionAssetChunk>& GetChunks() const;
		const DestructionAssetStats& GetStats() const;
	private:
		friend class DestructionSet;
		struct Storage;
		std::shared_ptr<const Storage> m_Data;
		const Storage& Data() const;
	};
}


namespace Riemann
{
	class DestructionSet;
	class DestructionCluster;
	struct PhysicsContactPoint;
	enum class DestructionEventKind : uint8_t
	{
		Contact, BondBroken, HierarchyBroken, ChunkReleased, ChunkSmashed, ClusterChanged, Count
	};
	enum class DestructionEventCause : uint8_t
	{
		GraphEdit, Command, Momentum, Partition, Stress, LegacyCommand, Unsupported, Contact, Field
	};
	struct DestructionEventSettings
	{
		static constexpr uint32_t AllKinds = (1u << (uint32_t)DestructionEventKind::Count) - 1;
		bool Enabled = false;
		uint32_t KindMask = AllKinds;
		size_t MaximumQueuedEvents = 2048;
		float MinimumContactImpulse = 0.1f;
		bool IsValid() const;
	};
	struct DestructionEventChunk
	{
		int ChunkIndex = -1;
		uint32_t ClusterId = 0;
		Transform Pose = Transform::Identity();
		Vector3 LinearVelocity = Vector3::Zero(), AngularVelocity = Vector3::Zero();
		float Mass = 0;
		bool IsStatic = false, IsFree = false, IsSmashed = false, HasRigidBody = false;
	};
	struct DestructionEventFragment
	{
		uint32_t ClusterId = 0;
		std::vector<int> ChunkIndices;
	};
	struct DestructionEvent
	{
		DestructionEventKind Kind = DestructionEventKind::Contact;
		DestructionEventCause Cause = DestructionEventCause::GraphEdit;
		uint64_t InstanceId = 0, Generation = 0, Sequence = 0, AssetFingerprint = 0;
		uint64_t WorldTick = 0, UpdateIndex = 0;
		int SubstepIndex = 0, SubstepCount = 1;
		float DeltaTime = 0;
		int BondIndex = -1, NodeA = -1, NodeB = -1, HierarchyNode = -1;
		DestructionBondType BondType = DestructionBondType::Invalid;
		float BondStrength = 0;
		uint32_t PreviousClusterId = 0;
		Vector3 Position = Vector3::Zero(), Normal = Vector3::Zero(), Impulse = Vector3::Zero();
		bool ImpulseEstimated = false;
		std::vector<DestructionEventChunk> Chunks;
		std::vector<DestructionEventFragment> Fragments;
	};
	struct DestructionEventStats
	{
		uint64_t Generated = 0, Filtered = 0, Dropped = 0, Invalid = 0;
		size_t Pending = 0;
	};
	class DestructionEvents
	{
	public:
		DestructionEvents(const DestructionSet& Set, const DestructionEventSettings& Settings);
		~DestructionEvents();
		DestructionEvents(const DestructionEvents&) = delete;
		DestructionEvents& operator=(const DestructionEvents&) = delete;
	private:
		friend class DestructionSet;
		void Reset(const DestructionSet& Set);
		void RefreshKinematics(const DestructionSet& Set);
		void Collect(DestructionSet& Set);
		void Contact(DestructionSet& Set, const DestructionCluster& Cluster, int Chunk, const PhysicsContactPoint& Point, const Vector3& Impulse, const Vector3& Normal, bool ImpulseEstimated);
		std::vector<DestructionEvent> Take();
		DestructionEventStats Stats() const;
		struct Storage;
		std::unique_ptr<Storage> m_Data;
	};
}


namespace Riemann
{
	class DestructionSet;
	struct DestructionConstants;
	struct DestructionCacheSettings
	{
		bool RecordEvents = true;
		size_t MaximumFrames = 18000;
		size_t MaximumPayloadBytes = 256u * 1024u * 1024u;
		float QuaternionTolerance = 1e-4f;
		float RelativeGeometryTolerance = 1e-5f, MinimumGeometryTolerance = 1e-6f;
		bool IsValid() const;
	};
	struct DestructionCacheCluster
	{
		uint32_t SourceId = 0;
		int Level = 0;
		bool IsStatic = false;
		std::vector<int> ChunkIndices;
		std::vector<Transform> LocalPoses;
	};
	struct DestructionCacheTopology
	{
		ConnectionGraph Graph;
		std::vector<DestructionCacheCluster> Clusters;
		std::vector<bool> BrokenHierarchy;
	};
	struct DestructionCacheClusterState
	{
		Transform Pose = Transform::Identity();
		Vector3 LinearVelocity = Vector3::Zero(), AngularVelocity = Vector3::Zero();
		float FrictionStatic = 0, FrictionDynamic = 0, Restitution = 0;
		bool Sleeping = false, Frozen = false;
	};
	struct DestructionCacheFrame
	{
		double Time = 0;
		uint64_t SourceWorldTick = 0;
		std::shared_ptr<const DestructionCacheTopology> Topology;
		std::vector<DestructionEventChunk> Chunks;
		std::vector<DestructionCacheClusterState> Clusters;
		std::vector<DestructionEvent> Events;
	};
	struct DestructionCacheSample
	{
		double Time = 0;
		size_t LowerFrame = 0, UpperFrame = 0;
		float Interpolation = 0;
		std::shared_ptr<const DestructionCacheTopology> Topology;
		std::vector<DestructionEventChunk> Chunks;
		std::vector<DestructionCacheClusterState> Clusters;
	};
	struct DestructionCacheStats
	{
		size_t FrameCount = 0, TopologyCount = 0, EventCount = 0, PayloadBytes = 0;
		double Duration = 0;
	};
	struct DestructionCacheEvent
	{
		double Time = 0;
		DestructionEvent Event;
	};
	class DestructionCache
	{
	public:
		bool IsValid() const { return m_Data != nullptr; }
		const DestructionAsset& GetAsset() const;
		const DestructionConstants& GetConstants() const;
		const DestructionCacheSettings& GetSettings() const;
		const DestructionEventSettings& GetEventSettings() const;
		const DestructionCacheStats& GetStats() const;
		const DestructionCacheFrame& GetFrame(size_t Index) const;
		uint64_t GetSourceInstanceId() const;
		uint64_t GetSourceGeneration() const;
		DestructionCacheSample Sample(double Time, const Transform& Placement = Transform::Identity()) const;
		std::vector<DestructionCacheEvent> GetEvents(double After, double Through, const Transform& Placement = Transform::Identity()) const;
	private:
		friend class DestructionCacheRecorder;
		struct Storage;
		std::shared_ptr<const Storage> m_Data;
		const Storage& Data() const;
	};
	class DestructionCacheRecorder
	{
	public:
		explicit DestructionCacheRecorder(const DestructionSet& Source, const DestructionCacheSettings& Settings = {});
		~DestructionCacheRecorder();
		DestructionCacheRecorder(const DestructionCacheRecorder&) = delete;
		DestructionCacheRecorder& operator=(const DestructionCacheRecorder&) = delete;
		void Record(double Time, const DestructionSet& Source, const std::vector<DestructionEvent>& Events = {});
		DestructionCacheStats GetStats() const;
		DestructionCache Finish();
	private:
		struct Storage;
		std::unique_ptr<Storage> m_Data;
	};
}


namespace Riemann
{
	class PhysicsWorld;
	class RigidBody;
	class Geometry;
	class DestructionSet;
	enum class DestructionPlaybackCommand { None, Replay, TogglePause, Seek };
	struct DestructionPlaybackSettings
	{
		bool CollisionEnabled = true, QueryEnabled = true;
		double Rate = 1;
		size_t MaximumQueuedEvents = 2048;
		bool IsValid() const;
	};
	struct DestructionPlaybackEvent
	{
		uint64_t PlaybackId = 0, Epoch = 0;
		DestructionCacheEvent Source;
	};
	class DestructionPlayback final : public GeometryAggregate
	{
	public:
		DestructionPlayback(PhysicsWorld& World, const DestructionCache& Cache,
			const Transform& Placement = Transform::Identity(), const DestructionPlaybackSettings& Settings = {});
		~DestructionPlayback();
		DestructionPlayback(const DestructionPlayback&) = delete;
		DestructionPlayback& operator=(const DestructionPlayback&) = delete;
		void SetPlaying(bool Playing);
		void Seek(double Time);
		void Detach();
		bool IsAttached() const { return m_World != nullptr; }
		bool IsPlaying() const { return m_Playing; }
		double GetTime() const { return m_Sample.Time; }
		uint64_t GetInstanceId() const { return m_InstanceId; }
		uint64_t GetEpoch() const { return m_Epoch; }
		uint64_t GetRevision() const { return m_Revision; }
		uint64_t GetDroppedEventCount() const { return m_DroppedEvents; }
		const DestructionCache& GetCache() const { return m_Cache; }
		const DestructionCacheSample& GetSample() const { return m_Sample; }
		Geometry* GetChunkGeometry(int Chunk) const;
		size_t GetBodyCount() const { return m_Bodies.size(); }
		RigidBody* GetBody(size_t Index) const;
		std::vector<DestructionPlaybackEvent> TakeEvents();
	private:
		friend class PhysicsWorld;
		struct Binding { RigidBody* Body = nullptr; std::vector<int> Chunks; };
		void RequireIdle() const;
		void Step(float Dt);
		void Apply(DestructionCacheSample Sample, bool Moving);
		void Rebuild(const DestructionCacheSample& Sample);
		void ClearBodies(std::vector<Binding>& Bodies);
		void StopVelocities();
		void OnBodyRemoved(RigidBody* Body);
		PhysicsWorld* m_World = nullptr;
		DestructionCache m_Cache;
		Transform m_Placement;
		DestructionPlaybackSettings m_Settings;
		std::unique_ptr<DestructionSet> m_Rest;
		DestructionCacheSample m_Sample;
		std::vector<Binding> m_Bodies;
		std::vector<Geometry*> m_Geometries;
		std::shared_ptr<const DestructionCacheTopology> m_Topology;
		std::vector<DestructionPlaybackEvent> m_Events;
		uint64_t m_InstanceId = 0, m_Epoch = 1, m_Revision = 0, m_DroppedEvents = 0;
		bool m_Playing = false;
	};
}

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
	struct DestructionBondStrain { int NodeA, NodeB; float Value; };
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
		CollisionData FilterData;                       // Inherited by chunk shapes and aggregate proxies on rebuild.
		float ContactOffset = 0.0f, RestOffset = 0.0f;
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
		bool IsValid() const;
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
		void InitializeAsset(const DestructionAsset& Asset, const Transform& Placement = Transform::Identity());
		const DestructionAsset* GetAuthoredAsset() const { return mAsset.IsValid() ? &mAsset : nullptr; }
		const Transform& GetAssetPlacement() const { return mAssetPlacement; }
		uint64_t GetInstanceId() const { return mInstanceId; }
		uint64_t GetGeneration() const { return mGeneration; }
		uint64_t GetEventRevision() const { return mEventRevision; }
		void ConfigureEvents(const DestructionEventSettings& Settings);
		const DestructionEventSettings& GetEventSettings() const { return mEventSettings; }
		DestructionEventStats GetEventStats() const;
		std::vector<DestructionEvent> TakeEvents();

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
		uint32_t ApplyBondStrain(const std::vector<DestructionBondStrain>& Strains);
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
		friend class DestructionAsset;
		friend class DestructionEvents;
		friend class DestructionCacheRecorder;
		static uint64_t AllocateInstanceId();
		void RefreshEventKinematics();
		void CollectEvents();
		void ResetEventState();
		class EventCauseScope
		{
		public:
			EventCauseScope(DestructionSet& Set, DestructionEventCause Cause);
			~EventCauseScope();
		private:
			DestructionSet& mSet;
			DestructionEventCause mPrevious;
		};
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
		void QueueContactMomentum(RigidBody* Body, Geometry* Geom, const PhysicsContactPoint& Point, const Vector3& Momentum, const Vector3& Normal, bool ImpulseEstimated);

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
		DestructionAsset mAsset;
		Transform mAssetPlacement = Transform::Identity();
		const uint64_t mInstanceId = AllocateInstanceId();
		uint64_t mGeneration = 1, mNextEventSequence = 1, mEventUpdateIndex = 0;
		uint64_t mEventRevision = 1;
		float mEventDeltaTime = 0;
		DestructionEventCause mEventCause = DestructionEventCause::GraphEdit;
		DestructionEventSettings mEventSettings;
		std::unique_ptr<DestructionEvents> mEvents;
		ConnectionGraph BuildStressGraph() const;
		void EnsureStressBondIndices();
		std::unique_ptr<StrainSolver> mStressSolver;
		std::vector<DestructionMomentum> mStressImpulses;
		Vector3 mStressGravity = Vector3::Zero();
	};

}	// namespace Riemann

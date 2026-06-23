#pragma once

#include <cstdint>
#include <map>
#include <string>
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
	class PhysicsWorld;
	struct PhysicsContactInfo;
	struct PhysicsContactPoint;
	class RigidBody;
	class RigidBodyDynamic;

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
		DestructionSet() = default;
		explicit DestructionSet(PhysicsWorld* Simulation);
		explicit DestructionSet(PhysicsWorld& Simulation);
		virtual ~DestructionSet();

		struct Stats
		{
			uint32_t NormalCount = 0;
			uint32_t StaticCount = 0;
			uint32_t FreeCount = 0;
			uint32_t SmashedCount = 0;
		};

		int AddChunk(const Box3& Bounds, float Mass = 1.0f, bool bStatic = false, const std::string& Name = std::string());
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

		BitSet FindUnsupported() const;
		std::vector<int> PruneUnsupported();
		std::vector<int> QueryChunksRadius(const Vector3& Center, float Radius) const;

		void RecalculateBounds();
		Stats GetStats() const;
		uint32_t AllocClusterID() { return mNextClusterID++; }
		void SetPhysicsWorld(PhysicsWorld* Simulation) { mPhysicsWorld = Simulation; }
		PhysicsWorld* GetPhysicsWorld() const { return mPhysicsWorld; }

		std::vector<float> GetMasses() const;
		float GetVolume(int Index) const;

		ConnectionGraph& GetGraph() { return mGraph; }
		const ConnectionGraph& GetGraph() const { return mGraph; }
		const std::vector<DestructionChunk>& GetChunks() const { return mChunks; }
		std::vector<DestructionChunk>& GetChunks() { return mChunks; }
		const Box3& GetBounds() const { return mBounds; }
		DestructionConstants& GetDestructionConstants() { return mConstants; }
		const DestructionConstants& GetDestructionConstants() const { return mConstants; }

	public:
		ConnectionGraph mGraph;
		Box3 mBounds = Box3::Empty();

	private:
		void BuildClusterRigidBody(DestructionCluster& Cluster);
		void ReleaseClusterRigidBody(DestructionCluster& Cluster, bool bReleaseGeometries);
		void ReleaseClusterRigidBodies(bool bReleaseGeometries);
		void MarkFree(int Index);
		bool BreakBondByIndex(int BondIndex);
		bool HasClusterRigidBodies() const;
		DestructionCluster* FindClusterByRigidBody(const RigidBody* Body);
		void QueueContactMomentum(RigidBody* Body, Geometry* Geom, const PhysicsContactPoint& Point, const Vector3& Momentum, const Vector3& Normal);

	private:
		std::vector<DestructionChunk> mChunks;
		std::map<uint32_t, DestructionCluster> mClusters;
		std::vector<DestructionMomentum> mMomentumQueue;
		std::vector<int> mBrokenIndices;
		BitSet mDestructionInfo[(int)DestructionEventType::Count];
		bool bHasDestructionInfo = false;
		uint32_t mNextClusterID = 1;
		PhysicsWorld* mPhysicsWorld = nullptr;
		DestructionConstants mConstants;
	};

}	// namespace Riemann

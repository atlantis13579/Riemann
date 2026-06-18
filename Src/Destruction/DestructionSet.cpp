
#include "DestructionSet.h"

#include <algorithm>
#include <queue>

namespace Riemann
{
	namespace
	{
		bool IntersectSphereAABB(const Vector3& Center, float Radius, const Box3& Bounds)
		{
			float DistanceSquared = 0.0f;
			const float Coordinates[3] = { Center.x, Center.y, Center.z };
			const float MinValues[3] = { Bounds.Min.x, Bounds.Min.y, Bounds.Min.z };
			const float MaxValues[3] = { Bounds.Max.x, Bounds.Max.y, Bounds.Max.z };
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				float Delta = 0.0f;
				if (Coordinates[Axis] < MinValues[Axis])
				{
					Delta = MinValues[Axis] - Coordinates[Axis];
				}
				else if (Coordinates[Axis] > MaxValues[Axis])
				{
					Delta = Coordinates[Axis] - MaxValues[Axis];
				}
				DistanceSquared += Delta * Delta;
			}
			return DistanceSquared <= Radius * Radius;
		}
	}

	DestructionConstants::DestructionConstants()
	{
		Levels[0] = { 10.0f, 3 };
		Levels[1] = { 100.0f, 2 };
		Levels[2] = { 1000.0f, 2 };
		Levels[3] = { 1000.0f, 2 };
		Levels[4] = { 1000.0f, 2 };
		Levels[5] = { 1000.0f, 2 };
		Levels[6] = { 1000.0f, 2 };
		Levels[7] = { 1000.0f, 2 };
	}

	int DestructionSet::AddChunk(const Box3& Bounds, float Mass, bool bStatic, const std::string& Name)
	{
		DestructionChunk Chunk;
		Chunk.Name = Name;
		Chunk.Bounds = Bounds;
		Chunk.Centroid = Bounds.GetCenter();
		Chunk.Volume = std::max(Bounds.GetVolume(), 1e-3f);
		Chunk.Mass = std::max(Mass, 1e-3f);
		Chunk.Density = Chunk.Mass / Chunk.Volume;
		Chunk.IsStatic = bStatic;

		DestructionBondList TempList;
		TempList.InitCollapseStrength(Bounds);
		Chunk.CollapseStrength = TempList.Strength;

		const int Index = (int)mChunks.size();
		mChunks.push_back(Chunk);
		mGraph.Resize(mChunks.size());
		RecalculateBounds();
		return Index;
	}

	void DestructionSet::SetChunks(const std::vector<DestructionChunk>& Chunks)
	{
		mChunks = Chunks;
		for (DestructionChunk& Chunk : mChunks)
		{
			if (Chunk.Volume <= 0.0f)
			{
				Chunk.Volume = std::max(Chunk.Bounds.GetVolume(), 1e-3f);
			}
			if (Chunk.Mass <= 0.0f)
			{
				Chunk.Mass = std::max(Chunk.Volume * Chunk.Density, 1e-3f);
			}
			Chunk.Centroid = Chunk.Bounds.GetCenter();
		}
		mGraph.Resize(mChunks.size());
		RecalculateBounds();
	}

	void DestructionSet::Clear()
	{
		mGraph.Clear();
		mBounds = Box3::Empty();
		mChunks.clear();
		mClusters.clear();
		mMomentumQueue.clear();
		mBrokenIndices.clear();
		for (int Type = 0; Type < (int)DestructionEventType::Count; ++Type)
		{
			mDestructionInfo[Type].clear();
		}
		bHasDestructionInfo = false;
		mNextClusterID = 1;
	}

	bool DestructionSet::AddBond(int NodeA, int NodeB, DestructionBondType Type, float Strength, int BondIndex)
	{
		return mGraph.AddBond(NodeA, NodeB, Type, Strength, BondIndex);
	}

	bool DestructionSet::SetBondStrength(int NodeA, int NodeB, float Strength)
	{
		return mGraph.SetBondStrength(NodeA, NodeB, Strength);
	}

	void DestructionSet::RemoveAllBonds()
	{
		mGraph.Clear();
		mGraph.Resize(mChunks.size());
	}

	uint32_t DestructionSet::AddCluster(int Level, const std::vector<int>& Indices)
	{
		DestructionCluster Cluster = DestructionCluster::Create(*this, Level, Indices);
		const uint32_t ID = Cluster.GetID();
		mClusters[ID] = Cluster;
		return ID;
	}

	bool DestructionSet::RemoveCluster(uint32_t ClusterID)
	{
		return mClusters.erase(ClusterID) > 0;
	}

	void DestructionSet::ClearClusters()
	{
		mClusters.clear();
	}

	void DestructionSet::RebuildClustersFromGraph()
	{
		mClusters.clear();
		BitSet Visited(mChunks.size());

		for (size_t Node = 0; Node < mChunks.size(); ++Node)
		{
			if (Visited.get(Node) || mChunks[Node].IsFree)
			{
				continue;
			}

			std::vector<int> Component;
			std::queue<int> Queue;
			Queue.push((int)Node);
			Visited.insert(Node);
			while (!Queue.empty())
			{
				const int Current = Queue.front();
				Queue.pop();
				Component.push_back(Current);
				if (!mGraph.IsValidNode(Current))
				{
					continue;
				}
				for (const Bond& ExistingBond : mGraph[(size_t)Current].Bonds)
				{
					const int Other = ExistingBond.GetOther(Current);
					if (Other >= 0 && (size_t)Other < mChunks.size() && !Visited.get((size_t)Other) && !mChunks[(size_t)Other].IsFree)
					{
						Visited.insert((size_t)Other);
						Queue.push(Other);
					}
				}
			}

			if (!Component.empty())
			{
				AddCluster(-1, Component);
			}
		}
	}

	DestructionCluster* DestructionSet::FindClusterContaining(int ChunkIndex)
	{
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			if (Entry.second.Contains(ChunkIndex))
			{
				return &Entry.second;
			}
		}
		return nullptr;
	}

	const DestructionCluster* DestructionSet::FindClusterContaining(int ChunkIndex) const
	{
		for (std::map<uint32_t, DestructionCluster>::const_reference Entry : mClusters)
		{
			if (Entry.second.Contains(ChunkIndex))
			{
				return &Entry.second;
			}
		}
		return nullptr;
	}

	DestructionCluster* DestructionSet::GetCluster(uint32_t ClusterID)
	{
		std::map<uint32_t, DestructionCluster>::iterator It = mClusters.find(ClusterID);
		return It == mClusters.end() ? nullptr : &It->second;
	}

	const DestructionCluster* DestructionSet::GetCluster(uint32_t ClusterID) const
	{
		std::map<uint32_t, DestructionCluster>::const_iterator It = mClusters.find(ClusterID);
		return It == mClusters.end() ? nullptr : &It->second;
	}

	std::vector<DestructionCluster*> DestructionSet::GetClusters()
	{
		std::vector<DestructionCluster*> Clusters;
		Clusters.reserve(mClusters.size());
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			Clusters.push_back(&Entry.second);
		}
		return Clusters;
	}

	void DestructionSet::BreakBondEx(int Index)
	{
		if (!mGraph.IsValidNode(Index) || mGraph.IsBroken(Index))
		{
			return;
		}

		mGraph.BreakBonds(Index);
		MarkFree(Index);
	}

	bool DestructionSet::ApplyMomentum(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		(void)Normal;
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return false;
		}

		const float Strength = Momentum.Length();
		bool bChanged = false;
		DestructionCluster* Cluster = FindClusterContaining(Index);
		if (Cluster && mConstants.PartitionAlgorithm == DestructionPartitionAlgorithm::Metis)
		{
			const int Level = Cluster->GetLevel();
			if (0 <= Level && Level < 8 && Cluster->Volume() > mConstants.MinClusterVolume)
			{
				const DestructionClusterLevelConstants& LevelConstants = mConstants.Levels[Level];
				if (Strength >= LevelConstants.BreakThreshold)
				{
					if (mGraph.BreakGraph(Cluster->GetSourceIndices(), LevelConstants.PartitionSize) == DestructionBreakStatus::Success)
					{
						RebuildClustersFromGraph();
						bChanged = true;
					}
					return bChanged;
				}
			}
		}

		std::vector<int> Collapse;
		const std::vector<int>* SourceIndices = Cluster ? &Cluster->GetSourceIndices() : nullptr;
		if (SourceIndices)
		{
			for (int ChunkIndex : *SourceIndices)
			{
				if (ChunkIndex < 0 || (size_t)ChunkIndex >= mChunks.size())
				{
					continue;
				}
				DestructionChunk& Chunk = mChunks[(size_t)ChunkIndex];
				if (Chunk.CollapseStrength > mConstants.ImpulseCollapseStrength || Chunk.IsStatic || Chunk.IsSmashed)
				{
					continue;
				}
				if (IntersectSphereAABB(Position, mConstants.ImpulseRadius, Chunk.Bounds))
				{
					Collapse.push_back(ChunkIndex);
				}
			}
		}
		else
		{
			DestructionChunk& Chunk = mChunks[(size_t)Index];
			if (Chunk.CollapseStrength <= mConstants.ImpulseCollapseStrength && !Chunk.IsStatic && !Chunk.IsSmashed)
			{
				Collapse.push_back(Index);
			}
		}

		if (!Collapse.empty())
		{
			AddDestructionInfo(DestructionEventType::SelfContact, Collapse);
			bChanged = true;
		}

		std::vector<std::pair<int, int>> ShockBreaks;
		mGraph.ShockPropagation(Index, Strength * mConstants.DamagePropagationStrength, mConstants.DamagePropagationAttenuation, ShockBreaks);
		for (const std::pair<int, int>& Pair : ShockBreaks)
		{
			mGraph.BreakBond(Pair.first, Pair.second);
			bChanged = true;
		}
		if (bChanged)
		{
			RebuildClustersFromGraph();
		}
		return bChanged;
	}

	void DestructionSet::AddMomentumQueue(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		if (Momentum.Length() < 1e-3f)
		{
			return;
		}
		for (const DestructionMomentum& ExistingMomentum : mMomentumQueue)
		{
			if (ExistingMomentum.Index == Index)
			{
				return;
			}
		}

		DestructionMomentum Info;
		Info.Index = Index;
		Info.Momentum = Momentum;
		Info.Position = Position;
		Info.Normal = Normal;
		mMomentumQueue.push_back(Info);
	}

	void DestructionSet::ProcessMomentumQueue()
	{
		int Count = std::max(mConstants.ImpulseProcessPerTick, 1);
		while (Count-- > 0 && !mMomentumQueue.empty())
		{
			const DestructionMomentum Info = mMomentumQueue.front();
			mMomentumQueue.erase(mMomentumQueue.begin());
			ApplyMomentum(Info.Index, Info.Momentum, Info.Position, Info.Normal);
		}
	}

	void DestructionSet::AddDestructionInfo(DestructionEventType Type, const std::vector<int>& Indices)
	{
		const int TypeIndex = (int)Type;
		if (TypeIndex < 0 || TypeIndex >= (int)DestructionEventType::Count)
		{
			return;
		}
		if (mDestructionInfo[TypeIndex].size() != mChunks.size())
		{
			mDestructionInfo[TypeIndex].resize(mChunks.size());
		}
		for (int Index : Indices)
		{
			if (Index >= 0 && (size_t)Index < mChunks.size() && !mChunks[(size_t)Index].IsStatic)
			{
				mDestructionInfo[TypeIndex].insert((size_t)Index);
				bHasDestructionInfo = true;
			}
		}
	}

	uint32_t DestructionSet::ProcessDestructionInfo()
	{
		if (!bHasDestructionInfo)
		{
			return 0;
		}
		bHasDestructionInfo = false;

		uint32_t Flags = 0;
		for (int TypeIndex = 0; TypeIndex < (int)DestructionEventType::Count; ++TypeIndex)
		{
			std::vector<uint32_t> Indices = mDestructionInfo[TypeIndex].to_vector();
			mDestructionInfo[TypeIndex].clear();
			if (Indices.empty())
			{
				continue;
			}

			Flags |= (1u << TypeIndex);
			for (uint32_t Index : Indices)
			{
				if (Index < mChunks.size())
				{
					if (TypeIndex == (int)DestructionEventType::SelfContact ||
						TypeIndex == (int)DestructionEventType::SelfCollapse ||
						TypeIndex == (int)DestructionEventType::Destruct)
					{
						mChunks[Index].IsSmashed = true;
					}
					MarkFree((int)Index);
				}
			}
		}
		RebuildClustersFromGraph();
		return Flags;
	}

	void DestructionSet::DoFractures(const DestructionFractureBuffer& Commands)
	{
		bool bChanged = false;
		for (int BondIndex : Commands.BondFractures)
		{
			bChanged = BreakBondByIndex(BondIndex) || bChanged;
		}
		if (bChanged)
		{
			RebuildClustersFromGraph();
		}
	}

	BitSet DestructionSet::FindUnsupported() const
	{
		BitSet Supported(mChunks.size());
		std::vector<int> Seeds;
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (mChunks[Index].IsStatic)
			{
				Seeds.push_back((int)Index);
			}
		}

		if (!Seeds.empty())
		{
			Supported = mGraph.FindSupported(Seeds, nullptr);
		}

		BitSet Unsupported(mChunks.size());
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (!mChunks[Index].IsStatic && !mChunks[Index].IsFree && !Supported.get(Index))
			{
				Unsupported.insert(Index);
			}
		}
		return Unsupported;
	}

	std::vector<int> DestructionSet::PruneUnsupported()
	{
		BitSet Unsupported = FindUnsupported();
		std::vector<uint32_t> UnsupportedIndices = Unsupported.to_vector();
		std::vector<int> Result;
		Result.reserve(UnsupportedIndices.size());
		for (uint32_t Index : UnsupportedIndices)
		{
			MarkFree((int)Index);
			Result.push_back((int)Index);
		}
		if (!Result.empty())
		{
			RebuildClustersFromGraph();
		}
		return Result;
	}

	std::vector<int> DestructionSet::QueryChunksRadius(const Vector3& Center, float Radius) const
	{
		std::vector<int> Result;
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (IntersectSphereAABB(Center, Radius, mChunks[Index].Bounds))
			{
				Result.push_back((int)Index);
			}
		}
		return Result;
	}

	void DestructionSet::RecalculateBounds()
	{
		mBounds = Box3::Empty();
		for (const DestructionChunk& Chunk : mChunks)
		{
			mBounds.Encapsulate(Chunk.Bounds);
		}
	}

	DestructionSet::Stats DestructionSet::GetStats() const
	{
		Stats Result;
		for (const DestructionChunk& Chunk : mChunks)
		{
			if (Chunk.IsStatic)
			{
				++Result.StaticCount;
			}
			else if (Chunk.IsSmashed)
			{
				++Result.SmashedCount;
			}
			else if (Chunk.IsFree)
			{
				++Result.FreeCount;
			}
			else
			{
				++Result.NormalCount;
			}
		}
		return Result;
	}

	std::vector<float> DestructionSet::GetMasses() const
	{
		std::vector<float> Masses;
		Masses.reserve(mChunks.size());
		for (const DestructionChunk& Chunk : mChunks)
		{
			Masses.push_back(Chunk.Mass);
		}
		return Masses;
	}

	float DestructionSet::GetVolume(int Index) const
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return 0.0f;
		}
		return mChunks[(size_t)Index].Volume;
	}

	void DestructionSet::MarkFree(int Index)
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return;
		}
		mChunks[(size_t)Index].IsFree = true;
		if (std::find(mBrokenIndices.begin(), mBrokenIndices.end(), Index) == mBrokenIndices.end())
		{
			mBrokenIndices.push_back(Index);
		}
	}

	bool DestructionSet::BreakBondByIndex(int BondIndex)
	{
		if (BondIndex < 0)
		{
			return false;
		}
		for (size_t Node = 0; Node < mGraph.Size(); ++Node)
		{
			for (const Bond& ExistingBond : mGraph[Node].Bonds)
			{
				if (ExistingBond.bond_index == BondIndex)
				{
					mGraph.BreakBond((int)Node, ExistingBond.GetOther((int)Node));
					return true;
				}
			}
		}
		return false;
	}

}	// namespace Riemann

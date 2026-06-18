
#include "ConnectionGraph.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "GraphPartition.h"

namespace Riemann
{
	namespace
	{
		bool IsTypeEnabled(DestructionBondType Type, uint32_t Mask)
		{
			const int TypeIndex = static_cast<int>(Type);
			return TypeIndex > 0 && ((1u << TypeIndex) & Mask) != 0;
		}

		bool ContainsNode(const std::vector<int>& Nodes, int Node)
		{
			return std::find(Nodes.begin(), Nodes.end(), Node) != Nodes.end();
		}
	}

	Bond::Bond(int InV0, int InV1, DestructionBondType InType, float InStrength, int InBondIndex)
		: v0(InV0)
		, v1(InV1)
		, strength(InStrength)
		, bond_index(InBondIndex)
	{
		SetDestructionType(InType);
	}

	int Bond::GetOther(int Node) const
	{
		if (Node == v0)
		{
			return v1;
		}
		if (Node == v1)
		{
			return v0;
		}
		return -1;
	}

	DestructionBondType Bond::GetDestructionType() const
	{
		return ConnectionGraph::FromLegacyType(type);
	}

	void Bond::SetDestructionType(DestructionBondType InType)
	{
		type = ConnectionGraph::ToLegacyType(InType);
	}

	void DestructionBondList::Clear()
	{
		Bonds.clear();
		IsSupport = false;
		SupportGroupID = 0;
	}

	bool DestructionBondList::SetBondStrength(int OtherNode, float InStrength)
	{
		for (Bond& ExistingBond : Bonds)
		{
			if (ExistingBond.GetOther(OtherNode) >= 0 || ExistingBond.v0 == OtherNode || ExistingBond.v1 == OtherNode)
			{
				ExistingBond.strength = InStrength;
				return true;
			}
		}
		return false;
	}

	void DestructionBondList::InitCollapseStrength(const Box3& Bounds)
	{
		const float StrengthDensity = 100.0f;
		const Vector3 Size = Bounds.GetSize();
		const float Mean = (Size.x + Size.y + Size.z) / 3.0f;
		const float Variance = std::sqrt((Size.x - Mean) * (Size.x - Mean) + (Size.y - Mean) * (Size.y - Mean) + (Size.z - Mean) * (Size.z - Mean));
		Strength = StrengthDensity / std::max(Variance, 1e-3f);
	}

	void ConnectionGraph::Clear() noexcept
	{
		mBondLists.clear();
		mLegacyBonds.clear();
		mBrokenNodes.clear();
		mBrokenBonds.clear();
	}

	void ConnectionGraph::Resize(size_t Size)
	{
		mBondLists.resize(Size);
		RebuildLegacyBonds();
	}

	bool ConnectionGraph::IsBroken(int Node) const noexcept
	{
		return !IsValidNode(Node) || mBondLists[(size_t)Node].Bonds.empty();
	}

	bool ConnectionGraph::IsValidNode(int Node) const noexcept
	{
		return Node >= 0 && (size_t)Node < mBondLists.size();
	}

	bool ConnectionGraph::HasBond(int NodeA, int NodeB) const noexcept
	{
		return FindBond(NodeA, NodeB) != nullptr;
	}

	const Bond* ConnectionGraph::FindBond(int NodeA, int NodeB) const noexcept
	{
		if (!IsValidNode(NodeA))
		{
			return nullptr;
		}

		for (const Bond& ExistingBond : mBondLists[(size_t)NodeA].Bonds)
		{
			if (ExistingBond.GetOther(NodeA) == NodeB)
			{
				return &ExistingBond;
			}
		}
		return nullptr;
	}

	Bond* ConnectionGraph::FindBond(int NodeA, int NodeB) noexcept
	{
		if (!IsValidNode(NodeA))
		{
			return nullptr;
		}

		for (Bond& ExistingBond : mBondLists[(size_t)NodeA].Bonds)
		{
			if (ExistingBond.GetOther(NodeA) == NodeB)
			{
				return &ExistingBond;
			}
		}
		return nullptr;
	}

	bool ConnectionGraph::AddBond(int NodeA, int NodeB, DestructionBondType Type, float Strength, int BondIndex)
	{
		if (NodeA < 0 || NodeB < 0 || NodeA == NodeB)
		{
			return false;
		}

		const size_t NewSize = (size_t)std::max(NodeA, NodeB) + 1;
		if (mBondLists.size() < NewSize)
		{
			mBondLists.resize(NewSize);
		}

		const bool bA = AddDirectedBond(NodeA, NodeB, Type, Strength, BondIndex);
		DestructionBondType ReverseType = Type;
		if (Type == DestructionBondType::Support)
		{
			ReverseType = DestructionBondType::BeSupported;
		}
		else if (Type == DestructionBondType::BeSupported)
		{
			ReverseType = DestructionBondType::Support;
		}
		const bool bB = AddDirectedBond(NodeB, NodeA, ReverseType, Strength, BondIndex);
		RebuildLegacyBonds();
		return bA || bB;
	}

	bool ConnectionGraph::AddDirectedBond(int FromNode, int ToNode, DestructionBondType Type, float Strength, int BondIndex)
	{
		if (FromNode < 0 || ToNode < 0 || FromNode == ToNode)
		{
			return false;
		}

		const size_t NewSize = (size_t)std::max(FromNode, ToNode) + 1;
		if (mBondLists.size() < NewSize)
		{
			mBondLists.resize(NewSize);
		}

		if (Bond* ExistingBond = FindBond(FromNode, ToNode))
		{
			ExistingBond->SetDestructionType(Type);
			ExistingBond->strength = Strength;
			ExistingBond->bond_index = BondIndex;
			RebuildLegacyBonds();
			return false;
		}

		mBondLists[(size_t)FromNode].Bonds.emplace_back(FromNode, ToNode, Type, Strength, BondIndex);
		RebuildLegacyBonds();
		return true;
	}

	DestructionBondType ConnectionGraph::GetBondType(int NodeA, int NodeB) const
	{
		if (const Bond* ExistingBond = FindBond(NodeA, NodeB))
		{
			return ExistingBond->GetDestructionType();
		}
		return DestructionBondType::Invalid;
	}

	bool ConnectionGraph::SetBondType(int NodeA, int NodeB, DestructionBondType Type)
	{
		bool bChanged = false;
		if (Bond* ExistingBond = FindBond(NodeA, NodeB))
		{
			ExistingBond->SetDestructionType(Type);
			bChanged = true;
		}
		if (Bond* ExistingBond = FindBond(NodeB, NodeA))
		{
			DestructionBondType ReverseType = Type;
			if (Type == DestructionBondType::Support)
			{
				ReverseType = DestructionBondType::BeSupported;
			}
			else if (Type == DestructionBondType::BeSupported)
			{
				ReverseType = DestructionBondType::Support;
			}
			ExistingBond->SetDestructionType(ReverseType);
			bChanged = true;
		}
		if (bChanged)
		{
			RebuildLegacyBonds();
		}
		return bChanged;
	}

	bool ConnectionGraph::SetBondStrength(int NodeA, int NodeB, float Strength)
	{
		bool bChanged = false;
		if (Bond* ExistingBond = FindBond(NodeA, NodeB))
		{
			ExistingBond->strength = Strength;
			bChanged = true;
		}
		if (Bond* ExistingBond = FindBond(NodeB, NodeA))
		{
			ExistingBond->strength = Strength;
			bChanged = true;
		}
		if (bChanged)
		{
			RebuildLegacyBonds();
		}
		return bChanged;
	}

	void ConnectionGraph::BreakBond(int NodeA, int NodeB) noexcept
	{
		if (!HasBond(NodeA, NodeB) && !HasBond(NodeB, NodeA))
		{
			return;
		}

		RemoveDirectedBond(NodeA, NodeB);
		RemoveDirectedBond(NodeB, NodeA);
		RecordBrokenBond(NodeA, NodeB);
		RebuildLegacyBonds();
	}

	void ConnectionGraph::BreakBonds(int Node) noexcept
	{
		if (!IsValidNode(Node) || mBondLists[(size_t)Node].Bonds.empty())
		{
			return;
		}

		std::vector<int> Neighbors;
		for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
		{
			Neighbors.push_back(ExistingBond.GetOther(Node));
		}

		for (int Neighbor : Neighbors)
		{
			RemoveDirectedBond(Neighbor, Node);
			RecordBrokenBond(Node, Neighbor);
		}
		mBondLists[(size_t)Node].Bonds.clear();
		mBrokenNodes.push_back(Node);
		RebuildLegacyBonds();
	}

	void ConnectionGraph::BreakBondsByType(int Node, uint32_t TypeMask) noexcept
	{
		if (!IsValidNode(Node))
		{
			return;
		}

		std::vector<int> Neighbors;
		for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
		{
			if (IsTypeEnabled(ExistingBond.GetDestructionType(), TypeMask))
			{
				Neighbors.push_back(ExistingBond.GetOther(Node));
			}
		}

		for (int Neighbor : Neighbors)
		{
			BreakBond(Node, Neighbor);
		}
	}

	DestructionBreakStatus ConnectionGraph::BreakGraph(const std::vector<int>& Nodes, int PartCount) noexcept
	{
		if (Nodes.size() <= 1)
		{
			return DestructionBreakStatus::Unchanged;
		}

		if (PartCount <= 1)
		{
			return DestructionBreakStatus::Unchanged;
		}

		if (Nodes.size() <= (size_t)PartCount)
		{
			for (size_t i = 0; i < Nodes.size(); ++i)
			{
				for (size_t j = i + 1; j < Nodes.size(); ++j)
				{
					BreakBond(Nodes[i], Nodes[j]);
				}
			}
			return DestructionBreakStatus::Success;
		}

		std::map<int, int> NodeToIndex;
		std::vector<std::vector<int>> Parts;
		if (!PartitionConnectionGraph(Nodes, this, PartCount, NodeToIndex, Parts))
		{
			return DestructionBreakStatus::Failed;
		}

		std::vector<int> NodePart(mBondLists.size(), -1);
		for (size_t PartIndex = 0; PartIndex < Parts.size(); ++PartIndex)
		{
			for (int Node : Parts[PartIndex])
			{
				if (IsValidNode(Node))
				{
					NodePart[(size_t)Node] = (int)PartIndex;
				}
			}
		}

		std::vector<std::pair<int, int>> BondsToBreak;
		for (int Node : Nodes)
		{
			if (!IsValidNode(Node))
			{
				continue;
			}
			for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
			{
				const int Other = ExistingBond.GetOther(Node);
				if (Other > Node && ContainsNode(Nodes, Other) && NodePart[(size_t)Node] != NodePart[(size_t)Other])
				{
					BondsToBreak.emplace_back(Node, Other);
				}
			}
		}

		for (const std::pair<int, int>& Pair : BondsToBreak)
		{
			BreakBond(Pair.first, Pair.second);
		}

		return DestructionBreakStatus::Success;
	}

	BitSet ConnectionGraph::FindSupported(int Seed, uint32_t TypeMask) const noexcept
	{
		BitSet Supported(mBondLists.size());
		if (!IsValidNode(Seed))
		{
			return Supported;
		}

		std::vector<int> ExpandQueue;
		ExpandQueue.push_back(Seed);
		while (!ExpandQueue.empty())
		{
			const int Node = ExpandQueue.back();
			ExpandQueue.pop_back();
			if (!IsValidNode(Node) || Supported.get((size_t)Node))
			{
				continue;
			}

			Supported.insert((size_t)Node);
			for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
			{
				if (IsTypeEnabled(ExistingBond.GetDestructionType(), TypeMask))
				{
					ExpandQueue.push_back(ExistingBond.GetOther(Node));
				}
			}
		}
		return Supported;
	}

	BitSet ConnectionGraph::FindSupported(int Seed, const std::vector<int>* AllowedNodes) const noexcept
	{
		std::vector<int> Seeds(1, Seed);
		return FindSupported(Seeds, AllowedNodes);
	}

	BitSet ConnectionGraph::FindSupported(const std::set<int>& Seeds, const std::vector<int>* AllowedNodes) const noexcept
	{
		std::vector<int> Queue(Seeds.begin(), Seeds.end());
		return FindSupported(Queue, AllowedNodes);
	}

	BitSet ConnectionGraph::FindSupported(std::vector<int>& ExpandQueue, const std::vector<int>* AllowedNodes) const noexcept
	{
		BitSet Allowed;
		if (AllowedNodes)
		{
			std::vector<uint32_t> Indices;
			Indices.reserve(AllowedNodes->size());
			for (int Node : *AllowedNodes)
			{
				if (IsValidNode(Node))
				{
					Indices.push_back((uint32_t)Node);
				}
			}
			Allowed = BitSet(mBondLists.size(), Indices);
		}

		BitSet Supported(mBondLists.size());
		while (!ExpandQueue.empty())
		{
			const int Node = ExpandQueue.back();
			ExpandQueue.pop_back();
			if (!IsValidNode(Node) || Supported.get((size_t)Node))
			{
				continue;
			}
			if (AllowedNodes && !Allowed.get((size_t)Node))
			{
				continue;
			}

			Supported.insert((size_t)Node);
			for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
			{
				const int Other = ExistingBond.GetOther(Node);
				if (!AllowedNodes || Allowed.get((size_t)Other))
				{
					ExpandQueue.push_back(Other);
				}
			}
		}
		return Supported;
	}

	void ConnectionGraph::ShockPropagation(int Seed, float Strength, float PropagationFactor, std::vector<std::pair<int, int>>& Breaks) const
	{
		Breaks.clear();
		if (!IsValidNode(Seed) || Strength <= 0.0f)
		{
			return;
		}

		std::vector<float> ShockStrength(mBondLists.size(), -1.0f);
		std::vector<int> ExpandQueue(1, Seed);
		ShockStrength[(size_t)Seed] = Strength;

		while (!ExpandQueue.empty())
		{
			const int Node = ExpandQueue.back();
			ExpandQueue.pop_back();
			for (const Bond& ExistingBond : mBondLists[(size_t)Node].Bonds)
			{
				const int Other = ExistingBond.GetOther(Node);
				if (!IsValidNode(Other) || ShockStrength[(size_t)Other] >= 0.0f)
				{
					continue;
				}

				const float Value = ShockStrength[(size_t)Node] * PropagationFactor;
				ShockStrength[(size_t)Other] = Value;
				ExpandQueue.push_back(Other);
				if (Value >= 1.0f && Value > ExistingBond.strength)
				{
					Breaks.emplace_back(Node, Other);
				}
			}
		}
	}

	void ConnectionGraph::ResetSupportGroups()
	{
		for (DestructionBondList& BondList : mBondLists)
		{
			BondList.SupportGroupID = 0;
		}
	}

	int ConnectionGraph::GetMaxBondCount() const
	{
		size_t MaxCount = 0;
		for (const DestructionBondList& BondList : mBondLists)
		{
			MaxCount = std::max(MaxCount, BondList.Bonds.size());
		}
		return (int)MaxCount;
	}

	void ConnectionGraph::ClearBreakHistory()
	{
		mBrokenNodes.clear();
		mBrokenBonds.clear();
	}

	Bond::BondType ConnectionGraph::ToLegacyType(DestructionBondType Type)
	{
		switch (Type)
		{
		case DestructionBondType::Connect:
			return Bond::BondType::Connect;
		case DestructionBondType::Support:
			return Bond::BondType::Supports;
		case DestructionBondType::BeSupported:
			return Bond::BondType::Supported;
		case DestructionBondType::HingeJoint:
			return Bond::BondType::HingeJoint;
		default:
			return Bond::BondType::Default;
		}
	}

	DestructionBondType ConnectionGraph::FromLegacyType(Bond::BondType Type)
	{
		switch (Type)
		{
		case Bond::BondType::Connect:
			return DestructionBondType::Connect;
		case Bond::BondType::Supports:
			return DestructionBondType::Support;
		case Bond::BondType::Supported:
			return DestructionBondType::BeSupported;
		case Bond::BondType::HingeJoint:
			return DestructionBondType::HingeJoint;
		default:
			return DestructionBondType::Invalid;
		}
	}

	void ConnectionGraph::RebuildLegacyBonds()
	{
		mLegacyBonds.clear();
		mLegacyBonds.resize((int)mBondLists.size());
		for (size_t Node = 0; Node < mBondLists.size(); ++Node)
		{
			for (const Bond& ExistingBond : mBondLists[Node].Bonds)
			{
				mLegacyBonds[(int)Node].push_back(ExistingBond);
			}
		}
	}

	void ConnectionGraph::RemoveDirectedBond(int FromNode, int ToNode) noexcept
	{
		if (!IsValidNode(FromNode))
		{
			return;
		}

		std::vector<Bond>& Bonds = mBondLists[(size_t)FromNode].Bonds;
		Bonds.erase(std::remove_if(Bonds.begin(), Bonds.end(),
			[FromNode, ToNode](const Bond& ExistingBond)
			{
				return ExistingBond.GetOther(FromNode) == ToNode;
			}), Bonds.end());
	}

	void ConnectionGraph::RecordBrokenBond(int NodeA, int NodeB)
	{
		if (NodeA > NodeB)
		{
			std::swap(NodeA, NodeB);
		}
		if (std::find(mBrokenBonds.begin(), mBrokenBonds.end(), std::make_pair(NodeA, NodeB)) == mBrokenBonds.end())
		{
			mBrokenBonds.emplace_back(NodeA, NodeB);
		}
	}

}	// namespace Riemann

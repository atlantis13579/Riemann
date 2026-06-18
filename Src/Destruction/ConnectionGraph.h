#pragma once

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "../Core/BitSet.h"
#include "../Core/ListSet.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	enum class DestructionBondType : int
	{
		Invalid = 0,
		Connect = 1,
		Support = 2,
		BeSupported = 3,
		HingeJoint = 4,
	};

	enum DestructionBondTypeMask : uint32_t
	{
		DestructionBondMaskConnect = 1u << static_cast<int>(DestructionBondType::Connect),
		DestructionBondMaskSupport = 1u << static_cast<int>(DestructionBondType::Support),
		DestructionBondMaskBeSupported = 1u << static_cast<int>(DestructionBondType::BeSupported),
		DestructionBondMaskHingeJoint = 1u << static_cast<int>(DestructionBondType::HingeJoint),
		DestructionBondMaskAll = DestructionBondMaskConnect | DestructionBondMaskSupport | DestructionBondMaskBeSupported | DestructionBondMaskHingeJoint,
	};

	enum class DestructionBreakStatus
	{
		Success = 0,
		Failed = 1,
		Unchanged = 2,
	};

	struct BondFrame
	{
		Vector3 Origin = Vector3::Zero();
		Vector3 Tangent = Vector3::UnitX();
		Vector3 Normal = Vector3::UnitY();
		Vector3 Binormal = Vector3::UnitZ();
	};

	struct Bond
	{
		enum class BondType
		{
			Default = 0,
			Connect = 1,
			Supports = 2,
			Supported = 3,
			HingeJoint = 4,
		};

		int				v0 = -1;
		int				v1 = -1;
		BondType		type{ BondType::Default };
		BondFrame		frame;
		float			strength{ 0.0f };
		int				bond_index{ -1 };

		Bond() = default;
		Bond(int InV0, int InV1, DestructionBondType InType, float InStrength = 0.0f, int InBondIndex = -1);

		inline bool operator< (const Bond& rhs) const
		{
			return (v0 < rhs.v0) || (v0 == rhs.v0 && v1 < rhs.v1);
		}

		inline bool operator==(const Bond& rhs) const
		{
			return v0 == rhs.v0 && v1 == rhs.v1;
		}

		int GetOther(int Node) const;
		DestructionBondType GetDestructionType() const;
		void SetDestructionType(DestructionBondType InType);
	};

	struct DestructionBondList
	{
		float Strength = 100000.0f;
		uint32_t SupportGroupID = 0;
		std::vector<Bond> Bonds;
		bool IsSupport = false;

		bool Empty() const { return Bonds.empty(); }
		size_t Size() const { return Bonds.size(); }
		void Clear();
		bool SetBondStrength(int OtherNode, float InStrength);
		void InitCollapseStrength(const Box3& Bounds);

		Bond& operator[](size_t Index) { return Bonds[Index]; }
		const Bond& operator[](size_t Index) const { return Bonds[Index]; }
	};

	struct DestructionSupportGroup
	{
		bool IsNotBreakable = false;
		std::vector<int> Supports;
		Box3 Bounds = Box3::Empty();
		Vector3 CenterOfMass = Vector3::Zero();
	};

	class ConnectionGraph
	{
	public:
		void Clear() noexcept;
		void Resize(size_t Size);
		size_t Size() const noexcept { return mBondLists.size(); }
		bool Empty() const noexcept { return mBondLists.empty(); }

		const ListSet<Bond>& GetBonds() const { return mLegacyBonds; }
		const std::vector<DestructionBondList>& GetBondLists() const { return mBondLists; }
		std::vector<DestructionBondList>& GetBondLists() { return mBondLists; }

		DestructionBondList& operator[](size_t Index) { return mBondLists[Index]; }
		const DestructionBondList& operator[](size_t Index) const { return mBondLists[Index]; }

		bool IsBroken(int Node) const noexcept;
		bool IsValidNode(int Node) const noexcept;
		bool HasBond(int NodeA, int NodeB) const noexcept;
		const Bond* FindBond(int NodeA, int NodeB) const noexcept;
		Bond* FindBond(int NodeA, int NodeB) noexcept;

		bool AddBond(int NodeA, int NodeB, DestructionBondType Type = DestructionBondType::Connect, float Strength = 1.0f, int BondIndex = -1);
		bool AddDirectedBond(int FromNode, int ToNode, DestructionBondType Type = DestructionBondType::Connect, float Strength = 1.0f, int BondIndex = -1);

		DestructionBondType GetBondType(int NodeA, int NodeB) const;
		bool SetBondType(int NodeA, int NodeB, DestructionBondType Type);
		bool SetBondStrength(int NodeA, int NodeB, float Strength);

		void BreakBond(int NodeA, int NodeB) noexcept;
		void BreakBonds(int Node) noexcept;
		void BreakBondsByType(int Node, uint32_t TypeMask) noexcept;
		DestructionBreakStatus BreakGraph(const std::vector<int>& Nodes, int PartCount) noexcept;

		BitSet FindSupported(int Seed, uint32_t TypeMask = DestructionBondMaskAll) const noexcept;
		BitSet FindSupported(int Seed, const std::vector<int>* AllowedNodes) const noexcept;
		BitSet FindSupported(const std::set<int>& Seeds, const std::vector<int>* AllowedNodes) const noexcept;
		BitSet FindSupported(std::vector<int>& ExpandQueue, const std::vector<int>* AllowedNodes) const noexcept;

		void ShockPropagation(int Seed, float Strength, float PropagationFactor, std::vector<std::pair<int, int>>& Breaks) const;

		void ResetSupportGroups();
		int GetMaxBondCount() const;

		const std::vector<int>& GetBrokenNodes() const { return mBrokenNodes; }
		const std::vector<std::pair<int, int>>& GetBrokenBonds() const { return mBrokenBonds; }
		void ClearBreakHistory();

		static Bond::BondType ToLegacyType(DestructionBondType Type);
		static DestructionBondType FromLegacyType(Bond::BondType Type);

	private:
		void RebuildLegacyBonds();
		void RemoveDirectedBond(int FromNode, int ToNode) noexcept;
		void RecordBrokenBond(int NodeA, int NodeB);

	private:
		std::vector<DestructionBondList> mBondLists;
		ListSet<Bond> mLegacyBonds;
		std::vector<int> mBrokenNodes;
		std::vector<std::pair<int, int>> mBrokenBonds;
	};

}	// namespace Riemann

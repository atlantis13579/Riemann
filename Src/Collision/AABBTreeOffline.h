#pragma once

#include <vector>

#include "../Maths/Box3.h"

namespace Riemann
{
	class AABBTreeOffline;
	struct CacheFriendlyAABBTree;

	enum class SplitHeuristic : uint8_t
	{
		Space,
		TreeBalance,
	};

	struct AABBTreeBuildData
	{
		AABBTreeBuildData(const Box3* pArray, const int nGeometries = 0, const int GeometriesPerNode = 1) :
			numGeometriesPerNode(GeometriesPerNode),
			numGeometries(nGeometries),
			pAABBArray(pArray),
			pCenterBuffer(nullptr),
			pIndicesBase(nullptr),
			pAABBTree(nullptr)
		{
			splitter = SplitHeuristic::Space;
		}

		~AABBTreeBuildData()
		{
			numGeometriesPerNode = 0;
			numGeometries = 0;
			pAABBArray = nullptr;
			Release();
		}

		void	Release()
		{
			if (pCenterBuffer)
			{
				delete[]pCenterBuffer;
				pCenterBuffer = nullptr;
			}
			pIndicesBase = nullptr;
		}

		int							numGeometriesPerNode;
		int							numGeometries;
		const Box3* pAABBArray;
		Vector3* pCenterBuffer;		// Holds the memory
		int* pIndicesBase;
		AABBTreeOffline* pAABBTree;
		SplitHeuristic				splitter;
	};

	class AABBTreeNodeOffline
	{
	public:
		AABBTreeNodeOffline()
		{
			child1 = child2 = nullptr;
		}

		~AABBTreeNodeOffline()
		{
			child1 = child2 = nullptr;
		}

		bool	IsLeafNode() const
		{
			if (child1 == nullptr)
			{
				return true;
			}
			return false;
		}

		int		SplitAxisBySpace(const AABBTreeBuildData& Params, int* pGeometries, int Num, int Axis);
		int		SplitAxisByNumGeometries(const AABBTreeBuildData& Params, int* pGeometries, int Num, int Axis);
		void	SubDivideAABBArray(AABBTreeBuildData& Params);
		void	BuildHierarchyRecursive(AABBTreeBuildData& Params);

	public:
		Box3					aabb;
		AABBTreeNodeOffline*	child1;
		AABBTreeNodeOffline*	child2;
		int						indexOffset = 0;
		int						numGeometries = 0;
	};


	class AABBTreeOffline
	{
	public:
		AABBTreeOffline() : pHead(nullptr), nCurrentBlockIndex(0), nTotalNodes(0)
		{

		}

		~AABBTreeOffline()
		{
			Release();
		}

		void Release();
		void Init(int nGeometries, int nGeometriesPerNode);
		void Build(AABBTreeBuildData& params);
		AABBTreeNodeOffline* AllocNodes();
		CacheFriendlyAABBTree* BuildCacheFriendlyTree();

	private:
		AABBTreeNodeOffline* pHead;

		struct NodeBlock
		{
			NodeBlock()  :
				pMem(nullptr),
				nUsedNodes(0),
				nMaxNodes(0) {}
			NodeBlock(AABBTreeNodeOffline* p, const int UsedNodes, const int maxNodes) :
				pMem(p),
				nUsedNodes(UsedNodes),
				nMaxNodes(maxNodes) {}
			AABBTreeNodeOffline* pMem;
			int						nUsedNodes;
			int						nMaxNodes;
		};
		std::vector<NodeBlock>		Blocks;
		int							nCurrentBlockIndex;
		int							nTotalNodes;
	};
}
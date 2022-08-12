#pragma once

#include <vector>

#include "../Maths/Box3d.h"

class AABBTreeOffline;
struct AABBTreeNodeInference;

enum class SplitHeuristic : uint8_t
{
	Space,
	TreeBalance,
};

struct AABBTreeBuildData
{
	AABBTreeBuildData(const Box3d* pArray, int nGeometries = 0, int GeometriesPerNode = 1) :
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
	const Box3d*				pAABBArray;
	Vector3*					pCenterBuffer;		// Holds the memory
	int*						pIndicesBase;
	AABBTreeOffline*			pAABBTree;
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
	Box3d					aabb;
	AABBTreeNodeOffline*	child1;
	AABBTreeNodeOffline*	child2;
	int						indexOffset;
	int						numGeometries;
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

	void Release()
	{
		for (size_t i = 0; i < Blocks.size(); ++i)
		{
			NodeBlock& s = Blocks[i];
			delete[]s.pMem;
		}

		Blocks.clear();
		nCurrentBlockIndex = 0;
		nTotalNodes = 0;
	}

	void Init(int nGeometries, int nGeometriesPerNode)
	{
		const int maxSize = nGeometries * 2 - 1;
		const int estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / nGeometriesPerNode;
		pHead = new  AABBTreeNodeOffline[estimatedFinalSize];
		memset(pHead, 0, sizeof(AABBTreeNodeOffline) * estimatedFinalSize);

		pHead->indexOffset = 0;
		pHead->numGeometries = nGeometries;

		Blocks.emplace_back(pHead, 1, estimatedFinalSize);
		nCurrentBlockIndex = 0;
		nTotalNodes = 1;
	}

	void Build(AABBTreeBuildData &params)
	{
		pHead->BuildHierarchyRecursive(params);
	}

	AABBTreeNodeOffline* AllocNodes()
	{
		nTotalNodes += 2;
		NodeBlock& currentBlock = Blocks[nCurrentBlockIndex];
		if (currentBlock.nUsedNodes + 2 <= currentBlock.nMaxNodes)
		{
			AABBTreeNodeOffline* p = currentBlock.pMem + currentBlock.nUsedNodes;
			currentBlock.nUsedNodes += 2;
			return p;
		}
		else
		{
			// Allocate new Block
			const int size = 1024;
			AABBTreeNodeOffline* p = new AABBTreeNodeOffline[size];
			memset(p, 0, sizeof(AABBTreeNodeOffline) * size);

			Blocks.emplace_back(p, 2, size);
			nCurrentBlockIndex++;
			return p;
		}
	}

	AABBTreeNodeInference* BuildInferenceTree();

private:
	AABBTreeNodeOffline* pHead;

	struct NodeBlock
	{
		NodeBlock() {}
		NodeBlock(AABBTreeNodeOffline* p, int UsedNodes, int maxNodes) :
			pMem(p),
			nUsedNodes(UsedNodes),
			nMaxNodes(maxNodes) {}
		AABBTreeNodeOffline*	pMem;
		int						nUsedNodes;
		int						nMaxNodes;
	};
	std::vector<NodeBlock>		Blocks;
	int							nCurrentBlockIndex;
	int							nTotalNodes;
};

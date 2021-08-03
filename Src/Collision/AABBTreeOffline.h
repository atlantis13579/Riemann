
#pragma once

#include "../Maths/Box3d.h"

#include <vector>

class AABBTreeOffline;
struct AABBTreeNodeInference;

struct AABBTreeBuildData
{
	AABBTreeBuildData(const Box3d* pArray, int nGeometries = 0, int GeometriesPerNode = 1) :
		NumGeometriesPerNode(GeometriesPerNode),
		NumGeometries(nGeometries),
		pAABBArray(pArray),
		pCenterBuffer(nullptr),
		pIndicesBase(nullptr),
		pAABBTree(nullptr)
	{

	}

	~AABBTreeBuildData()
	{
		NumGeometriesPerNode = 0;
		NumGeometries = 0;
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

	int							NumGeometriesPerNode;	
	int							NumGeometries;
	const Box3d*				pAABBArray;
	Vector3d*					pCenterBuffer;		// Holds the memory
	int*						pIndicesBase;
	AABBTreeOffline*			pAABBTree;
};

class AABBTreeNodeOffline
{
public:
	AABBTreeNodeOffline()
	{
		Children[0] = Children[1] = nullptr;
	}

	~AABBTreeNodeOffline()
	{
		Children[0] = Children[1] = nullptr;
	}

	bool	IsLeafNode() const
	{
		if (Children[0] == nullptr)
		{
			return true;
		}
		return false;
	}

	int		SplitAxis(const AABBTreeBuildData& Params, int* pGeometries, int Num, int Axis);
	void	SubDivideAABBArray(AABBTreeBuildData& Params);
	void	BuildHierarchyRecursive(AABBTreeBuildData& Params);

public:
	Box3d							BV;
	AABBTreeNodeOffline*			Children[2];
	int								IndexOffset;
	int								NumGeometries;
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

		pHead->IndexOffset = 0;
		pHead->NumGeometries = nGeometries;

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

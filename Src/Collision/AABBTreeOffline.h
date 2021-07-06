
#pragma once

#include "../Maths/BoundingBox3d.h"

#include <vector>

class AABBTreeOffline;
struct AABBTreeNodeInference;

struct AABBTreeBuildData
{
	AABBTreeBuildData(const BoundingBox3d* pArray, int nPrimitives = 0, int PrimitivesPerNode = 1) :
		NumPrimitivesPerNode(PrimitivesPerNode),
		NumPrimitives(nPrimitives),
		pAABBArray(pArray),
		pCenterBuffer(nullptr),
		pIndexBase(nullptr),
		pAABBTree(nullptr)
	{

	}

	~AABBTreeBuildData()
	{
		NumPrimitivesPerNode = 0;
		NumPrimitives = 0;
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
		pIndexBase = nullptr;
	}

	int							NumPrimitivesPerNode;	
	int							NumPrimitives;
	const BoundingBox3d*		pAABBArray;
	Vector3d*					pCenterBuffer;		// Holds the memory
	int*						pIndexBase;
	AABBTreeOffline*			pAABBTree;
};

class AABBTreeNodeOffline
{
public:
	AABBTreeNodeOffline()
	{
		pLeftNode = pRightNode = nullptr;
	}

	~AABBTreeNodeOffline()
	{
		pLeftNode = pRightNode = nullptr;
	}

	bool	IsLeafNode() const
	{
		if (pLeftNode == nullptr)
		{
			return true;
		}
		return false;
	}

	int		SplitAxis(const AABBTreeBuildData& params, int* prims, int nb, int axis);
	void	SubDivideAABBArray(AABBTreeBuildData& params);
	void	BuildHierarchyRecursive(AABBTreeBuildData& params);

public:
	BoundingBox3d					mBV;
	AABBTreeNodeOffline				*pLeftNode;
	AABBTreeNodeOffline				*pRightNode;
	int								IndexOffset;
	int								NumPrimitives;
};


class AABBTreeOffline
{
public:
	AABBTreeOffline() : pMem(nullptr), nCurrentBlockIndex(0), nTotalNodes(0)
	{

	}

	~AABBTreeOffline()
	{
		Release();
	}

	void Release()
	{
		for (size_t i = 0; i < Blocks.size(); i++)
		{
			Block& s = Blocks[i];
			delete[]s.pMem;
		}

		Blocks.clear();
		nCurrentBlockIndex = 0;
		nTotalNodes = 0;
	}

	void Init(int nPrimitives, int nPrimitivesPerNode)
	{
		const int maxSize = nPrimitives * 2 - 1;
		const int estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / nPrimitivesPerNode;
		pMem = new  AABBTreeNodeOffline[estimatedFinalSize];
		memset(pMem, 0, sizeof(AABBTreeNodeOffline) * estimatedFinalSize);

		pMem->IndexOffset = 0;
		pMem->NumPrimitives = nPrimitives;

		Blocks.emplace_back(pMem, 1, estimatedFinalSize);
		nCurrentBlockIndex = 0;
		nTotalNodes = 1;
	}

	void Build(AABBTreeBuildData &params)
	{
		pMem->BuildHierarchyRecursive(params);
	}

	AABBTreeNodeOffline* AllocNodes()
	{
		nTotalNodes += 2;
		Block& currentBlock = Blocks[nCurrentBlockIndex];
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

	AABBTreeNodeInference* BuildCompactTree();

private:
	AABBTreeNodeOffline* pMem;

	struct Block
	{
		Block() {}
		Block(AABBTreeNodeOffline* p, int UsedNodes, int maxNodes) :
			pMem(p),
			nUsedNodes(UsedNodes),
			nMaxNodes(maxNodes) {}
		AABBTreeNodeOffline*	pMem;
		int						nUsedNodes;
		int						nMaxNodes;
	};
	std::vector<Block>		Blocks;
	int						nCurrentBlockIndex;
	int						nTotalNodes;
};

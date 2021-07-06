
#pragma once

#include "../Maths/BoundingBox3d.h"

#include <vector>

struct AABBTreeBuildParams
{
	AABBTreeBuildParams(const BoundingBox3d* pArray, int nPrimitives = 0, int PrimitivesPerNode = 1) :
		NumPrimitivesPerNode(PrimitivesPerNode),
		NumPrimitives(nPrimitives),
		pAABBArray(pArray),
		pCenterBuffer(nullptr),
		pIndexBase(nullptr)
	{

	}

	~AABBTreeBuildParams()
	{
		Relase();
	}

	void	Relase()
	{
		NumPrimitivesPerNode = 0;
		NumPrimitives = 0;
		pAABBArray = nullptr;
		if (pCenterBuffer)
		{
			delete[]pCenterBuffer;
			pCenterBuffer = nullptr;
		}
	}

	int			NumPrimitivesPerNode;	
	int			NumPrimitives;
	const	BoundingBox3d* pAABBArray;
	Vector3d*	pCenterBuffer;
	int*		pIndexBase;
};

class AABBTreeNodePool;

class AABBTreeNode
{
public:
	AABBTreeNode()
	{
		pLeftNode = nullptr;
	}

	~AABBTreeNode()
	{
		pLeftNode = nullptr;
	}

	const BoundingBox3d& GetBoundingBox() const
	{
		return mBV;
	}

	const AABBTreeNode* GetLeftNode() const
	{
		return pLeftNode;
	}

	const AABBTreeNode* GetRightNode() const
	{
		const AABBTreeNode* p = pLeftNode;
		if (p)
		{
			return p + 1;
		}
		return nullptr;
	}

	bool	IsLeafNode() const
	{
		if (pLeftNode == nullptr)
		{
			return true;
		}
		return false;
	}

	int		SplitAxis(const AABBTreeBuildParams& params, int* prims, int nb, int axis);
	void	SubDivideAABBArray(AABBTreeBuildParams& params, AABBTreeNodePool* Pool);
	void	BuildHierarchyRecursive(AABBTreeBuildParams& params, AABBTreeNodePool* Pool);

	const int* GetPrimitives(const int* Base) const
	{
		return Base + IndexOffset;
	}

	int* GetPrimitives(int* Base)
	{
		return Base + IndexOffset;
	}

public:
	BoundingBox3d					mBV;
	AABBTreeNode					*pLeftNode;
	int								IndexOffset;
	int								NumPrimitives;
};


class AABBTreeNodePool
{
public:
	AABBTreeNodePool() : pMem(nullptr), nCurrentSlabIndex(0), nTotalNodes(0)
	{

	}

	~AABBTreeNodePool()
	{
		Release();
	}

	void Release()
	{
		const size_t nbSlabs = Blocks.size();
		for (size_t i = 0; i < nbSlabs; i++)
		{
			Block& s = Blocks[i];
			delete[]s.pMem;
		}

		Blocks.clear();
		nCurrentSlabIndex = 0;
		nTotalNodes = 0;
	}

	void Init(int nbPrimitives, int limit)
	{
		const int maxSize = nbPrimitives * 2 - 1;
		const int estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / limit;
		pMem = new  AABBTreeNode[estimatedFinalSize];
		memset(pMem, 0, sizeof(AABBTreeNode) * estimatedFinalSize);

		pMem->IndexOffset = 0;
		pMem->NumPrimitives = nbPrimitives;

		Blocks.emplace_back(pMem, 1, estimatedFinalSize);
		nCurrentSlabIndex = 0;
		nTotalNodes = 1;
	}

	AABBTreeNode* Malloc2()
	{
		nTotalNodes += 2;
		Block& currentSlab = Blocks[nCurrentSlabIndex];
		if (currentSlab.nUsedNodes + 2 <= currentSlab.nMaxNodes)
		{
			AABBTreeNode* biNode = currentSlab.pMem + currentSlab.nUsedNodes;
			currentSlab.nUsedNodes += 2;
			return biNode;
		}
		else
		{
			// Allocate new Block
			const int size = 1024;
			AABBTreeNode* pool = new AABBTreeNode[size];
			memset(pool, 0, sizeof(AABBTreeNode) * size);

			Blocks.emplace_back(pool, 2, size);
			nCurrentSlabIndex++;
			return pool;
		}
	}

	AABBTreeNode* pMem;

	struct Block
	{
		Block() {}
		Block(AABBTreeNode* p, int UsedNodes, int maxNodes) :
			pMem(p),
			nUsedNodes(UsedNodes),
			nMaxNodes(maxNodes) {}
		AABBTreeNode*		pMem;
		int					nUsedNodes;
		int					nMaxNodes;
	};
	std::vector<Block>		Blocks;
	int						nCurrentSlabIndex;
	int						nTotalNodes;
};


class AABBTree
{
public:
	AABBTree();
	~AABBTree();

	void Release();

public:
	void StaticBuild(AABBTreeBuildParams& params);

private:
	void InitAABBTreeBuild(AABBTreeBuildParams& params);

private:
	int*		m_PrimitiveNodeIndices;
	int						m_NumPrimitives;
	AABBTreeNodePool*		m_NodePool;

};
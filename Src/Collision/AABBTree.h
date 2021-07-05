
#pragma once

#include "../Maths/BoundingBox3d.h"

typedef unsigned int	PrimitiveNode;

struct AABBTreeBuildParams
{
	AABBTreeBuildParams(const BoundingBox3d* pArray, int nPrimitives = 0, int PrimitivesPerNode = 1) :
		mPrimitivesPerNode(PrimitivesPerNode),
		mPrimitives(nPrimitives),
		mAABBArray(pArray)
	{

	}

	~AABBTreeBuildParams()
	{
		Relase();
	}

	void	Relase()
	{
		mPrimitivesPerNode = 0;
		mPrimitives = 0;
		mAABBArray = nullptr;
	}

	int			mPrimitivesPerNode;	
	int			mPrimitives;
	const	BoundingBox3d* mAABBArray;
};

class AABBTreeNode
{
public:
	AABBTreeNode()
	{
		m_NodePtr = nullptr;
	}

	~AABBTreeNode()
	{
		m_NodePtr = nullptr;
	}

	const BoundingBox3d& GetBoundingBox() const
	{
		return m_BoundingBox;
	}

	const AABBTreeNode* GetLeftNode() const
	{
		return m_NodePtr;
	}

	const AABBTreeNode* GetRightNode() const
	{
		const AABBTreeNode* p = m_NodePtr;
		if (p)
		{
			return p + 1;
		}
		return nullptr;
	}

	bool IsLeafNode() const
	{
		if (m_NodePtr == nullptr)
		{
			return true;
		}
		return false;
	}

	void SubDivideSpace(AABBTreeBuildParams& params, AABBTreeNode* Pool, PrimitiveNode*& Primitives);

	void BuildHierarchyRecursive(AABBTreeBuildParams& params, AABBTreeNode* Pool, PrimitiveNode*& Primitives);

	const PrimitiveNode* GetPrimitives(const PrimitiveNode* Base) const
	{
		return Base + m_NodeIndex;
	}

	PrimitiveNode* GetPrimitives(PrimitiveNode* Base)
	{
		return Base + m_NodeIndex;
	}

private:
	BoundingBox3d					m_BoundingBox;
	AABBTreeNode					*m_NodePtr;
	int								m_NodeIndex;
	int								mNumPrimitives;
};

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void Build(AABBTreeBuildParams& params);

private:
	void InitAABBTreeBuild(AABBTreeBuildParams& params);

	void Release();

private:
	PrimitiveNode*			m_Primitives;
	int						m_NumPrimitives;
	AABBTreeNode*			m_NodePool;
	Vector3d*				m_CenterCache;

};
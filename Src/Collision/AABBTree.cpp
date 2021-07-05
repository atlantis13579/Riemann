
#include "AABBTree.h"

void AABBTreeNode::BuildHierarchyRecursive(AABBTreeBuildParams& params, AABBTreeNode* Pool, PrimitiveNode*& Primitives)
{
	SubDivideSpace(params, Pool, Primitives);

	if (!IsLeafNode())
	{
		AABBTreeNode* Left = m_NodePtr;
		AABBTreeNode* Right = Left + 1;
		Left->BuildHierarchyRecursive(params, Pool, Primitives);
		Right->BuildHierarchyRecursive(params, Pool, Primitives);
	}

}

void AABBTreeNode::SubDivideSpace(AABBTreeBuildParams& params, AABBTreeNode* Pool, PrimitiveNode*& Primitives)
{

}

AABBTree::AABBTree()
{
	m_Primitives = nullptr;
	m_NumPrimitives = 0;
	m_NodePool = nullptr;
}

AABBTree::~AABBTree()
{
	Release();
}

void AABBTree::Release()
{
	if (m_NodePool)
	{
		delete []m_NodePool;
		m_NodePool = nullptr;
	}
	m_Primitives = nullptr;
	m_NumPrimitives = 0;
	m_CenterCache = nullptr;
}

void AABBTree::Build(AABBTreeBuildParams& params)
{
	InitAABBTreeBuild(params);

	m_NodePool->BuildHierarchyRecursive(params, m_NodePool, m_Primitives);

	if (m_CenterCache)
	{
		delete[]m_CenterCache;
		m_CenterCache = nullptr;
	}
}

void AABBTree::InitAABBTreeBuild(AABBTreeBuildParams& params)
{
	if (m_Primitives)
		return;

	m_Primitives = new PrimitiveNode[params.mPrimitives];
	for (int i = 0; i < params.mPrimitives; ++i)
		m_Primitives[i] = (PrimitiveNode)i;

	const int maxSize = params.mPrimitives * 2 - 1;
	const int estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / params.mPrimitivesPerNode;
	m_NodePool = new AABBTreeNode[estimatedFinalSize];
	memset(m_NodePool, 0, sizeof(m_NodePool[0]) * estimatedFinalSize);

	m_CenterCache = new Vector3d[params.mPrimitives + 1];
	for (int i = 0; i < params.mPrimitives; i++)
	{
		m_CenterCache[i] = params.mAABBArray[i].GetCenter();
	}

	return;
}


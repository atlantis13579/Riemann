
#include "AABBTree.h"

#include <assert.h>

#include "AABBTreeOffline.h"
#include "AABBTreeInference.h"

AABBTree::AABBTree()
{
	m_PrimitiveIndicesBase = nullptr;
	m_NumPrimitives = 0;
	m_AABBTreeInference = nullptr;
}

AABBTree::~AABBTree()
{
	Release();
}

void AABBTree::Release()
{
	m_NumPrimitives = 0;
	if (m_PrimitiveIndicesBase)
	{
		delete[]m_PrimitiveIndicesBase;
		m_PrimitiveIndicesBase = nullptr;
	}
	if (m_AABBTreeInference)
	{
		delete[]m_AABBTreeInference;
		m_AABBTreeInference = nullptr;
	}
}

void AABBTree::StaticBuild(AABBTreeBuildData& params)
{
	InitAABBTreeBuild(params);

	params.pAABBTree->Build(params);

	params.Release();

	assert(params.pAABBTree);
	m_AABBTreeInference = params.pAABBTree->BuildCompactTree();

	delete params.pAABBTree;
	params.pAABBTree = nullptr;
}

int AABBTree::Traverse(const Vector3d& Point) const
{
	AABBTreeNodeInference* p = m_AABBTreeInference;
	if (p == nullptr || !p->mBV.IsInside(Point))
	{
		return -1;
	}

	while (p)
	{
		if (p->IsLeafNode())
		{
			return *p->GetPrimitiveIndices(m_PrimitiveIndicesBase);
		}

		if (p->GetLeftNode(m_AABBTreeInference)->mBV.IsInside(Point))
		{
			p = p->GetLeftNode(m_AABBTreeInference);
			continue;
		}
		else if (p->GetRightNode(m_AABBTreeInference)->mBV.IsInside(Point))
		{
			p = p->GetRightNode(m_AABBTreeInference);
			continue;
		}

		return *p->GetPrimitiveIndices(m_PrimitiveIndicesBase);
	}

	return -1;
}

void AABBTree::InitAABBTreeBuild(AABBTreeBuildData& params)
{
	if (m_PrimitiveIndicesBase)
		return;

	m_PrimitiveIndicesBase = new int[params.NumPrimitives];
	for (int i = 0; i < params.NumPrimitives; ++i)
		m_PrimitiveIndicesBase[i] = (int)i;

	params.pAABBTree = new AABBTreeOffline();
	params.pAABBTree->Init(params.NumPrimitives, params.NumPrimitivesPerNode);

	params.pIndexBase = m_PrimitiveIndicesBase;
	params.pCenterBuffer = (Vector3d*) new float[(params.NumPrimitives + 1)*3];
	for (int i = 0; i < params.NumPrimitives; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


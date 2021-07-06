
#include "AABBTree.h"
#include <assert.h>

void AABBTreeNode::BuildHierarchyRecursive(AABBTreeBuildParams& params, AABBTreeNodePool* Pool)
{
	SubDivideAABBArray(params, Pool);

	if (!IsLeafNode())
	{
		AABBTreeNode* Left = this->pLeftNode;
		AABBTreeNode* Right = Left + 1;
		Left->BuildHierarchyRecursive(params, Pool);
		Right->BuildHierarchyRecursive(params, Pool);
	}
}

int AABBTreeNode::SplitAxis(const AABBTreeBuildParams& params, int *pPrimitives, int numPrimitives, int axis)
{
	const float splitValue = (mBV.Min[axis] + mBV.Max[axis]) * 0.5f;
	int nbPos = 0;

	for (int i = 0; i < numPrimitives; ++i)
	{
		const int index = pPrimitives[i];
		const float primitiveValue = params.pCenterBuffer[index][axis];
		assert(primitiveValue == params.pCenterBuffer[index][axis]);

		if (primitiveValue > splitValue)
		{
			pPrimitives[i] = pPrimitives[nbPos];
			pPrimitives[nbPos] = index;
			nbPos++;
		}
	}
	return nbPos;
}

void AABBTreeNode::SubDivideAABBArray(AABBTreeBuildParams& params, AABBTreeNodePool* Pool)
{
	int* primitives = params.pIndexBase + this->IndexOffset;
	int nbPrims = NumPrimitives;

	Vector3d meansV = params.pCenterBuffer[primitives[0]];
	const BoundingBox3d* pAABB = params.pAABBArray;

	Vector3d minV = pAABB[primitives[0]].Min;
	Vector3d maxV = pAABB[primitives[0]].Max;

	for (int i = 1; i < nbPrims; i++)
	{
		int index = primitives[i];
		const Vector3d& curMinV = pAABB[index].Min;
		const Vector3d& curMaxV = pAABB[index].Max;

		meansV += params.pCenterBuffer[index];
		
		minV = minV.Min(curMinV);
		maxV = maxV.Max(curMaxV);
	}

	mBV = BoundingBox3d(minV, maxV);

	if (nbPrims <= params.NumPrimitivesPerNode)
		return;

	const float coeff = 1.0f / float(nbPrims);
	meansV *= coeff;

	Vector3d varsV = Vector3d::Zero();
	for (int i = 0; i < nbPrims; i++)
	{
		int index = primitives[i];
		Vector3d centerV = params.pCenterBuffer[index];
		centerV = centerV - meansV;
		centerV = centerV * centerV;
		varsV = varsV + centerV;
	}
	
	const float coeffNb1 = 1.0f / float(nbPrims - 1);
	varsV *= coeffNb1;

	const int axis = varsV.LargestAxis();

	int	nbPos = SplitAxis(params, primitives, NumPrimitives, axis);
	
	bool validSplit = true;
	if (!nbPos || nbPos == nbPrims)
		validSplit = false;

	if (!validSplit)
	{
		if (nbPrims > params.NumPrimitivesPerNode)
		{
			nbPos = nbPrims >> 1;
		}
		else
		{
			return;
		}
	}

	this->pLeftNode = Pool->Malloc2();

	// Assign children
	assert(!IsLeafNode());
	AABBTreeNode* Left = this->pLeftNode;
	AABBTreeNode* Right = Left + 1;
	Left->IndexOffset = this->IndexOffset;
	Left->NumPrimitives = nbPos;
	Right->IndexOffset = this->IndexOffset + nbPos;
	Right->NumPrimitives = NumPrimitives - nbPos;
}

AABBTree::AABBTree()
{
	m_PrimitiveNodeIndices = nullptr;
	m_NumPrimitives = 0;
	m_NodePool = nullptr;
}

AABBTree::~AABBTree()
{
	Release();
}

void AABBTree::Release()
{
	m_NumPrimitives = 0;
	if (m_NodePool)
	{
		delete m_NodePool;
		m_NodePool = nullptr;
	}
	if (m_PrimitiveNodeIndices)
	{
		delete[]m_PrimitiveNodeIndices;
		m_PrimitiveNodeIndices = nullptr;
	}
}

void AABBTree::StaticBuild(AABBTreeBuildParams& params)
{
	InitAABBTreeBuild(params);

	m_NodePool->pMem->BuildHierarchyRecursive(params, m_NodePool);
}

void AABBTree::InitAABBTreeBuild(AABBTreeBuildParams& params)
{
	if (m_PrimitiveNodeIndices)
		return;

	m_PrimitiveNodeIndices = new int[params.NumPrimitives];
	for (int i = 0; i < params.NumPrimitives; ++i)
		m_PrimitiveNodeIndices[i] = (int)i;

	m_NodePool = new AABBTreeNodePool();
	m_NodePool->Init(params.NumPrimitives, params.NumPrimitivesPerNode);

	params.pIndexBase = m_PrimitiveNodeIndices;
	params.pCenterBuffer = (Vector3d*) new float[(params.NumPrimitives + 1)*3];
	for (int i = 0; i < params.NumPrimitives; i++)
	{
		params.pCenterBuffer[i] = params.pAABBArray[i].GetCenter();
	}

	return;
}


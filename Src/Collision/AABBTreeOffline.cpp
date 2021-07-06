
#include "AABBTree.h"
#include <assert.h>

void AABBTreeNodeOffline::BuildHierarchyRecursive(AABBTreeBuildData& params)
{
	SubDivideAABBArray(params);

	if (!IsLeafNode())
	{
		this->pLeftNode->BuildHierarchyRecursive(params);
		this->pRightNode->BuildHierarchyRecursive(params);
	}
}

int AABBTreeNodeOffline::SplitAxis(const AABBTreeBuildData& params, int *pPrimitives, int numPrimitives, int axis)
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

void AABBTreeNodeOffline::SubDivideAABBArray(AABBTreeBuildData& params)
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

	this->pLeftNode = params.pAABBTree->AllocNodes();
	this->pRightNode = this->pLeftNode + 1;

	// Assign children
	assert(!IsLeafNode());
	this->pLeftNode->IndexOffset = this->IndexOffset;
	this->pLeftNode->NumPrimitives = nbPos;
	this->pRightNode->IndexOffset = this->IndexOffset + nbPos;
	this->pRightNode->NumPrimitives = NumPrimitives - nbPos;
}


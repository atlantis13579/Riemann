
#include "AABBTree.h"
#include <assert.h>

void AABBTreeNodeOffline::BuildHierarchyRecursive(AABBTreeBuildData& params)
{
	SubDivideAABBArray(params);

	if (!IsLeafNode())
	{
		this->Children[0]->BuildHierarchyRecursive(params);
		this->Children[1]->BuildHierarchyRecursive(params);
	}
}

int AABBTreeNodeOffline::SplitAxis(const AABBTreeBuildData& params, int *pPrimitives, int numPrimitives, int axis)
{
	const float SplitValue = (mBV.Min[axis] + mBV.Max[axis]) * 0.5f;
	int nSplitLeft = 0;

	for (int i = 0; i < numPrimitives; ++i)
	{
		const int index = pPrimitives[i];
		const float primitiveValue = params.pCenterBuffer[index][axis];
		assert(primitiveValue == params.pCenterBuffer[index][axis]);

		if (primitiveValue > SplitValue)
		{
			pPrimitives[i] = pPrimitives[nSplitLeft];
			pPrimitives[nSplitLeft] = index;
			++nSplitLeft;
		}
	}
	return nSplitLeft;
}

void AABBTreeNodeOffline::SubDivideAABBArray(AABBTreeBuildData& params)
{
	int* primitives = params.pIndexBase + this->IndexOffset;
	int nPrims = NumPrimitives;

	Vector3d meansV = params.pCenterBuffer[primitives[0]];
	const BoundingBox3d* pAABB = params.pAABBArray;

	Vector3d minV = pAABB[primitives[0]].Min;
	Vector3d maxV = pAABB[primitives[0]].Max;

	for (int i = 1; i < nPrims; ++i)
	{
		int index = primitives[i];
		const Vector3d& curMinV = pAABB[index].Min;
		const Vector3d& curMaxV = pAABB[index].Max;

		meansV += params.pCenterBuffer[index];
		
		minV = minV.Min(curMinV);
		maxV = maxV.Max(curMaxV);
	}

	mBV = BoundingBox3d(minV, maxV);

	if (nPrims <= params.NumPrimitivesPerNode)
		return;

	meansV *= 1.0f / float(nPrims);

	Vector3d varsV = Vector3d::Zero();
	for (int i = 0; i < nPrims; ++i)
	{
		int index = primitives[i];
		Vector3d centerV = params.pCenterBuffer[index];
		centerV = centerV - meansV;
		centerV = centerV * centerV;
		varsV = varsV + centerV;
	}
	
	varsV *= 1.0f / float(nPrims - 1);

	const int axis = varsV.LargestAxis();

	int	nSplitLeft = SplitAxis(params, primitives, NumPrimitives, axis);
	
	bool validSplit = true;
	if (!nSplitLeft || nSplitLeft == nPrims)
		validSplit = false;

	if (!validSplit)
	{
		if (nPrims > params.NumPrimitivesPerNode)
		{
			nSplitLeft = nPrims >> 1;
		}
		else
		{
			return;
		}
	}

	this->Children[0] = params.pAABBTree->AllocNodes();
	this->Children[1] = this->Children[0] + 1;

	// Assign children
	assert(!IsLeafNode());
	this->Children[0]->IndexOffset = this->IndexOffset;
	this->Children[0]->NumPrimitives = nSplitLeft;
	this->Children[1]->IndexOffset = this->IndexOffset + nSplitLeft;
	this->Children[1]->NumPrimitives = NumPrimitives - nSplitLeft;
}


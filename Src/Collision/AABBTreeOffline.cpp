
#include "AABBTree.h"
#include <assert.h>

void AABBTreeNodeOffline::BuildHierarchyRecursive(AABBTreeBuildData& params)
{
	SubDivideAABBArray(params);

	if (!IsLeafNode())
	{
		this->child1->BuildHierarchyRecursive(params);
		this->child2->BuildHierarchyRecursive(params);
	}
}

int AABBTreeNodeOffline::SplitAxis(const AABBTreeBuildData& Params, int *pGeometries, int Num, int Axis)
{
	const float SplitValue = (aabb.mMin[Axis] + aabb.mMax[Axis]) * 0.5f;
	int nSplitLeft = 0;

	for (int i = 0; i < Num; ++i)
	{
		const int Index = pGeometries[i];
		const float Value = Params.pCenterBuffer[Index][Axis];
		assert(Value == Params.pCenterBuffer[Index][Axis]);

		if (Value > SplitValue)
		{
			pGeometries[i] = pGeometries[nSplitLeft];
			pGeometries[nSplitLeft] = Index;
			++nSplitLeft;
		}
	}
	return nSplitLeft;
}

void AABBTreeNodeOffline::SubDivideAABBArray(AABBTreeBuildData& Params)
{
	int* geoms = Params.pIndicesBase + this->indexOffset;
	int nPrims = numGeometries;

	Vector3 meansV = Params.pCenterBuffer[geoms[0]];
	const Box3d* pAABB = Params.pAABBArray;

	Vector3 minV = pAABB[geoms[0]].mMin;
	Vector3 maxV = pAABB[geoms[0]].mMax;

	for (int i = 1; i < nPrims; ++i)
	{
		int index = geoms[i];
		const Vector3& curMinV = pAABB[index].mMin;
		const Vector3& curMaxV = pAABB[index].mMax;

		meansV += Params.pCenterBuffer[index];
		
		minV = minV.Min(curMinV);
		maxV = maxV.Max(curMaxV);
	}

	aabb = Box3d(minV, maxV);

	if (nPrims <= Params.numGeometriesPerNode)
		return;

	meansV *= 1.0f / float(nPrims);

	Vector3 varsV = Vector3::Zero();
	for (int i = 0; i < nPrims; ++i)
	{
		int index = geoms[i];
		Vector3 centerV = Params.pCenterBuffer[index];
		centerV = centerV - meansV;
		centerV = centerV * centerV;
		varsV = varsV + centerV;
	}
	
	varsV *= 1.0f / float(nPrims - 1);

	const int axis = varsV.LargestAxis();

	int	nSplitLeft = SplitAxis(Params, geoms, numGeometries, axis);
	
	bool validSplit = true;
	if (!nSplitLeft || nSplitLeft == nPrims)
		validSplit = false;

	if (!validSplit)
	{
		if (nPrims > Params.numGeometriesPerNode)
		{
			nSplitLeft = nPrims >> 1;
		}
		else
		{
			return;
		}
	}

	this->child1 = Params.pAABBTree->AllocNodes();
	this->child2 = this->child1 + 1;

	// Assign children
	assert(!IsLeafNode());
	this->child1->indexOffset = this->indexOffset;
	this->child1->numGeometries = nSplitLeft;
	this->child2->indexOffset = this->indexOffset + nSplitLeft;
	this->child2->numGeometries = numGeometries - nSplitLeft;
}


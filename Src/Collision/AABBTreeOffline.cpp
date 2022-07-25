
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

int AABBTreeNodeOffline::SplitAxis(const AABBTreeBuildData& Params, int *pGeometries, int Num, int Axis)
{
	const float SplitValue = (BV.Min[Axis] + BV.Max[Axis]) * 0.5f;
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
	int* geoms = Params.pIndicesBase + this->IndexOffset;
	int nPrims = NumGeometries;

	Vector3 meansV = Params.pCenterBuffer[geoms[0]];
	const Box3d* pAABB = Params.pAABBArray;

	Vector3 minV = pAABB[geoms[0]].Min;
	Vector3 maxV = pAABB[geoms[0]].Max;

	for (int i = 1; i < nPrims; ++i)
	{
		int index = geoms[i];
		const Vector3& curMinV = pAABB[index].Min;
		const Vector3& curMaxV = pAABB[index].Max;

		meansV += Params.pCenterBuffer[index];
		
		minV = minV.Min(curMinV);
		maxV = maxV.Max(curMaxV);
	}

	BV = Box3d(minV, maxV);

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

	int	nSplitLeft = SplitAxis(Params, geoms, NumGeometries, axis);
	
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

	this->Children[0] = Params.pAABBTree->AllocNodes();
	this->Children[1] = this->Children[0] + 1;

	// Assign children
	assert(!IsLeafNode());
	this->Children[0]->IndexOffset = this->IndexOffset;
	this->Children[0]->NumGeometries = nSplitLeft;
	this->Children[1]->IndexOffset = this->IndexOffset + nSplitLeft;
	this->Children[1]->NumGeometries = NumGeometries - nSplitLeft;
}


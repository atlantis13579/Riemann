
#include "AABBTree.h"
#include <assert.h>
#include <algorithm>

void AABBTreeNodeOffline::BuildHierarchyRecursive(AABBTreeBuildData& params)
{
	SubDivideAABBArray(params);

	if (!IsLeafNode())
	{
		this->child1->BuildHierarchyRecursive(params);
		this->child2->BuildHierarchyRecursive(params);
	}
}

int AABBTreeNodeOffline::SplitAxisBySpace(const AABBTreeBuildData& Params, int *pGeometries, int Num, int Axis)
{
	const float SplitValue = (aabb.mMin[Axis] + aabb.mMax[Axis]) * 0.5f;
	int nSplitLeft = 0;

	for (int i = 0; i < Num; ++i)
	{
		const int Index = pGeometries[i];
		const float Value = Params.pCenterBuffer[Index][Axis];
		if (Value > SplitValue)
		{
			pGeometries[i] = pGeometries[nSplitLeft];
			pGeometries[nSplitLeft] = Index;
			++nSplitLeft;
		}
	}
	return nSplitLeft;
}

int	AABBTreeNodeOffline::SplitAxisByNumGeometries(const AABBTreeBuildData& Params, int* pGeometries, int Num, int Axis)
{
	std::vector<float> AxisValue;
	AxisValue.resize(Num);
	
	for (int i = 0; i < Num; ++i)
	{
		AxisValue[i] = Params.pCenterBuffer[pGeometries[i]][Axis];
	}
	std::nth_element(AxisValue.begin(), AxisValue.begin() + (Num >> 1), AxisValue.end());
	
	const float SplitValue = AxisValue[Num >> 1];
	int nSplitLeft = 0;
	for (int i = 0; i < Num; ++i)
	{
		const int Index = pGeometries[i];
		const float Value = Params.pCenterBuffer[Index][Axis];
		if (Value >= SplitValue)
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
	int nGeoms = numGeometries;

	Vector3 means = Params.pCenterBuffer[geoms[0]];
	const Box3d* pAABB = Params.pAABBArray;

	Vector3 bMin = pAABB[geoms[0]].mMin;
	Vector3 bMax = pAABB[geoms[0]].mMax;

	for (int i = 1; i < nGeoms; ++i)
	{
		int index = geoms[i];
		const Vector3& mMin = pAABB[index].mMin;
		const Vector3& mMax = pAABB[index].mMax;

		means += Params.pCenterBuffer[index];
		
		bMin = bMin.Min(mMin);
		bMax = bMax.Max(mMax);
	}

	aabb = Box3d(bMin, bMax);

	if (nGeoms <= Params.numGeometriesPerNode)
		return;

	means *= 1.0f / float(nGeoms);

	Vector3 vars = Vector3::Zero();
	for (int i = 0; i < nGeoms; ++i)
	{
		int index = geoms[i];
		Vector3 center = Params.pCenterBuffer[index];
		vars = vars + (center - means) * (center - means);
	}
	
	vars *= 1.0f / float(nGeoms - 1);

	const int axis = vars.LargestAxis();

	int	nSplitLeft = 0;
	if (Params.splitter == SplitHeuristic::Space)
	{
		nSplitLeft = SplitAxisBySpace(Params, geoms, numGeometries, axis);
	}
	else if (Params.splitter == SplitHeuristic::TreeBalance)
	{
		nSplitLeft = SplitAxisByNumGeometries(Params, geoms, numGeometries, axis);
	}
	else
	{
		assert(false);
	}
	
	if (nSplitLeft == 0 || nSplitLeft == nGeoms)
	{
		if (nGeoms > Params.numGeometriesPerNode)
		{
			nSplitLeft = nGeoms >> 1;
		}
		else
		{
			return;
		}
	}

	this->child1 = Params.pAABBTree->AllocNodes();
	this->child2 = this->child1 + 1;

	assert(!IsLeafNode());
	this->child1->indexOffset = this->indexOffset;
	this->child1->numGeometries = nSplitLeft;
	this->child2->indexOffset = this->indexOffset + nSplitLeft;
	this->child2->numGeometries = numGeometries - nSplitLeft;
}



#include "AABBTreeOffline.h"
#include "AABBTreeInference.h"
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


void AABBTreeOffline::Release()
{
	for (size_t i = 0; i < Blocks.size(); ++i)
	{
		NodeBlock& s = Blocks[i];
		delete[]s.pMem;
	}

	Blocks.clear();
	nCurrentBlockIndex = 0;
	nTotalNodes = 0;
}

void AABBTreeOffline::Init(int nGeometries, int nGeometriesPerNode)
{
	const int maxSize = nGeometries * 2 - 1;
	const int estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / nGeometriesPerNode;
	pHead = new  AABBTreeNodeOffline[estimatedFinalSize];
	memset(pHead, 0, sizeof(AABBTreeNodeOffline) * estimatedFinalSize);

	pHead->indexOffset = 0;
	pHead->numGeometries = nGeometries;

	Blocks.emplace_back(pHead, 1, estimatedFinalSize);
	nCurrentBlockIndex = 0;
	nTotalNodes = 1;
}

void AABBTreeOffline::Build(AABBTreeBuildData &params)
{
	pHead->BuildHierarchyRecursive(params);
}

AABBTreeNodeOffline* AABBTreeOffline::AllocNodes()
{
	nTotalNodes += 2;
	NodeBlock& currentBlock = Blocks[nCurrentBlockIndex];
	if (currentBlock.nUsedNodes + 2 <= currentBlock.nMaxNodes)
	{
		AABBTreeNodeOffline* p = currentBlock.pMem + currentBlock.nUsedNodes;
		currentBlock.nUsedNodes += 2;
		return p;
	}
	else
	{
		// Allocate new Block
		const int size = 1024;
		AABBTreeNodeOffline* p = new AABBTreeNodeOffline[size];
		memset(p, 0, sizeof(AABBTreeNodeOffline) * size);

		Blocks.emplace_back(p, 2, size);
		nCurrentBlockIndex++;
		return p;
	}
}

AABBTreeNodeInference* AABBTreeOffline::BuildInferenceTree()
{
	if (nTotalNodes <= 0)
	{
		return nullptr;
	}

	AABBTreeNodeInference* Compact = (AABBTreeNodeInference*)new char[sizeof(AABBTreeNodeInference) * nTotalNodes];
	memset(Compact, 0, sizeof(AABBTreeNodeInference) * nTotalNodes);

	int offset = 0;
	for (size_t k = 0; k < Blocks.size(); ++k)
	{
		const NodeBlock& block = Blocks[k];

		AABBTreeNodeOffline* p = block.pMem;
		for (int i = 0; i < block.nUsedNodes; ++i)
		{
			Compact[offset].aabb = p[i].aabb;
			if (p[i].IsLeafNode())
			{
				const int index = p[i].indexOffset;
				const int nPrimitives = p[i].numGeometries;
				assert(nPrimitives <= 16);

				Compact[offset].Data = (index << 5) | ((nPrimitives & 15) << 1) | 1;
			}
			else
			{
				assert(p[i].child1);
				assert(p[i].child2);
				uint32_t localNodeIndex = 0xffffffff;
				uint32_t nodeBase = 0;
				for (size_t j = 0; j < Blocks.size(); ++j)
				{
					if (p[i].child1 >= Blocks[j].pMem && p[i].child1 < Blocks[j].pMem + Blocks[j].nUsedNodes)
					{
						localNodeIndex = (uint32_t)(p[i].child1 - Blocks[j].pMem);
						break;
					}
					nodeBase += Blocks[j].nUsedNodes;
				}
				const uint32_t nodeIndex = nodeBase + localNodeIndex;
				Compact[offset].Data = nodeIndex << 1;
			}
			offset++;
		}
	}

	return Compact;
}

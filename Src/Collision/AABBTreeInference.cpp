
#include "AABBTreeInference.h"
#include "AABBTreeOffline.h"

#include <assert.h>
#include <vector>

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

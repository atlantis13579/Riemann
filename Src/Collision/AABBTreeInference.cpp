
#include "AABBTreeInference.h"
#include "AABBTreeOffline.h"

#include <assert.h>
#include <vector>

AABBTreeNodeInference* AABBTreeOffline::BuildCompactTree()
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
		const MemoryBlock& block = Blocks[k];

		AABBTreeNodeOffline* p = block.pMem;
		for (int i = 0; i < block.nUsedNodes; ++i)
		{
			Compact[offset].BV = p[i].mBV;
			if (p[i].IsLeafNode())
			{
				const int index = p[i].IndexOffset;

				const int nbPrims = p[i].NumPrimitives;
				assert(nbPrims <= 16);

				Compact[offset].Data = (index << 5) | ((nbPrims & 15) << 1) | 1;
			}
			else
			{
				assert(p[i].Children[0]);
				unsigned int localNodeIndex = 0xffffffff;
				unsigned int nodeBase = 0;
				for (size_t j = 0; j < Blocks.size(); ++j)
				{
					if (p[i].Children[0] >= Blocks[j].pMem && p[i].Children[0] < Blocks[j].pMem + Blocks[j].nUsedNodes)
					{
						localNodeIndex = (unsigned int)(p[i].Children[0] - Blocks[j].pMem);
						break;
					}
					nodeBase += Blocks[j].nUsedNodes;
				}
				const unsigned int nodeIndex = nodeBase + localNodeIndex;
				Compact[offset].Data = nodeIndex << 1;
			}
			offset++;
		}
	}

	return Compact;
}

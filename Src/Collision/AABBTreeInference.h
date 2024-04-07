#pragma once

#include <stdint.h>

#include "../Maths/Box3.h"

namespace Riemann
{
	class AABBTreeOffline;

	struct CacheFriendlyAABBTree
	{
		inline const Box3& GetBoundingVolume() const
		{
			return aabb;
		}

		inline bool			IsLeaf() const
		{
			return Data & 1;
		}

		inline const int* GetGeometryIndices(const int* Base) const
		{
			return Base + (Data >> 5);
		}

		inline int* GetGeometryIndices(int* Base) const
		{
			return Base + (Data >> 5);
		}

		inline int			GetNumGeometries() const
		{
			return (Data >> 1) & 15;
		}

		inline int			GetLeftNode() const
		{
			return Data >> 1;
		}

		inline int			GetRightNode() const
		{
			return (Data >> 1) + 1;
		}

		inline const CacheFriendlyAABBTree* GetLeftNode(const CacheFriendlyAABBTree* Base)	const
		{
			return Base + (Data >> 1);
		}

		inline CacheFriendlyAABBTree* GetLeftNode(CacheFriendlyAABBTree* Base)
		{
			return Base + (Data >> 1);
		}

		inline const CacheFriendlyAABBTree* GetRightNode(const CacheFriendlyAABBTree* Base) const
		{
			return Base + (Data >> 1) + 1;
		}

		inline CacheFriendlyAABBTree* GetRightNode(CacheFriendlyAABBTree* Base)
		{
			return Base + (Data >> 1) + 1;
		}

		Box3				aabb;
		uint32_t			Data;		// 27 bits node index | 4 bits #Geometries | 1 bit leaf
	};

	static_assert(sizeof(CacheFriendlyAABBTree) == 28, "Size of AABBTreeNodeInference is not valid");
}
#pragma once

#include <stdint.h>

#include "../Maths/Box3d.h"

class AABBTreeOffline;

struct AABBTreeNodeInference
{
	inline const Box3d& GetBoundingVolume() const
	{
		return aabb;
	}

	inline bool			IsLeaf() const
	{
		return Data & 1;
	}

	inline const int*	GetGeometryIndices(const int* Base) const
	{
		return Base + (Data >> 5);
	}

	inline int*			GetGeometryIndices(int* Base) const
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

	inline const AABBTreeNodeInference* GetLeftNode(const AABBTreeNodeInference* Base)	const
	{
		return Base + (Data >> 1);
	}

	inline AABBTreeNodeInference* GetLeftNode(AABBTreeNodeInference* Base)
	{
		return Base + (Data >> 1);
	}

	inline const AABBTreeNodeInference* GetRightNode(const AABBTreeNodeInference* Base) const
	{
		return Base + (Data >> 1) + 1;
	}

	inline AABBTreeNodeInference* GetRightNode(AABBTreeNodeInference* Base)
	{
		return Base + (Data >> 1) + 1;
	}

	Box3d				aabb;
	uint32_t			Data;		// 27 bits node index | 4 bits #Geometries | 1 bit leaf
};

static_assert(sizeof(AABBTreeNodeInference) == 28, "Size of AABBTreeNodeInference is not valid");

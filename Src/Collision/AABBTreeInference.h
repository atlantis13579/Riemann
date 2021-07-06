
#pragma once

#include "../Maths/BoundingBox3d.h"

class AABBTreeOffline;

struct AABBTreeNodeInference
{
	inline const BoundingBox3d& GetBoundingBox() const
	{
		return mBV;
	}

	inline bool			IsLeafNode() const
	{
		return mData & 1;
	}

	inline const int*	GetPrimitiveIndices(const int* Base) const
	{
		return Base + (mData >> 5);
	}

	inline int*			GetPrimitiveIndices(int* Base) const
	{
		return Base + (mData >> 5);
	}

	inline int			GetNumPrimitives() const
	{
		return (mData >> 1) & 15;
	}

	inline int			GetLeftNode() const
	{
		return mData >> 1;
	}

	inline int			GetRightNode() const
	{
		return (mData >> 1) + 1;
	}

	inline const AABBTreeNodeInference* GetLeftNode(const AABBTreeNodeInference* Base)	const
	{
		return Base + (mData >> 1);
	}

	inline AABBTreeNodeInference* GetLeftNode(AABBTreeNodeInference* Base)
	{
		return Base + (mData >> 1);
	}

	inline const AABBTreeNodeInference* GetRightNode(const AABBTreeNodeInference* Base) const
	{
		const AABBTreeNodeInference* Left = GetLeftNode(Base);
		if (Left)
		{
			return Left + 1;
		}
		return nullptr;
	}

	inline AABBTreeNodeInference* GetRightNode(AABBTreeNodeInference* Base)
	{
		AABBTreeNodeInference* Left = GetLeftNode(Base);
		if (Left)
		{
			return Left + 1;
		}
		return nullptr;
	}

	BoundingBox3d				mBV;
	unsigned int				mData;	// 27 bits node or prim index|4 bits #prims|1 bit leaf
};

static_assert(sizeof(AABBTreeNodeInference) == 28, "Size of AABBTreeNodeInference is not valid");
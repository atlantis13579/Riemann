
#pragma once

#include "../Maths/BoundingBox3d.h"
#include "AABBTreeOffline.h"

struct AABBTreeNodeInference;

class AABBTree
{
public:
	AABBTree();
	~AABBTree();

public:
	void Release();

	void StaticBuild(AABBTreeBuildData& params);

	int	 Traverse(const Vector3d& Point) const;

private:
	void InitAABBTreeBuild(AABBTreeBuildData& params);

private:
	int*						m_PrimitiveIndicesBase;
	int							m_NumPrimitives;

	AABBTreeNodeInference		*m_AABBTreeInference;
};
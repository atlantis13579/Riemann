
#pragma once

#include <assert.h>
#include <float.h>
#include "../Maths/Vector3d.h"
#include "../Maths/Box3d.h"

inline void* AlignMemory(void *Memory, int Width)
{
#if INTPTR_MAX == INT32_MAX
	return = (void*)((((unsigned int)(Memory + Width - 1) / Width) * Width);
#else
	return (void*)((((unsigned long long)Memory + Width - 1) / Width) * Width);
#endif
}

#define RTREE_N		4

struct RTreePage;

struct RTreeNodeQ
{
	float			minx, miny, minz, maxx, maxy, maxz;
	unsigned int	ptr; // lowest bit is leaf flag

	 void setLeaf(bool set) { if (set) ptr |= 1; else ptr &= ~1; }
	 unsigned int isLeaf() const { return ptr & 1; }
	 void setEmpty()
	{
		minx = miny = minz = FLT_MAX;
		maxx = maxy = maxz = -FLT_MAX;
	}
	 void grow(const RTreePage& page, int nodeIndex);
	 void grow(const RTreeNodeQ& node);
};

struct RTreePage
{
	float minx[RTREE_N];
	float miny[RTREE_N];
	float minz[RTREE_N];
	float maxx[RTREE_N];
	float maxy[RTREE_N];
	float maxz[RTREE_N];
	unsigned int ptrs[RTREE_N];

	unsigned int	nodeCount() const; // returns the number of occupied nodes in this page
	void	setEmpty(unsigned int startIndex = 0);
	bool	isEmpty(unsigned int index) const { return minx[index] > maxx[index]; }
	void	copyNode(unsigned int targetIndex, const RTreePage& sourcePage, unsigned int sourceIndex);
	void	setNode(unsigned int targetIndex, const RTreeNodeQ& node);
	void	clearNode(unsigned int nodeIndex);
	void	getNode(unsigned int nodeIndex, RTreeNodeQ& result) const;
	void	computeBounds(RTreeNodeQ& bounds);
	void	adjustChildBounds(unsigned int index, const RTreeNodeQ& adjustedChildBounds);
	void	growChildBounds(unsigned int index, const RTreeNodeQ& adjustedChildBounds);
	unsigned int	getNodeHandle(unsigned int index) const;
	unsigned int	isLeaf(unsigned int index) const { return ptrs[index] & 1; }

};

__declspec(align(16))
class RTree
{
public:
	RTree()
	{
		mFlags = 0;
		mPages = nullptr;
		mTotalNodes = 0;
		mNumLevels = 0;
		mPageSize = RTREE_N;
	}

	~RTree()
	{
		release();
	}

	void release()
	{
		if ((mFlags & USER_ALLOCATED) != 0)
		{
			delete mPages;
			mPages = nullptr;
		}
	}

	// bool load(PxInputStream& stream, unsigned int meshVersion, bool mismatch);

	class CallbackRaycast
	{
	public:
		virtual ~CallbackRaycast() {}
		virtual bool processResults(unsigned int count, unsigned int* buf, float& newMaxT) = 0;
	};

	void		traverseRay(const Vector3d& Origin, const Vector3d& Dir, CallbackRaycast* cb, float maxT = FLT_MAX) const;

	void		openTextDump();
	void		closeTextDump();
	void		textDump(const char* prefix);
	void		maxscriptExport();
	unsigned int		computeBottomLevelCount(unsigned int storedToMemMultiplier) const;

	void		validate();
	void		validateRecursive(unsigned int level, RTreeNodeQ parentBounds, RTreePage* page);

	Vector4d			mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler; // 16
	unsigned int		mPageSize;
	unsigned int		mNumRootPages;
	unsigned int		mNumLevels;
	unsigned int		mTotalNodes; // 16
	unsigned int		mTotalPages;
	unsigned int		mFlags; enum { USER_ALLOCATED = 0x1, IS_EDGE_SET = 0x2 };
	RTreePage			*mPages;

protected:
	typedef unsigned int NodeHandle;

	friend struct RTreePage;
};


// bit 1 is always expected to be set to differentiate between leaf and non-leaf node
inline unsigned int LeafGetNbTriangles(unsigned int Data) { return ((Data >> 1) & 15) + 1; }
inline unsigned int LeafGetTriangleIndex(unsigned int Data) { return Data >> 5; }
inline unsigned int LeafSetData(unsigned int nb, unsigned int index)
{
	assert(nb > 0 && nb <= 16); assert(index < (1 << 27));
	return (index << 5) | (((nb - 1) & 15) << 1) | 1;
}

struct LeafTriangles
{
	unsigned int			Data;

	// Gets number of triangles in the leaf, returns the number of triangles N, with 0 < N <= 16
		unsigned int	GetNbTriangles()				const { return LeafGetNbTriangles(Data); }

	// Gets triangle index for this leaf. Indexed model's array of indices retrieved with RTreeMidphase::GetIndices()
		unsigned int	GetTriangleIndex()				const { return LeafGetTriangleIndex(Data); }
		void	SetData(unsigned int nb, unsigned int index) { Data = LeafSetData(nb, index); }
};

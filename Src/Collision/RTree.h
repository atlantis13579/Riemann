
#pragma once

#include <assert.h>
#include <float.h>
#include "../Maths/Vector3d.h"
#include "../Maths/Box3d.h"

inline void* AlignMemory(void *Memory, int Width)
{
#if INTPTR_MAX == INT32_MAX
	return = (void*)((((uint32_t)(Memory + Width - 1) / Width) * Width);
#else
	return (void*)((((uint64_t)Memory + Width - 1) / Width) * Width);
#endif
}

#define RTREE_N		4

struct RTreePage;

struct RTreeNodeQ
{
	float			minx, miny, minz, maxx, maxy, maxz;
	uint32_t	ptr; // lowest bit is leaf flag

	 void setLeaf(bool set) { if (set) ptr |= 1; else ptr &= ~1; }
	 uint32_t isLeaf() const { return ptr & 1; }
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
	uint32_t ptrs[RTREE_N];

	uint32_t	nodeCount() const; // returns the number of occupied nodes in this page
	void	setEmpty(uint32_t startIndex = 0);
	bool	isEmpty(uint32_t index) const { return minx[index] > maxx[index]; }
	void	copyNode(uint32_t targetIndex, const RTreePage& sourcePage, uint32_t sourceIndex);
	void	setNode(uint32_t targetIndex, const RTreeNodeQ& node);
	void	clearNode(uint32_t nodeIndex);
	void	getNode(uint32_t nodeIndex, RTreeNodeQ& result) const;
	void	computeBounds(RTreeNodeQ& bounds);
	void	adjustChildBounds(uint32_t index, const RTreeNodeQ& adjustedChildBounds);
	void	growChildBounds(uint32_t index, const RTreeNodeQ& adjustedChildBounds);
	uint32_t	getNodeHandle(uint32_t index) const;
	uint32_t	isLeaf(uint32_t index) const { return ptrs[index] & 1; }

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

	// bool load(PxInputStream& stream, uint32_t meshVersion, bool mismatch);

	class CallbackRaycast
	{
	public:
		virtual ~CallbackRaycast() {}
		virtual bool processResults(uint32_t count, uint32_t* buf, float& newMaxT) = 0;
	};

	void		traverseRay(const Vector3d& Origin, const Vector3d& Dir, CallbackRaycast* cb, float maxT = FLT_MAX) const;

	void		openTextDump();
	void		closeTextDump();
	void		textDump(const char* prefix);
	void		maxscriptExport();
	uint32_t		computeBottomLevelCount(uint32_t storedToMemMultiplier) const;

	void		validate();
	void		validateRecursive(uint32_t level, RTreeNodeQ parentBounds, RTreePage* page);

	Vector4d			mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler; // 16
	uint32_t		mPageSize;
	uint32_t		mNumRootPages;
	uint32_t		mNumLevels;
	uint32_t		mTotalNodes; // 16
	uint32_t		mTotalPages;
	uint32_t		mFlags; enum { USER_ALLOCATED = 0x1, IS_EDGE_SET = 0x2 };
	RTreePage			*mPages;

protected:
	typedef uint32_t NodeHandle;

	friend struct RTreePage;
};


// bit 1 is always expected to be set to differentiate between leaf and non-leaf node
inline uint32_t LeafGetNbTriangles(uint32_t Data) { return ((Data >> 1) & 15) + 1; }
inline uint32_t LeafGetTriangleIndex(uint32_t Data) { return Data >> 5; }
inline uint32_t LeafSetData(uint32_t nb, uint32_t index)
{
	assert(nb > 0 && nb <= 16); assert(index < (1 << 27));
	return (index << 5) | (((nb - 1) & 15) << 1) | 1;
}

struct LeafTriangles
{
	uint32_t			Data;

	// Gets number of triangles in the leaf, returns the number of triangles N, with 0 < N <= 16
		uint32_t	GetNbTriangles()				const { return LeafGetNbTriangles(Data); }

	// Gets triangle index for this leaf. Indexed model's array of indices retrieved with RTreeMidphase::GetIndices()
		uint32_t	GetTriangleIndex()				const { return LeafGetTriangleIndex(Data); }
		void	SetData(uint32_t nb, uint32_t index) { Data = LeafSetData(nb, index); }
};

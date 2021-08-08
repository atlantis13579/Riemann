
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

#define SIMD_WIDTH		4

struct BVHNodeBatch;

struct BVHNode
{
	float			minx;
	float			miny;
	float			minz;
	float			maxx;
	float			maxy;
	float			maxz;
	uint32_t		Data;
	void			SetLeaf(bool isLeaf)
	{
		if (isLeaf)
		{
			Data |= 1;
		}
		else
		{
			Data &= ~1;
		}
	}
	uint32_t		IsLeaf() const
	{
		return Data & 1;
	}
	void			SetEmpty()
	{
		minx = miny = minz = FLT_MAX;
		maxx = maxy = maxz = -FLT_MAX;
	}
	void Grow(const BVHNodeBatch& page, int nodeIndex);
	void Grow(const BVHNode& node);
};

struct LeafNode
{
	LeafNode(uint32_t nb, uint32_t index)
	{
		assert(nb > 0 && nb <= 16); assert(index < (1 << 27));
		Data = (index << 5) | (((nb - 1) & 15) << 1) | 1;
	}

	LeafNode(uint32_t _Data)
	{
		Data = _Data;
	}

	uint32_t			Data;

	uint32_t			GetNumTriangles()				const
	{
		return ((Data >> 1) & 15) + 1;
	}

	uint32_t			GetTriangleIndex()				const
	{
		return Data >> 5;
	
	}
};

struct BVHNodeBatch
{
	float		minx[SIMD_WIDTH];
	float		miny[SIMD_WIDTH];
	float		minz[SIMD_WIDTH];
	float		maxx[SIMD_WIDTH];
	float		maxy[SIMD_WIDTH];
	float		maxz[SIMD_WIDTH];
	uint32_t	Data[SIMD_WIDTH];

	uint32_t	GetNodeCount() const;
	BVHNode		GetNode(uint32_t nodeIndex) const;
	bool		IsEmpty(uint32_t index) const
	{
		return minx[index] > maxx[index];
	}
	Box3d		ComputeBounds();
	uint32_t	IsLeaf(uint32_t index) const
	{
		return Data[index] & 1;
	}
};

struct alignas(16) MeshBVH4
{
public:
	enum
	{
		USER_ALLOCATED = 0x1,
	};

	MeshBVH4()
	{
		memset(this, 0, sizeof(MeshBVH4));
		BatchSize = SIMD_WIDTH;
	}

	~MeshBVH4()
	{
		Release();
	}

	void			Release()
	{
		if ((Flags & USER_ALLOCATED) != 0)
		{
			delete BatchPtr;
			BatchPtr = nullptr;
		}
	}

	void			Validate(void* p);
	void			ValidateRecursive(void* p, uint32_t Depth, const Box3d& parentBounds, BVHNodeBatch* batch);

	static void		BuildFromBounds(MeshBVH4& bvh, uint8_t*& Memory, const std::vector<Box3d>& allBounds, std::vector<uint32_t>& Permute, const Box3d& meshBounds);

	Vector4d		BoundsMin, BoundsMax;
	float			____Unuse1[8];
	uint32_t		BatchSize;
	uint32_t		NumRoots;
	uint32_t		MaxDepth;
	uint32_t		NumNodes;
	uint32_t		NumBatches;
	uint32_t		Flags; 
	BVHNodeBatch	*BatchPtr;

protected:
	typedef uint32_t NodeHandle;

	friend struct BVHNodeBatch;
};

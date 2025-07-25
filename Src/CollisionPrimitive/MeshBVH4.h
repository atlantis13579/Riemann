#pragma once

#include <assert.h>
#include <float.h>
#include <stdint.h>
#include <vector>

#include "../Core/Base.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
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
		LeafNode(const uint32_t nb, const uint32_t index)
		{
			assert(nb > 0 && nb <= 16); assert(index < (1 << 27));
			Data = (index << 5) | (((nb - 1) & 15) << 1) | 1;
		}

		explicit LeafNode(uint32_t iData)
		{
			Data = iData;
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
		Box3		ComputeBounds();
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
			BatchPtr = nullptr;
			Memory = nullptr;
		}

		~MeshBVH4()
		{
			Release();
		}

		void			Release()
		{
			if ((Flags & USER_ALLOCATED) != 0)
			{
				delete Memory;
			}
			Memory = nullptr;
			BatchPtr = nullptr;
		}

		void			Validate(void* p);
		void			ValidateRecursive(void* p, uint32_t Depth, const Box3& parentBounds, BVHNodeBatch* batch);

		void* AllocMemory(int Size, int Width)
		{
			if (Memory != nullptr)
			{
				delete[] Memory;
			}
			Memory = new uint8_t[Size + Width - 1];
			BatchPtr = (BVHNodeBatch*)AlignMemory(Memory, Width);
			Flags |= USER_ALLOCATED;
			return BatchPtr;
		}

		static void		BuildFromBounds(MeshBVH4& bvh, const std::vector<Box3>& allBounds, std::vector<uint32_t>& Permute, const Box3& meshBounds);

		Vector4			BoundsMin, BoundsMax;
		uint8_t*		Memory;
		float			____unuse1[6];
		uint32_t		BatchSize;
		uint32_t		NumRoots;
		uint32_t		MaxDepth;
		uint32_t		NumNodes;
		uint32_t		NumBatches;
		uint32_t		Flags;
		BVHNodeBatch*	BatchPtr;

	protected:
		typedef uint32_t NodeHandle;

		friend struct BVHNodeBatch;
	};
}
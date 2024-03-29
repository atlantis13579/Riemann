
#pragma once

#include <math.h>
#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	struct SpatialEntiry
	{
		void* Entity;
		SpatialEntiry* Next;
	};

	inline uint32_t hash_id_3d(int x, int y, int z, int bucketSize)
	{
		const uint32_t h1 = 0x8da6b343; // large multiplicative constants
		const uint32_t h2 = 0xd8163841; // here arbitrarialy chosen primes
		const uint32_t h3 = 0xcb1ab31f;
		uint32_t n = h1 * x + h2 * y + h3 * z;
		n = n % bucketSize;
		if (n < 0) n += bucketSize;
		return n;
	}

	inline uint32_t hash_id_2d(int x, int y, int bucketSize)
	{
		const uint32_t h1 = 0x8da6b343; // large multiplicative constants
		const uint32_t h2 = 0xd8163841; // here arbitrarialy chosen primes
		uint32_t n = h1 * x + h2 * y;
		n = n % bucketSize;
		if (n < 0) n += bucketSize;
		return n;
	}

	class SparseSpatialHash
	{
	public:
		SparseSpatialHash()
		{
			m_Origin = Vector3::Zero();
			m_CellSize = Vector3::One();
			m_InvCellSize = Vector3::One();
		}

		~SparseSpatialHash()
		{
			m_Buckets.clear();
		}

		int GetBucketSize() const
		{
			return (int)m_Buckets.size();
		}

		uint32_t ComputeHashBucketIndex2D(const Vector3& Pos) const
		{
			int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
			int z = (int)(floorf((Pos.z - m_Origin.z) * m_InvCellSize.z));
			return hash_id_2d(x, z, GetBucketSize());
		}

		uint32_t ComputeHashBucketIndex3D(const Vector3& Pos) const
		{
			int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
			int y = (int)(floorf((Pos.y - m_Origin.y) * m_InvCellSize.y));
			int z = (int)(floorf((Pos.z - m_Origin.z) * m_InvCellSize.z));
			return hash_id_3d(x, y, z, GetBucketSize());
		}

	private:
		Vector3 m_Origin;
		Vector3 m_CellSize, m_InvCellSize;
		std::vector<SpatialEntiry>	m_Buckets;
	};
}
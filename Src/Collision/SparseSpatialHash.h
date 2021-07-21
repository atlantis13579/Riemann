
#pragma once

#include <math.h>
#include <vector>

#include "../Maths/Vector3d.h"

struct SpatialEntiry
{
	void* Entity;
	SpatialEntiry* Next;
};

inline unsigned int hash_id_3d(int x, int y, int z, int bucketSize)
{
	const unsigned int h1 = 0x8da6b343; // large multiplicative constants
	const unsigned int h2 = 0xd8163841; // here arbitrarialy chosen primes
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	n = n % bucketSize;
	if (n < 0) n += bucketSize;
	return n;
}

inline unsigned int hash_id_2d(int x, int y, int bucketSize)
{
	const unsigned int h1 = 0x8da6b343; // large multiplicative constants
	const unsigned int h2 = 0xd8163841; // here arbitrarialy chosen primes
	unsigned int n = h1 * x + h2 * y;
	n = n % bucketSize;
	if (n < 0) n += bucketSize;
	return n;
}

class SparseSpatialHash
{
public:
	SparseSpatialHash()
	{
		m_Origin = Vector3d::Zero();
		m_CellSize = Vector3d::One();
		m_InvCellSize = Vector3d::One();
	}

	~SparseSpatialHash()
	{
		m_Buckets.clear();
	}

	int GetBucketSize() const
	{
		return (int)m_Buckets.size();
	}

	unsigned int ComputeHashBucketIndex2D(const Vector3d& Pos) const
	{
		int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
		int z = (int)(floorf((Pos.z - m_Origin.z) * m_InvCellSize.z));
		return hash_id_2d(x, z, GetBucketSize());
	}

	unsigned int ComputeHashBucketIndex3D(const Vector3d& Pos) const
	{
		int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
		int y = (int)(floorf((Pos.y - m_Origin.y) * m_InvCellSize.y));
		int z = (int)(floorf((Pos.z - m_Origin.z) * m_InvCellSize.z));
		return hash_id_3d(x, y, z, GetBucketSize());
	}

private:
	Vector3d m_Origin;
	Vector3d m_CellSize, m_InvCellSize;
	std::vector<SpatialEntiry>	m_Buckets;
};
#pragma once

#include <math.h>
#include <vector>

#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	inline uint32_t hash_id_2d(int x, int y, int bucketSize)
	{
		const uint32_t h1 = 0x8da6b343; // large multiplicative constants
		const uint32_t h2 = 0xd8163841; // here arbitrarialy chosen primes
		uint32_t n = h1 * x + h2 * y;
		n = n % bucketSize;
		if (n < 0) n += bucketSize;
		return n;
	}

	template<class T>
	class SparseSpatialHash2
	{
	public:
		struct Element
		{
			Element(const Vector2& _pos, const T& _value)
			{
				pos = _pos;
				value = _value;
			}
			Vector2	pos;
			T		value;
		};

		SparseSpatialHash2(float cellsize, int bucket_size)
		{
			m_Origin = Vector2::Zero();
			m_CellSize = Vector2(cellsize, cellsize);
			m_InvCellSize = Vector2(1.0f) / m_CellSize;
			m_Buckets.resize(bucket_size);
		}

		~SparseSpatialHash2()
		{
			m_Buckets.clear();
		}

		int GetBucketSize() const
		{
			return (int)m_Buckets.size();
		}

		void Insert(const Vector2& pos, const T& val)
		{
			uint32_t idx = ComputeHashBucketIndex2D(pos);
			m_Buckets[idx].emplace_back(pos, val);
		}

		bool Reomve(const Vector3& pos, const T& val)
		{
			uint32_t idx = ComputeHashBucketIndex2D(pos);
			std::vector<Element>& bucket = m_Buckets[idx];
			for (size_t i = 0; i < bucket.size(); ++i)
			{
				if (bucket[i] == val)
				{
					bucket[i] = bucket[bucket.size() - 1];
					return true;
				}
			}
			return false;
		}

		void AllIteration(std::function<void(const Vector2&, const T&)> func) const
		{
			for (size_t j = 0; j < m_Buckets.size(); ++j)
			{
				std::vector<Element>& bucket = m_Buckets[j];
				for (size_t i = 0; i < bucket.size(); ++i)
				{
					const Element& e = bucket[i];
					func(e.pos, e.value);
				}
			}
		}

		void RangeIteration(const Vector2& center, float radius, std::function<void(const Vector2&, const T&)> func) const
		{
			const int x1 = static_cast<int>(floorf((center.x - radius) * m_InvCellSize.x));
			const int y1 = static_cast<int>(floorf((center.y - radius) * m_InvCellSize.y));
			const int x2 = static_cast<int>(floorf((center.x + radius) * m_InvCellSize.x));
			const int y2 = static_cast<int>(floorf((center.y + radius) * m_InvCellSize.y));
			const float radius2 = radius * radius;
			for (int x = x1; x <= x2; ++x)
			for (int y = y1; y <= y2; ++y)
			{
				uint32_t idx = hash_id_2d(x, y, GetBucketSize());
				const std::vector<Element>& bucket = m_Buckets[idx];
				for (size_t i = 0; i < bucket.size(); ++i)
				{
					const Element &e = bucket[i];
					const float sqr = (e.pos - center).SquareLength();
					if (sqr <= radius2)
					{
						func(e.pos, e.value);
					}
				}
			}
		}

		bool RangeQuery(const Vector2& center, float radius, std::vector<T>& Results) const
		{
			RangeIteration(center, radius,
				[&](const Vector2& pos, const T& value)
				{
					Results.push_back(value);
				}
			);
			return Results.size() > 0;
		}

		bool FindNearest(const Vector2& center, float radius, T& val)
		{
			float sqr_min = FLT_MAX;
			RangeIteration(center, radius,
				[&](const Vector2& pos, const T& value)
				{
					const float sqr = (pos - center).SquareLength();
					if (sqr < sqr_min)
					{
						sqr_min = sqr;
						val = value;
					}
				}
			);
			return sqr_min != FLT_MAX;
		}

		uint32_t ComputeHashBucketIndex2D(const Vector2& Pos) const
		{
			int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
			int y = (int)(floorf((Pos.y - m_Origin.y) * m_InvCellSize.y));
			return hash_id_2d(x, y, GetBucketSize());
		}

	private:
		Vector2								m_Origin;
		Vector2								m_CellSize;
		Vector2								m_InvCellSize;
		std::vector<std::vector<Element>>	m_Buckets;
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

	template<typename T>
	class SparseSpatialHash3
	{
	public:
		struct Element
		{
			Element(const Vector3 &_pos, const T &_value)
			{
				pos = _pos;
				value = _value;
			}
			Vector3	pos;
			T		value;
		};

		SparseSpatialHash3(float cellsize, int bucket_size)
		{
			m_Origin = Vector3::Zero();
			m_CellSize = Vector3(cellsize, cellsize, cellsize);
			m_InvCellSize = Vector3(1.0f) / m_CellSize;
			m_Buckets.resize(bucket_size);
		}

		~SparseSpatialHash3()
		{
			m_Buckets.clear();
		}

		inline int GetBucketSize() const
		{
			return (int)m_Buckets.size();
		}

		void Insert(const Vector3 &pos, const T& val)
		{
			uint32_t idx = ComputeHashBucketIndex3D(pos);
			m_Buckets[idx].emplace_back(pos, val);
		}

		bool Reomve(const Vector3& pos, const T& val)
		{
			uint32_t idx = ComputeHashBucketIndex3D(pos);
			std::vector<Element> &bucket = m_Buckets[idx];
			for (size_t i = 0; i < bucket.size(); ++i)
			{
				if (bucket[i] == val)
				{
					bucket[i] = bucket[bucket.size() - 1];
					return true;
				}
			}
			return false;
		}

		void AllIteration(std::function<void(const Vector3&, const T&)> func) const
		{
			for (size_t j = 0; j < m_Buckets.size(); ++j)
			{
				std::vector<Element>& bucket = m_Buckets[j];
				for (size_t i = 0; i < bucket.size(); ++i)
				{
					const Element& e = bucket[i];
					func(e.pos, e.value);
				}
			}
		}

		void RangeIteration(const Vector3& center, float radius, std::function<void(const Vector3&, const T&)> func) const
		{
			const int x1 = static_cast<int>(floorf((center.x - radius) * m_InvCellSize.x));
			const int y1 = static_cast<int>(floorf((center.y - radius) * m_InvCellSize.y));
			const int z1 = static_cast<int>(floorf((center.z - radius) * m_InvCellSize.z));
			const int x2 = static_cast<int>(floorf((center.x + radius) * m_InvCellSize.x));
			const int y2 = static_cast<int>(floorf((center.y + radius) * m_InvCellSize.y));
			const int z2 = static_cast<int>(floorf((center.z + radius) * m_InvCellSize.z));
			const float radius2 = radius * radius;
			for (int x = x1; x <= x2; ++x)
			for (int y = y1; y <= y2; ++y)
			for (int z = z1; z <= z2; ++z)
			{
				uint32_t idx = hash_id_3d(x, y, z, GetBucketSize());
				const std::vector<Element>& bucket = m_Buckets[idx];
				for (size_t i = 0; i < bucket.size(); ++i)
				{
					const Element& e = bucket[i];
					const float sqr = (e.pos - center).SquareLength();
					if (sqr <= radius2)
					{
						func(e.pos, e.value);
					}
				}
			}
		}

		bool RangeQuery(const Vector3& center, float radius, std::vector<T> &Results) const
		{
			RangeIteration(center, radius,
				[&](const Vector3& pos, const T& value)
				{
					Results.push_back(value);
				}
			);
			return Results.size() > 0;
		}

		bool FindNearest(const Vector3& center, float radius, T &val)
		{
			float sqr_min = FLT_MAX;
			RangeIteration(center, radius,
				[&](const Vector3& pos, const T& value)
				{
					const float sqr = (pos - center).SquareLength();
					if (sqr < sqr_min)
					{
						sqr_min = sqr;
						val = value;
					}
				}
			);
			return sqr_min != FLT_MAX;
		}

		uint32_t ComputeHashBucketIndex3D(const Vector3& Pos) const
		{
			int x = (int)(floorf((Pos.x - m_Origin.x) * m_InvCellSize.x));
			int y = (int)(floorf((Pos.y - m_Origin.y) * m_InvCellSize.y));
			int z = (int)(floorf((Pos.z - m_Origin.z) * m_InvCellSize.z));
			return hash_id_3d(x, y, z, GetBucketSize());
		}

	private:
		Vector3								m_Origin;
		Vector3								m_CellSize;
		Vector3								m_InvCellSize;
		std::vector<std::vector<Element>>	m_Buckets;
	};
}

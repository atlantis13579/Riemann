#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace Riemann
{
	// Reusable open-addressed cell lookup replaces sorting and binary searches.
	// Particle order within each cell is stable, so SPH reduction order is
	// unchanged. Density/velocity sampling and rigid coupling share this index.
	// Reference: Ihmsen et al., A Parallel SPH Implementation on Multi-Core CPUs,
	// https://diglib.eg.org/items/f0dc0c20-6271-4fe3-a8f7-3ac3b8be7b8c
	class ParticleBucketGrid
	{
	public:
		bool empty() const { return m_Used.empty(); }
		size_t size() const { return m_Used.size(); }

		struct Bucket
		{
			uint64_t Key = 0;
			std::vector<int> Indices;
		};

		void clear()
		{
			for (size_t Index : m_Used) m_Buckets[Index].Indices.clear();
			m_Used.clear();
		}

		void Prepare(size_t ParticleCount)
		{
			clear();
			size_t Capacity = 16;
			while (Capacity < ParticleCount * 2) Capacity *= 2;
			if (m_Buckets.size() < Capacity) m_Buckets.resize(Capacity);
		}

		void Insert(uint64_t Key, int ParticleIndex)
		{
			size_t Index = (size_t)Key & (m_Buckets.size() - 1);
			while (!m_Buckets[Index].Indices.empty() && m_Buckets[Index].Key != Key)
				Index = (Index + 1) & (m_Buckets.size() - 1);
			Bucket& Cell = m_Buckets[Index];
			if (Cell.Indices.empty())
			{
				Cell.Key = Key;
				m_Used.push_back(Index);
			}
			Cell.Indices.push_back(ParticleIndex);
		}

		const Bucket* Find(uint64_t Key) const
		{
			if (m_Buckets.empty()) return nullptr;
			size_t Index = (size_t)Key & (m_Buckets.size() - 1);
			while (!m_Buckets[Index].Indices.empty())
			{
				if (m_Buckets[Index].Key == Key) return &m_Buckets[Index];
				Index = (Index + 1) & (m_Buckets.size() - 1);
			}
			return nullptr;
		}

	private:
		std::vector<Bucket> m_Buckets;
		std::vector<size_t> m_Used;
	};
}

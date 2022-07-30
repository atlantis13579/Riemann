#pragma once

#include <vector>

template<class T>
class BatchList
{
public:
	struct OneBatch
	{
		std::vector<T>	stores;
		int				current;
		
		bool			Full() const
		{
			return current >= (int)stores.size();
		}
	};

	BatchList()
	{
	}

	void	Init(int nBatchs, int BatchSize)
	{
		m_allocCount = 0;
		m_freeList = nullptr;
		m_batchSize = BatchSize;
		m_allBatchs.clear();
		m_allBatchs.resize(nBatchs);
		for (int i = 0; i < nBatchs; ++i)
		{
			m_allBatchs[i].current = 0;
			m_allBatchs[i].stores.resize(BatchSize);
		}
		m_currBatch = nBatchs - 1;
	}

	int		GetCount() const
	{
		return m_allocCount;
	}

	T*		Alloc()
	{
		m_allocCount++;

		if (m_freeList)
		{
			T* p = m_freeList;
			m_freeList = m_freeList->next;
			return p;
		}

		OneBatch *Batch = m_allBatchs.empty() ? nullptr : &m_allBatchs[m_currBatch];
		if (Batch == nullptr || (Batch->Full() && m_currBatch >= (int)m_allBatchs.size() - 1))
		{
			m_allBatchs.resize(m_allBatchs.size() + 1);
			m_currBatch = (int)m_allBatchs.size() - 1;
			Batch = &m_allBatchs[m_currBatch];
			Batch->stores.resize(m_batchSize);
			Batch->current = 0;
		}
		else if (Batch->Full() && m_currBatch < (int)m_allBatchs.size() - 1)
		{
			m_currBatch++;
			Batch = &m_allBatchs[m_currBatch];
		}
		T* p = &Batch->stores[Batch->current++];
		return p;
	}

	T*		AllocOneBatch()
	{
		m_allocCount += m_batchSize;

		OneBatch *Batch = m_allBatchs.empty() ? nullptr : &m_allBatchs[m_currBatch];
		if (Batch == nullptr || (Batch->Full() && m_currBatch >= (int)m_allBatchs.size() - 1))
		{
			m_allBatchs.resize(m_allBatchs.size() + 1);
			m_currBatch = (int)m_allBatchs.size() - 1;
			Batch = &m_allBatchs[m_currBatch];
			Batch->stores.resize(m_batchSize);
			Batch->current = 0;
		}
		else if (Batch->Full() && m_currBatch < (int)m_allBatchs.size() - 1)
		{
			m_currBatch++;
			Batch = &m_allBatchs[m_currBatch];
		}
		Batch->current = m_batchSize;
		return &Batch->stores[0];
	}

	void	Free(T* p)
	{
		if (p)
		{
			m_allocCount--;

			p->next = m_freeList;
			m_freeList = p;
		}
	}
	
	void	Clear()
	{
		for (size_t i = 0; i < m_allBatchs.size(); ++i)
		{
			m_allBatchs[i].current = 0;
		}
		m_allocCount = 0;
		m_currBatch = m_allBatchs.empty() ? -1 : 0;
		m_freeList = nullptr;
	}

private:
	std::vector<OneBatch>	m_allBatchs;
	T*						m_freeList;
	int						m_allocCount;
	int						m_currBatch;
	int						m_batchSize;
};

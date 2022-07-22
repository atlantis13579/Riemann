#pragma once

#include <vector>

template<class T>
class BatchList
{
public:
	struct OneBatch
	{
		std::vector<T>	Stores;
		int				Current;
	};

	BatchList()
	{
	}

	void	Init(int nBatchs, int BatchSize)
	{
		m_Count = 0;
		m_FreeList = nullptr;
		m_BatchSize = BatchSize;
		m_Batchs.clear();
		m_Batchs.resize(nBatchs);
		for (int i = 0; i < nBatchs; ++i)
		{
			m_Batchs[i].Current = 0;
			m_Batchs[i].Stores.resize(BatchSize);
		}
	}

	int		GetCount() const
	{
		return m_Count;
	}

	T*		GetBatchData(int i)
	{
		if (i >= (int)m_Batchs.size())
		{
			return nullptr;
		}
		return &m_Batchs[i].Stores[0];
	}

	T*		Alloc()
	{
		m_Count++;

		if (m_FreeList)
		{
			T* p = m_FreeList;
			m_FreeList = m_FreeList->next;
			return p;
		}

		if (m_Batchs.empty() || m_Batchs.back().Current >= (int)m_Batchs.back().Stores.size())
		{
			m_Batchs.resize(m_Batchs.size() + 1);
			m_Batchs.back().Stores.resize(m_BatchSize);
			m_Batchs.back().Current = 0;
		}
		OneBatch& Batch = m_Batchs.back();
		T* p = &Batch.Stores[Batch.Current++];
		return p;
	}

	T*		ReserveOneBatch()
	{
		m_Count += m_BatchSize;

		if (m_Batchs.empty() || m_Batchs.back().Current >= (int)m_Batchs.back().Stores.size())
		{
			m_Batchs.resize(m_Batchs.size() + 1);
			m_Batchs.back().Stores.resize(m_BatchSize);
			m_Batchs.back().Current = 0;
		}
		m_Batchs.back().Current = m_BatchSize;
		return &m_Batchs.back().Stores[0];
	}

	void	Free(T* p)
	{
		if (p)
		{
			m_Count--;

			p->next = m_FreeList;
			m_FreeList = p;
		}
	}

private:
	std::vector<OneBatch>	m_Batchs;
	T*						m_FreeList;
	int						m_Count;
	int						m_BatchSize;
};
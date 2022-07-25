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
		
		bool			Full() const
		{
			return Current >= (int)Stores.size();
		}
	};

	BatchList()
	{
	}

	void	Init(int nBatchs, int BatchSize)
	{
		m_Count = 0;
		m_CurrBatch = 0;
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

	T*		Alloc()
	{
		m_Count++;

		if (m_FreeList)
		{
			T* p = m_FreeList;
			m_FreeList = m_FreeList->next;
			return p;
		}

		OneBatch *Batch = m_Batchs.empty() ? nullptr : &m_Batchs[m_CurrBatch];
		if (Batch == nullptr || (Batch->Full() && m_CurrBatch >= (int)m_Batchs.size() - 1))
		{
			m_Batchs.resize(m_Batchs.size() + 1);
			m_CurrBatch = (int)m_Batchs.size() - 1;
			Batch = &m_Batchs[m_CurrBatch];
			Batch->Stores.resize(m_BatchSize);
			Batch->Current = 0;
		}
		else if (Batch->Full() && m_CurrBatch < (int)m_Batchs.size() - 1)
		{
			m_CurrBatch++;
			Batch = &m_Batchs[m_CurrBatch];
		}
		T* p = &Batch->Stores[Batch->Current++];
		return p;
	}

	T*		AllocOneBatch()
	{
		m_Count += m_BatchSize;

		OneBatch *Batch = m_Batchs.empty() ? nullptr : &m_Batchs[m_CurrBatch];
		if (Batch == nullptr || (Batch->Full() && m_CurrBatch >= (int)m_Batchs.size() - 1))
		{
			m_Batchs.resize(m_Batchs.size() + 1);
			m_CurrBatch = (int)m_Batchs.size() - 1;
			Batch = &m_Batchs[m_CurrBatch];
			Batch->Stores.resize(m_BatchSize);
			Batch->Current = 0;
		}
		else if (Batch->Full() && m_CurrBatch < (int)m_Batchs.size() - 1)
		{
			m_CurrBatch++;
			Batch = &m_Batchs[m_CurrBatch];
		}
		Batch->Current = m_BatchSize;
		return &Batch->Stores[0];
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
	
	void	Clear()
	{
		for (size_t i = 0; i < m_Batchs.size(); ++i)
		{
			m_Batchs[i].Current = 0;
		}
		m_Count = 0;
		m_CurrBatch = m_Batchs.empty() ? -1 : 0;
		m_FreeList = nullptr;
	}

private:
	std::vector<OneBatch>	m_Batchs;
	T*						m_FreeList;
	int						m_Count;
	int						m_CurrBatch;
	int						m_BatchSize;
};

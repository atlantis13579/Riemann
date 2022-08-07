#pragma once

#include <string.h>
#include <vector>
#include "DenseVector.h"

template<typename T>
class TSparseVector
{
public:
	TSparseVector(const T *A, int m)
	{
		m_Size = m;
		m_NumEntries = 0;
		for (int i = 0; i < m; i++)
		{
			if (!FuzzyZero(A[i]))
			{
				++m_NumEntries;
			}
		}

		m_Entries = nullptr;
		m_Indices = nullptr;

		if (m_NumEntries > 0)
		{
			m_Entries = new T[m_NumEntries];
			m_Indices = new int[m_NumEntries];

			memset(m_Entries, 0, sizeof(m_Entries[0])*m_NumEntries);
			memset(m_Indices, 0, sizeof(m_Indices[0])*m_NumEntries);

			int k = 0;
			for (int i = 0; i < m; i++)
			{
				if (!FuzzyZero(A[i]))
				{
					m_Entries[k] = A[i];
					m_Indices[k] = i;
					k++;
				}
			}
		}
	}

	~TSparseVector()
	{
		if (m_Entries)
		{
			delete[]m_Entries;
			m_Entries = nullptr;
		}
		if (m_Indices)
		{
			delete[]m_Indices;
			m_Indices = nullptr;
		}
	}
	
	TDenseVector<T>	ToDense() const
	{
		TDenseVector<T> R(m_Size);
		R.LoadZero();
		for (int k = 0; k < m_NumEntries; ++k)
		{
			int i = m_Indices[k];
			R[i] = m_Entries[k];
		}
		return std::move(R);
	}

private:
	T 	*m_Entries;
	int *m_Indices;
	int m_NumEntries;
	int m_Size;
};

using SparseVector = TSparseVector<float>;

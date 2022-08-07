#pragma once

#include <string.h>
#include <vector>
#include "DenseMatrix.h"

template<typename T>
class TSparseMatrix
{
public:
	TSparseMatrix(const T *A, int m, int n)
	{
		m_Rows = m;
		m_Cols = n;
		m_NumEntries = 0;
		for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
		{
			if (!FuzzyZero(A[i * n + j]))
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
			for (int j = 0; j < n; j++)
			{
				if (!FuzzyZero(A[i*n+j]))
				{
					m_Entries[k] = A[i * n + j];
					m_Indices[k] = i * n + j;
					k++;
				}
			}
		}
	}

	~TSparseMatrix()
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
	
	void MulXCompressed3x3(const T *pIn, T *pOut)
	{
		memset(pOut, 0, sizeof(T) * m_Cols * 3);
		for (int i = 0; i < m_NumEntries; i++)
		{
			int ind = m_Indices[i];
			int x = ind / m_Cols;
			int y = ind % m_Cols;
			T entry = m_Entries[i];
			pOut[x * 3] += entry * pIn[y * 3];
			pOut[x * 3 + 1] += entry * pIn[y * 3 + 1];
			pOut[x * 3 + 2] += entry * pIn[y * 3 + 2];
		}
	}

	void MulXCompressedN(const T* pIn, int nDof, T* pOut)
	{
		memset(pOut, 0, sizeof(T) * m_Cols * nDof);

		for (int i = 0; i < m_NumEntries; i++)
		{
			int ind = m_Indices[i];
			int x = ind / m_Cols;
			int y = ind % m_Cols;
			T entry = m_Entries[i];

			int bx = x * nDof, by = y * nDof;
			for (int j = 0; j < nDof; ++j)
			{
				pOut[bx + j] += entry * pIn[by + j];
			}
		}
	}
	
	TDenseMatrix<T>	ToDense() const
	{
		TDenseMatrix<T> R(m_Rows, m_Cols);
		R.LoadZero();
		for (int k = 0; k < m_NumEntries; ++k)
		{
			int ind = m_Indices[k];
			int i = ind / m_Cols;
			int j = ind % m_Cols;
			R[i][j] = m_Entries[k];
		}
		return std::move(R);
	}

private:
	T 	*m_Entries;
	int *m_Indices;
	int m_NumEntries;
	int m_Rows, m_Cols;
};

using SparseMatrix = TSparseMatrix<float>;

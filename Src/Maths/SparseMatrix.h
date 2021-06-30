#pragma once

#include <math.h>
#include <vector>

class SparseMatrix
{
public:
	SparseMatrix(const float *A, int n)
	{
		m_nSize = n;
		m_numEntries = 0;
		for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
		{
			if (fabsf(A[i * n + j]) > 1e-9)
			{
				++m_numEntries;
			}
		}

		m_entries = nullptr;
		m_indices = nullptr;

		if (m_numEntries > 0)
		{
			m_entries = new float[m_numEntries];
			m_indices = new int[m_numEntries];

			memset(m_entries, 0, sizeof(m_entries[0])*m_numEntries);
			memset(m_indices, 0, sizeof(m_indices[0])*m_numEntries);

			int counter = 0;
			for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
			{
				if (fabsf(A[i*n+j]) > 1e-9)
				{
					m_entries[counter] = A[i * n + j];
					m_indices[counter] = i * n + j;
					counter++;
				}
			}
		}


	}

	~SparseMatrix()
	{
		if (m_entries)
		{
			delete[]m_entries;
			m_entries = nullptr;
		}
		if (m_indices)
		{
			delete[]m_indices;
			m_indices = nullptr;
		}

	}

	void MulXCompressed3x3(const float *in, float *out)
	{
		for (int i = 0; i < m_nSize * 3; i++)
			out[i] = 0;

		for (int i = 0; i < m_numEntries; i++)
		{
			int ind = m_indices[i];
			int x = ind / m_nSize;
			int y = ind % m_nSize;
			float entry = m_entries[i];

			out[x * 3] += entry * in[y * 3];
			out[x * 3 + 1] += entry * in[y * 3 + 1];
			out[x * 3 + 2] += entry * in[y * 3 + 2];
		}
	}

private:
	float *m_entries;
	int *m_indices;
	int m_numEntries;
	int m_nSize;
};


#pragma once

#include <math.h>
#include <vector>

class SparseMatrix
{
public:
	SparseMatrix(const float *A, int n)
	{
		m_Size = n;
		m_NumEntries = 0;
		for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
		{
			if (fabsf(A[i * n + j]) > 1e-9)
			{
				++m_NumEntries;
			}
		}

		m_Entries = nullptr;
		m_Indices = nullptr;

		if (m_NumEntries > 0)
		{
			m_Entries = new float[m_NumEntries];
			m_Indices = new int[m_NumEntries];

			memset(m_Entries, 0, sizeof(m_Entries[0])*m_NumEntries);
			memset(m_Indices, 0, sizeof(m_Indices[0])*m_NumEntries);

			int counter = 0;
			for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
			{
				if (fabsf(A[i*n+j]) > 1e-9)
				{
					m_Entries[counter] = A[i * n + j];
					m_Indices[counter] = i * n + j;
					counter++;
				}
			}
		}
	}

	~SparseMatrix()
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

	void MulXCompressed3x3(const float *In, float *out)
	{
		for (int i = 0; i < m_Size * 3; i++)
			out[i] = 0;

		for (int i = 0; i < m_NumEntries; i++)
		{
			int ind = m_Indices[i];
			int x = ind / m_Size;
			int y = ind % m_Size;
			float entry = m_Entries[i];

			out[x * 3] += entry * In[y * 3];
			out[x * 3 + 1] += entry * In[y * 3 + 1];
			out[x * 3 + 2] += entry * In[y * 3 + 2];
		}
	}

	void MulXCompressedN(const float* In, int nDof, float* out)
	{
		for (int i = 0; i < m_Size * nDof; i++)
			out[i] = 0;

		for (int i = 0; i < m_NumEntries; i++)
		{
			int ind = m_Indices[i];
			int x = ind / m_Size;
			int y = ind % m_Size;
			float entry = m_Entries[i];

			int kx = x * nDof, ky = y * nDof;
			for (int j = 0; j < nDof; ++j)
			{
				out[kx + j] += entry * In[ky + j];
			}
		}
	}

private:
	float *m_Entries;
	int *m_Indices;
	int m_NumEntries;
	int m_Size;
};


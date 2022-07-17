#pragma once

#include <vector>
#include "../Maths/Maths.h"
#include "../LinearSystem/SparseMatrix.h"

class SparseConjugateGradientSolver
{
public:
	SparseConjugateGradientSolver()
	{
		m_SparseA = nullptr;
		r = nullptr;
		d = nullptr;
		q = nullptr;
		temp = nullptr;
		m_MaxIterations = 100;
		m_Epsilon = 0.0001f;
	}

	~SparseConjugateGradientSolver()
	{
		if (m_SparseA) delete m_SparseA;
		if (r) delete[]r;
		if (d) delete[]d;
		if (q) delete[]q;
		if (temp) delete[]temp;
	}

	void InitSparseSolverCompressed3x3(const float* A, int n)
	{
		m_Size = n * 3;
		m_SparseA = new SparseMatrix(A, n, n);

		r = new float[m_Size];
		d = new float[m_Size];
		q = new float[m_Size];
		temp = new float[m_Size];
	}

	void SolveSparseCompressed3x3(float* x, const float* b)
	{
		for (int i = 0; i < m_Size; ++i)
		{
			r[i] = 0;
			d[i] = 0;
			q[i] = 0;
		}

		m_SparseA->MulXCompressed3x3(x, temp);

		for (int i = 0; i < m_Size; ++i)
		{
			d[i] = r[i] = b[i] - temp[i];
		}

		int it = 0;
		float deltaNew = Dot(r, r, m_Size);
		float delta0 = deltaNew;
		float alpha = 0, beta = 0;

		while (it < m_MaxIterations && deltaNew > m_Epsilon * m_Epsilon * delta0)
		{
			it++;

			m_SparseA->MulXCompressed3x3(d, q);

			alpha = deltaNew / Dot(d, q, m_Size);

			for (int i = 0; i < m_Size; ++i)
			{
				x[i] = x[i] + alpha * d[i];
			}

			if (it % 50 == 0)		// floating point errors ???
			{
				m_SparseA->MulXCompressed3x3(x, temp);
				for (int i = 0; i < m_Size; ++i)
					r[i] = b[i] - temp[i];
			}
			else
			{
				for (int i = 0; i < m_Size; ++i)
					r[i] = r[i] - alpha * q[i];
			}

			float deltaOld = deltaNew;
			deltaNew = Dot(r, r, m_Size);
			beta = deltaNew / deltaOld;

			for (int i = 0; i < m_Size; ++i)
				d[i] = r[i] + beta * d[i];
		}
	}

	static float Dot(const float *a, const float *b, int n)
	{
		float dp = 0;
		for(int i = 0; i < n; ++i)
			dp += a[i] * b[i];
		return dp;
	}

private:
	float *r, *x, *d, *q, *temp;
	int m_Size;
	int m_MaxIterations;
	float m_Epsilon;
	SparseMatrix* m_SparseA;
};


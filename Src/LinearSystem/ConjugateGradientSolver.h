#pragma once

#include <string.h>

// solve ax = b, given a, b
// https://en.wikipedia.org/wiki/Conjugate_gradient_method

template<typename T>
class ConjugateGradientSolver
{
public:
	static bool Solve(const T* A, const T *B, int n, T* X, const int maxIterations = 100, const T Eps = (T)0.0001)
	{
		T stack_mem[1024 * 3 * 3];
		T *heap_mem = nullptr;
		if (3 * n > sizeof(stack_mem) / sizeof(stack_mem[0]))
		{
			heap_mem = new T[3 * n];
		}
		T* r = heap_mem ? heap_mem : stack_mem;
		T* d = r + n;
		T* q = r + 2 * n;

		memset(r, 0, sizeof(T) * n);
		memset(d, 0, sizeof(T) * n);
		memset(q, 0, sizeof(T) * n);

		for (int i = 0; i < n; ++i)
		{
			d[i] = r[i] = B[i] - Dot(A + i * n, X, n);
		}

		int it = 0;
		T deltaNew = Dot(r, r, n);
		T delta0 = deltaNew;
		T alpha = 0, beta = 0;

		while (it < maxIterations)
		{
			it++;
			for (int i = 0; i < n; i++)
				q[i] = Dot(A + i * n, d, n);

			alpha = deltaNew / Dot(d, q, n);

			for (int i = 0; i < n; i++)
				X[i] = X[i] + alpha * d[i];

			if (it % 50 == 0)		// floating point errors ???
			{
				for (int i = 0; i < n; ++i)
					r[i] = B[i] - Dot(A + i * n, X, n);
			}
			else
			{
				for (int i = 0; i < n; ++i)
					r[i] = r[i] - alpha * q[i];
			}

			T deltaOld = deltaNew;
			deltaNew = Dot(r, r, n);
			beta = deltaNew / deltaOld;

			for (int i = 0; i < n; ++i)
				d[i] = r[i] + beta * d[i];

			if (deltaNew < Eps * Eps * delta0)
			{
				break;
			}
		}

		if (heap_mem) delete[]heap_mem;

		return true;
	}

	static T Dot(const T* a, const T* b, int n)
	{
		T dp = 0;
		for (int i = 0; i < n; ++i)
			dp += a[i] * b[i];
		return dp;
	}
};

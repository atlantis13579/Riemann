#pragma once

#include <string.h>

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Jacobi_method
template<typename T>
class JacobiIteration_CPU
{
public:
	bool Solve(const T* A, const T* B, int N, T* X, const int MaxIteration, const T kEps = (T)0.00001)
	{
		T stack_mem[1024];
		T* heap_mem = nullptr;
		if (N > sizeof(stack_mem) / sizeof(stack_mem[0]))
		{
			heap_mem = new T[N];
		}
		T* X0 = heap_mem ? heap_mem : stack_mem;

		bool succ = false;
		int Iter = 0;
		while (Iter++ < MaxIteration)
		{
			for (int i = 0; i < N; ++i)
			{
				T beta = (T)0;
				for (int j = 0; j < i; ++j)
				{
					beta += A[i * N + j] * X[j];
				}
				for (int j = i + 1; j < N; ++j)
				{
					beta += A[i * N + j] * X[j];
				}
				T denom = A[i * N + i];
				if (-kEps < denom && denom < kEps)
					denom = kEps;
				X0[i] = (B[i] - beta) / denom;
			}

			T Norm = (T)0;
			for (int i = 0; i < N; i++)
			{
				Norm += (X[i] - X0[i]) * (X[i] - X0[i]);
			}

			memcpy(X, &X0[0], sizeof(X[0]) * N);

			if (Norm < kEps)
			{
				succ = true;
				break;
			}
		}

		if (heap_mem) delete[]heap_mem;
		return succ;
	}
};

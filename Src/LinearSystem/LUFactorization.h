#pragma once

// https://en.wikipedia.org/wiki/LU_decomposition

#include <string.h>

namespace Maths
{
namespace LinearAlgebra
{

template<typename T>
class LUFactorization
{
public:
	bool operator()(const T* A, int N, T* L, T* U) const
	{
		memset(U, 0, sizeof(U[0]) * N * N);
		memset(L, 0, sizeof(L[0]) * N * N);

		for (int i = 0; i < N; ++i)
		{
			L[i*N + i] = (T)1;
		}

		for (int i = 0; i < N; ++i)
		{
			for (int j = i; j < N; ++j)
			{
				T s = (T)0;
				for (int k = 0; k < i; ++k)
				{
					s += L[i * N + k] * U[k * N + i];
				}
				for (int k = i + 1; k < N; ++k)
				{
					s += L[i * N + k] * U[k * N + i];
				}

				U[i * N+j] = A[i * N + j] - s;
			}
			
			if (fabs(U[i * N + i]) < (T)1e-9)
			{
				return false;
			}

			if (i + 1 < N)
			{
				for (int j = i + 1; j < N; ++j)
				{
					T s = (T)0;
					for (int k = 0; k < i; ++k)
					{
						s += L[i * N + k] * U[k * N + i];
					}
					for (int k = i+1; k < N; ++k)
					{
						s += L[i * N + k] * U[k * N + i];
					}

					L[j * N + i] = (A[j * N + i] - s) / U[i * N + i];
				}

			}
		}
		
		return true;
	}
};

}	// namespace LinearAlgebra
}	// namespace Maths
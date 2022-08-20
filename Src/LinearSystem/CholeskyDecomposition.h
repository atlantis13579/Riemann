#pragma once

// The Cholesky decomposition is roughly twice
// as efficient as the LU decomposition for solving
// systems of linear equations.

// A = L * L^T where L is a lower triangular matrix

#include <math.h>

template<typename T>
class CholeskyDecomposition
{
public:
	bool operator()(const T* A, int n, T* L) const
	{
		for (int i = 0; i < n; ++i)
		for (int j = 0; j <= i; ++j)
		{
			if (j == i)
			{
				T sum = (T)0;
				for (int k = 0; k < j; ++k)
				{
					sum += L[j*n+k] * L[j*n+k];
				}
				L[j*n+j] = A[j*n+j] - sum;
				
				if (L[j*n+j] < (T)1e-9)
				{
					return false;
				}
				
				L[j*n+j] = sqrt(L[j*n+j]);
			}
			else
			{
				T sum = (T)0;
				for (int k = 0; k < j; ++k)
				{
					sum += L[i*n+k] * L[j*n+k];
				}
				L[i*n+j] = (A[i*n+j] - sum) / L[j*n+j];
			}
		}
		return true;
	}
};

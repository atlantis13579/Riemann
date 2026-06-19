#pragma once

// The Cholesky decomposition is roughly twice
// as efficient as the LU decomposition for solving
// systems of linear equations.

// A = L * L^T where L is a lower triangular matrix

#include <math.h>

namespace Maths
{
namespace LinearAlgebra
{

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

// Solve LX = b where L is a lower triangular matrix
template<typename T>
class LowerTriangularSolver
{
public:
	bool operator()(const T* L, int n, const T* B, T *X) const
	{
		for (int i = 0; i < n; ++i)
		{
			T sum = (T)0;
			for (int j = 0; j < i; ++j)
			{
				sum += L[i*n+j] * X[j];
			}
			
			if (fabs(L[i*n+i]) < (T)1e-9)
			{
				return false;
			}
			
			X[i] = (B[i] - sum) / L[i*n+i];
		}
		
		return true;
	}
};

// Solve UX = b where U is a upper triangular matrix
template<typename T>
class UpperTriangularSolver
{
public:
	bool operator()(const T* U, int n, const T* B, T *X) const
	{
		for (int i = n - 1; i >= 0; --i)
		{
			T sum = (T)0;
			for (int j = i + 1; j < n; ++j)
			{
				sum += U[i*n+j] * X[j];
			}
			
			if (fabs(U[i*n+i]) < (T)1e-9)
			{
				return false;
			}
			
			X[i] = (B[i] - sum) / U[i*n+i];
		}
		
		return true;
	}
};

}	// namespace LinearAlgebra
}	// namespace Maths
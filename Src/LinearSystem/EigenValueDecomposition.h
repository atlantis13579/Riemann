#pragma once

// https://en.wikipedia.org/wiki/Eigenvalue_algorithm

#include <algorithm>
#include <math.h>
#include <string.h>
#include "gemm.inl"

// Solve Eigen with QR Decompose (using householder method)
// 
// X <- A
// for it = 1 to 30
//   Q , R <- qr_decompose(X);
//   X <- R * Q;
//   EigenVectors <- EigenVectors * Q;
// EigenValues = diag(X)

namespace Maths
{
namespace LinearAlgebra
{

template<typename T>
class SymmetricEigenSolver
{
public:
	void operator()(const T* A, int n, T* EigenValues, T* EigenVectors) const
	{
		solve_eigen_qr_iterations(A, n, EigenValues, EigenVectors);
	}

private:
	void solve_eigen_qr_iterations(const T* A, int n, T* EigenValues, T* EigenVectors) const
	{
		memset(EigenVectors, 0, sizeof(T) * n * n);
		for (int i = 0; i < n; ++i)
		{
			EigenVectors[i*n + i] = (T)1;
		}

		T stack_mem[32 * 32];
		T* heap_mem = nullptr;
		if (n * n > sizeof(stack_mem) / sizeof(stack_mem[0]))
		{
			heap_mem = new T[n * n];
		}

		T* x = heap_mem ? heap_mem : stack_mem;
		memcpy(x, A, sizeof(T) * n * n);

		const int maxIterations = std::max(32, n * n * 32);
		const T convergeEps = (T)(1e-6);

		for (int it = 0; it < maxIterations; ++it)
		{
			int p = 0;
			int q = 1;
			T maxOffDiag = (T)0;
			for (int i = 0; i < n; ++i)
			for (int j = i + 1; j < n; ++j)
			{
				T v = (T)fabs(x[i * n + j]);
				if (v > maxOffDiag)
				{
					maxOffDiag = v;
					p = i;
					q = j;
				}
			}

			if (maxOffDiag < convergeEps)
			{
				break;
			}

			T app = x[p * n + p];
			T aqq = x[q * n + q];
			T apq = x[p * n + q];
			T theta = (T)0.5 * (T)atan2((T)2 * apq, aqq - app);
			T c = (T)cos(theta);
			T s = (T)sin(theta);

			for (int k = 0; k < n; ++k)
			{
				if (k == p || k == q)
				{
					continue;
				}
				T akp = x[k * n + p];
				T akq = x[k * n + q];
				T new_kp = c * akp - s * akq;
				T new_kq = s * akp + c * akq;
				x[k * n + p] = new_kp;
				x[p * n + k] = new_kp;
				x[k * n + q] = new_kq;
				x[q * n + k] = new_kq;
			}

			x[p * n + p] = c * c * app - (T)2 * s * c * apq + s * s * aqq;
			x[q * n + q] = s * s * app + (T)2 * s * c * apq + c * c * aqq;
			x[p * n + q] = (T)0;
			x[q * n + p] = (T)0;

			for (int k = 0; k < n; ++k)
			{
				T vkp = EigenVectors[k * n + p];
				T vkq = EigenVectors[k * n + q];
				EigenVectors[k * n + p] = c * vkp - s * vkq;
				EigenVectors[k * n + q] = s * vkp + c * vkq;
			}
		}

		for (int i = 0; i < n; ++i)
		{
			EigenValues[i] = x[i * n + i];
		}

		for (int i = 0; i < n - 1; ++i)
		for (int j = i + 1; j < n; ++j)
		{
			if (EigenValues[j] > EigenValues[i])
			{
				std::swap(EigenValues[i], EigenValues[j]);
				for (int k = 0; k < n; ++k)
				{
					std::swap(EigenVectors[k * n + i], EigenVectors[k * n + j]);
				}
			}
		}

		if (heap_mem) delete[]heap_mem;
	}

	static T compute_norm(T* v, int k)
	{
		T sum = (T)0;
		for (int i = 0; i < k; i++) {
			sum += v[i] * v[i];
		}
		return sqrt(sum);
	}
};

}	// namespace LinearAlgebra
}	// namespace Maths

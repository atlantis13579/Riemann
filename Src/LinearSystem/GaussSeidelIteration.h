#pragma once

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method

template<typename T>
class GaussSeidelIteration_CPU
{
public:
	// Relaxation:
	// In numerical linear algebra, the method of successive over - relaxation(SOR)
	// is a variant of the Gauss¨CSeidel method for solving a linear system of equations,
	// resulting in faster convergence.
	// http://www.gamedev.ru/community/gd_physcomm/articles/?id=709
	static bool Solve(const T* A, const T* B, int N, T* X, const int MaxIteration, const T kEps = (T)0.00001, const float Relaxation = (T)1.0)
	{
		int Iter = 0;
		while (Iter++ < MaxIteration)
		{
			T Norm = (T)0;
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
				T delta = (B[i] - beta) / denom;
				Norm += (X[i] - delta) * (X[i] - delta);
				X[i] += Relaxation * (delta - X[i]);
			}

			if (Norm < kEps)
			{
				return true;
			}
		}
		return false;
	}
};


#pragma once

// Projected Successive Over Relaxation (PSOR)
// Projected Gauss Seidel (PGS)
template<typename T>
class ProjectedGaussSeidel_CPU
{
public:
	// Solve X, A * X + B >= 0, X >= 0 and dot(A * X + B, X) = 0,
	// A is square Matrix of size N, B is Vector of size N
	static bool Solve(const T* A, const T* B, int N, T* X, const int MaxIteration, const T kEps = (T)0.00001, const T Relaxation = (T)1)
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
				T X0 = (-B[i] - beta) / A[i * N + i];
				X0 = X[i] + Relaxation * (X0 - X[i]);
				X0 = std::max(T(0), X0);
				Norm += (X[i] - X0) * (X[i] - X0);
				X[i] = X0;
			}

			if (Norm < kEps)
			{
				return true;
			}
		}
		return false;
	}
	
	// Solve X, Y, X2 >= X >= X1, Y == A * X + B and (X_i == X1_i and Y_i > 0) or (X_i == X2_i and Y_i < 0) or (X1_i < X_i < X2_i and Y_i == 0)
	// A is square Matrix of size N, B is Vector of size N
	static bool Solve(const T* A, const T* B, int N, T* X, const T* X1, const T* X2, const int MaxIteration, const T kEps = (T)0.00001, const T Relaxation = (T)1)
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
				T X0 = (-B[i] - beta) / A[i * N + i];
				X0 = X[i] + Relaxation * (X0 - X[i]);
				X0 = std::min(std::max(X1[i], X0), X2[i]);
				Norm += (X[i] - X0) * (X[i] - X0);
				X[i] = X0;
			}

			if (Norm < kEps)
			{
				return true;
			}
		}
		return false;
	}
};

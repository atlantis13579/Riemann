
#pragma once

#include <vector>

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Jacobi_method
template<typename T>
class JacobiIteration_CPU
{
public:
	JacobiIteration_CPU() {}

	void Solve(const T* A, const T* B, int N, int MaxIteration, T* X)
	{
		if (m_buf.size() < N)
		{
			m_buf.resize(N);
		}
		T* DX = &m_buf[0];

		int Iter = 0;
		while (Iter++ < MaxIteration)
		{
			for (int i = 0; i < N; ++i)
			{
				T s = (T)0;
				for (int j = 0; j < i; ++j)
					s += A[i * N + j] * X[j];

				for (int j = i + 1; j < N; ++j)
					s += A[i * N + j] * X[j];

				DX[i] = (B[i] - s) / A[i * N + i];
			}

			memcpy(X, DX, sizeof(X[0]) * N);

			T Norm = (T)0;
			for (int i = 0; i < N; i++)
				Norm += X[i] * X[i];

			bool converge = Norm < (T)0.00001;
			if (converge)
			{
				break;
			}
		}
	}

private:
	std::vector<T> m_buf;
};
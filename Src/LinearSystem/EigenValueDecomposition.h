#pragma once

// https://en.wikipedia.org/wiki/Eigenvalue_algorithm

#include <string.h>
#include "DenseMatrix.h"

template<typename T>
class SolveEigenSymmetric
{
public:
	bool operator()(const T* A, int n, T* EignValues, T *EigenVectors) const
	{
		memset(EigenVectors, 0, sizeof(T) * n * n);
		for (int i = 0; i < n; ++i)
		{
			EigenVectors[i*n + i] = (T)1;
		}

		T* buf = new T[5 * n * n + n];

		T* X = buf;
		T* Q = buf + n * n;
		T* R = buf + 2 * n * n;
		T* H = buf + 3 * n * n;
		T* buffer = buf + 4 * n * n;
		T* v = buf + 5 * n * n;

		memcpy(X, A, sizeof(T) * n * n);

		const int maxIterations = 30;
		int it = 0;
		while (it++ < maxIterations)
		{
			// Q , R = QRDecompose(X);
			// X = R * Q;
			// EigenVectors = EigenVectors * Q;

			memset(Q, 0, sizeof(T) * n * n);
			for (int i = 0; i < n; ++i)
			{
				Q[i * n + i] = (T)1;
			}

			memcpy(R, X, sizeof(T) * n * n);

			for (int i = 0; i < n; ++i)
			{
				for (int j = 0; j < n - i; ++j)
				{
					v[j] = R[(j + i) * n + i];
				}
				T L = compute_norm(v, n - i);

				v[0] += v[0] > 0 ? L : -L;
				L = compute_norm(v, n - i);

				for (int p = 0; p < n - i; p++)
				{
					v[p] /= L;
				}

				memset(H, 0, sizeof(T) * n * n);
				for (int r = i; r < n; ++r)
				for (int c = i; c < n; ++c)
				{
					H[r * n + c] = -2 * v[r - i] * v[c - i];
				}

				// R += H * R;
				gemm_block<T>(H, R, n, n, n, i, n - 1, 0, n - 1, 0, n - 1, buffer);
				gema_block<T>(R, buffer, n, n, i, n - 1, 0, n - 1, R);

				// Q += Q * H;
				gemm_block<T>(Q, H, n, n, n, 0, n - 1, i, n - 1, i, n - 1, buffer);
				gema_block<T>(Q, buffer, n, n, 0, n - 1, i, n - 1, Q);
			}

			for (int c = 0; c < n; ++c)
			{
				if (R[c * n + c] < 0)
				{
					for (int r = 0; r < n; ++r)
					{
						R[c * n + r] = -R[c * n + r];
						Q[r * n + c] = -Q[r * n + c];
					}
					for (int r = n; r < n; ++r)
					{
						Q[r * n + c] = -Q[r * n + c];
					}
				}
			}

			gemm<T>(EigenVectors, Q, n, n, n, X);
			memcpy(EigenVectors, X, sizeof(T) * n * n);
			gemm<T>(R, Q, n, n, n, X);
		}

		for (int i = 0; i < n; ++i)
		{
			EignValues[i] = X[i * n + i];
		}

		delete[]buf;

		return true;
	}

private:
	static T compute_norm(T* v, int k)
	{
		T sum = (T)0;
		for (int i = 0; i < k; i++) {
			sum += v[i] * v[i];
		}
		return sqrt(sum);
	}
};

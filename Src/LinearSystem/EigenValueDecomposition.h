#pragma once

// https://en.wikipedia.org/wiki/Eigenvalue_algorithm

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

		T* buf = new T[4 * n * n + n];

		T* x = buf;
		T* q = buf + n * n;
		T* r = buf + 2 * n * n;
		T* h = buf + 3 * n * n;
		T* v = buf + 4 * n * n;

		memcpy(x, A, sizeof(T) * n * n);

		T prev_norm = (T)0;

		const int maxIterations = 32;
		const T convergeEps = (T)(1e-5);

		int it = 0;
		while (it++ < maxIterations)
		{
			// Q, R <- QRDecompose(X) , householder method

			memset(q, 0, sizeof(T) * n * n);
			for (int i = 0; i < n; ++i)
			{
				q[i * n + i] = (T)1;
			}

			memcpy(r, x, sizeof(T) * n * n);

			for (int i = 0; i < n; ++i)
			{
				for (int j = 0; j < n - i; ++j)
				{
					v[j] = r[(j + i) * n + i];
				}
				T norn = compute_norm(v, n - i);

				v[0] += v[0] > 0 ? norn : -norn;
				norn = compute_norm(v, n - i);

				for (int j = 0; j < n - i; ++j)
				{
					v[j] /= norn;
				}

				memset(h, 0, sizeof(T) * n * n);
				for (int j = i; j < n; ++j)
				for (int k = i; k < n; ++k)
				{
					h[j * n + k] = -2 * v[j - i] * v[k - i];
				}

				// R += H * R;
				gemm_block(h, r, n, n, n, i, n - 1, 0, n - 1, 0, n - 1, x);
				gema_block(r, x, n, n, i, n - 1, 0, n - 1, r);

				// Q += Q * H;
				gemm_block(q, h, n, n, n, 0, n - 1, i, n - 1, i, n - 1, x);
				gema_block(q, x, n, n, 0, n - 1, i, n - 1, q);
			}

			for (int i = 0; i < n; ++i)
			{
				if (r[i * n + i] >= 0)
					continue;

				for (int j = 0; j < n; ++j)
				{
					r[i * n + j] = -r[i * n + j];
					q[j * n + i] = -q[j * n + i];
				}
			}

			// EigenVectors = EigenVectors * Q;
			gemm(EigenVectors, q, n, n, n, x);
			memcpy(EigenVectors, x, sizeof(T) * n * n);

			// X = R * Q;
			gemm(r, q, n, n, n, x);

			// check converge
			T norm = (T)0;
			for (int i = 0; i < n; ++i)
			{
				norm += x[i * n + i] * x[i * n + i];
			}
			norm /= n;

			if (fabs(norm - prev_norm) < convergeEps)
			{
				break;
			}
			prev_norm = norm;
		}

		for (int i = 0; i < n; ++i)
		{
			EigenValues[i] = x[i * n + i];
		}

		delete[]buf;
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

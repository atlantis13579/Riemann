#pragma once

// A = QR where Q is an orthogonal matrix and R is an upper triangular matrix

// https://en.wikipedia.org/wiki/QR_decomposition
// https://en.wikipedia.org/wiki/Householder_transformation

#include <math.h>
#include <string.h>
#include "DenseMatrix.h"

template <typename T>
class QRDecomposition
{
public:
	void operator()(const T* A, int nRows, int nCols, T* Q, T* R) const
	{
        compute_householder(A, Q, R, nRows, nCols);
		return;
	}

private:
	static void compute_householder(const T* A, T* Q, T *R, int m, int n)
	{
		int max_dim = m > n ? m : n;

		T* buf = new T[m * m + m + max_dim * max_dim];

		T* H = buf;
		T* v = H + m * m;
		T* buffer = H + m * m + m;

		memset(Q, 0, sizeof(T) * m * m);
		for (int i = 0; i < m; ++i)
		{
			Q[i * m + i] = (T)1;
		}

		memcpy(R, A, sizeof(T) * m * n);

		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < m - i; ++j)
			{
				v[j] = R[(j + i) * n + i];
			}
			T L = compute_norm(v, m - i);

			v[0] += v[0] > 0 ? L : -L;
			L = compute_norm(v, m - i);

			for (int p = 0; p < m-i; p++)
			{
				v[p] /= L;
			}

			memset(H, 0, sizeof(T) * m * m);
			for (int r = i; r < m; ++r)
			for (int c = i; c < m; ++c)
			{
				H[r*m+c] = - 2 * v[r - i] * v[c - i];
			}

			// R += H * R;
			gemm_block<T>(H, R, m, m, n, i, m - 1, 0, m - 1, 0, n - 1, buffer);
			gema_block<T>(R, buffer, m, n, i, m - 1, 0, n - 1, R);

			// Q += Q * H;
			gemm_block<T>(Q, H, m, m, m, 0, m - 1, i, m - 1, i, m - 1, buffer);
			gema_block<T>(Q, buffer, m, n, 0, m - 1, i, m - 1, Q);
		}

		for (int c = 0; c < n; ++c)
		{
			if (R[c*n+c] < 0)
			{
				for (int r = 0; r < n; ++r)
				{
					R[c*n+r] = -R[c*n+r];
					Q[r*m+c] = -Q[r*m+c];
				}
				for (int r = n; r < m; ++r)
				{
					Q[r*m+c] = -Q[r*m+c];
				}
			}
		}

		delete []buf;

		return;
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
#pragma once

// A = QR where Q is an orthogonal matrix and R is an upper triangular matrix

// https://en.wikipedia.org/wiki/QR_decomposition

#include <string.h>
#include "gemm.inl"

namespace Maths
{
namespace LinearAlgebra
{

template <typename T>
class QRDecomposition
{
public:
	bool operator()(const T* A, int nRows, int nCols, T* Q, T* R) const
	{
		if (nCols < nRows)
			return false;

		qr_householder_reflections(A, Q, R, nRows, nCols);
		return true;
	}

private:
	// https://en.wikipedia.org/wiki/Householder_transformation
	static void qr_householder_reflections(const T* a, T* q, T *r, int m, int n)
	{
		int max_dim = m > n ? m : n;

		T* buf = new T[m * m + m + max_dim * max_dim];

		T* h = buf;
		T* v = h + m * m;
		T* buffer = h + m * m + m;

		memset(q, 0, sizeof(T) * m * m);
		for (int i = 0; i < m; ++i)
		{
			q[i * m + i] = (T)1;
		}

		memcpy(r, a, sizeof(T) * m * n);

		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < m - i; ++j)
			{
				v[j] = r[(j + i) * n + i];
			}
			T norm = compute_norm(v, m - i);

			v[0] += v[0] > 0 ? norm : -norm;
			norm = compute_norm(v, m - i);

			for (int j = 0; j < m - i; ++j)
			{
				v[j] /= norm;
			}

			memset(h, 0, sizeof(T) * m * m);
			for (int j = i; j < m; ++j)
			for (int k = i; k < m; ++k)
			{
				h[j*m+k] = - 2 * v[j - i] * v[k - i];
			}

			// R += H * R;
			gemm_block(h, r, m, m, n, i, m - 1, 0, m - 1, 0, n - 1, buffer);
			gema_block(r, buffer, m, n, i, m - 1, 0, n - 1, r);

			// Q += Q * H;
			gemm_block(q, h, m, m, m, 0, m - 1, i, m - 1, i, m - 1, buffer);
			gema_block(q, buffer, m, n, 0, m - 1, i, m - 1, q);
		}

		for (int i = 0; i < n; ++i)
		{
			if (r[i*n+i] >= 0)
				continue;

			for (int j = 0; j < n; ++j)
			{
				r[i*n+j] = -r[i*n+j];
				q[j*m+i] = -q[j*m+i];
			}
			for (int r = n; r < m; ++r)
			{
				q[r*m+i] = -q[r*m+i];
			}
		}

		delete []buf;

		return;
	}

	static void qr_gram_schmidt(const T* a, T* q, T* r, int m, int n)
	{
	}

	static void qr_givens_rotations(const T* a, T* q, T* r, int m, int n)
	{
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
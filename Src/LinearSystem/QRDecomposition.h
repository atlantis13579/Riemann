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
	static void compute_householder(const T* pA, T* pQ, T *pR, int m, int n)
	{
		TDenseMatrix<T> Q(m, m), R(m, n), H(m, m);
		TDenseVector<T> V(m);
		Q.LoadIdentity();
		memcpy(R.GetData(), pA, sizeof(T) * m * n);

		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < m - i; ++j)
			{
				V[j] = R[j + i][i];
			}
			T L = Norm(V.GetData(), m - i);

			V[0] += V[0] > 0 ? L : -L;
			L = Norm(V.GetData(), m - i);

			for (int p = 0; p < m-i; p++) {
				V[p] /= L;
			}

			for (int r = 0; r < m; ++r)
			for (int c = 0; c < m; ++c)
			{
				if (r >= i && c >= i)
				{
					H[r][c] = KroneckerDelta(r, c) - 2 * V[r - i] * V[c - i];
				}
				else
				{
					H[r][c] = KroneckerDelta(r, c);
				}
			}

			R = H * R;
			Q = Q * H;
		}

		for (int c = 0; c < n; ++c)
		{
			if (R[c][c] < 0)
			{
				for (int r = 0; r < n; ++r)
				{
					R[c][r] = -R[c][r];
					Q[r][c] = -Q[r][c];
				}
				for (int r = n; r < m; ++r)
				{
					Q[r][c] = -Q[r][c];
				}
			}
		}

		TDenseMatrix<T>(pQ, m, m).Assign(Q);
		TDenseMatrix<T>(pR, m, n).Assign(R);
		return;
	}

	static inline T KroneckerDelta(int a, int b)
	{
		return a == b ? (T)1 : (T)0;
	}

	static T Norm(T* v, int k)
	{
		T sum = (T)0;
		for (int i = 0; i < k; i++) {
			sum += v[i] * v[i];
		}
		return sqrt(sum);
	}
};
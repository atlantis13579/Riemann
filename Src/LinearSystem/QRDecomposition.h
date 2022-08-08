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

			H.LoadZero();
			for (int r = i; r < m; ++r)
			for (int c = i; c < m; ++c)
			{
				H[r][c] = - 2 * V[r - i] * V[c - i];
			}

			// R += H * R;
			// Q += Q * H;
			R.SubAdd(SubMultiply(H, R, i, m - 1, 0, m - 1, 0, n - 1), i, m - 1, 0, n - 1);
			Q.SubAdd(SubMultiply(Q, H, 0, m - 1, i, m - 1, i, m - 1), 0, m - 1, i, m - 1);
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

	static T Norm(T* v, int k)
	{
		T sum = (T)0;
		for (int i = 0; i < k; i++) {
			sum += v[i] * v[i];
		}
		return sqrt(sum);
	}
};
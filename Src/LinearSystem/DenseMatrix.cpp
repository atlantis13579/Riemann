#include "DenseMatrix.h"
#include "CholeskyDecomposition.h"
#include "GaussianElimination.h"
#include "LUFactorization.h"
#include "MoorePenrosePseudoInverse.h"
#include "PolarDecomposition.h"
#include "QRDecomposition.h"
#include "SingularValueDecomposition.h"
#include "EigenValueDecomposition.h"

// #define GEMM_USE_SSE
// #define GEMM_USE_AVX

/////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
void gemm_block_slow(const T* mat1, const T* mat2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int k = k0; k <= k1; ++k)
	{
		T dp = (T)0;
		const T* p = mat1 + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * mat2[j * c2 + k];
		m[i * c2 + k] = dp;
	}
}

template<typename T>
void gemv_block_slow(const T* mat, const T* vec1, int r1, int c1, int i0, int i1, int j0, int j1, T* v)
{
	for (int i = i0; i <= i1; ++i)
	{
		T dp = (T)0;
		const T* p = mat + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * vec1[j];
		v[i] = dp;
	}
}

template<typename T>
void gema_block_slow(const T* mat1, const T* mat2, int r, int c, int i0, int i1, int j0, int j1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int j = j0; j <= j1; ++j)
	{
		m[i * c + j] = mat1[i * c + j] + mat2[i * c + j];
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////

template<>
void gemm_block(const float* m1, const float* m2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, float* m)
{
	gemm_block_slow<float>(m1, m2, r1, c1, c2, i0, i1, j0, j1, k0, k1, m);
}

template<>
void gemv_block(const float* m1, const float* v1, int r1, int c1, int i0, int i1, int k0, int k1, float* v)
{
	gemv_block_slow<float>(m1, v1, r1, c1, i0, i1, k0, k1, v);
}

template<>
void gema_block(const float* mat1, const float* mat2, int r, int c, int i0, int i1, int j0, int j1, float* m)
{
	gema_block_slow<float>(mat1, mat2, r, c, i0, i1, j0, j1, m);
}

template<>
void gemm_block(const double* m1, const double* m2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, double* m)
{
	gemm_block_slow(m1, m2, r1, c1, c2, i0, i1, j0, j1, k0, k1, m);
}

template<>
void gemv_block(const double* m1, const double* v1, int r1, int c1, int i0, int i1, int k0, int k1, double* v)
{
	gemv_block_slow(m1, v1, r1, c1, i0, i1, k0, k1, v);
}

template<>
void gema_block(const double* mat1, const double* mat2, int r, int c, int i0, int i1, int j0, int j1, double* m)
{
	gema_block_slow<double>(mat1, mat2, r, c, i0, i1, j0, j1, m);
}

template<>
bool TDenseMatrix<float>::GetInverse(TDenseMatrix<float>& InvM) const
{
	if (!IsSquare())
		return false;
	InvM.SetSize(mRows, mRows);
	return GaussianElimination<float>()(pData, mRows, InvM.pData, nullptr);
}

template<>
float TDenseMatrix<float>::Determinant() const
{
	float Det;
	if (GaussianElimination<float>()(pData, mRows, nullptr, &Det))
	{
		return Det;
	}
	return 0.0f;
}

template<>
bool TDenseMatrix<float>::GetPseudoInverse(TDenseMatrix<float>& pinv) const
{
	if (!IsSquare())
		return false;
	pinv.SetSize(mRows, mRows);
	return MoorePenrosePseudoInverse<float>()(pData, mRows, pinv.pData);
}

template<>
bool TDenseMatrix<float>::SingularValueDecompose(TDenseMatrix<float> &U, TDenseVector<float> &S, TDenseMatrix<float> &V) const
{
	S.SetSize(std::min(mRows, mCols));
	U.SetSize(mRows, mRows);
	V.SetSize(mCols, mCols);
	if (!::SingularValueDecomposition<float>()(pData, mRows, mCols, U.pData, S.GetData(), V.pData))
	{
		return false;
	}
	return true;
}

template<>
bool TDenseMatrix<float>::PolarDecompose(TDenseMatrix<float> &U, TDenseMatrix<float> &P) const
{
	if (!IsSquare())
		return false;
	U.SetSize(mRows, mRows);
	P.SetSize(mRows, mRows);
	return ::PolarDecomposition<float>()(pData, mRows, U.pData, P.pData);
}

template<>
void TDenseMatrix<float>::QRDecompose(TDenseMatrix<float>& Q, TDenseMatrix<float>& R) const
{
	Q.SetSize(mRows, mRows);
	R.SetSize(mRows, mCols);
	::QRDecomposition<float>()(pData, mRows, mCols, Q.pData, R.pData);
}

template<>
bool TDenseMatrix<float>::EigenDecompose(TDenseVector<float>& EigenValues, TDenseMatrix<float>& EigenVectors) const
{
	if (!IsSquare())
		return false;

	if (!IsSymmetric())
		return false;

	EigenValues.SetSize(mRows);
	EigenVectors.SetSize(mRows, mRows);
	::SolveEigenSymmetric<float>()(pData, mRows, EigenValues.GetData(), EigenVectors.GetData());
	return true;
}
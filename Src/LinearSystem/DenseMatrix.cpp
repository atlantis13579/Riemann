#include "DenseMatrix.h"
#include "CholeskyDecomposition.h"
#include "GaussianElimination.h"
#include "LUFactorization.h"
#include "MoorePenrosePseudoInverse.h"
#include "PolarDecomposition.h"
#include "QRDecomposition.h"
#include "SingularValueDecomposition.h"

// #define GEMM_USE_SSE
// #define GEMM_USE_AVX

void gemm_sub_slow(const float* mat1, const float* mat2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, float* mat)
{
	for (int i = i0; i <= i1; ++i)
	for (int k = k0; k <= k1; ++k)
	{
		float dp = 0.0f;
		const float* p = mat1 + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * mat2[j * c2 + k];
		mat[i * c2 + k] = dp;
	}
}

void gemv_sub_slow(const float* mat, const float* vec1, int r1, int c1, int i0, int i1, int j0, int j1, float* v)
{
	for (int i = i0; i <= i1; ++i)
	{
		float dp = 0.0f;
		const float* p = mat + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * vec1[j];
		v[i] = dp;
	}
}

template<>
void gemm_sub(const float* m1, const float* m2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, float* m)
{
	gemm_sub_slow(m1, m2, r1, c1, c2, i0, i1, j0, j1, k0, k1, m);
}

template<>
void gemv_sub(const float* m1, const float* v1, int r1, int c1, int i0, int i1, int k0, int k1, float* v)
{
	gemv_sub_slow(m1, v1, r1, c1, i0, i1, k0, k1, v);
}

template<>
bool	TDenseMatrix<float>::GetInverse(TDenseMatrix<float>& InvM) const
{
	if (!IsSquare())
		return false;
	InvM.SetSize(mRows, mRows);
	return GaussianElimination<float>()(GetData(), mRows, InvM.GetData(), nullptr);
}

template<>
float	TDenseMatrix<float>::Determinant() const
{
	float Det;
	if (GaussianElimination<float>()(GetData(), mRows, nullptr, &Det))
	{
		return Det;
	}
	return 0.0f;
}

template<>
bool	TDenseMatrix<float>::GetPseudoInverse(TDenseMatrix<float>& pinv) const
{
	if (!IsSquare())
		return false;
	pinv.SetSize(mRows, mRows);
	return MoorePenrosePseudoInverse<float>()(GetData(), mRows, pinv.GetData());
}

template<>
bool 	TDenseMatrix<float>::SingularValueDecompose(TDenseMatrix<float> &U, TDenseVector<float> &S, TDenseMatrix<float> &V) const
{
	S.SetSize(std::min(mRows, mCols));
	U.SetSize(mRows, mRows);
	V.SetSize(mCols, mCols);
	if (!::SingularValueDecomposition<float>()(GetData(), mRows, mCols, U.GetData(), S.GetData(), V.GetData()))
	{
		return false;
	}
	return true;
}

template<>
bool 	TDenseMatrix<float>::PolarDecompose(TDenseMatrix<float> &U, TDenseMatrix<float> &P) const
{
	assert(IsSquare());
	U.SetSize(mRows, mRows);
	P.SetSize(mRows, mRows);
	return ::PolarDecomposition<float>()(GetData(), mRows, U.GetData(), P.GetData());
}

template<>
void 	TDenseMatrix<float>::QRDecompose(TDenseMatrix<float>& Q, TDenseMatrix<float>& R) const
{
	Q.SetSize(mRows, mRows);
	R.SetSize(mRows, mCols);
	::QRDecomposition<float>()(GetData(), mRows, mCols, Q.GetData(), R.GetData());
}
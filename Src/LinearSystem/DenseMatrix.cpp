#include "DenseMatrix.h"
#include "GaussianElimination.h"
#include "MoorePenrosePseudoInverse.h"
#include "SingularValueDecomposition.h"
#include "PolarDecomposition.h"

void gemm_slow(const float* m1, const float* m2, int r1, int c1, int c2, float* m)
{
	for (int i = 0; i < r1; ++i)
	for (int j = 0; j < c2; ++j)
	{
		float dp = 0.0f;
		const float* p = m1 + i * c1;
		for (int k = 0; k < c1; ++k)
			dp += p[k] * m2[k * c2 + j];
		m[i * c2 + j] = dp;
	}
}

void gemv_slow(const float* m1, const float* v1, int r1, int c1, float* v)
{
	for (int i = 0; i < r1; ++i)
	{
		float dp = 0.0f;
		const float* p = m1 + i * c1;
		for (int k = 0; k < c1; ++k)
			dp += p[k] * v1[k];
		v[i] = dp;
	}
}

template<>
TDenseMatrix<float> TDenseMatrix<float>::operator*(const TDenseMatrix<float>& v) const
{
	if (mCols != v.mRows)
	{
		return TDenseMatrix<float>();
	}

	TDenseMatrix<float> Ret(mRows, v.mCols);
	gemm_slow(GetData(), v.GetData(), mRows, mCols, v.mCols, Ret.GetData());
	return Ret;
}

template<>
TDenseVector<float> TDenseMatrix<float>::operator*(const TDenseVector<float>& v) const
{
	if (mCols != v.GetSize())
	{
		return TDenseVector<float>();
	}

	TDenseVector<float> Ret(mRows);
	gemv_slow(GetData(), v.GetData(), mRows, mCols, Ret.GetData());
	return Ret;
}

template<>
bool	TDenseMatrix<float>::GetInverse(TDenseMatrix<float>& InvM) const
{
	assert(IsSquare());
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
TDenseMatrix<float> TDenseMatrix<float>::PseudoInverse() const
{
	assert(IsSquare());
	TDenseMatrix<float> pinv;
	bool succ = MoorePenrosePseudoInverse<float>()(GetData(), mRows, pinv.GetData());
	assert(succ);
	return pinv;
}

template<>
bool	TDenseMatrix<float>::GetPseudoInverse(TDenseMatrix<float>& pinv) const
{
	assert(IsSquare());
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

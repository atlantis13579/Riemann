#include "DenseMatrix.h"
#include "CholeskyDecomposition.h"
#include "GaussianElimination.h"
#include "LUFactorization.h"
#include "MoorePenrosePseudoInverse.h"
#include "PolarDecomposition.h"
#include "QRDecomposition.h"
#include "SingularValueDecomposition.h"
#include "EigenValueDecomposition.h"

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
bool TDenseMatrix<float>::QRDecompose(TDenseMatrix<float>& Q, TDenseMatrix<float>& R) const
{
	if (mCols < mRows)
		return false;
	Q.SetSize(mRows, mRows);
	R.SetSize(mRows, mCols);
	return ::QRDecomposition<float>()(pData, mRows, mCols, Q.pData, R.pData);
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
	::SymmetricEigenSolver<float>()(pData, mRows, EigenValues.GetData(), EigenVectors.GetData());
	return true;
}
#pragma once

#include "DenseVector.h"
#include "DenseMatrix.h"
#include "SingularValueDecomposition.h"

// https://en.wikipedia.org/wiki/Polar_decomposition

namespace Maths
{
namespace LinearAlgebra
{

template<typename T>
class PolarDecomposition
{
public:
	bool operator()(const T* A, int nDim, T* U, T* P) const
	{
		TDenseVector<T> S(nDim);
		TDenseMatrix<T> sU(U, nDim, nDim);
		TDenseMatrix<T> sV(P, nDim, nDim);
		if (!SingularValueDecomposition<T>()(A, nDim, nDim, sU.GetData(), S.GetData(), sV.GetData()))
		{
			return false;
		}
		TDenseMatrix<T> mU(nDim, nDim), mP(nDim, nDim);
		mP = sV * TDenseMatrix<T>(S) * sV.Transpose();
		mU = sU * sV.Transpose();
		memcpy(U, mU.GetData(), sizeof(T)*nDim*nDim);
		memcpy(P, mP.GetData(), sizeof(T)*nDim*nDim);
		return true;
	}
};

}	// namespace LinearAlgebra
}	// namespace Maths
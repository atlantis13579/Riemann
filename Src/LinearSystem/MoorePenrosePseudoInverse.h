#pragma once

// Compute the Moore-Penrose Pseudo Inverse of a Matrix
// pinvM = (M^T * M)^(-1) * M^T
// the pinvM has the property that : M * pinvM * M == M,

// https://en.wikipedia.org/wiki/Pseudoinversa_de_Moore-Penrose
// https://www.mathworks.com/help/matlab/ref/pinv.html

#include <string.h>
#include "DenseVector.h"
#include "DenseMatrix.h"
#include "SingularValueDecomposition.h"

template<typename T>
class MoorePenrosePseudoInverse
{
public:
	bool operator()(const T* M, int nDim, T* pinvM) const
	{
		TDenseVector<T> S(nDim);
		TDenseMatrix<T> InvS(nDim, nDim), UT(nDim, nDim), V(nDim, nDim);
		if (!SingularValueDecomposition<T>()(M, nDim, nDim, UT.GetData(), S.GetData(), V.GetData()))
		{
			return false;
		}
		UT.TransposeInPlace();
		
		const T pinvTolerance = (T)1e-6;
		for (int i = 0; i < nDim; ++i)
		{
			S[i] = S[i] > pinvTolerance ? (T)1 / S[i] : (T)0;
		}
		InvS.LoadDiagonal(S);

		TDenseMatrix<T> P = V * InvS * UT;
		memcpy(pinvM, P.GetData(), sizeof(T)*nDim*nDim);
		return true;
	}
};


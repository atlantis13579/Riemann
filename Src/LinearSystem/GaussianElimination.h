#pragma once

#include <string.h>

#define matInvM(i, j)		(InvM[i*nDim + j])

template<typename T>
class GaussianElimination
{
public:
	bool operator()(const T* M, int nDim, T* InvM, T* Determinant) const
	{
		int nSize = nDim * nDim;
		T* bufferInvM = nullptr;
		bool NeedInverse = InvM != nullptr;
		if (!NeedInverse)
		{
			bufferInvM = new T[nSize];
			InvM = bufferInvM;
		}
		memcpy(InvM, M, nSize * sizeof(T));

		int *buffer = new int[3*nDim];
		int *colIndex = buffer;
		int *rowIndex = buffer + nDim;
		int *pivoted = buffer + 2 * nDim;
		memset(pivoted, 0, sizeof(pivoted[0])*nDim);

		const T zero = (T)0;
		const T one = (T)1;
		bool odd = false;
		if (Determinant)
		{
			*Determinant = one;
		}

		int i1, i2, row = 0, col = 0;
		for (int i0 = 0; i0 < nDim; ++i0)
		{
			T maxVal = zero;
			for (i1 = 0; i1 < nDim; ++i1)
			{
				if (pivoted[i1] != 0)
				{
					continue;
				}
				
				for (i2 = 0; i2 < nDim; ++i2)
				{
					if (pivoted[i2] != 0)
					{
						continue;
					}
					
					T Val = matInvM(i1, i2);
					T absVal = (Val >= zero ? Val : -Val);
					if (absVal > maxVal)
					{
						maxVal = absVal;
						row = i1;
						col = i2;
					}
				}
			}

			if (fabs(maxVal) < zero)
			{
				if (NeedInverse)
				{
					memset(InvM, 0, nSize * sizeof(T));
				}
				if (Determinant)
				{
					*Determinant = zero;
				}
				
				if (bufferInvM) delete []bufferInvM;
				if (buffer) delete []buffer;
				return false;
			}

			pivoted[col] = true;

			if (row != col)
			{
				odd = !odd;
				for (int i = 0; i < nDim; ++i)
				{
					T t = matInvM(row, i);
					matInvM(row, i) = matInvM(col, i);
					matInvM(col, i) = t;
				}
			}

			rowIndex[i0] = row;
			colIndex[i0] = col;

			T diagonal = matInvM(col, col);
			if (Determinant) *Determinant *= diagonal;
			T inv = one / diagonal;
			matInvM(col, col) = one;
			for (i2 = 0; i2 < nDim; ++i2)
			{
				matInvM(col, i2) *= inv;
			}

			for (i1 = 0; i1 < nDim; ++i1)
			{
				if (i1 == col)
				{
					continue;
				}
				
				T save = matInvM(i1, col);
				matInvM(i1, col) = zero;
				for (i2 = 0; i2 < nDim; ++i2)
				{
					matInvM(i1, i2) -= matInvM(col, i2) * save;
				}
			}
		}

		if (InvM)
		{
			for (i1 = nDim - 1; i1 >= 0; --i1)
			{
				if (rowIndex[i1] == colIndex[i1])
				{
					continue;
				}
				
				for (i2 = 0; i2 < nDim; ++i2)
				{
					T t = matInvM(i2, rowIndex[i1]);
					matInvM(i2, rowIndex[i1]) = matInvM(i2, colIndex[i1]);
					matInvM(i2, colIndex[i1]) = t;
				}
			}
		}

		if (odd && Determinant)
		{
			*Determinant = -*Determinant;
		}
		
		if (bufferInvM) delete []bufferInvM;
		if (buffer) delete []buffer;
		return true;
	}
};

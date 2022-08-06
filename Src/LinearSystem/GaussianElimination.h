#pragma once

#include <string.h>

#define matInvM(i, j)		(InvM[i*nRows + j])

template<typename T>
class GaussianElimination
{
public:
	bool operator()(const T* M, int nRows, T* InvM, T* Determinant) const
	{
		int nSize = nRows * nRows;
		T* localInverseM = nullptr;
		bool NeedInverse = InvM != nullptr;
		if (!NeedInverse)
		{
			localInverseM = new T[nSize];
			InvM = localInverseM;
		}
		memcpy(InvM, M, nSize * sizeof(T));

		int *buffer = new int[3*nRows];
		int *colIndex = buffer;
		int *rowIndex = buffer + nRows;
		int *pivoted = buffer + 2 * nRows;
		memset(pivoted, 0, sizeof(pivoted[0])*nRows);

		const T zero = (T)0;
		const T one = (T)1;
		bool odd = false;
		if (Determinant)
		{
			*Determinant = one;
		}

		int i1, i2, row = 0, col = 0;
		for (int i0 = 0; i0 < nRows; ++i0)
		{
			T maxVal = zero;
			for (i1 = 0; i1 < nRows; ++i1)
			{
				if (pivoted[i1] != 0)
				{
					continue;
				}
				
				for (i2 = 0; i2 < nRows; ++i2)
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

			if (FuzzyEqual(maxVal, zero))
			{
				if (NeedInverse)
				{
					memset(InvM, 0, nSize * sizeof(T));
				}
				if (Determinant)
				{
					*Determinant = zero;
				}
				
				if (localInverseM) delete []localInverseM;
				if (buffer) delete []buffer;
				return false;
			}

			pivoted[col] = true;

			if (row != col)
			{
				odd = !odd;
				for (int i = 0; i < nRows; ++i)
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
			for (i2 = 0; i2 < nRows; ++i2)
			{
				matInvM(col, i2) *= inv;
			}

			for (i1 = 0; i1 < nRows; ++i1)
			{
				if (i1 == col)
				{
					continue;
				}
				
				T save = matInvM(i1, col);
				matInvM(i1, col) = zero;
				for (i2 = 0; i2 < nRows; ++i2)
				{
					matInvM(i1, i2) -= matInvM(col, i2) * save;
				}
			}
		}

		if (InvM)
		{
			for (i1 = nRows - 1; i1 >= 0; --i1)
			{
				if (rowIndex[i1] == colIndex[i1])
				{
					continue;
				}
				
				for (i2 = 0; i2 < nRows; ++i2)
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
		
		if (localInverseM) delete []localInverseM;
		if (buffer) delete []buffer;
		return true;
	}
};

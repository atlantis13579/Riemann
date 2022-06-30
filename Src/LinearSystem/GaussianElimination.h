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
		std::vector<T> localInverseM;
		bool NeedInverse = InvM != nullptr;
		if (!NeedInverse)
		{
			localInverseM.resize(nSize);
			InvM = localInverseM.data();
		}
		memcpy(InvM, M, nSize * sizeof(T));

		std::vector<int> colIndex(nRows), rowIndex(nRows), pivoted(nRows);
		std::fill(pivoted.begin(), pivoted.end(), 0);

		const T zero = (T)0;
		const T one = (T)1;
		bool odd = false;
		if (Determinant) *Determinant = one;

		int i1, i2, row = 0, col = 0;
		for (int i0 = 0; i0 < nRows; ++i0)
		{
			T maxVal = zero;
			for (i1 = 0; i1 < nRows; ++i1)
			{
				if (!pivoted[i1])
				{
					for (i2 = 0; i2 < nRows; ++i2)
					{
						if (!pivoted[i2])
						{
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
				}
			}

			if (FuzzyEqual(maxVal, zero))
			{
				if (NeedInverse)
				{
					memset(InvM, 0, nSize * sizeof(T));
				}
				if (Determinant) *Determinant = zero;
				return false;
			}

			pivoted[col] = true;

			if (row != col)
			{
				odd = !odd;
				for (int i = 0; i < nRows; ++i)
				{
					std::swap(matInvM(row, i), matInvM(col, i));
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
				if (i1 != col)
				{
					T save = matInvM(i1, col);
					matInvM(i1, col) = zero;
					for (i2 = 0; i2 < nRows; ++i2)
					{
						matInvM(i1, i2) -= matInvM(col, i2) * save;
					}
				}
			}
		}

		if (InvM)
		{
			for (i1 = nRows - 1; i1 >= 0; --i1)
			{
				if (rowIndex[i1] != colIndex[i1])
				{
					for (i2 = 0; i2 < nRows; ++i2)
					{
						std::swap(matInvM(i2, rowIndex[i1]), matInvM(i2, colIndex[i1]));
					}
				}
			}
		}

		if (odd && Determinant)
		{
			*Determinant = -*Determinant;
		}

		return true;
	}
};

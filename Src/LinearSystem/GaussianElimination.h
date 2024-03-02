#pragma once

#include <string.h>

namespace Maths
{
namespace LinearAlgebra
{

#define matInvM(i, j)		(InvM[i*nDim + j])

template<typename T>
class GaussianElimination
{
public:
	bool operator()(const T* M, int nDim, T* InvM, T* Determinant) const
	{
		char  stack_mem[8 * 64 * 64 + 3 * 4 * 64];
		char* heap_mem = nullptr;

		int nSize = nDim * nDim;
		int* colIndex = nullptr;
		bool needInverse = InvM != nullptr;

		int mem_size = (!needInverse ? sizeof(T) * nSize : 0) + sizeof(int) * 3 * nDim;
		if (mem_size > sizeof(stack_mem))
		{
			heap_mem = new char[mem_size];
		}

		if (!needInverse)
		{
			InvM = heap_mem ? (T*)heap_mem : (T*)stack_mem;
			colIndex = (int*)(InvM + nSize);
		}
		else
		{
			colIndex = heap_mem ? (int*)heap_mem : (int*)stack_mem;
		}

		int* rowIndex = colIndex + nDim;
		int* pivoted = colIndex + 2 * nDim;

		memcpy(InvM, M, nSize * sizeof(T));
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
				if (needInverse)
				{
					memset(InvM, 0, nSize * sizeof(T));
				}
				if (Determinant)
				{
					*Determinant = zero;
				}
				
				if (heap_mem) delete []heap_mem;
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
		
		if (heap_mem) delete []heap_mem;

		return true;
	}
};

}	// namespace LinearAlgebra
}	// namespace Maths
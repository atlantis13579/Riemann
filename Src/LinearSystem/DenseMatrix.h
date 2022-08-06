#pragma once

#include <assert.h>
#include <math.h>
#include <cmath>
#include <vector>
#include "DenseVector.h"
#include "../Maths/Maths.h"

template<typename T>
class TDenseMatrix
{
public:
	TDenseMatrix() : mRows(0), mCols(0)
	{

	}

	TDenseMatrix(int nSize) : TDenseMatrix(nSize, nSize)
	{

	}

	TDenseMatrix(int nRows, int nCols) : mRows(nRows), mCols(nCols), mData(nCols* nRows)
	{
		pData = &mData[0];
	}

	TDenseMatrix(const TDenseMatrix& v) : mRows(v.mRows), mCols(v.mCols), mData(mCols* mRows)
	{
		pData = &mData[0];
		memcpy(pData, v.pData, sizeof(T) * mRows * mCols);
	}

	TDenseMatrix(TDenseMatrix&& v) : mRows(v.mRows), mCols(v.mCols)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	TDenseMatrix(T* p, int nRows, int nCols) : mRows(nRows), mCols(nCols)
	{
		pData = p;
	}

	void		SetSize(int nRows, int nCols)
	{
		mRows = nRows;
		mCols = nCols;
		mData.resize(nCols * nRows);
		pData = &mData[0];
	}

	inline int	GetLength() const
	{
		return mRows * mCols;
	}

	inline int	GetRows() const
	{
		return mRows;
	}

	inline	int	GetCols() const
	{
		return mCols;
	}

	inline T*	GetData()
	{
		return pData;
	}

	inline const T*	GetData() const
	{
		return pData;
	}

	TDenseVector<T>	GetRow(int i) const
	{
		TDenseVector<T> Ret(mCols);
		memcpy(Ret.GetData(), pData + i * mCols, sizeof(T) * mCols);
		return std::move(Ret);
	}

	TDenseVector<T>	GetCol(int i) const
	{
		TDenseVector<T> Ret(mRows);
		for (int j = 0; j < mRows; ++j)
		{
			Ret[j] = pData[j * mCols + i];
		}
		return std::move(Ret);
	}

	inline bool		IsSquare() const
	{
		return mCols == mRows;
	}

	inline bool		IsColumnVector() const
	{
		return mCols == 1;
	}

	inline bool		IsRowVector() const
	{
		return mRows == 1;
	}

	inline const T* operator[](int i) const
	{
		return pData + i * mCols;
	}

	inline T* operator[](int i)
	{
		return pData + i * mCols;
	}

	inline T operator()(int i, int j) const
	{
		return pData[i * mCols + j];
	}

	inline T& operator()(int i, int j)
	{
		return pData[i * mCols + j];
	}

	TDenseMatrix<T>& operator=(const TDenseMatrix<T>& v)
	{
		mRows = v.mRows;
		mCols = v.mCols;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TDenseMatrix<T>& operator=(TDenseMatrix<T>&& v)
	{
		mRows = v.mRows;
		mCols = v.mCols;
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TDenseMatrix<T> operator+(const TDenseMatrix<T>& v) const
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return TDenseMatrix<T>();
		}

		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] += v.pData[i];
		}
		return std::move(Ret);
	}

	TDenseMatrix<T> operator-(const TDenseMatrix<T>& v) const
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return TDenseMatrix<T>();
		}

		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] -= v.pData[i];
		}
		return std::move(Ret);
	}

	TDenseMatrix<T> operator*(T k) const
	{
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] *= k;
		}
		return std::move(Ret);
	}

	TDenseMatrix<T>	operator*(const TDenseMatrix<T>& v) const;
	TDenseVector<T>	operator*(const TDenseVector<T>& v) const;

	TDenseMatrix<T>	operator-() const
	{
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] = -Ret.pData[i];
		}
		return std::move(Ret);
	}

	void		operator+= (const TDenseMatrix<T>& v)
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return;
		}

		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] += v.pData[i];
		}
	}

	void		operator-= (const TDenseMatrix<T>& v)
	{
		if (GetLength() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] -= v.pData[i];
		}
	}

	void		operator*= (T k)
	{
		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] *= k;
		}
	}

	void		TransposeInPlace()
	{
		for (int i = 0; i < mRows; ++i)
		for (int j = i + 1; j < mCols; ++j)
		{
			T t = pData[i * mRows + j];
			pData[i * mRows + j] = pData[j * mCols + i];
			pData[j * mCols + i] = t;
		}
		std::swap(mRows, mCols);
	}

	TDenseMatrix<T>		DotMatrix() const		// X^T * X
	{
		TDenseMatrix<T> Ret(mCols, mCols);
		for (int i = 0; i < mCols; ++i)
		for (int j = i; j < mCols; ++j)
		{
			T sum = (T)0;
			for (int k = 0; k < mRows; ++k)
			{
				sum += pData[k * mCols + i] * pData[k * mCols + j];
			}
			Ret(i, j) = sum;
		}
		for (int i = 0; i < mCols; ++i)
		for (int j = 0; j < i; ++j)
		{
			Ret(i, j) = Ret(j, i);
		}
		return std::move(Ret);
	}

	void		LoadZero()
	{
		memset(pData, 0, sizeof(T) * mRows * mCols);
	}

	void		LoadIdentity()
	{
		if (!IsSquare())
		{
			return;
		}

		LoadZero();
		for (int i = 0; i < mRows; ++i)
		{
			pData[i * mRows + i] = (T)1;
		}
	}
	
	void 		LoadDiagonal(const TDenseVector<T> &diag)
	{
		if (!IsSquare() || mRows != diag.GetSize())
		{
			return;
		}
		
		LoadZero();
		for (int i = 0; i < mRows; ++i)
		{
			pData[i * mRows + i] = diag[i];
		}
	}

	bool		IsZero() const
	{
		for (int i = 0; i < GetLength(); ++i)
		{
			if (!FuzzyZero(pData[i]))
				return false;
		}
		return true;
	}

	bool		IsIdentity(float Eplison = 1e-6f) const
	{
		if (!IsSquare())
		{
			return false;
		}

		int n = mRows;
		for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			T v = pData[i * n + j];
			if (i == j)
			{
				if (!FuzzyEqual(v, (T)1, Eplison))
					return false;
			}
			else
			{
				if (!FuzzyEqual(v, (T)0, Eplison))
					return false;
			}
		}
		return true;
	}

	T			Trace() const
	{
		assert(IsSquare());
		T tr = (T)0;
		for (int i = 0; i < mRows; ++i)
		{
			tr += pData[i * mRows + i];
		}
		return tr;
	}

	T			Determinant() const;

	TDenseMatrix<T>	Inverse() const
	{
		assert(IsSquare());
		TDenseMatrix<T> InvM(GetRows());
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return std::move(InvM);
	}
	
	TDenseMatrix<T> PseudoInverse() const;

	bool		GetInverse(TDenseMatrix<T>& InvM) const;

	TDenseMatrix<T>	Transpose() const
	{
		TDenseMatrix<T> Ret(*this);
		Ret.TransposeInPlace();
		return std::move(Ret);
	}

	T			LpNorm(int p) const
	{
		T sum = (T)0;
		for (int i = 1; i < mRows * mCols; ++i)
		{
			sum += std::pow(pData[i], p);
		}
		return std::pow(sum, (T)1 / p);
	}

	T			LInfinityNorm() const
	{
		T maxVal = pData[0];
		for (int i = 1; i < mRows * mCols; ++i)
		{
			maxVal = std::max(maxVal, pData[i]);
		}
		return maxVal;
	}

protected:
	int				mRows;
	int				mCols;
	std::vector<T>	mData;
	T* 				pData;
};

template <typename T>
inline TDenseMatrix<T> operator* (T s, const TDenseMatrix<T>& vv)
{
	return vv * s;
}

using DenseMatrix = TDenseMatrix<float>;

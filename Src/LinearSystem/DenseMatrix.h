#pragma once

#include <math.h>
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

	int			GetLength() const
	{
		return mRows * mCols;
	}

	int			GetRows() const
	{
		return mRows;
	}

	int			GetCols() const
	{
		return mCols;
	}

	T* GetData()
	{
		return pData;
	}

	const T* GetData() const
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

	TDenseMatrix<T>	operator-()
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

protected:
	int				mRows;
	int				mCols;
	std::vector<T>	mData;
	T* pData;
};

template <typename T>
inline TDenseMatrix<T> operator* (T s, const TDenseMatrix<T>& vv)
{
	return vv * s;
}

template<typename T>
class TDenseSquare
{
public:
	TDenseSquare() : mSize(0), pData(nullptr)
	{
	}

	TDenseSquare(int nSize, bool Identity = false) : mSize(nSize), mData(nSize* nSize)
	{
		pData = &mData[0];

		if (Identity)
		{
			LoadIdentity();
		}
	}

	TDenseSquare(const TDenseSquare& v) : mSize(v.mSize), mData(mSize* mSize)
	{
		pData = &mData[0];
		memcpy(pData, v.pData, sizeof(T) * mSize * mSize);
	}

	TDenseSquare(TDenseSquare&& v)
	{
		mSize = v.mSize;
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	inline const T* operator[](int i) const
	{
		return pData + i * mSize;
	}

	inline T* operator[](int i)
	{
		return pData + i * mSize;
	}

	inline T operator()(int i, int j) const
	{
		return pData[i * mSize + j];
	}

	inline T& operator()(int i, int j)
	{
		return pData[i * mSize + j];
	}

	TDenseSquare<T>& operator=(const TDenseSquare<T>& v)
	{
		mSize = v.mSize;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TDenseSquare<T>& operator=(TDenseSquare<T>&& v)
	{
		mSize = v.mSize;
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	void		SetSize(int nSize)
	{
		mSize = nSize;
		mData.resize(nSize * nSize);
		pData = &mData[0];
	}

	int			GetSize() const
	{
		return mSize;
	}

	int			GetLength() const
	{
		return mSize * mSize;
	}

	T* GetData()
	{
		return pData;
	}

	const T* GetData() const
	{
		return pData;
	}

	TDenseVector<T>	GetRow(int i) const
	{
		TDenseVector<T> Ret(mSize);
		memcpy(Ret.GetData(), pData + i * mSize, sizeof(T) * mSize);
		return std::move(Ret);
	}

	TDenseVector<T>	GetCol(int i) const
	{
		TDenseVector<T> Ret(mSize);
		for (int j = 0; j < mSize; ++j)
		{
			Ret[j] = pData[j * mSize + i];
		}
		return std::move(Ret);
	}


	void		LoadZero()
	{
		memset(pData, 0, sizeof(T) * mSize * mSize);
	}

	void		LoadIdentity()
	{
		LoadZero();
		for (int i = 0; i < mSize; ++i)
		{
			pData[i * mSize + i] = (T)1;
		}
	}

	bool		IsZero() const
	{
		for (int i = 0; i < GetLength(); ++i)
		{
			if (!FuzzyEqual(pData[i], (T)0))
				return false;
		}
		return true;
	}

	bool		IsIdentity() const
	{
		int n = mSize;
		for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j)
			{
				T v = pData[i * n + j];
				if (i == j)
				{
					if (!FuzzyEqual(v, (T)1))
						return false;
				}
				else
				{
					if (!FuzzyEqual(v, (T)0))
						return false;
				}
			}
		return true;
	}

	TDenseSquare<T> operator+(const TDenseSquare<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TDenseSquare<T>();
		}

		TDenseSquare<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] += v.pData[i];
		}
		return std::move(Ret);
	}

	TDenseSquare<T> operator-(const TDenseSquare<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TDenseSquare<T>();
		}

		TDenseSquare<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] -= v.pData[i];
		}
		return std::move(Ret);
	}

	TDenseSquare<T> operator*(T k) const
	{
		TDenseSquare<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] *= k;
		}
		return std::move(Ret);
	}

	TDenseSquare<T>	operator*(const TDenseSquare<T>& v) const;
	TDenseVector<T>			operator*(const TDenseVector<T>& v) const;

	TDenseSquare<T>	operator-()
	{
		TDenseSquare<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] = -Ret.pData[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TDenseSquare<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] += v.pData[i];
		}
	}

	void	 operator-= (const TDenseSquare<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] -= v.pData[i];
		}
	}

	void				operator*= (T k)
	{
		for (int i = 0; i < GetLength(); ++i)
		{
			pData[i] *= k;
		}
	}

	void				operator*= (const TDenseMatrix<T>& v)
	{
		*this = *this * v;
	}

	T					Trace() const
	{
		T tr = (T)0;
		for (int i = 0; i < mSize; ++i)
		{
			tr += pData[i * mSize + i];
		}
		return tr;
	}

	T					Determinant() const;

	TDenseSquare<T>	Inverse() const
	{
		TDenseSquare<T> InvM(GetSize());
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return std::move(InvM);
	}

	bool				GetInverse(TDenseSquare<T>& InvM) const;

	void				TransposeInPlace()
	{
		TDenseMatrix<T> Ret(pData, mSize, mSize);
		Ret.TransposeInPlace();
	}

	TDenseSquare<T>	Transpose() const
	{
		TDenseSquare<T> Ret(*this);
		Ret.TransposeInPlace();
		return std::move(Ret);
	}

protected:
	int				mSize;
	std::vector<T>	mData;
	T* pData;
};

template <typename T>
inline TDenseSquare<T> operator* (T s, const TDenseSquare<T>& vv)
{
	return vv * s;
}

using DenseMatrix = TDenseMatrix<float>;
using DenseSquare = TDenseSquare<float>;
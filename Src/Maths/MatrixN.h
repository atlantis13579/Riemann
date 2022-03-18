#pragma once

#include <vector>
#include "VectorN.h"
#include "Maths.h"

template<typename T>
class TMatrix
{
public:
	TMatrix() : mRows(0), mCols(0)
	{

	}

	TMatrix(int nRows, int nCols) : mRows(nRows), mCols(nCols), mData(nCols* nRows)
	{
		pData = &mData[0];
	}

	TMatrix(const TMatrix& v) : mRows(v.mRows), mCols(v.mCols), mData(mCols* mRows)
	{
		pData = &mData[0];
		memcpy(pData, v.pData, sizeof(T) * mRows * mCols);
	}

	TMatrix(TMatrix&& v) : mRows(v.mRows), mCols(v.mCols)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	TMatrix(T* p, int nRows, int nCols) : mRows(nRows), mCols(nCols)
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

	T*			GetData()
	{
		return pData;
	}

	const T*	GetData() const
	{
		return pData;
	}

	TVector<T>	GetRow(int i) const
	{
		TVector<T> Ret(mCols);
		memcpy(Ret.GetData(), pData + i * mCols, sizeof(T) * mCols);
		return std::move(Ret);
	}

	TVector<T>	GetCol(int i) const
	{
		TVector<T> Ret(mRows);
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

	TMatrix<T>& operator=(const TMatrix<T>& v)
	{
		mRows = v.mRows;
		mCols = v.mCols;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TMatrix<T>& operator=(TMatrix<T>&& v)
	{
		mRows = v.mRows;
		mCols = v.mCols;
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TMatrix<T> operator+(const TMatrix<T>& v) const
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return TMatrix<T>();
		}

		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] += v.pData[i];
		}
		return std::move(Ret);
	}

	TMatrix<T> operator-(const TMatrix<T>& v) const
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return TMatrix<T>();
		}

		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] -= v.pData[i];
		}
		return std::move(Ret);
	}

	TMatrix<T> operator*(T k) const
	{
		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] *= k;
		}
		return std::move(Ret);
	}

	TMatrix<T>	operator*(const TMatrix<T>& v) const;
	TVector<T>	operator*(const TVector<T>& v) const;

	TMatrix<T>	operator-()
	{
		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] = -Ret.pData[i];
		}
		return std::move(Ret);
	}

	void		operator+= (const TMatrix<T>& v)
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

	void		operator-= (const TMatrix<T>& v)
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

	void		operator*= (const TMatrix<T>& v)
	{
		*this = *this * v;
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
	T*				pData;
};

template <typename T>
inline TMatrix<T> operator* (T s, const TMatrix<T>& vv)
{
	return vv * s;
}

template<typename T>
class TSquareMatrix
{
public:
	TSquareMatrix() : mSize(0), pData(nullptr)
	{
	}

	TSquareMatrix(int nSize, bool Identity = false) : mSize(nSize), mData(nSize * nSize)
	{
		pData = &mData[0];

		if (Identity)
		{
			LoadIdentity();
		}
	}

	TSquareMatrix(const TSquareMatrix& v) : mSize(v.mSize), mData(mSize* mSize)
	{
		pData = &mData[0];
		memcpy(pData, v.pData, sizeof(T) * mSize * mSize);
	}

	TSquareMatrix(TSquareMatrix&& v)
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

	TSquareMatrix<T>& operator=(const TSquareMatrix<T>& v)
	{
		mSize = v.mSize;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TSquareMatrix<T>& operator=(TSquareMatrix<T>&& v)
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

	T*			GetData()
	{
		return pData;
	}

	const T*	GetData() const
	{
		return pData;
	}

	TVector<T>	GetRow(int i) const
	{
		TVector<T> Ret(mSize);
		memcpy(Ret.GetData(), pData + i * mSize, sizeof(T) * mSize);
		return std::move(Ret);
	}

	TVector<T>	GetCol(int i) const
	{
		TVector<T> Ret(mSize);
		for (int j = 0; j < mSize; ++j)
		{
			Ret[j] = pData[j * mSize + i];
		}
		return std::move(Ret);
	}


	void		LoadZero()
	{
		memset(pData, 0, sizeof(T)* mSize * mSize);
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

	TSquareMatrix<T> operator+(const TSquareMatrix<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TSquareMatrix<T>();
		}

		TSquareMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] += v.pData[i];
		}
		return std::move(Ret);
	}

	TSquareMatrix<T> operator-(const TSquareMatrix<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TSquareMatrix<T>();
		}

		TSquareMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] -= v.pData[i];
		}
		return std::move(Ret);
	}

	TSquareMatrix<T> operator*(T k) const
	{
		TSquareMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] *= k;
		}
		return std::move(Ret);
	}

	TSquareMatrix<T>	operator*(const TSquareMatrix<T>& v) const;
	TVector<T>			operator*(const TVector<T>& v) const;

	TSquareMatrix<T>	operator-()
	{
		TSquareMatrix<T> Ret(*this);
		for (int i = 0; i < GetLength(); ++i)
		{
			Ret.pData[i] = -Ret.pData[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TSquareMatrix<T>& v)
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

	void	 operator-= (const TSquareMatrix<T>& v)
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

	void				operator*= (const TMatrix<T>& v)
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

	TSquareMatrix<T>	Inverse() const
	{
		TSquareMatrix<T> InvM(GetSize());
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return std::move(InvM);
	}
	
	bool				GetInverse(TSquareMatrix<T> &InvM) const;

	void				TransposeInPlace()
	{
		TMatrix<T> Ret(pData, mSize, mSize);
		Ret.TransposeInPlace();
	}

	TSquareMatrix<T>	Transpose() const
	{
		TSquareMatrix<T> Ret(*this);
		Ret.TransposeInPlace();
		return std::move(Ret);
	}

protected:
	int				mSize;
	std::vector<T>	mData;
	T*				pData;
};

template <typename T>
inline TSquareMatrix<T> operator* (T s, const TSquareMatrix<T>& vv)
{
	return vv * s;
}

using MatrixN = TSquareMatrix<float>;
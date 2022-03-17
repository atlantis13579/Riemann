#pragma once

#include <vector>
#include "VectorN.h"

template<typename T>
class TMatrix
{
public:
	TMatrix() : mRows(0), mCols(0)
	{

	}

	TMatrix(int nRows, int nCols) : mRows(nRows), mCols(nCols), mData(nCols * nRows)
	{
	}

	int			GetRows() const
	{
		return mRows;
	}

	int			GetCols() const
	{
		return mCols;
	}

	int			GetSize() const
	{
		return mRows * mCols;
	}

	T*			GetData()
	{
		return &mData[0];
	}

	const T*	GetData() const
	{
		return &mData[0];
	}

	inline const TVector<T>& operator[](int i) const
	{
		return TVector<T>(mData + i * mCols, mCols);
	}

	inline TVector<T>& operator[](int i)
	{
		return TVector<T>(mData + i * mCols, mCols);
	}

	inline T operator()(int i, int j) const
	{
		return mData[i * mCols + j];
	}

	inline T& operator()(int i, int j)
	{
		return mData[i * mCols + j];
	}

	TMatrix<T>& operator=(const TMatrix<T>& v)
	{
		mRows = v.mRows;
		mCols = v.mCols;
		mData = v.mData;
		return *this;
	}

	TMatrix<T> operator+(const TMatrix<T>& v) const
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return TMatrix<T>();
		}

		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.mData[i] += v.mData[i];
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
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.mData[i] -= v.mData[i];
		}
		return std::move(Ret);
	}

	TMatrix<T> operator*(const TMatrix<T>& v) const;

	TMatrix<T> operator*(T k) const
	{
		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.mData[i] *= k;
		}
		return std::move(Ret);
	}

	TMatrix<T> operator-()
	{
		TMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.mData[i] = -Ret[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TMatrix<T>& v)
	{
		if (GetCols() != v.GetCols() || GetRows() != v.GetRows())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			mData[i] += v.mData[i];
		}
	}

	void	 operator-= (const TMatrix<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			mData[i] -= v.mData[i];
		}
	}

	void	 operator*= (const TMatrix<T>& v);

	void				TransposeInPlace()
	{
		for (int i = 0; i < mRows; ++i)
		for (int j = i + 1; i < mCols; ++j)
		{
			T t = mData[i * mRows + j];
			mData[i * mRows + j] = mData[j * mCols + i];
			mData[j * mCols + i] = t;
		}
		std::swap(mRows, mCols);
	}

protected:
	int				mRows;
	int				mCols;
	std::vector<T>	mData;
};

template<typename T>
class TSquareMatrix : public TMatrix<T>
{
public:
	TSquareMatrix(int nSize) : TMatrix(nSize, nSize)
	{
	}

	TSquareMatrix(const TSquareMatrix& m) : TMatrix(m)
	{
	}

	T					Trace() const
	{
		T tr = (T)0;
		for (int i = 0; i < this->mRows; ++i)
		{
			tr += this->mData[i * this->mRows + i];
		}
		return tr;
	}

	T					Determinant() const
	{
		// TODO
	}

	TSquareMatrix<T>	Inverse() const
	{
		// TODO
	}

	TSquareMatrix<T>	Transpose() const
	{
		TSquareMatrix<T> Ret(*this);
		Ret.TransposeInPlace();
		return std::move(Ret);
	}
};

void gemm_slow(const float *m1, const float* m2, int r1, int c1, int c2, float* m);
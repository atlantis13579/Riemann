#pragma once

#include <vector>
#include "VectorNd.h"
#include "Maths.h"
#include "../LinearSystem/GaussianElimination.h"

template<typename T, int ROWS, int COLS>
class TMatrixMxN
{
public:
	TMatrixMxN()
	{

	}

	TMatrixMxN(const TMatrixMxN<T, ROWS, COLS>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * ROWS * COLS);
	}

	TVectorNd<T, COLS>	GetRow(int i) const
	{
		TVectorNd<T, COLS> Ret;
		memcpy(*Ret, mData + i * COLS, sizeof(T) * COLS);
		return std::move(Ret);
	}

	TVectorNd<T, ROWS>	GetCol(int i) const
	{
		TVectorNd<T, ROWS> Ret;
		for (int j = 0; j < ROWS; ++j)
		{
			Ret[j] = mData[j * COLS + i];
		}
		return std::move(Ret);
	}

	inline const T* operator*() const
	{
		return mData;
	}

	inline T* operator*()
	{
		return mData;
	}

	inline const T* operator[](int i) const
	{
		return mData + i * COLS;
	}

	inline T* operator[](int i)
	{
		return mData + i * COLS;
	}

	inline T operator()(int i, int j) const
	{
		return mData[i * COLS + j];
	}

	inline T& operator()(int i, int j)
	{
		return mData[i * COLS + j];
	}

	TMatrixMxN<T, ROWS, COLS>& operator=(const TMatrixMxN<T, ROWS, COLS>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * ROWS * COLS);
		return *this;
	}

	TMatrixMxN<T, ROWS, COLS> operator+(const TMatrixMxN<T, ROWS, COLS>& v) const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] += v.mData[i];
		}
		return std::move(Ret);
	}

	TMatrixMxN<T, ROWS, COLS> operator-(const TMatrixMxN<T, ROWS, COLS>& v) const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] -= v.mData[i];
		}
		return std::move(Ret);
	}

	TMatrixMxN<T, ROWS, COLS> operator*(T k) const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] *= k;
		}
		return std::move(Ret);
	}

	template<int COLS2>
	TMatrixMxN<T, ROWS, COLS2>	operator*(const TMatrixMxN<T, COLS, COLS2>& v) const
	{
		TMatrixMxN<T, ROWS, COLS2> Ret;
		T* pm = *Ret;
		const T* pv = *v;
		for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS2; ++j)
		{
			T dp = (T)0;
			const T* p = mData + i * COLS;
			for (int k = 0; k < COLS; ++k)
				dp += p[k] * pv[k * COLS2 + j];
			pm[i * ROWS + j] = dp;
		}
		return Ret;
	}

	TVectorNd<T, ROWS>			operator*(const TVectorNd<T, COLS>& v) const
	{
		TVectorNd<T, ROWS> Ret;
		T* pm = *Ret;
		const T* pv = *v;
		for (int i = 0; i < ROWS; ++i)
		{
			T dp = (T)0;
			const T* p = mData + i * COLS;
			for (int k = 0; k < COLS; ++k)
				dp += p[k] * pv[k];
			pm[i] = dp;
		}
		return Ret;
	}

	TMatrixMxN<T, ROWS, COLS>	operator-()
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] = -Ret.mData[i];
		}
		return std::move(Ret);
	}

	void		operator+= (const TMatrixMxN<T, ROWS, COLS>& v)
	{
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			mData[i] += v.mData[i];
		}
	}

	void		operator-= (const TMatrixMxN<T, ROWS, COLS>& v)
	{
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			mData[i] -= v.mData[i];
		}
	}

	void		operator*= (T k)
	{
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			mData[i] *= k;
		}
	}

	TMatrixMxN<T, COLS, ROWS>	Transpose()
	{
		TMatrixMxN<T, COLS, ROWS> Ret;
		for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
		{
			Ret[i * ROWS + j] = mData[j * COLS + i];
		}
		return Ret;
	}

protected:
	T				mData[ROWS * COLS];
};

template <typename T, int ROWS, int COLS>
inline TMatrixMxN<T, ROWS, COLS> operator* (T s, const TMatrixMxN<T, ROWS, COLS>& vv)
{
	return vv * s;
}


template<typename T, int SIZE>
class TMatrixNd
{
public:
	TMatrixNd(bool Identity = false)
	{
		if (Identity)
		{
			LoadIdentity();
		}
	}

	TMatrixNd(const TMatrixNd<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE * SIZE);
	}

	inline const T* operator*() const
	{
		return mData;
	}

	inline T* operator*()
	{
		return mData;
	}

	inline const T* operator[](int i) const
	{
		return mData + i * SIZE;
	}

	inline T* operator[](int i)
	{
		return mData + i * SIZE;
	}

	inline T operator()(int i, int j) const
	{
		return mData[i * SIZE + j];
	}

	inline T& operator()(int i, int j)
	{
		return mData[i * SIZE + j];
	}

	TMatrixNd<T, SIZE>& operator=(const TMatrixNd<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE * SIZE);
		return *this;
	}

	TVectorNd<T, SIZE>	GetRow(int i) const
	{
		TVectorNd<T, SIZE> Ret;
		memcpy(*Ret, mData + i * SIZE, sizeof(T) * SIZE);
		return Ret;
	}

	TVectorNd<T, SIZE>	GetCol(int i) const
	{
		TVectorNd<T, SIZE> Ret;
		for (int j = 0; j < SIZE; ++j)
		{
			Ret[j] = mData[j * SIZE + i];
		}
		return Ret;
	}

	void		LoadZero()
	{
		memset(mData, 0, sizeof(T)* SIZE * SIZE);
	}

	void		LoadIdentity()
	{
		LoadZero();
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i * SIZE + i] = (T)1;
		}
	}

	bool		IsIdentity() const
	{
		int n = SIZE;
		for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			T v = mData[i * n + j];
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

	TMatrixNd<T, SIZE> operator+(const TMatrixNd<T, SIZE>& v) const
	{
		TMatrixNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			Ret.mData[i] += v.mData[i];
		}
		return Ret;
	}

	TMatrixNd<T, SIZE> operator-(const TMatrixNd<T, SIZE>& v) const
	{
		TMatrixNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			Ret.mData[i] -= v.mData[i];
		}
		return Ret;
	}

	TMatrixNd<T, SIZE> operator*(T k) const
	{
		TMatrixNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			Ret.mData[i] *= k;
		}
		return Ret;
	}

	TMatrixNd<T, SIZE>	operator*(const TMatrixNd<T, SIZE>& v) const
	{
		TMatrixNd<T, SIZE> Ret;
		T* pm = *Ret;
		const T* pv = *v;
		for (int i = 0; i < SIZE; ++i)
		for (int j = 0; j < SIZE; ++j)
		{
			T dp = (T)0;
			const T* p = mData + i * SIZE;
			for (int k = 0; k < SIZE; ++k)
				dp += p[k] * pv[k * SIZE + j];
			pm[i * SIZE + j] = dp;
		}
		return Ret;
	}

	TVectorNd<T, SIZE>	operator*(const TVectorNd<T, SIZE>& v) const
	{
		TVectorNd<T, SIZE> Ret;
		T* pm = *Ret;
		const T* pv = *v;
		for (int i = 0; i < SIZE; ++i)
		{
			T dp = (T)0;
			const T* p = mData + i * SIZE;
			for (int k = 0; k < SIZE; ++k)
				dp += p[k] * pv[k];
			pm[i] = dp;
		}
		return Ret;
	}

	TMatrixNd<T, SIZE>	operator-()
	{
		TMatrixNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			Ret.mData[i] = -Ret.mData[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TMatrixNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			mData[i] += v.mData[i];
		}
	}

	void	 operator-= (const TMatrixNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			mData[i] -= v.mData[i];
		}
	}

	void				operator*= (T k)
	{
		for (int i = 0; i < SIZE * SIZE; ++i)
		{
			mData[i] *= k;
		}
	}

	void				operator*= (const TMatrixNd<T, SIZE>& v)
	{
		*this = *this * v;
	}

	T					Trace() const
	{
		T tr = (T)0;
		for (int i = 0; i < SIZE; ++i)
		{
			tr += mData[i * SIZE + i];
		}
		return tr;
	}

	T					Determinant() const
	{
		float Det;
		if (GaussianElimination<float>()(mData, SIZE, nullptr, &Det))
		{
			return Det;
		}
		return 0.0f;
	}

	TMatrixNd<T, SIZE>	Inverse() const
	{
		TMatrixNd<T, SIZE> InvM;
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return InvM;
	}
	
	bool				GetInverse(TMatrixNd<T, SIZE>& InvM) const
	{
		return GaussianElimination<float>()(mData, SIZE, *InvM, nullptr);
	}

	void				TransposeInPlace()
	{
		for (int i = 0; i < SIZE; ++i)
		for (int j = i + 1; j < SIZE; ++j)
		{
			T t = mData[i * SIZE + j];
			mData[i * SIZE + j] = mData[j * SIZE + i];
			mData[j * SIZE + i] = t;
		}
	}

	TMatrixNd<T, SIZE>	Transpose() const
	{
		TMatrixNd<T, SIZE> Ret(*this);
		Ret.TransposeInPlace();
		return Ret;
	}

protected:
	T			mData[SIZE * SIZE];
};

template <typename T, int SIZE>
inline TMatrixNd<T, SIZE> operator* (T s, const TMatrixNd<T, SIZE>& vv)
{
	return vv * s;
}

template<int ROWS, int COLS>
using MatrixMxN = TMatrixMxN<float, ROWS, COLS>;

template<int SIZE>
using SquareMatrix = TMatrixNd<float, SIZE>;

#pragma once

#include <type_traits>
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

	void		LoadZero()
	{
		memset(mData, 0, sizeof(T) * ROWS * COLS);
	}

	template<typename = typename std::enable_if<ROWS == COLS>::type>
	void		LoadIdentity()
	{
		LoadZero();
		for (int i = 0; i < ROWS; ++i)
		{
			mData[i * ROWS + i] = (T)1;
		}
	}

	template<typename = typename std::enable_if<ROWS == COLS>::type>
	bool		IsIdentity() const
	{
		int n = ROWS;
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

	template<typename = typename std::enable_if<ROWS == COLS>::type>
	T							Determinant() const
	{
		float Det;
		if (GaussianElimination<float>()(mData, ROWS, nullptr, &Det))
		{
			return Det;
		}
		return 0.0f;
	}

	template<typename = typename std::enable_if<ROWS == COLS>::type>
	TMatrixMxN<T, ROWS, ROWS>	Inverse() const
	{
		TMatrixMxN<T, ROWS, ROWS> InvM;
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return InvM;
	}

	T					InfinityNorm() const
	{
		T maxVal = mData[0];
		for (int i = 1; i < ROWS * COLS; ++i)
		{
			maxVal = std::max(maxVal, mData[i]);
		}
		return maxVal;
	}

	template<typename = typename std::enable_if<ROWS == COLS>::type>
	bool				GetInverse(TMatrixMxN<T, ROWS, ROWS>& InvM) const
	{
		return GaussianElimination<float>()(mData, ROWS, *InvM, nullptr);
	}

	TMatrixMxN<T, COLS, ROWS>	Transpose()
	{
		TMatrixMxN<T, COLS, ROWS> Ret;
		T* p = *Ret;
		for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
		{
			p[i * ROWS + j] = mData[j * COLS + i];
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
using TMatrixNd = TMatrixMxN<T, SIZE, SIZE>;

template<int ROWS, int COLS>
using MatrixMxN = TMatrixMxN<float, ROWS, COLS>;

template<int SIZE>
using SquareMatrix = TMatrixNd<float, SIZE>;

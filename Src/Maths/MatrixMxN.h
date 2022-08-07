#pragma once

#include <string.h>
#include <cmath>
#include <type_traits>
#include <vector>
#include "VectorN.h"
#include "Maths.h"
#include "../LinearSystem/GaussianElimination.h"

#ifdef _MSC_VER
#define DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX	template<typename = typename std::enable_if<ROWS == COLS>::type>
#else
#define DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX   
#endif

template<typename T, int ROWS, int COLS>
class TMatrixMxN
{
public:
	TMatrixMxN()
	{
	}

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
	explicit TMatrixMxN(bool Identity)
	{
		if (Identity)
		{
			LoadIdentity();
		}
		else
		{
			memset(mData, 0, sizeof(T) * ROWS * COLS);
		}
	}

	TMatrixMxN(const std::initializer_list<T>& v)
	{
		memset(mData, 0, sizeof(T) * ROWS * COLS);
		int i = 0;
		for (const T& vv : v)
		{
			mData[i] = vv;
			if (i++ >= ROWS * COLS)
				break;
		}
	}

	TMatrixMxN(const TMatrixMxN<T, ROWS, COLS>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * ROWS * COLS);
	}

	TVectorN<T, COLS>	GetRow(int i) const
	{
		TVectorN<T, COLS> Ret;
		memcpy(*Ret, mData + i * COLS, sizeof(T) * COLS);
		return Ret;
	}

	TVectorN<T, ROWS>	GetCol(int i) const
	{
		TVectorN<T, ROWS> Ret;
		for (int j = 0; j < ROWS; ++j)
		{
			Ret[j] = mData[j * COLS + i];
		}
		return Ret;
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
		return Ret;
	}

	TMatrixMxN<T, ROWS, COLS> operator-(const TMatrixMxN<T, ROWS, COLS>& v) const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] -= v.mData[i];
		}
		return Ret;
	}

	TMatrixMxN<T, ROWS, COLS> operator*(T k) const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] *= k;
		}
		return Ret;
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
			pm[i * COLS2 + j] = dp;
		}
		return Ret;
	}

	TVectorN<T, ROWS>			operator*(const TVectorN<T, COLS>& v) const
	{
		TVectorN<T, ROWS> Ret;
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

	TMatrixMxN<T, ROWS, COLS>	operator-() const
	{
		TMatrixMxN<T, ROWS, COLS> Ret(*this);
		for (int i = 0; i < ROWS * COLS; ++i)
		{
			Ret.mData[i] = -Ret.mData[i];
		}
		return Ret;
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

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
	void		LoadIdentity()
	{
		LoadZero();
		for (int i = 0; i < ROWS; ++i)
		{
			mData[i * ROWS + i] = (T)1;
		}
	}

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
	bool		IsIdentity(T eps = Epsilon<T>()) const
	{
		int n = ROWS;
		for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			T v = mData[i * n + j];
			if (i == j)
			{
				if (!FuzzyEqual(v, (T)1, eps))
					return false;
			}
			else
			{
				if (!FuzzyEqual(v, (T)0, eps))
					return false;
			}
		}
		return true;
	}

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
	T					Determinant() const
	{
		T Det;
		if (GaussianElimination<T>()(mData, ROWS, nullptr, &Det))
		{
			return Det;
		}
		return (T)0;
	}

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
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

	T					LpNorm(int p) const
	{
		T sum = (T)0;
		for (int i = 1; i < ROWS * COLS; ++i)
		{
			sum += std::pow(mData[i], p);
		}
		return std::pow(sum, (T)1 / p);
	}

	T					LInfinityNorm() const
	{
		T maxVal = mData[0];
		for (int i = 1; i < ROWS * COLS; ++i)
		{
			maxVal = std::max(maxVal, mData[i]);
		}
		return maxVal;
	}

	DIASBLE_DECLARATION_FOR_NON_SQUARE_MATRIX
	bool				GetInverse(TMatrixMxN<T, ROWS, ROWS>& InvM) const
	{
		return GaussianElimination<T>()(mData, ROWS, *InvM, nullptr);
	}

	TMatrixMxN<T, COLS, ROWS>	Transpose() const
	{
		TMatrixMxN<T, COLS, ROWS> Ret;
		T* p = *Ret;
		for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
		{
			p[j * ROWS + i] = mData[i * COLS + j];
		}
		return Ret;
	}

	static TMatrixMxN<T, ROWS, COLS> Zero()
	{
		return TMatrixMxN<T, ROWS, COLS>(false);
	}

	static TMatrixMxN<T, ROWS, COLS> Identity()
	{
		return TMatrixMxN<T, ROWS, COLS>(true);
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
using TMatrixN = TMatrixMxN<T, SIZE, SIZE>;

template<int ROWS, int COLS>
using MatrixMxN = TMatrixMxN<float, ROWS, COLS>;

template<int SIZE>
using MatrixN = TMatrixN<float, SIZE>;

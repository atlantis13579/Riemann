#pragma once

#include<string.h>

template<typename T, int DIM>
class TVectorN
{
public:
	TVectorN()
	{
	}
	
	explicit TVectorN(bool Zero)
	{
		if (Zero)
		{
			LoadZero();
		}
	}

	TVectorN(const std::initializer_list<T>& v)
	{
		memset(mData, 0, sizeof(T) * DIM);
		int i = 0;
		for (const T &vv : v)
		{
			mData[i] = vv;
			if (i++ >= DIM)
				break;
		}
	}

	TVectorN(const TVectorN<T, DIM>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * DIM);
	}

	inline const T* Data() const
	{
		return mData;
	}

	inline T* Data()
	{
		return mData;
	}

	inline const T& operator[](int i) const
	{
		return mData[i];
	}

	inline T& operator[](int i)
	{
		return mData[i];
	}

	inline TVectorN<T, DIM>& operator=(const TVectorN<T, DIM>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * DIM);
		return *this;
	}

	inline TVectorN<T, DIM> operator+(const TVectorN<T, DIM>& v) const
	{
		TVectorN<T, DIM> Ret(*this);
		for (int i = 0; i < DIM; ++i)
		{
			Ret[i] += v[i];
		}
		return Ret;
	}

	inline TVectorN<T, DIM> operator-(const TVectorN<T, DIM>& v) const
	{
		TVectorN<T, DIM> Ret(*this);
		for (int i = 0; i < DIM; ++i)
		{
			Ret[i] -= v[i];
		}
		return Ret;
	}

	inline TVectorN<T, DIM> operator*(const TVectorN<T, DIM>& v) const
	{
		TVectorN<T, DIM> Ret(*this);
		for (int i = 0; i < DIM; ++i)
		{
			Ret[i] *= v[i];
		}
		return Ret;
	}

	inline TVectorN<T, DIM> operator*(T k) const
	{
		TVectorN<T, DIM> Ret(*this);
		for (int i = 0; i < DIM; ++i)
		{
			Ret[i] *= k;
		}
		return Ret;
	}

	inline TVectorN<T, DIM> operator/(T k) const
	{
		return operator*((T)1 / k);
	}

	inline TVectorN<T, DIM> operator-()
	{
		TVectorN<T, DIM> Ret(*this);
		for (int i = 0; i < DIM; ++i)
		{
			Ret[i] = -Ret[i];
		}
		return Ret;
	}

	inline void	 operator+= (const TVectorN<T, DIM>& v)
	{
		for (int i = 0; i < DIM; ++i)
		{
			mData[i] += v[i];
		}
	}

	inline void	 operator-= (const TVectorN<T, DIM>& v)
	{
		for (int i = 0; i < DIM; ++i)
		{
			mData[i] -= v[i];
		}
	}

	inline void	 operator*= (const TVectorN<T, DIM>& v)
	{
		for (int i = 0; i < DIM; ++i)
		{
			mData[i] *= v[i];
		}
	}

	inline void	 operator/= (const TVectorN<T, DIM>& v)
	{
		for (int i = 0; i < DIM; ++i)
		{
			mData[i] /= v[i];
		}
	}

	inline void	 operator*= (T k)
	{
		for (int i = 0; i < DIM; ++i)
		{
			mData[i] *= k;
		}
	}

	inline void	 operator/= (T k)
	{
		operator*= ((T)1 / k);
	}

	inline T	Dot(const TVectorN<T, DIM>& v) const
	{
		T dp = (T)0;
		for (int i = 0; i < DIM; ++i)
		{
			dp += mData[i] * v.mData[i];
		}
		return dp;
	}
	
	inline T	Length() const
	{
		return L2Norm();
	}

	inline T	SquareLength() const
	{
		return Dot(*this);
	}

	T					L1Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < DIM; ++i)
		{
			sum += std::abs(mData[i]);
		}
		return sum;
	}

	T					L2Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < DIM; ++i)
		{
			sum += mData[i] * mData[i];
		}
		return std::sqrt(sum);
	}

	T					LpNorm(int p) const
	{
		T sum = (T)0;
		for (int i = 1; i < DIM; ++i)
		{
			sum += std::pow(mData[i], p);
		}
		return std::pow(sum, (T)1 / p);
	}

	T					LInfinityNorm() const
	{
		T maxVal = mData[0];
		for (int i = 1; i < DIM; ++i)
		{
			maxVal = std::max(maxVal, mData[i]);
		}
		return maxVal;
	}

	void		LoadZero()
	{
		memset(mData, 0, sizeof(T) * DIM);
	}

	static TVectorN<T, DIM> Zero()
	{
		return TVectorN<T, DIM>(true);
	}

private:
	T	mData[DIM];
};

template<int DIM>
using VectorN = TVectorN<float, DIM>;

template<typename T>
using TVector8 = TVectorN<T, 8>;

template<typename T>
using TVector12 = TVectorN<T, 12>;

template<typename T>
using TVector16 = TVectorN<T, 16>;

typedef TVectorN<float, 8>	Vector8;
typedef TVectorN<float, 12>	Vector12;
typedef TVectorN<float, 16>	Vector16;

static_assert(sizeof(Vector8) == 32, "sizeof Vector8 is not valid");
static_assert(sizeof(Vector12) == 48, "sizeof Vector12 is not valid");
static_assert(sizeof(Vector16) == 64, "sizeof Vector16 is not valid");

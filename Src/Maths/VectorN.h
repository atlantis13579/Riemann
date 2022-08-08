#pragma once

#include<string.h>

template<typename T, int SIZE>
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
		memset(mData, 0, sizeof(T) * SIZE);
		int i = 0;
		for (const T &vv : v)
		{
			mData[i] = vv;
			if (i++ >= SIZE)
				break;
		}
	}

	TVectorN(const TVectorN<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE);
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

	TVectorN<T, SIZE>& operator=(const TVectorN<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE);
		return *this;
	}

	TVectorN<T, SIZE> operator+(const TVectorN<T, SIZE>& v) const
	{
		TVectorN<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] += v[i];
		}
		return Ret;
	}

	TVectorN<T, SIZE> operator-(const TVectorN<T, SIZE>& v) const
	{
		TVectorN<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] -= v[i];
		}
		return Ret;
	}

	TVectorN<T, SIZE> operator*(const TVectorN<T, SIZE>& v) const
	{
		TVectorN<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] *= v[i];
		}
		return Ret;
	}

	TVectorN<T, SIZE> operator*(T k) const
	{
		TVectorN<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] *= k;
		}
		return Ret;
	}

	TVectorN<T, SIZE> operator/(T k) const
	{
		return operator*((T)1 / k);
	}

	TVectorN<T, SIZE> operator-()
	{
		TVectorN<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] = -Ret[i];
		}
		return Ret;
	}

	void	 operator+= (const TVectorN<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] += v[i];
		}
	}

	void	 operator-= (const TVectorN<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] -= v[i];
		}
	}

	void	 operator*= (const TVectorN<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] *= v[i];
		}
	}

	void	 operator/= (const TVectorN<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] /= v[i];
		}
	}

	void	 operator*= (T k)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] *= k;
		}
	}

	void	 operator/= (T k)
	{
		operator*= ((T)1 / k);
	}

	T			Dot(const TVectorN<T, SIZE>& v) const
	{
		T dp = (T)0;
		for (int i = 0; i < SIZE; ++i)
		{
			dp += mData[i] * v.mData[i];
		}
		return dp;
	}

	T			SquaredNorm() const
	{
		return Dot(*this);
	}

	void		LoadZero()
	{
		memset(mData, 0, sizeof(T) * SIZE);
	}

	static TVectorN<T, SIZE> Zero()
	{
		return TVectorN<T, SIZE>(true);
	}

private:
	T	mData[SIZE];
};

template<int SIZE>
using VectorNd = TVectorN<float, SIZE>;

typedef TVectorN<float, 8>	Vector8;
typedef TVectorN<float, 16>	Vector16;


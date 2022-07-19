#pragma once

#include<string.h>

template<typename T, int SIZE>
class TVectorNd
{
public:
	TVectorNd()
	{
	}
	
	TVectorNd(bool Zero)
	{
		if (Zero)
		{
			LoadZero();
		}
	}

	TVectorNd(const std::initializer_list<T>& v)
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

	TVectorNd(const TVectorNd<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE);
	}

	inline constexpr const T* operator*() const
	{
		return mData;
	}

	inline constexpr T* operator*()
	{
		return mData;
	}

	inline constexpr const T& operator[](int i) const
	{
		return mData[i];
	}

	inline constexpr T& operator[](int i)
	{
		return mData[i];
	}

	TVectorNd<T, SIZE>& operator=(const TVectorNd<T, SIZE>& v)
	{
		memcpy(mData, v.mData, sizeof(T) * SIZE);
		return *this;
	}

	TVectorNd<T, SIZE> operator+(const TVectorNd<T, SIZE>& v) const
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] += v[i];
		}
		return Ret;
	}

	TVectorNd<T, SIZE> operator-(const TVectorNd<T, SIZE>& v) const
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] -= v[i];
		}
		return Ret;
	}

	TVectorNd<T, SIZE> operator*(const TVectorNd<T, SIZE>& v) const
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] *= v[i];
		}
		return Ret;
	}

	TVectorNd<T, SIZE> operator+(T k) const
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] += k;
		}
		return Ret;
	}

	TVectorNd<T, SIZE> operator-(T k) const
	{
		return operator+(-k);
	}

	TVectorNd<T, SIZE> operator*(T k) const
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] *= k;
		}
		return Ret;
	}

	TVectorNd<T, SIZE> operator/(T k) const
	{
		return operator*((T)1 / k);
	}

	TVectorNd<T, SIZE> operator-()
	{
		TVectorNd<T, SIZE> Ret(*this);
		for (int i = 0; i < SIZE; ++i)
		{
			Ret[i] = -Ret[i];
		}
		return Ret;
	}

	void	 operator+= (const TVectorNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] += v[i];
		}
	}

	void	 operator-= (const TVectorNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] -= v[i];
		}
	}

	void	 operator*= (const TVectorNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] *= v[i];
		}
	}

	void	 operator/= (const TVectorNd<T, SIZE>& v)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] /= v[i];
		}
	}

	void	 operator+= (T k)
	{
		for (int i = 0; i < SIZE; ++i)
		{
			mData[i] += k;
		}
	}

	void	 operator-= (T k)
	{
		operator+=(-k);
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

	T			Dot(const TVectorNd<T, SIZE>& v) const
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

	static TVectorNd<T, SIZE> Zero()
	{
		static TVectorNd<T, SIZE> s_Zero(true);
		return s_Zero;
	}

private:
	T	mData[SIZE];
};

template<int SIZE>
using VectorNd = TVectorNd<float, SIZE>;

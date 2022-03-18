#pragma once

#include <memory>
#include <vector>

template<typename T>
class TDenseVector
{
public:
	TDenseVector() : mSize(0), pData(nullptr)
	{
	}

	TDenseVector(int nSize) : mSize(nSize), mData(nSize)
	{
		pData = &mData[0];
	}

	TDenseVector(int nSize, T Val) : mSize(nSize), mData(nSize, Val)
	{
		pData = &mData[0];
	}

	TDenseVector(const TDenseVector<T>&& v) : mSize(v.mSize)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	TDenseVector(T* p, int nSize) : mSize(nSize)
	{
		pData = p;
	}

	int			GetSize() const
	{
		return mSize;
	}

	T* GetData()
	{
		return pData;
	}

	const T* GetData() const
	{
		return pData;
	}

	bool		HoldsMemory() const
	{
		return mData.size() > 0;
	}

	inline const T& operator[](int i) const
	{
		return pData[i];
	}

	inline T& operator[](int i)
	{
		return pData[i];
	}

	TDenseVector<T>& operator=(const TDenseVector<T>& v)
	{
		mSize = v.mSize;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TDenseVector<T> operator+(const TDenseVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TDenseVector<T>();
		}

		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] += v[i];
		}
		return std::move(Ret);
	}

	TDenseVector<T> operator-(const TDenseVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TDenseVector<T>();
		}

		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] -= v[i];
		}
		return std::move(Ret);
	}

	TDenseVector<T> operator*(const TDenseVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TDenseVector<T>();
		}

		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] *= v[i];
		}
		return std::move(Ret);
	}

	TDenseVector<T> operator+(T k) const
	{
		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] += k;
		}
		return std::move(Ret);
	}

	TDenseVector<T> operator-(T k) const
	{
		return operator+(-k);
	}

	TDenseVector<T> operator*(T k) const
	{
		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] *= k;
		}
		return std::move(Ret);
	}

	TDenseVector<T> operator/(T k) const
	{
		return operator*((T)1 / k);
	}

	TDenseVector<T> operator-()
	{
		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] = -Ret[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TDenseVector<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] += v[i];
		}
	}

	void	 operator-= (const TDenseVector<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] -= v[i];
		}
	}

	void	 operator*= (const TDenseVector<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] *= v[i];
		}
	}

	void	 operator/= (const TDenseVector<T>& v)
	{
		if (GetSize() != v.GetSize())
		{
			return;
		}

		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] /= v[i];
		}
	}

	void	 operator+= (T k)
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] += k;
		}
	}

	void	 operator-= (T k)
	{
		operator+=(-k);
	}

	void	 operator*= (T k)
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] *= k;
		}
	}

	void	 operator/= (T k)
	{
		operator*= ((T)1 / k);
	}

	T			Dot(const TDenseVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return (T)0;
		}

		T dp = (T)0;
		for (int i = 0; i < mSize; ++i)
		{
			dp += pData[i] * v.pData[i];
		}
		return dp;
	}

	void		LoadZero()
	{
		memset(pData, 0, sizeof(T) * mSize);
	}

private:
	int				mSize;
	std::vector<T>	mData;
	T* pData;
};

using DenseVector = TDenseVector<float>;

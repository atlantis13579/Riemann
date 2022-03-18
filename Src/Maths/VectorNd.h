#pragma once

#include <vector>

template<typename T>
class TVector
{
public:
	TVector() : mSize(0), pData(nullptr)
	{
	}

	TVector(int nSize) : mSize(nSize), mData(nSize)
	{
		pData = &mData[0];
	}

	TVector(int nSize, T Val) : mSize(nSize), mData(nSize, Val)
	{
		pData = &mData[0];
	}

	TVector(const TVector<T>&& v) : mSize(v.mSize)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	TVector(T* p, int nSize) : mSize(nSize)
	{
		pData = p;
	}

	~TVector()
	{
		mData.clear();
		pData = nullptr;
	}

	int			GetSize() const
	{
		return mSize;
	}

	T*			GetData()
	{
		return pData;
	}

	const T*	GetData() const
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

	TVector<T>& operator=(const TVector<T>& v)
	{
		mSize = v.mSize;
		mData = v.mData;
		pData = mData.size() > 0 ? &mData[0] : v.pData;
		return *this;
	}

	TVector<T> operator+(const TVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TVector<T>();
		}

		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] += v[i];
		}
		return std::move(Ret);
	}

	TVector<T> operator-(const TVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TVector<T>();
		}

		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] -= v[i];
		}
		return std::move(Ret);
	}

	TVector<T> operator*(const TVector<T>& v) const
	{
		if (GetSize() != v.GetSize())
		{
			return TVector<T>();
		}

		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] *= v[i];
		}
		return std::move(Ret);
	}

	TVector<T> operator+(T k) const
	{
		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] += k;
		}
		return std::move(Ret);
	}

	TVector<T> operator-(T k) const
	{
		return operator+(-k);
	}

	TVector<T> operator*(T k) const
	{
		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] *= k;
		}
		return std::move(Ret);
	}

	TVector<T> operator/(T k) const
	{
		return operator*((T)1 / k);
	}

	TVector<T> operator-()
	{
		TVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] = -Ret[i];
		}
		return std::move(Ret);
	}

	void	 operator+= (const TVector<T>& v)
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

	void	 operator-= (const TVector<T>& v)
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

	void	 operator*= (const TVector<T>& v)
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

	void	 operator/= (const TVector<T>& v)
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

	T			Dot(const TVector<T>& v) const
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

	T			SquareNorm() const
	{
		return Dot(*this);
	}

	void		LoadZero()
	{
		memset(pData, 0, sizeof(T) * mSize);
	}

private:
	int				mSize;
	std::vector<T>	mData;
	T*				pData;
};

using VectorN = TVector<float>;

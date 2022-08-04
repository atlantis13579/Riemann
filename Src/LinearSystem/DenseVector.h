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

	TDenseVector(const TDenseVector<T>& v) : mSize(v.mSize)
	{
		mData = v.mData;
		pData = &mData[0];
	}

	TDenseVector(TDenseVector<T>&& v) : mSize(v.mSize)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	TDenseVector(T* p, int nSize) : mSize(nSize)
	{
		pData = p;
	}

	inline int		GetSize() const
	{
		return mSize;
	}

	inline T*		GetData()
	{
		return pData;
	}

	inline const T*	GetData() const
	{
		return pData;
	}

	inline int		GetDataSize() const
	{
		return mSize * sizeof(T);
	}

	bool			HoldsMemory() const
	{
		return mData.size() > 0;
	}

	inline const T& operator[](int i) const
	{
		return pData[i];
	}

	inline T&		operator[](int i)
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

	TDenseVector<T>& operator=(TDenseVector<T>&& v)
	{
		mSize = v.mSize;
		mData = std::move(v.mData);
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
			Ret[i] += v.pData[i];
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
			Ret[i] -= v.pData[i];
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
			Ret[i] *= v.pData[i];
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
			pData[i] += v.pData[i];
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
			pData[i] -= v.pData[i];
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
			pData[i] *= v.pData[i];
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
			pData[i] /= v.pData[i];
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

	bool	operator>(T val) const
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] > val))
				return false;
		}
		return true;
	}

	bool	operator>=(T val) const
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] >= val))
				return false;
		}
		return true;
	}

	bool	operator<(T val) const
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] < val))
				return false;
		}
		return true;
	}

	bool	operator<=(T val) const
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] <= val))
				return false;
		}
		return true;
	}

	bool	operator>(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] > rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool	operator>=(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] >= rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool	operator<(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] < rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool	operator<=(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] <= rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool	operator==(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] == rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool	operator!=(const TDenseVector<T>& rhs) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!(pData[i] != rhs.pData[i]))
				return false;
		}
		return true;
	}

	bool		FuzzyEqual(const TDenseVector<T>& rhs, T Eps) const
	{
		if (GetSize() != rhs.GetSize())
			return false;
		for (int i = 0; i < GetSize(); ++i)
		{
			if (std::abs(pData[i] - rhs.pData[i]) > Eps)
				return false;
		}
		return true;
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

	bool		IsZero(T Eps) const
	{
		T dp = (T)0;
		for (int i = 0; i < mSize; ++i)
		{
			dp += pData[i] * pData[i];
		}
		return dp < Eps;
	}

private:
	int				mSize;
	std::vector<T>	mData;
	T* 				pData;
};

using DenseVector = TDenseVector<float>;

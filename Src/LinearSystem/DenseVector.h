#pragma once

#include <cmath>
#include <memory>
#include <vector>

namespace Maths
{
namespace LinearAlgebra
{

template<typename T>
class TDenseVector
{
public:
	explicit TDenseVector() : mSize(0), pData(nullptr)
	{
	}

	explicit TDenseVector(int nSize) : mSize(nSize), mData(nSize)
	{
		pData = &mData[0];
	}

	explicit TDenseVector(int nSize, T Val) : mSize(nSize), mData(nSize, Val)
	{
		pData = &mData[0];
	}

	explicit TDenseVector(const TDenseVector<T>& v) : mSize(v.mSize)
	{
		if (!v.mData.empty())
			mData = v.mData;
		pData = &mData[0];
	}

	TDenseVector(TDenseVector<T>&& v) : mSize(v.mSize)
	{
		mData = std::move(v.mData);
		pData = mData.size() > 0 ? &mData[0] : v.pData;
	}

	explicit TDenseVector(T* p, int nSize) : mSize(nSize)
	{
		pData = p;
	}
	
	void SetSize(int nSize)
	{
		mSize = nSize;
		mData.resize(nSize);
		pData = &mData[0];
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
		return mData.size() > 0 || pData == nullptr;
	}

	void			Assign(const TDenseVector<T>& rhs)
	{
		if (!HoldsMemory())
		{
			memcpy(pData, rhs.GetData(), mSize * sizeof(T));
		}
		else
		{
			SetSize(rhs.GetSize());
			memcpy(pData, rhs.GetData(), mSize * sizeof(T));
		}
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

	TDenseVector<T> operator-() const
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
	
	TDenseVector<T> Inverse() const
	{
		TDenseVector<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] = (T)1 / Ret[i];
		}
		return std::move(Ret);
	}

	bool		FuzzyEqual(const TDenseVector<T>& rhs, const T Eps = (T)1e-6f) const
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

	TDenseVector<T> Min(const TDenseVector<T>& v) const
	{
		TDenseVector<T> Ret(mSize);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] = std::min(pData[i], v.pData[i]);
		}
		return Ret;
	}

	TDenseVector<T> Max(const TDenseVector<T>& v) const
	{
		TDenseVector<T> Ret(mSize);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret[i] = std::max(pData[i], v.pData[i]);
		}
		return Ret;
	}

	static TDenseVector<T> Lerp(const TDenseVector<T>& start, const TDenseVector<T>& end, float t)
	{
		TDenseVector<T> Ret(start.mSize);
        for (int i = 0; i < start.mSize; ++i)
		{
			Ret[i] = start[i] * (1.0f - t) + end[i] * t;
		}
		return Ret;
	}

	T			L1Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < mSize; ++i)
		{
			sum += std::abs(pData[i]);
		}
		return sum;
	}

	T			L2Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < mSize; ++i)
		{
			sum += pData[i] * pData[i];
		}
		return std::sqrt(sum);
	}

	T			LpNorm(int p) const
	{
		T sum = (T)0;
		for (int i = 1; i < mSize; ++i)
		{
			sum += std::pow(pData[i], p);
		}
		return std::pow(sum, (T)1 / p);
	}

	T			LInfinityNorm() const
	{
		T maxVal = pData[0];
		for (int i = 1; i < mSize; ++i)
		{
			maxVal = std::max(maxVal, pData[i]);
		}
		return maxVal;
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

template <typename T>
inline TDenseVector<T> operator* (T s, const TDenseVector<T>& v)
{
	return v * s;
}

using DenseVector = TDenseVector<float>;

}	// namespace LinearAlgebra
}	// namespace Maths

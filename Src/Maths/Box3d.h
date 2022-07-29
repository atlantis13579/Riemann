
#pragma once

#include <float.h>
#include <algorithm>
#include <vector>

#include "Vector3.h"
#include "Matrix4.h"
#include "Quaternion.h"

#define _USE_MATH_DEFINES

#undef max
#undef min 

template <typename T>
class TAABB3
{
public:
	TVector3<T> mMin;
	TVector3<T> mMax;

public:
	TAABB3<T>() { }
	TAABB3<T>(const TVector3<T>& InMin, const TVector3<T>& InMax)
		: mMin(InMin)
		, mMax(InMax)
	{ }

	TAABB3<T>(T InMin, T InMax)
		: mMin(InMin, InMin, InMin)
		, mMax(InMax, InMax, InMax)
	{ }

	TAABB3<T>(const TVector3<T>* v, int Num)
	{
		mMin = mMax = v[0];
		for (int i = 1; i < Num; ++i)
		{
			this->Encapsulate(v[i]);
		}
	}

	TAABB3<T>(const TVector3<T>& v0, const TVector3<T>& v1, const TVector3<T>& v2)
	{
		mMin = mMax = v0;
		this->Encapsulate(v1, v2);
	}

	TAABB3<T>(const std::vector<TVector3<T>> &v)
	{
		mMin = mMax = v[0];
		for (size_t i = 1; i < v.size(); ++i)
		{

			this->Encapsulate(v[i]);
		}
	}

public:
	bool operator==(const TAABB3<T>& Other) const
	{
		return (mMin == Other.mMin) && (mMax == Other.mMax);
	}
	
	TAABB3<T>& operator=(const TAABB3<T>& rhs)
	{
		mMin = rhs.mMin;
		mMax = rhs.mMax;
		return *this;
	}

	TAABB3<T>& operator+=(const TVector3<T>& rhs)
	{
		mMin += rhs;
		mMax += rhs;
		return *this;
	}

	TAABB3<T> operator+(const TVector3<T>& rhs) const
	{
		return TAABB3<T>(*this) += rhs;
	}
	
	void SetEmpty()
	{
		*this = TAABB3<T>::Empty();
	}

	TAABB3<T>& Thicken(float Thickness)
	{
		mMin -= Thickness;
		mMax += Thickness;
		return *this;
	}
	
	TAABB3<T>& Encapsulate(const TVector3<T>& a)
	{
		mMin = mMin.Min(a);
		mMax = mMax.Max(a);
		return *this;
	}
	
	TAABB3<T>& Encapsulate(const TVector3<T>& a, const TVector3<T>& b)
	{
		mMin = mMin.Min(a);
		mMax = mMax.Max(a);
		mMin = mMin.Min(b);
		mMax = mMax.Max(b);
		return *this;
	}
	
	TAABB3<T>& Encapsulate(const TVector3<T>& a, const TVector3<T>& b, const TVector3<T>& c)
	{
		mMin = mMin.Min(a);
		mMax = mMax.Max(a);
		mMin = mMin.Min(b);
		mMax = mMax.Max(b);
		mMin = mMin.Min(c);
		mMax = mMax.Max(c);
		return *this;
	}

	TAABB3<T>& Encapsulate(const TAABB3<T>& aabb)
	{
		mMin = mMin.Min(aabb.mMin);
		mMax = mMax.Max(aabb.mMax);
		return *this;
	}
	
	TAABB3<T>& Encapsulate(const TAABB3<T>& aabb1, const TAABB3<T>& aabb2)
	{
		mMin = mMin.Min(aabb1.mMin);
		mMax = mMax.Max(aabb1.mMax);
		mMin = mMin.Min(aabb2.mMin);
		mMax = mMax.Max(aabb2.mMax);
		return *this;
	}

	TVector3<T> GetCenter() const
	{
		return TVector3<T>((mMin + mMax) * 0.5f);
	}

	TVector3<T> GetExtent() const
	{
		return (mMax - mMin) * 0.5f;
	}

	TVector3<T> GetSize() const
	{
		return mMax - mMin;
	}
	
	T GetVolume() const
	{
		return (mMax.x - mMin.x) * (mMax.y - mMin.y) * (mMax.z - mMin.z);
	}

	inline T GetLengthX() const
	{
		return mMax.x - mMin.x;
	}

	inline T GetLengthY() const
	{
		return mMax.y - mMin.y;
	}

	inline T GetLengthZ() const
	{
		return mMax.z - mMin.z;
	}

	void GetCenterAndExtent(TVector3<T>* center, TVector3<T>* Extent) const
	{
		*center = GetCenter();
		*Extent = GetExtent();
	}

	void BuildFromCenterAndExtent(const TVector3<T>& Center, const TVector3<T>& Extent)
	{
		mMin = Center - Extent;
		mMax = Center + Extent;
	}

	inline bool Intersect(const TAABB3<T>& rhs) const
	{
		if (mMin.x > rhs.mMax.x || rhs.mMin.x > mMax.x)
		{
			return false;
		}

		if (mMin.y > rhs.mMax.y || rhs.mMin.y > mMax.y)
		{
			return false;
		}

		if (mMin.z > rhs.mMax.z || rhs.mMin.z > mMax.z)
		{
			return false;
		}

		return true;
	}

	inline bool Intersect(const TVector3<T>& Bmin, const TVector3<T>& Bmax) const
	{
		if (mMin.x > Bmax.x || Bmin.x > mMax.x)
		{
			return false;
		}

		if (mMin.y > Bmax.y || Bmin.y > mMax.y)
		{
			return false;
		}

		if (mMin.z > Bmax.z || Bmin.z > mMax.z)
		{
			return false;
		}

		return true;
	}

	inline bool IsInside(const TVector3<T>& rhs) const
	{
		return ((rhs.x >= mMin.x) && (rhs.x <= mMax.x) && (rhs.y >= mMin.y) && (rhs.y <= mMax.y) && (rhs.z >= mMin.z) && (rhs.z <= mMax.z));
	}

	inline bool IsInside(const TAABB3<T>& rhs) const
	{
		return ((rhs.mMin.x <= mMin.x) && (rhs.mMax.x >= mMax.x) && (rhs.mMin.y <= mMin.y) && (rhs.mMax.y >= mMax.y) && (rhs.mMin.z <= mMin.z) && (rhs.mMax.z >= mMax.z));
	}

	bool		GetIntersection(const TVector3<T>& Bmin, const TVector3<T>& Bmax, TAABB3<T>& OutBox) const
	{
		if (!Intersect(Bmin, Bmax))
		{
			return false;
		}

		OutBox.mMin.x = std::max(mMin.x, Bmin.x);
		OutBox.mMax.x = std::min(mMax.x, Bmax.x);

		OutBox.mMin.y = std::max(mMin.y, Bmin.y);
		OutBox.mMax.y = std::min(mMax.y, Bmax.y);

		OutBox.mMin.z = std::max(mMin.z, Bmin.z);
		OutBox.mMax.z = std::min(mMax.z, Bmax.z);

		return true;
	}

	bool		GetIntersection(const TAABB3<T>& Box, TAABB3<T> &OutBox) const
	{
		return GetIntersection(Box.mMin, Box.mMax, OutBox);
	}

	TAABB3<T>	Transform(const Matrix4& M) const
	{
		TAABB3<T> Box;

		TVector3<T> Center = GetCenter();
		TVector3<T> Extent = GetExtent();

		TVector3<T> m0 = M.Column(0).xyz();
		TVector3<T> m1 = M.Column(1).xyz();
		TVector3<T> m2 = M.Column(2).xyz();
		TVector3<T> m3 = M.Column(3).xyz();

		TVector3<T> NewCenter = m0 * Center.x + m1 * Center.y + m2 * Center.z + m3;
		TVector3<T> NewExtent = (m0 * Extent.x).Abs() + (m1 * Extent.y).Abs() + (m2 * Extent.z).Abs();

		Box.BuildFromCenterAndExtent(NewCenter, NewExtent);

		return Box;
	}

	TAABB3<T>	Transform(const Vector3& t, const Quaternion& q) const
	{
		Matrix4 mat = q.ToRotationMatrix4();
		mat[0][3] += t.x;
		mat[1][3] += t.y;
		mat[2][3] += t.z;
		return Transform(mat);
	}
	
	static void GetVertices(const TVector3<T>& Bmin, const TVector3<T>& Bmax, TVector3<T> *v)
	{
		//   6   7
		// 4   5
		// 	 2   3		
		// 0   1
		v[0] = TVector3<T>(Bmin.x, Bmin.y, Bmin.z);
		v[1] = TVector3<T>(Bmax.x, Bmin.y, Bmin.z);
		v[2] = TVector3<T>(Bmin.x, Bmax.y, Bmin.z);
		v[3] = TVector3<T>(Bmax.x, Bmax.y, Bmin.z);
		v[4] = TVector3<T>(Bmin.x, Bmin.y, Bmax.z);
		v[5] = TVector3<T>(Bmax.x, Bmin.y, Bmax.z);
		v[6] = TVector3<T>(Bmin.x, Bmax.y, Bmax.z);
		v[7] = TVector3<T>(Bmax.x, Bmax.y, Bmax.z);
	}

	static TAABB3<T> Unit()
	{
		return TAABB3<T>(-TVector3<T>::One(), TVector3<T>::One());
	}

	static TAABB3<T> One()
	{
		return TAABB3<T>(TVector3<T>::Zero(), TVector3<T>::One());
	}

	static TAABB3<T> Empty()
	{
		return TAABB3<T>(FLT_MAX * 0.25f, -FLT_MAX * 0.25f);
	}
};

template <typename T>
class TCE3
{
public:
	TVector3<T> Center;
	TVector3<T> Extent;

	TAABB3<T> GetAABB() const
	{
		return TAABB3<T>(Center - Extent, Center + Extent);
	}

	static TCE3<T> Empty()
	{
		TCE3<T> Ret;
		Ret.Center = TVector3<T>::Zero();
		Ret.Extent = TVector3<T>(FLT_MAX * 0.25f, FLT_MAX * 0.25f, FLT_MAX * 0.25f);
		return Ret;
	}
};

typedef TAABB3<float> Box3d;

static_assert(sizeof(Box3d) == 24, "sizeof Box3d is not valid");

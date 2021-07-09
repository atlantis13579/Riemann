
#pragma once

#include <algorithm>

#include "Vector3d.h"
#include "Matrix4d.h"

template <typename T>
class TBoundingBox3
{
public:
	TVector3<T> Min;
	TVector3<T> Max;

public:
	TBoundingBox3<T>() { }
	TBoundingBox3<T>(const TVector3<T>& InMin, const TVector3<T>& InMax)
		: Min(InMin)
		, Max(InMax)
	{ }

	TBoundingBox3<T>(const TVector3<T>* v, int Num)
	{
		for (int i = 0; i < Num; ++i)
		{
			*this += v[i];
		}
	}

public:
	bool operator==(const TBoundingBox3<T>& Other) const
	{
		return (Min == Other.Min) && (Max == Other.Max);
	}

	TBoundingBox3<T>& operator+=(const TVector3<T>& rhs)
	{
		Min += rhs;
		Max += rhs;
		return *this;
	}

	TBoundingBox3<T> operator+(const TVector3<T>& rhs) const
	{
		return TBoundingBox3<T>(*this) += rhs;
	}
	
	TBoundingBox3<T>& Extend(const TVector3<T>& rhs)
	{
		Min.x = std::min(Min.x, rhs.x);
		Min.y = std::min(Min.y, rhs.y);
		Min.z = std::min(Min.z, rhs.z);

		Max.x = std::max(Max.x, rhs.x);
		Max.y = std::max(Max.y, rhs.y);
		Max.z = std::max(Max.z, rhs.z);

		return *this;
	}

	TBoundingBox3<T>& Extend(const TBoundingBox3<T>& rhs)
	{
		Min.x = std::min(Min.x, rhs.Min.x);
		Min.y = std::min(Min.y, rhs.Min.y);
		Min.z = std::min(Min.z, rhs.Min.z);
		
		Max.x = std::max(Max.x, rhs.Max.x);
		Max.y = std::max(Max.y, rhs.Max.y);
		Max.z = std::max(Max.z, rhs.Max.z);

		return *this;
	}

	TVector3<T> GetCenter() const
	{
		return TVector3<T>((Min + Max) * 0.5f);
	}

	TVector3<T> GetExtent() const
	{
		return (Max - Min) * 0.5f;
	}

	TVector3<T> GetSize() const
	{
		return Max - Min;
	}

	T GetSizeX() const
	{
		return Max.x - Min.x;
	}

	T GetSizeY() const
	{
		return Max.y - Min.y;
	}

	T GetSizeZ() const
	{
		return Max.z - Min.z;
	}

	void GetCenterAndExtent(TVector3<T>* center, TVector3<T>* Extent) const
	{
		*center = GetCenter();
		*Extent = GetExtent();
	}

	void BuildFromCenterAndExtent(const TVector3<T>& Center, const TVector3<T>& Extent)
	{
		Min = Center - Extent;
		Max = Center + Extent;
	}

	bool Intersect(const TBoundingBox3<T>& rhs) const
	{
		if (Min.x > rhs.Max.x || rhs.Min.x > Max.x)
		{
			return false;
		}

		if (Min.y > rhs.Max.y || rhs.Min.y > Max.y)
		{
			return false;
		}

		if (Min.z > rhs.Max.z || rhs.Min.z > Max.z)
		{
			return false;
		}

		return true;
	}

	bool IsInside(const TVector3<T>& rhs) const
	{
		return ((rhs.x > Min.x) && (rhs.x < Max.x) && (rhs.y > Min.y) && (rhs.y < Max.y) && (rhs.z > Min.z) && (rhs.z < Max.z));
	}

	TBoundingBox3<T> Overlap(const TBoundingBox3<T>& rhs) const
	{
		if (!Intersect(rhs))
		{
			TBoundingBox3<T> Zero(TVector3<T>::Zero(), TVector3<T>::Zero());
			return Zero;
		}

		TBoundingBox3<T> Box;

		Box.Min.x = std::max(Min.x, rhs.Min.x);
		Box.Max.x = std::min(Max.x, rhs.Max.x);

		Box.Min.y = std::max(Min.y, rhs.Min.y);
		Box.Max.y = std::min(Max.y, rhs.Max.y);

		Box.Min.z = std::max(Min.z, rhs.Min.z);
		Box.Max.z = std::min(Max.z, rhs.Max.z);

		return Box;
	}

	TBoundingBox3<T> Transform(const Matrix4d& M) const
	{
		TBoundingBox3<T> Box;

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
	
	static void GetVertices(const TVector3<T>& Bmin, const TVector3<T>& Bmax, TVector3<T> v[8])
	{
		v[0] = TVector3<T>(Bmin.x, Bmin.y, Bmin.z);
		v[1] = TVector3<T>(Bmax.x, Bmin.y, Bmin.z);
		v[2] = TVector3<T>(Bmin.x, Bmax.y, Bmin.z);
		v[3] = TVector3<T>(Bmax.x, Bmax.y, Bmin.z);
		v[4] = TVector3<T>(Bmin.x, Bmin.y, Bmax.z);
		v[5] = TVector3<T>(Bmax.x, Bmin.y, Bmax.z);
		v[6] = TVector3<T>(Bmin.x, Bmax.y, Bmax.z);
		v[7] = TVector3<T>(Bmax.x, Bmax.y, Bmax.z);
	}
};

typedef TBoundingBox3<float> BoundingBox3d;

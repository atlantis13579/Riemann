
#pragma once

#include <algorithm>
#include <vector>

#include "Vector3d.h"
#include "Matrix4d.h"

class BoundingBox3d
{
public:
	Vector3d Min;
	Vector3d Max;

public:
	BoundingBox3d() { }
	BoundingBox3d(const Vector3d& InMin, const Vector3d& InMax)
		: Min(InMin)
		, Max(InMax)
	{ }

	BoundingBox3d(const Vector3d* v, int Num)
	{
		for (int i = 0; i < Num; ++i)
		{
			*this += v[i];
		}
	}

	BoundingBox3d(const std::vector<Vector3d>& v)
	{
		for (size_t i = 0; i < v.size(); ++i)
		{
			*this += v[i];
		}
	}

public:
	bool operator==(const BoundingBox3d& Other) const
	{
		return (Min == Other.Min) && (Max == Other.Max);
	}

	BoundingBox3d& operator+=(const Vector3d& rhs)
	{
		Min.x = std::min(Min.x, rhs.x);
		Min.y = std::min(Min.y, rhs.y);
		Min.z = std::min(Min.z, rhs.z);

		Max.x = std::max(Max.x, rhs.x);
		Max.y = std::max(Max.y, rhs.y);
		Max.z = std::max(Max.z, rhs.z);
	}

	BoundingBox3d operator+(const BoundingBox3d& rhs) const
	{
		return BoundingBox3d(*this) += rhs;
	}

	BoundingBox3d& operator+=(const BoundingBox3d& rhs)
	{
		Min.x = std::min(Min.x, rhs.Min.x);
		Min.y = std::min(Min.y, rhs.Min.y);
		Min.z = std::min(Min.z, rhs.Min.z);
		
		Max.x = std::max(Max.x, rhs.Max.x);
		Max.y = std::max(Max.y, rhs.Max.y);
		Max.z = std::max(Max.z, rhs.Max.z);
	}

	Vector3d GetCenter() const
	{
		return Vector3d((Min + Max) * 0.5f);
	}

	Vector3d GetExtent() const
	{
		return (Max - Min) * 0.5f;
	}

	Vector3d GetSize() const
	{
		return Max - Min;
	}

	float GetSizeX() const
	{
		return Max.x - Min.x;
	}

	float GetSizeY() const
	{
		return Max.y - Min.y;
	}

	float GetSizeZ() const
	{
		return Max.z - Min.z;
	}

	void GetCenterAndExtent(Vector3d* center, Vector3d* Extent) const
	{
		*center = GetCenter();
		*Extent = GetExtent();
	}

	void BuildFromCenterAndExtent(const Vector3d& Center, const Vector3d& Extent)
	{
		Min = Center - Extent;
		Max = Center + Extent;
	}

	float CalcVolume() const
	{
		return ((Max.x - Min.x) * (Max.y - Min.y) * (Max.z - Min.z));
	}

	bool Intersect(const BoundingBox3d& rhs) const
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

	bool IsInside(const Vector3d& rhs) const
	{
		return ((rhs.x > Min.x) && (rhs.x < Max.x) && (rhs.y > Min.y) && (rhs.y < Max.y) && (rhs.z > Min.z) && (rhs.z < Max.z));
	}

	BoundingBox3d Overlap(const BoundingBox3d& rhs) const
	{
		if (!Intersect(rhs))
		{
			BoundingBox3d Zero(Vector3d(0,0,0), Vector3d(0, 0, 0));
			return Zero;
		}

		BoundingBox3d Box;

		Box.Min.x = std::max(Min.x, rhs.Min.x);
		Box.Max.x = std::min(Max.x, rhs.Max.x);

		Box.Min.y = std::max(Min.y, rhs.Min.y);
		Box.Max.y = std::min(Max.y, rhs.Max.y);

		Box.Min.z = std::max(Min.z, rhs.Min.z);
		Box.Max.z = std::min(Max.z, rhs.Max.z);

		return Box;
	}

	BoundingBox3d Transform(const Matrix4d& M) const
	{
		BoundingBox3d Box;

		Vector3d Center = GetCenter();
		Vector3d Extent = GetExtent();

		Vector3d m0 = M.Row(0).xyz();
		Vector3d m1 = M.Row(1).xyz();
		Vector3d m2 = M.Row(2).xyz();
		Vector3d m3 = M.Row(3).xyz();

		Vector3d NewCenter = m0 * Center.x + m1 * Center.y + m2 * Center.z + m3;
		Vector3d NewExtent = (m0 * Extent.x).Abs() + (m1 * Extent.y).Abs() + (m2 * Extent.z).Abs();

		Box.BuildFromCenterAndExtent(NewCenter, NewExtent);

		return Box;
	}
	
	static void GetVertices(const Vector3d& Bmin, const Vector3d& Bmax, Vector3d v[8])
	{
		v[0] = Vector3d(Bmin.x, Bmin.y, Bmin.z);
		v[1] = Vector3d(Bmax.x, Bmin.y, Bmin.z);
		v[2] = Vector3d(Bmin.x, Bmax.y, Bmin.z);
		v[3] = Vector3d(Bmax.x, Bmax.y, Bmin.z);
		v[4] = Vector3d(Bmin.x, Bmin.y, Bmax.z);
		v[5] = Vector3d(Bmax.x, Bmin.y, Bmax.z);
		v[6] = Vector3d(Bmin.x, Bmax.y, Bmax.z);
		v[7] = Vector3d(Bmax.x, Bmax.y, Bmax.z);
	}
};
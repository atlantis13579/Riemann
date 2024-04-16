#pragma once

#include <float.h>
#include <algorithm>
#include <limits>

#include "Vector3.h"
#include "Matrix4.h"
#include "Quaternion.h"

namespace Maths
{
	#define _USE_MATH_DEFINES
	#undef max
	#undef min

	template <typename T>
	class TAABB3
	{
	public:
		TVector3<T> Min;
		TVector3<T> Max;

	public:
		TAABB3<T>() { }
		explicit TAABB3<T>(const TVector3<T>& InMin, const TVector3<T>& InMax)
			: Min(InMin)
			, Max(InMax)
		{ }

		explicit TAABB3<T>(T InMin, T InMax)
			: Min(InMin, InMin, InMin)
			, Max(InMax, InMax, InMax)
		{ }

		explicit TAABB3<T>(const TVector3<T>& InCenter, const T extent) 
			: Min(InCenter.x - extent, InCenter.y - extent, InCenter.z - extent)
			, Max(InCenter.x + extent, InCenter.y + extent, InCenter.z + extent)
		{ }

		explicit TAABB3<T>(const TVector3<T>* v, size_t Num)
		{
			Min = Max = v[0];
			for (size_t i = 1; i < Num; ++i)
			{
				this->Encapsulate(v[i]);
			}
		}

		explicit TAABB3<T>(const TVector3<T>& v0, const TVector3<T>& v1, const TVector3<T>& v2)
		{
			Min = Max = v0;
			this->Encapsulate(v1, v2);
		}

		explicit TAABB3<T>(const TAABB3<T>& aabb1, const TAABB3<T>& aabb2)
		{
			Min = aabb2.Min.Min(aabb1.Min);
			Max = aabb2.Max.Max(aabb1.Max);
		}

		TAABB3<T>(const TAABB3<T>& aabb1)
		{
			Min = aabb1.Min;
			Max = aabb1.Max;
		}

	public:
		bool operator==(const TAABB3<T>& Other) const
		{
			return (Min == Other.Min) && (Max == Other.Max);
		}

		TAABB3<T>& operator=(const TAABB3<T>& rhs)
		{
			Min = rhs.Min;
			Max = rhs.Max;
			return *this;
		}

		TAABB3<T> operator+(const TVector3<T>& rhs) const
		{
			return TAABB3<T>(*this) += rhs;
		}

		TAABB3<T>& operator+=(const TVector3<T>& rhs)
		{
			Min += rhs;
			Max += rhs;
			return *this;
		}

		TAABB3<T>& operator+=(const TAABB3<T>& rhs)
		{
			Min.x = std::min(Min.x, rhs.Min.x);
			Min.y = std::min(Min.y, rhs.Min.y);
			Min.z = std::min(Min.z, rhs.Min.z);

			Max.x = std::max(Max.x, rhs.Max.x);
			Max.y = std::max(Max.y, rhs.Max.y);
			Max.z = std::max(Max.z, rhs.Max.z);
			return *this;
		}

		void SetEmpty()
		{
			*this = TAABB3<T>::Empty();
		}

		TAABB3<T>& Thicken(float Thickness)
		{
			Min -= Thickness;
			Max += Thickness;
			return *this;
		}

		TAABB3<T>& Encapsulate(const TVector3<T>& a)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			return *this;
		}

		TAABB3<T>& Encapsulate(const TVector3<T>& a, const TVector3<T>& b)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			Min = Min.Min(b);
			Max = Max.Max(b);
			return *this;
		}

		TAABB3<T>& Encapsulate(const TVector3<T>& a, const TVector3<T>& b, const TVector3<T>& c)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			Min = Min.Min(b);
			Max = Max.Max(b);
			Min = Min.Min(c);
			Max = Max.Max(c);
			return *this;
		}

		TAABB3<T>& Encapsulate(const TAABB3<T>& aabb)
		{
			Min = Min.Min(aabb.Min);
			Max = Max.Max(aabb.Max);
			return *this;
		}

		TAABB3<T>& Encapsulate(const TAABB3<T>& aabb1, const TAABB3<T>& aabb2)
		{
			Min = Min.Min(aabb1.Min);
			Max = Max.Max(aabb1.Max);
			Min = Min.Min(aabb2.Min);
			Max = Max.Max(aabb2.Max);
			return *this;
		}

		inline TVector3<T> GetCenter() const
		{
			return (Min + Max) * (T)0.5;
		}

		inline TVector3<T> GetExtent() const
		{
			return (Max - Min) * (T)0.5;
		}

		inline TVector3<T> GetSize() const
		{
			return Max - Min;
		}

		inline T GetVolume() const
		{
			return (Max.x - Min.x) * (Max.y - Min.y) * (Max.z - Min.z);
		}

		inline T GetLengthX() const
		{
			return Max.x - Min.x;
		}

		inline T GetLengthY() const
		{
			return Max.y - Min.y;
		}

		inline T GetLengthZ() const
		{
			return Max.z - Min.z;
		}

		T MaxDim() const
		{
			return std::max(std::max(GetLengthX(), GetLengthY()), GetLengthZ());
		}

		T MinDim() const
		{
			return std::min(std::min(GetLengthX(), GetLengthY()), GetLengthZ());
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

		inline bool Intersect(const TAABB3<T>& rhs) const
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

		inline bool Intersect(const TVector3<T>& Bmin, const TVector3<T>& Bmax) const
		{
			if (Min.x > Bmax.x || Bmin.x > Max.x)
			{
				return false;
			}

			if (Min.y > Bmax.y || Bmin.y > Max.y)
			{
				return false;
			}

			if (Min.z > Bmax.z || Bmin.z > Max.z)
			{
				return false;
			}

			return true;
		}

		inline bool IsInside(const TVector3<T>& rhs) const
		{
			return ((rhs.x >= Min.x) && (rhs.x <= Max.x) && (rhs.y >= Min.y) && (rhs.y <= Max.y) && (rhs.z >= Min.z) && (rhs.z <= Max.z));
		}

		inline bool IsInside(const TAABB3<T>& rhs) const
		{
			return ((Min.x <= rhs.Min.x) && (Max.x >= rhs.Max.x) && (Min.y <= rhs.Min.y) && (Max.y >= rhs.Max.y) && (Min.z <= rhs.Min.z) && (Max.z >= rhs.Max.z));
		}

		bool		GetIntersection(const TVector3<T>& Bmin, const TVector3<T>& Bmax, TAABB3<T>& OutBox) const
		{
			if (!Intersect(Bmin, Bmax))
			{
				return false;
			}

			OutBox.Min.x = std::max(Min.x, Bmin.x);
			OutBox.Max.x = std::min(Max.x, Bmax.x);

			OutBox.Min.y = std::max(Min.y, Bmin.y);
			OutBox.Max.y = std::min(Max.y, Bmax.y);

			OutBox.Min.z = std::max(Min.z, Bmin.z);
			OutBox.Max.z = std::min(Max.z, Bmax.z);

			return true;
		}

		bool		GetIntersection(const TAABB3<T>& Box, TAABB3<T>& OutBox) const
		{
			return GetIntersection(Box.Min, Box.Max, OutBox);
		}

		static TAABB3<T> Transform(const TAABB3<T>& src, const Matrix4& M)
		{
			TAABB3<T> Box;

			TVector3<T> Center = src.GetCenter();
			TVector3<T> Extent = src.GetExtent();

			TVector3<T> m0 = M.Column(0).xyz();
			TVector3<T> m1 = M.Column(1).xyz();
			TVector3<T> m2 = M.Column(2).xyz();
			TVector3<T> m3 = M.Column(3).xyz();

			TVector3<T> NewCenter = m0 * Center.x + m1 * Center.y + m2 * Center.z + m3;
			TVector3<T> NewExtent = (m0 * Extent.x).Abs() + (m1 * Extent.y).Abs() + (m2 * Extent.z).Abs();

			Box.BuildFromCenterAndExtent(NewCenter, NewExtent);

			return Box;
		}

		static TAABB3<T> Transform(const TAABB3<T>& src, const Vector3& t, const Quaternion& q)
		{
			Matrix4 mat = q.ToRotationMatrix4();
			mat[0][3] += t.x;
			mat[1][3] += t.y;
			mat[2][3] += t.z;
			return Transform(src, mat);
		}

		static TAABB3<T> Transform(const TAABB3<T>& src, const Vector3& t, const Quaternion& q, const Vector3& s)
		{
			Matrix4 mat = q.ToRotationMatrix4();
			mat[0] *= s.x;
			mat[1] *= s.y;
			mat[2] *= s.z;
			mat[0][3] += t.x;
			mat[1][3] += t.y;
			mat[2][3] += t.z;
			return Transform(src, mat);
		}

		static void GetVertices(const TVector3<T>& Bmin, const TVector3<T>& Bmax, TVector3<T>* v)
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
			return TAABB3<T>(std::numeric_limits<T>::max(), -std::numeric_limits<T>::max());
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
			Ret.Extent = TVector3<T>::Zero();
			return Ret;
		}
	};

	typedef TAABB3<float> Box3;
	typedef TAABB3<int> Box3i;

	static_assert(sizeof(Box3) == 24, "sizeof Box3 is not valid");
	static_assert(sizeof(Box3i) == 24, "sizeof Box3i is not valid");
}

using Box3 = Maths::Box3;
using Box3i = Maths::Box3i;
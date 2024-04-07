#pragma once

#include <float.h>
#include <algorithm>
#include <limits>

#include "Vector2.h"
#include "Matrix2.h"

namespace Maths
{
#define _USE_MATH_DEFINES

#undef max
#undef min

	template <typename T>
	class TAABB2
	{
	public:
		TVector2<T> Min;
		TVector2<T> Max;

	public:
		TAABB2<T>() { }
		explicit TAABB2<T>(const TVector2<T>& InMin, const TVector2<T>& InMax)
			: Min(InMin)
			, Max(InMax)
		{ }

		explicit TAABB2<T>(T InMin, T InMax)
			: Min(InMin, InMin)
			, Max(InMax, InMax)
		{ }

		explicit TAABB2<T>(const TVector2<T>* v, size_t Num)
		{
			Min = Max = v[0];
			for (size_t i = 1; i < Num; ++i)
			{
				this->Encapsulate(v[i]);
			}
		}

		explicit TAABB2<T>(const TVector2<T>& v0, const TVector2<T>& v1, const TVector2<T>& v2)
		{
			Min = Max = v0;
			this->Encapsulate(v1, v2);
		}

		explicit TAABB2<T>(const TAABB2<T>& aabb1, const TAABB2<T>& aabb2)
		{
			Min = aabb2.Min.Min(aabb1.Min);
			Max = aabb2.Max.Max(aabb1.Max);
		}

		TAABB2<T>(const TAABB2<T>& aabb1)
		{
			Min = aabb1.Min;
			Max = aabb1.Max;
		}

	public:
		bool operator==(const TAABB2<T>& Other) const
		{
			return (Min == Other.Min) && (Max == Other.Max);
		}

		TAABB2<T>& operator=(const TAABB2<T>& rhs)
		{
			Min = rhs.Min;
			Max = rhs.Max;
			return *this;
		}

		TAABB2<T> operator+(const TVector2<T>& rhs) const
		{
			return TAABB2<T>(*this) += rhs;
		}

		TAABB2<T>& operator+=(const TVector2<T>& rhs)
		{
			Min += rhs;
			Max += rhs;
			return *this;
		}

		TAABB2<T>& operator+=(const TAABB2<T>& rhs)
		{
			Min.x = std::min(Min.x, rhs.Min.x);
			Min.y = std::min(Min.y, rhs.Min.y);

			Max.x = std::max(Max.x, rhs.Max.x);
			Max.y = std::max(Max.y, rhs.Max.y);
			return *this;
		}

		void SetEmpty()
		{
			*this = TAABB2<T>::Empty();
		}

		TAABB2<T>& Thicken(float Thickness)
		{
			Min -= Thickness;
			Max += Thickness;
			return *this;
		}

		TAABB2<T>& Encapsulate(const TVector2<T>& a)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			return *this;
		}

		TAABB2<T>& Encapsulate(const TVector2<T>& a, const TVector2<T>& b)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			Min = Min.Min(b);
			Max = Max.Max(b);
			return *this;
		}

		TAABB2<T>& Encapsulate(const TVector2<T>& a, const TVector2<T>& b, const TVector2<T>& c)
		{
			Min = Min.Min(a);
			Max = Max.Max(a);
			Min = Min.Min(b);
			Max = Max.Max(b);
			Min = Min.Min(c);
			Max = Max.Max(c);
			return *this;
		}

		TAABB2<T>& Encapsulate(const TAABB2<T>& aabb)
		{
			Min = Min.Min(aabb.Min);
			Max = Max.Max(aabb.Max);
			return *this;
		}

		TAABB2<T>& Encapsulate(const TAABB2<T>& aabb1, const TAABB2<T>& aabb2)
		{
			Min = Min.Min(aabb1.Min);
			Max = Max.Max(aabb1.Max);
			Min = Min.Min(aabb2.Min);
			Max = Max.Max(aabb2.Max);
			return *this;
		}

		TVector2<T> GetCenter() const
		{
			return TVector2<T>((Min + Max) * 0.5f);
		}

		TVector2<T> GetExtent() const
		{
			return (Max - Min) * 0.5f;
		}

		TVector2<T> GetSize() const
		{
			return Max - Min;
		}

		T CalculateVolume() const
		{
			return (Max.x - Min.x) * (Max.y - Min.y);
		}

		inline T GetLengthX() const
		{
			return Max.x - Min.x;
		}

		inline T GetLengthY() const
		{
			return Max.y - Min.y;
		}

		void GetCenterAndExtent(TVector2<T>* center, TVector2<T>* Extent) const
		{
			*center = GetCenter();
			*Extent = GetExtent();
		}

		void BuildFromCenterAndExtent(const TVector2<T>& Center, const TVector2<T>& Extent)
		{
			Min = Center - Extent;
			Max = Center + Extent;
		}

		inline bool Intersect(const TAABB2<T>& rhs) const
		{
			if (Min.x > rhs.Max.x || rhs.Min.x > Max.x)
			{
				return false;
			}

			if (Min.y > rhs.Max.y || rhs.Min.y > Max.y)
			{
				return false;
			}

			return true;
		}

		inline bool Intersect(const TVector2<T>& Bmin, const TVector2<T>& Bmax) const
		{
			if (Min.x > Bmax.x || Bmin.x > Max.x)
			{
				return false;
			}

			if (Min.y > Bmax.y || Bmin.y > Max.y)
			{
				return false;
			}

			return true;
		}

		inline bool IsInside(const TVector2<T>& rhs) const
		{
			return (rhs.x >= Min.x) && (rhs.x <= Max.x) && (rhs.y >= Min.y) && (rhs.y <= Max.y);
		}

		inline bool IsInside(const TAABB2<T>& rhs) const
		{
			return  (Min.x <= rhs.Min.x) && (Max.x >= rhs.Max.x) && (Min.y <= rhs.Min.y) && (Max.y >= rhs.Max.y);
		}

		bool		GetIntersection(const TVector2<T>& Bmin, const TVector2<T>& Bmax, TAABB2<T>& OutBox) const
		{
			if (!Intersect(Bmin, Bmax))
			{
				return false;
			}

			OutBox.Min.x = std::max(Min.x, Bmin.x);
			OutBox.Max.x = std::min(Max.x, Bmax.x);

			OutBox.Min.y = std::max(Min.y, Bmin.y);
			OutBox.Max.y = std::min(Max.y, Bmax.y);

			return true;
		}

		bool		GetIntersection(const TAABB2<T>& Box, TAABB2<T>& OutBox) const
		{
			return GetIntersection(Box.Min, Box.Max, OutBox);
		}

		static TAABB2<T> Unit()
		{
			return TAABB2<T>(-TVector2<T>::One(), TVector2<T>::One());
		}

		static TAABB2<T> One()
		{
			return TAABB2<T>(TVector2<T>::Zero(), TVector2<T>::One());
		}

		static TAABB2<T> Empty()
		{
			return TAABB2<T>(std::numeric_limits<T>::max(), -std::numeric_limits<T>::max());
		}
	};

	template <typename T>
	class TCE2
	{
	public:
		TVector2<T> Center;
		TVector2<T> Extent;

		TAABB2<T> GetAABB() const
		{
			return TAABB2<T>(Center - Extent, Center + Extent);
		}

		static TCE2<T> Empty()
		{
			TCE2<T> Ret;
			Ret.Center = TVector2<T>::Zero();
			Ret.Extent = TVector2<T>::Zero();
			return Ret;
		}
	};

	typedef TAABB2<float> Box2;

	static_assert(sizeof(Box2) == 16, "sizeof Box2 is not valid");
}

using Box2 = Maths::Box2;
#pragma once

#include <float.h>
#include <algorithm>
#include <limits>

namespace Maths
{
	#define _USE_MATH_DEFINES
	#undef max
	#undef min

	template <typename T>
	class TAABB1
	{
	public:
		T Min;
		T Max;

	public:
		TAABB1<T>() { }
		explicit TAABB1<T>(const T InMin, const T InMax)
			: Min(std::min(InMin, InMax))
			, Max(std::max(InMin, InMax))
		{ }

		explicit TAABB1<T>(const T* v, size_t Num)
		{
			Min = Max = v[0];
			for (size_t i = 1; i < Num; ++i)
			{
				this->Encapsulate(v[i]);
			}
		}

		explicit TAABB1<T>(const T& v0, const T& v1, const T& v2)
		{
			Min = Max = v0;
			this->Encapsulate(v1, v2);
		}

		explicit TAABB1<T>(const TAABB1<T>& aabb1, const TAABB1<T>& aabb2)
		{
			Min = std::min(aabb2.Min, aabb1.Min);
			Max = std::max(aabb2.Max, aabb1.Max);
		}

		TAABB1<T>(const TAABB1<T>& aabb1)
		{
			Min = aabb1.Min;
			Max = aabb1.Max;
		}

	public:
		bool operator==(const TAABB1<T>& Other) const
		{
			return (Min == Other.Min) && (Max == Other.Max);
		}

		TAABB1<T>& operator=(const TAABB1<T>& rhs)
		{
			Min = rhs.Min;
			Max = rhs.Max;
			return *this;
		}

		TAABB1<T> operator+(const T& rhs) const
		{
			return TAABB1<T>(*this) += rhs;
		}

		TAABB1<T>& operator+=(const T& rhs)
		{
			Min += rhs;
			Max += rhs;
			return *this;
		}

		TAABB1<T>& operator+=(const TAABB1<T>& rhs)
		{
			Min = std::min(Min, rhs.Min);
			Max = std::max(Max, rhs.Max);
			return *this;
		}

		void SetEmpty()
		{
			*this = TAABB1<T>::Empty();
		}

		TAABB1<T>& Thicken(float Thickness)
		{
			Min -= Thickness;
			Max += Thickness;
			return *this;
		}

		TAABB1<T>& Encapsulate(const T& a)
		{
			Min = std::min(Min, a);
			Max = std::max(Max, a);
			return *this;
		}

		TAABB1<T>& Encapsulate(const T& a, const T& b)
		{
			Min = std::min(Min, a);
			Max = std::max(Max, a);
			Min = std::min(Min, b);
			Max = std::max(Max, b);
			return *this;
		}

		TAABB1<T>& Encapsulate(const TAABB1<T>& aabb)
		{
			Min = Min.Min(aabb.Min);
			Max = Max.Max(aabb.Max);
			return *this;
		}

		TAABB1<T>& Encapsulate(const TAABB1<T>& aabb1, const TAABB1<T>& aabb2)
		{
			Min = Min.Min(aabb1.Min);
			Max = Max.Max(aabb1.Max);
			Min = Min.Min(aabb2.Min);
			Max = Max.Max(aabb2.Max);
			return *this;
		}

		inline T GetCenter() const
		{
			return T((Min + Max) * 0.5f);
		}

		inline T GetExtent() const
		{
			return (Max - Min) * 0.5f;
		}

		inline T GetLength() const
		{
			return Max - Min;
		}

		void GetCenterAndExtent(T* center, T* Extent) const
		{
			*center = GetCenter();
			*Extent = GetExtent();
		}

		void BuildFromCenterAndExtent(const T& Center, const T& Extent)
		{
			Min = Center - Extent;
			Max = Center + Extent;
		}

		inline bool Intersect(const TAABB1<T>& rhs) const
		{
			if (Min > rhs.Max || rhs.Min > Max)
			{
				return false;
			}

			return true;
		}

		inline bool Intersect(const T& Bmin, const T& Bmax) const
		{
			if (Min > Bmax || Bmin > Max)
			{
				return false;
			}

			return true;
		}

		inline bool IsInside(const T& rhs) const
		{
			return (rhs >= Min) && (rhs <= Max);
		}

		inline bool IsInside(const TAABB1<T>& rhs) const
		{
			return  (Min <= rhs.Min) && (Max >= rhs.Max);
		}

		bool IsPoint() const
		{
			return (Max - Min) < std::numeric_limits<T>::epsilon();
		}

		bool		GetIntersection(const T& Bmin, const T& Bmax, TAABB1<T>& OutBox) const
		{
			if (!Intersect(Bmin, Bmax))
			{
				return false;
			}

			OutBox.Min = std::max(Min, Bmin);
			OutBox.Max = std::min(Max, Bmax);

			return true;
		}

		bool		GetIntersection(const TAABB1<T>& Box, TAABB1<T>& OutBox) const
		{
			return GetIntersection(Box.Min, Box.Max, OutBox);
		}

		static TAABB1<T> Unit()
		{
			return TAABB1<T>((T)-1, (T)1);
		}

		static TAABB1<T> One()
		{
			return TAABB1<T>(0, (T)1);
		}

		static TAABB1<T> Empty()
		{
			return TAABB1<T>(std::numeric_limits<T>::max(), -std::numeric_limits<T>::max());
		}
	};

	template <typename T>
	class TCE1
	{
	public:
		T Center;
		T Extent;

		TAABB1<T> GetAABB() const
		{
			return TAABB1<T>(Center - Extent, Center + Extent);
		}

		static TCE1<T> Empty()
		{
			TCE1<T> Ret;
			Ret.Center = (T)0;
			Ret.Extent = (T)0;
			return Ret;
		}
	};

	typedef TAABB1<float> Box1;

	static_assert(sizeof(Box1) == 8, "sizeof Box1 is not valid");
}

using Box1 = Maths::Box1;
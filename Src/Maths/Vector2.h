#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

namespace Maths
{
	template <typename T>
	class TVector2
	{
	public:
		T x, y;

		TVector2()
		{
		}

		explicit TVector2(T _x, T _y)
		{
			x = _x;
			y = _y;
		}

		explicit inline TVector2<T>(T k)
		{
			x = y = k;
		}

		TVector2(const TVector2<T>& v)
		{
			x = v.x;
			y = v.y;
		}

		inline T	Dot(const TVector2<T>& v) const
		{
			return x * v.x + y * v.y;
		}

		inline T	Cross(const TVector2<T>& v) const
		{
			return x * v.y - y * v.x;
		}

		TVector2<T>	Unit() const
		{
			T m = (T)Length();
			return TVector2<T>(x / m, y / m);
		}

		TVector2<T> SafeUnit() const
		{
			T m = Length();
			if (m <= std::numeric_limits<T>::epsilon())
			{
				return TVector2<T>::Zero();
			}
			return TVector2<T>(x / m, y / m);
		}

		T Normalize()
		{
			T m = Length();
			x /= m;
			y /= m;
			return m;
		}

		T SafeNormalize()
		{
			T m = Length();
			if (m <= std::numeric_limits<T>::epsilon())
			{
				x = y = (T)0;
				return (T)0;
			}
			x /= m;
			y /= m;
			return m;
		}

		inline T	Length() const
		{
			return (T)sqrtf(x * x + y * y);
		}

		inline T	SquareLength() const
		{
			return x * x + y * y;
		}

		TVector2<T> Rotate(float theta) const
		{
			const T m0 = cosf(theta);
			const T m1 = sinf(theta);
			return TVector2<T>(m0 * x - m1 * y, m1 * x + m0 * y);
		}

		inline TVector2<T>& operator=(const TVector2<T>& v)
		{
			x = v.x;
			y = v.y;
			return *this;
		}

		inline TVector2<T> operator+(const TVector2<T>& v) const
		{
			return TVector2<T>(x + v.x, y + v.y);
		}

		inline TVector2<T> operator-(const TVector2<T>& v) const
		{
			return TVector2<T>(x - v.x, y - v.y);
		}

		inline TVector2<T> operator*(const TVector2<T>& v) const
		{
			return TVector2<T>(x * v.x, y * v.y);
		}

		inline TVector2<T> operator*(T k) const
		{
			return TVector2<T>(x * k, y * k);
		}

		inline TVector2<T> operator/(T k) const
		{
			return TVector2<T>(x / k, y / k);
		}

		inline TVector2<T> operator-()
		{
			return TVector2<T>(-x, -y);
		}

		inline void	operator+= (const TVector2<T>& v)
		{
			x += v.x;
			y += v.y;
		}

		inline void operator-= (const TVector2<T>& v)
		{
			x -= v.x;
			y -= v.y;
		}

		inline void operator*= (const TVector2<T>& v)
		{
			x *= v.x;
			y *= v.y;
		}

		inline void	operator/= (const TVector2<T>& v)
		{
			x /= v.x;
			y /= v.y;
		}

		inline void	operator*= (T k)
		{
			x *= k;
			y *= k;
		}

		inline void	operator/= (T k)
		{
			x /= k;
			y /= k;
		}

		bool	operator> (T k) const
		{
			return x > k && y > k;
		}

		bool	operator>= (T k) const
		{
			return x >= k && y >= k;
		}

		bool	operator< (T k) const
		{
			return x < k&& y < k;
		}

		bool	operator<= (T k) const
		{
			return x <= k && y <= k;
		}

		bool	operator> (const TVector2<T>& rhs) const
		{
			return x > rhs.x && y > rhs.y;
		}

		bool	operator>= (const TVector2<T>& rhs) const
		{
			return x >= rhs.x && y >= rhs.y;
		}

		bool	operator< (const TVector2<T>& rhs) const
		{
			return x < rhs.x&& y < rhs.y;
		}

		bool	operator<= (const TVector2<T>& rhs) const
		{
			return x <= rhs.x && y <= rhs.y;
		}

		bool	operator==(const TVector2<T>& rhs) const
		{
			return x == rhs.x && y == rhs.y;
		}

		bool	operator!=(const TVector2<T>& rhs) const
		{
			return x != rhs.x || y != rhs.y;
		}

		inline const T* Data() const
		{
			return &x;
		}

		inline T* Data()
		{
			return &x;
		}

		inline const T& operator[](int i) const
		{
			const T* p = (const T*)this;
			return p[i];
		}

		inline T& operator[](int i)
		{
			T* p = (T*)this;
			return p[i];
		}

		inline TVector2<T> PerpCCW() const
		{
			return TVector2<T>(-y, x);
		}

		inline TVector2<T> PerpCW() const
		{
			return TVector2<T>(y, -x);
		}

		inline TVector2<T> Min(const TVector2<T>& v) const
		{
			return TVector2<T>(std::min(v.x, x), std::min(v.y, y));
		}

		inline TVector2<T> Max(const TVector2<T>& v) const
		{
			return TVector2<T>(std::max(v.x, x), std::max(v.y, y));
		}

		static TVector2<T> Lerp(const TVector2<T>& start, const TVector2<T>& end, float t)
		{
			return TVector2<T>(
				start.x * (1.0f - t) + end.x * t,
				start.y * (1.0f - t) + end.y * t);
		}

		static TVector2<T> UnitLerp(TVector2<T>& start, TVector2<T>& end, float t)
		{
			return Lerp(start, end, t).Unit();
		}

		static TVector2<T> Slerp(TVector2<T>& start, TVector2<T>& end, float t)
		{
			float innerp = start.Dot(end);
			innerp = std::min(std::max(-1.0f, innerp), 1.0f);
			float angle = acosf(innerp) * t;
			TVector2<T> base = (end - start * innerp).Unit();
			return start * cosf(angle) + base * sinf(angle);
		}

		constexpr static TVector2<T> Zero()
		{
			return TVector2<T>(0, 0);
		}

		constexpr static TVector2<T> One()
		{
			return TVector2<T>((T)1, (T)1);
		}

		constexpr static TVector2<T> UnitX()
		{
			return TVector2<T>((T)1, 0);
		}

		constexpr static TVector2<T> UnitY()
		{
			return TVector2<T>(0, (T)1);
		}

		static TVector2<T> Random()
		{
			return TVector2<T>((T)rand() / RAND_MAX, (T)rand() / RAND_MAX);
		}
	};

	template <typename T>
	inline TVector2<T> operator* (T s, const TVector2<T>& vv)
	{
		return vv * s;
	}

	template <typename T>
	inline T DotProduct(const TVector2<T>& a, const TVector2<T>& b)
	{
		return a.Dot(b);
	}

	template <typename T>
	inline TVector2<T> Rotate(const TVector2<T>& v, T theta)
	{
		const T m0 = cos(theta);
		const T m1 = sin(theta);
		return TVector2<T>(m0 * v.x - m1 * v.y, m1 * v.x + m0 * v.y);
	}

	template <typename T>
	T DotPerp(const TVector2<T>& v1, const TVector2<T>& v2)
	{
		return v1.x * v2.y - v1.y * v2.x;
	}

	template<typename T>
	int Orient(const TVector2<T>& A, const TVector2<T> & B, const TVector2<T>& C)
	{
		T dp = DotPerp((B - A), (C - A));
		if (dp > std::numeric_limits<T>::epsilon())
		{
			return 1;
		}
		else if (dp < -std::numeric_limits<T>::epsilon())
		{
			return -1;
		}
		return 0;
	}

	typedef TVector2<float> Vector2;
	typedef TVector2<int>	Vector2i;

	static_assert(sizeof(Vector2) == 8, "sizeof Vector2 is not valid");
	static_assert(sizeof(Vector2i) == 8, "sizeof Vector2i is not valid");
}

using Vector2 = Maths::Vector2;
using Vector2i = Maths::Vector2i;
#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

#include "Vector3.h"

namespace Maths
{
	template <typename T>
	class TVector4
	{
	public:
		T x, y, z, w;

		TVector4<T>()
		{
		}

		explicit TVector4<T>(T k)
		{
			x = k;
			y = k;
			z = k;
			w = k;
		}

		explicit TVector4<T>(T _x, T _y, T _z, T _w)
		{
			x = _x;
			y = _y;
			z = _z;
			w = _w;
		}

		explicit TVector4<T>(const TVector3<T>& v, float vw)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = vw;
		}

		TVector4<T>(const TVector4<T>& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;
		}

		TVector3<T> xyz() const
		{
			return TVector3<T>(x, y, z);
		}

		inline T Dot(const TVector4<T>& v) const
		{
			return x * v.x + y * v.y + z * v.z + w * v.w;
		}

		TVector4<T> Unit() const
		{
			T m = Length();
			return TVector4<T>(x / m, y / m, z / m, w / m);
		}

		void Normalize()
		{
			T m = Length();
			x /= m;
			y /= m;
			z /= m;
			w /= m;
		}

		inline T Length() const
		{
			return sqrtf(x * x + y * y + z * z + w * w);
		}

		inline T SquareLength() const
		{
			return x * x + y * y + z * z + w * w;
		}

		TVector4<T> Abs() const
		{
			return TVector4<T>(std::abs(x), std::abs(y), std::abs(z), std::abs(w));
		}

		inline TVector4<T>& operator=(const TVector4<T>& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;
			return *this;
		}

		inline TVector4<T> operator+(const TVector4<T>& v) const
		{
			return TVector4<T>(x + v.x, y + v.y, z + v.z, w + v.w);
		}

		inline TVector4<T> operator-(const TVector4<T>& v) const
		{
			return TVector4<T>(x - v.x, y - v.y, z - v.z, w - v.w);
		}

		inline TVector4<T> operator*(const TVector4<T>& v) const
		{
			return TVector4<T>(x * v.x, y * v.y, z * v.z, w * v.w);
		}

		inline TVector4<T> operator*(T k) const
		{
			return TVector4<T>(x * k, y * k, z * k, w * k);
		}

		inline TVector4<T> operator/(T k) const
		{
			return TVector4<T>(x / k, y / k, z / k, w / k);
		}

		inline TVector4<T> operator-()
		{
			return TVector4<T>(-x, -y, -z, -w);
		}

		inline void	operator+= (const TVector4<T>& v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
			w += v.w;
		}

		inline void	operator-= (const TVector4<T>& v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
			w -= v.w;
		}

		inline void	operator*= (const TVector4<T>& v)
		{
			x *= v.x;
			y *= v.y;
			z *= v.z;
			w *= v.w;
		}

		inline void	operator/= (const TVector4<T>& v)
		{
			x /= v.x;
			y /= v.y;
			z /= v.z;
			w /= v.w;
		}

		inline void	operator*= (T k)
		{
			x *= k;
			y *= k;
			z *= k;
			w *= k;
		}

		inline void	operator/= (T k)
		{
			x /= k;
			y /= k;
			z /= k;
			w /= k;
		}

		inline bool	operator> (T k) const
		{
			return x > k && y > k && z > k && w > k;
		}

		inline bool	operator>= (T k) const
		{
			return x >= k && y >= k && z >= k && w >= k;
		}

		inline bool	operator< (T k) const
		{
			return x < k&& y < k&& z < k&& w < k;
		}

		inline bool	operator<= (T k) const
		{
			return x <= k && y <= k && z <= k && w <= k;
		}

		inline bool	operator> (const TVector4<T>& rhs) const
		{
			return x > rhs.x && y > rhs.y && z > rhs.z && w > rhs.w;
		}

		inline bool	operator>= (const TVector4<T>& rhs) const
		{
			return x >= rhs.x && y >= rhs.y && z >= rhs.z && w >= rhs.w;
		}

		inline bool	operator< (const TVector4<T>& rhs) const
		{
			return x < rhs.x&& y < rhs.y&& z < rhs.z&& w < rhs.w;
		}

		inline bool	operator<= (const TVector4<T>& rhs) const
		{
			return x <= rhs.x && y <= rhs.y && z <= rhs.z && w <= rhs.w;
		}

		inline bool	operator==(const TVector4<T>& rhs) const
		{
			return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
		}

		inline bool	operator!=(const TVector4<T>& rhs) const
		{
			return x != rhs.x || y != rhs.y || z != rhs.z || w != rhs.w;
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
			return reinterpret_cast<const T*>(this)[i];
		}

		inline T& operator[](int i)
		{
			return reinterpret_cast<T*>(this)[i];
		}

		inline TVector4<T> Min(const TVector4<T>& v) const
		{
			return TVector4<T>(std::min(v.x, x), std::min(v.y, y), std::min(v.z, z), std::min(v.w, w));
		}

		inline TVector4<T> Max(const TVector4<T>& v) const
		{
			return TVector4<T>(std::max(v.x, x), std::max(v.y, y), std::max(v.z, z), std::max(v.w, w));
		}

		static TVector4<T> Lerp(TVector4<T>& start, TVector4<T>& end, float t)
		{
			return TVector4<T>(
				start.x * (1.0f - t) + end.x * t,
				start.y * (1.0f - t) + end.y * t,
				start.z * (1.0f - t) + end.z * t,
				start.w * (1.0f - t) + end.w * t);
		}

		static TVector4<T> NLerp(TVector4<T>& start, TVector4<T>& end, float t)
		{
			return Lerp(start, end, t).Unit();
		}

		static TVector4<T> Slerp(TVector4<T>& start, TVector4<T>& end, float t)
		{
			float innerp = start.Dot(end);
			innerp = std::min(std::max(-1.0f, innerp), 1.0f);
			float angle = acosf(innerp) * t;
			TVector4<T> base = (end - start * innerp).Unit();
			return start * cosf(angle) + base * sinf(angle);
		}

		static const TVector4<T>& Zero()
		{
			static TVector4<T> zero(0, 0, 0, 0);
			return zero;
		}
	};

	typedef TVector4<float> Vector4;

	static_assert(sizeof(Vector4) == 16, "sizeof Vector4 is not valid");

}

using Vector4 = Maths::Vector4;
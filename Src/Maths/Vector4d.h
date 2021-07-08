#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

#include "Vector3d.h"

template <typename T>
class TVector4
{
public:
	union
	{
		struct { T x, y, z, w; };
		struct { T r, g, b, a; };
		struct { T coords[4]; };
	};

	TVector4<T>(T _x, T _y, T _z, T _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	TVector4<T>(T v[4])
	{
		x = v[0];
		y = v[1];
		z = v[2];
		w = v[3];
	}

	TVector4<T>(const TVector4<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
	}

	TVector4<T>(T v)
	{
		x = v;
		y = v;
		z = v;
		w = v;
	}

	TVector4<T>()
	{
	}

	TVector3<T> xyz()
	{
		return TVector3<T>(x, y, z);
	}

	inline T    Dot(const TVector4<T>& v) const
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

	T    Length() const
	{
		return sqrtf(x * x + y * y + z * z + w * w);
	}

	T    SquareLength() const
	{
		return x * x + y * y + z * z + w * w;
	}

	TVector4<T>& operator=(const TVector4<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
		return *this;
	}

	TVector4<T> operator+(const TVector4<T>& v) const
	{
		return TVector4<T>(x + v.x, y + v.y, z + v.z, w + v.w);
	}

	TVector4<T> operator-(const TVector4<T>& v) const
	{
		return TVector4<T>(x - v.x, y - v.y, z - v.z, w - v.w);
	}

	TVector4<T> operator*(const TVector4<T>& v) const
	{
		return TVector4<T>(x * v.x, y * v.y, z * v.z, w * v.w);
	}

	TVector4<T> operator*(T k) const
	{
		return TVector4<T>(x * k, y * k, z * k, w * k);
	}

	TVector4<T> operator/(T k) const
	{
		return TVector4<T>(x / k, y / k, z / k, w / k);
	}

	TVector4<T> operator-()
	{
		return TVector4<T>(-x, -y, -z, -w);
	}

	void	 operator+= (const TVector4<T>& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
	}

	void	 operator-= (const TVector4<T>& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		w -= v.w;
	}

	void	 operator*= (const TVector4<T>& v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
		w *= v.w;
	}

	void	 operator/= (const TVector4<T>& v)
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
		w /= v.w;
	}

	void	 operator*= (T k)
	{
		x *= k;
		y *= k;
		z *= k;
		w *= k;
	}

	void	 operator/= (T k)
	{
		x /= k;
		y /= k;
		z /= k;
		w /= k;
	}

	inline T operator[](int i) const
	{
		return coords[i];
	}

	inline T& operator[](int i)
	{
		return coords[i];
	}

	static TVector4<T> Lerp(TVector4<T>& start, TVector4<T>& end, float t)
	{
		return TVector4<T>(
			start.x * (1.0f - t) + end.x * t,
			start.y * (1.0f - t) + end.y * t,
			start.z * (1.0f - t) + end.z * t,
			start.w * (1.0f - t) + end.z * w);
	}

	static TVector4<T> UnitLerp(TVector4<T>& start, TVector4<T>& end, float t)
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
};

typedef TVector4<float> Vector4d;

static_assert(sizeof(Vector4d) == 16, "sizeof Vector4d is not valid");
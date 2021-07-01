#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

class Vector4d
{
public:
	union
	{
		struct { float x, y, z, w; };
		struct { float r, g, b, a; };
		struct { float coords[4]; };
	};

	Vector4d(float _x, float _y, float _z, float _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	Vector4d(float v[4])
	{
		x = v[0];
		y = v[1];
		z = v[2];
		w = v[3];
	}

	Vector4d(const Vector4d& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
	}
	Vector4d()
	{
	}

	inline float    Dot(const Vector4d& v) const
	{
		return x * v.x + y * v.y + z * v.z + w * v.w;
	}


	Vector4d Unit() const
	{
		float m = Length();
		return Vector4d(x / m, y / m, z / m, w / m);
	}

	void Normalize()
	{
		float m = Length();
		x /= m;
		y /= m;
		z /= m;
		w /= m;
	}

	float    Length() const
	{
		return sqrtf(x * x + y * y + z * z + w * w);
	}

	float    SquareLength() const
	{
		return x * x + y * y + z * z + w * w;
	}

	Vector4d& operator=(const Vector4d& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
		return *this;

	}

	Vector4d operator*(float k) const
	{
		return Vector4d(x * k, y * k, z * k, w * k);
	}

	Vector4d operator*(const Vector4d& v) const
	{
		return Vector4d(x * v.x, y * v.y, z * v.z, w * v.w);
	}

	Vector4d operator/(float k) const
	{
		return Vector4d(x / k, y / k, z / k, w / k);
	}

	Vector4d operator+(const Vector4d& v) const
	{
		return Vector4d(x + v.x, y + v.y, z + v.z, w + v.w);
	}

	Vector4d operator-(const Vector4d& v) const
	{
		return Vector4d(x - v.x, y - v.y, z - v.z, w - v.w);
	}

	Vector4d operator-()
	{
		return Vector4d(-x, -y, -z, -w);
	}

	void	 operator+= (const Vector4d& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
	}

	void	 operator-= (const Vector4d& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		w -= v.w;
	}

	void	 operator*= (const Vector4d& v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
		w *= v.w;
	}

	void	 operator/= (const Vector4d& v)
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
		w /= v.w;
	}

	void	 operator*= (float k)
	{
		x *= k;
		y *= k;
		z *= k;
		w *= k;
	}

	void	 operator/= (float k)
	{
		x /= k;
		y /= k;
		z /= k;
		w /= k;
	}

	static Vector4d Lerp(Vector4d& start, Vector4d& end, float t)
	{
		return (start*(1 - t) + end * t);
	}

	static Vector4d UnitLerp(Vector4d& start, Vector4d& end, float t)
	{
		return Lerp(start, end, t).Unit();
	}

	static Vector4d Slerp(Vector4d& start, Vector4d& end, float t)
	{
		float innerp = start.Dot(end);
		innerp = std::min(std::max(-1.0f, innerp), 1.0f);
		float angle = acosf(innerp) * t;
		Vector4d base = (end - start * innerp).Unit();
		return start * cosf(angle) + base * sinf(angle);
	}
};

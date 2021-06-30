#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

class Vector2d
{
public:
	union
	{
		struct { float x, y; };
		struct { float coords[2]; };
	};

	Vector2d(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	Vector2d(float v[2])
	{
		x = v[0];
		y = v[1];
	}

	Vector2d(const Vector2d& v)
	{
		x = v.x;
		y = v.y;
	}

	Vector2d()
	{
	}

	inline float  Dot(const Vector2d& v) const
	{
		return x * v.x + y * v.y;
	}

	inline float Cross(const Vector2d& b) const
	{
		return x * b.y - y * b.x;
	}

	Vector2d Unit() const
	{
		float m = Length();
		return Vector2d(x / m, y / m);
	}

	void Normalize()
	{
		float m = Length();
		x /= m;
		y /= m;
	}

	float    Length() const
	{
		return sqrtf(x * x + y * y);
	}

	float    SquareLength() const
	{
		return x * x + y * y;
	}

	Vector2d& operator=(const Vector2d& v)
	{
		x = v.x;
		y = v.y;
		return *this;
	}

	Vector2d operator*(float k) const
	{
		return Vector2d(x * k, y * k);
	}

	Vector2d operator*(const Vector2d& v) const
	{
		return Vector2d(x * v.x, y * v.y);
	}

	Vector2d operator/(const float k) const
	{
		return Vector2d(x / k, y / k);
	}

	Vector2d operator+(const Vector2d& v) const
	{
		return Vector2d(x + v.x, y + v.y);
	}

	Vector2d operator-(const Vector2d& v) const
	{
		return Vector2d(x - v.x, y - v.y);
	}

	Vector2d operator-()
	{
		return Vector2d(-x, -y);
	}

	void	 operator+= (const Vector2d& v)
	{
		x += v.x;
		y += v.y;
	}

	void	 operator-= (const Vector2d& v)
	{
		x -= v.x;
		y -= v.y;
	}

	void	 operator*= (const Vector2d& v)
	{
		x *= v.x;
		y *= v.y;
	}

	void	 operator/= (const Vector2d& v)
	{
		x /= v.x;
		y /= v.y;
	}

	void	 operator*= (float k)
	{
		x *= k;
		y *= k;
	}

	void	 operator/= (float k)
	{
		x /= k;
		y /= k;
	}

	static Vector2d Lerp(Vector2d& start, Vector2d& end, float t)
	{
		return (start*(1 - t) + end * t);
	}

	static Vector2d UnitLerp(Vector2d& start, Vector2d& end, float t)
	{
		return Lerp(start, end, t).Unit();
	}

	static Vector2d Slerp(Vector2d& start, Vector2d& end, float t)
	{
		float innerp = start.Dot(end);
		innerp = std::min(std::max(-1.0f, innerp), 1.0f);
		float angle = acosf(innerp) * t;
		Vector2d base = (end - start * innerp).Unit();
		return start * cosf(angle) + base * sinf(angle);
	}
};
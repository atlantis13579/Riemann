#pragma once

#include <math.h>

class Point2
{
public:
	float x, y;

	Point2(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	Point2(const Point2& v)
	{
		x = v.x;
		y = v.y;
	}

	Point2()
	{
	}

	inline float	Dot(const Point2& v) const
	{
		return x * v.x + y * v.y;
	}

	inline float	Cross(const Point2& v) const
	{
		return x * v.y - y * v.x;
	}

	Point2			Unit() const
	{
		float m = Length();
		return Point2(x / m, y / m);
	}

	void			Normalize()
	{
		float m = Length();
		x /= m;
		y /= m;
	}

	Point2			Rotate(float theta) const
	{
		const float m0 = cosf(theta);
		const float m1 = sinf(theta);
		return Point2(m0 * x - m1 * y, m1 * x + m0 * y);
	}

	inline float	Length() const
	{
		return sqrtf(x * x + y * y);
	}

	inline float	SquareLength() const
	{
		return x * x + y * y;
	}

	inline Point2& operator=(const Point2& v)
	{
		x = v.x;
		y = v.y;
		return *this;
	}

	Point2 operator+(const Point2& v) const
	{
		return Point2(x + v.x, y + v.y);
	}

	Point2 operator-(const Point2& v) const
	{
		return Point2(x - v.x, y - v.y);
	}

	Point2 operator*(float k) const
	{
		return Point2(x * k, y * k);
	}

	Point2 operator/(float k) const
	{
		return Point2(x / k, y / k);
	}

	Point2 operator-()
	{
		return Point2(-x, -y);
	}

	void	operator+= (const Point2& v)
	{
		x += v.x;
		y += v.y;
	}

	void	operator-= (const Point2& v)
	{
		x -= v.x;
		y -= v.y;
	}
};

static_assert(sizeof(Point2) == 8, "sizeof Point2 is not valid");

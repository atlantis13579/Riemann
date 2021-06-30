#pragma once

class Vector2i
{
public:
	union
	{
		struct { int x, y; };
	};

	Vector2i(int _x, int _y)
	{
		x = _x;
		y = _y;
	}

	Vector2i()
	{

	}

	Vector2i operator+(const Vector2i& v) const
	{
		return Vector2i(x + v.x, y + v.y);
	}

	Vector2i operator-(const Vector2i& v) const
	{
		return Vector2i(x - v.x, y - v.y);
	}

	inline int Dot(const Vector2i& b) const
	{
		return x * b.y + y * b.x;
	}

	inline int Cross(const Vector2i& b) const
	{
		return x * b.y - y * b.x;
	}
};

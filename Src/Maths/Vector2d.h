#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

template <typename T>
class TVector2
{
public:
	T x, y;

	TVector2(T _x, T _y)
	{
		x = _x;
		y = _y;
	}

	TVector2(T v[2])
	{
		x = v[0];
		y = v[1];
	}

	TVector2(const TVector2<T>& v)
	{
		x = v.x;
		y = v.y;
	}

	TVector2()
	{
	}

	inline T	Dot(const TVector2<T>& v) const
	{
		return x * v.x + y * v.y;
	}

	inline T	Cross(const TVector2<T>& b) const
	{
		return x * b.y - y * b.x;
	}

	TVector2<T>	Unit() const
	{
		T m = (T)Length();
		return TVector2<T>(x / m, y / m);
	}

	void Normalize()
	{
		T m = Length();
		x /= m;
		y /= m;
	}

	inline T	Length() const
	{
		return (T)sqrtf(x * x + y * y);
	}

	inline T	SquareLength() const
	{
		return x * x + y * y;
	}

	inline TVector2<T>& operator=(const TVector2<T>& v)
	{
		x = v.x;
		y = v.y;
		return *this;
	}

	TVector2<T> operator+(const TVector2<T>& v) const
	{
		return TVector2<T>(x + v.x, y + v.y);
	}

	TVector2<T> operator-(const TVector2<T>& v) const
	{
		return TVector2<T>(x - v.x, y - v.y);
	}

	TVector2<T> operator*(const TVector2<T>& v) const
	{
		return TVector2<T>(x * v.x, y * v.y);
	}

	TVector2<T> operator+(T k) const
	{
		return TVector2<T>(x + k, y + k);
	}

	TVector2<T> operator-(T k) const
	{
		return TVector2<T>(x - k, y - k);
	}

	TVector2<T> operator*(T k) const
	{
		return TVector2<T>(x * k, y * k);
	}

	TVector2<T> operator/(T k) const
	{
		return TVector2<T>(x / k, y / k);
	}

	TVector2<T> operator-()
	{
		return TVector2<T>(-x, -y);
	}

	void	operator+= (const TVector2<T>& v)
	{
		x += v.x;
		y += v.y;
	}

	void	operator-= (const TVector2<T>& v)
	{
		x -= v.x;
		y -= v.y;
	}

	void	operator*= (const TVector2<T>& v)
	{
		x *= v.x;
		y *= v.y;
	}

	void	operator/= (const TVector2<T>& v)
	{
		x /= v.x;
		y /= v.y;
	}

	void	operator+= (T k)
	{
		x += k;
		y += k;
	}

	void	operator-= (T k)
	{
		x -= k;
		y -= k;
	}

	void	operator*= (T k)
	{
		x *= k;
		y *= k;
	}

	void	operator/= (T k)
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
		return x < k && y < k;
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
		return x < rhs.x && y < rhs.y;
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

	inline T operator[](int i) const
	{
		const T* p = (const T*)this;
		return p[i];
	}

	inline T& operator[](int i)
	{
		T* p = (T*)this;
		return p[i];
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

	static const TVector2<T>& Zero()
	{
		static TVector2<T> zero(0, 0);
		return zero;
	}

	static const TVector2<T>& One()
	{
		static TVector2<T> One((T)1, (T)1);
		return One;
	}

	static const TVector2<T>& UnitX()
	{
		static TVector2<T> inf((T)1, 0);
		return inf;
	}

	static const TVector2<T>& UnitY()
	{
		static TVector2<T> inf(0, (T)1);
		return inf;
	}
};

typedef TVector2<float> Vector2d;
typedef TVector2<int>	Vector2i;

inline Vector2d Rotate(const Vector2d& v, float theta)
{
	const float m0 = cosf(theta);
	const float m1 = sinf(theta);
	return Vector2d(m0 * v.x - m1 * v.y, m1 * v.x + m0 * v.y);
}

static_assert(sizeof(Vector2d) == 8, "sizeof Vector2d is not valid");
static_assert(sizeof(Vector2i) == 8, "sizeof Vector2i is not valid");

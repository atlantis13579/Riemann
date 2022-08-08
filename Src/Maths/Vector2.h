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
	
	TVector2()
	{
	}

	explicit TVector2(T _x, T _y)
	{
		x = _x;
		y = _y;
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

	inline const T* Data() const
	{
		return &x;
	}

	inline T* Data()
	{
		return &x;
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
};

typedef TVector2<float> Vector2;
typedef TVector2<int>	Vector2i;

inline Vector2 Rotate(const Vector2& v, float theta)
{
	const float m0 = cosf(theta);
	const float m1 = sinf(theta);
	return Vector2(m0 * v.x - m1 * v.y, m1 * v.x + m0 * v.y);
}

static_assert(sizeof(Vector2) == 8, "sizeof Vector2 is not valid");
static_assert(sizeof(Vector2i) == 8, "sizeof Vector2i is not valid");

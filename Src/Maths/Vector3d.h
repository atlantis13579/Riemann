#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

template <typename T>
class TVector3
{
public:
	union
	{
		struct { T x, y, z; };
	};

	inline TVector3<T>(T _x, T _y, T _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	inline TVector3<T>(T v[3])
	{
		x = v[0];
		y = v[1];
		z = v[2];
	}

	inline TVector3<T>(const TVector3<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	inline TVector3<T>(T v)
	{
		x = v;
		y = v;
		z = v;
	}

	inline TVector3<T>()
	{
	}

	inline void	SetEmpty()
	{
		x = y = z = 0.0f;
	}

	inline T Dot(const TVector3<T>& v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}

	inline TVector3<T> Cross(const TVector3<T>& v) const
	{
		return TVector3<T>(y * v.z - v.y * z, z * v.x - v.z * x, x * v.y - v.x * y);
	}

	TVector3<T> Unit() const
	{
		T m = Length();
		return TVector3<T>(x / m, y / m, z / m);
	}

	TVector3<T> SafeUnit() const
	{
		T m = SquareLength();
		if (m < 1e-6f)
		{
			return TVector3<T>::Zero();
		}
		m = sqrtf(m);
		return TVector3<T>(x / m, y / m, z / m);
	}

	void Normalize()
	{
		T m = Length();
		x /= m;
		y /= m;
		z /= m;
	}

	inline T    Length() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	inline T    SquareLength() const
	{
		return x * x + y * y + z * z;
	}

	TVector3<T>& operator=(const TVector3<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	inline TVector3<T> operator+(const TVector3<T>& v) const
	{
		return TVector3<T>(x + v.x, y + v.y, z + v.z);
	}

	inline TVector3<T> operator-(const TVector3<T>& v) const
	{
		return TVector3<T>(x - v.x, y - v.y, z - v.z);
	}

	inline TVector3<T> operator*(const TVector3<T>& v) const
	{
		return TVector3<T>(x * v.x, y * v.y, z * v.z);
	}

	inline TVector3<T> operator/(const TVector3<T>& v) const
	{
		return TVector3<T>(x / v.x, y / v.y, z / v.z);
	}

	inline TVector3<T> operator*(T k) const
	{
		return TVector3<T>(x * k, y * k, z * k);
	}

	inline TVector3<T> operator/(T k) const
	{
		return TVector3<T>(x / k, y / k, z / k);
	}

	inline TVector3<T> operator-() const
	{
		return TVector3<T>(-x, -y, -z);
	}

	inline void	 operator+= (const TVector3<T>& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}

	inline void	 operator-= (const TVector3<T>& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	inline void	 operator*= (const TVector3<T>& v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	inline void	 operator/= (const TVector3<T>& v)
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
	}

	inline void	 operator*= (T k)
	{
		x *= k;
		y *= k;
		z *= k;
	}

	inline void	 operator/= (T k)
	{
		x /= k;
		y /= k;
		z /= k;
	}

	inline bool operator==(const TVector3<T>& v) const
	{
		return x == v.x && y == v.y && z == v.z;
	}

	inline bool operator!=(const TVector3<T>& v) const
	{
		return x != v.x || y != v.y || z != v.z;
	}

	inline T operator[](int i) const
	{
		const float* p = (const float*)this;
		return p[i];
	}

	inline T& operator[](int i)
	{
		float* p = (float*)this;
		return p[i];
	}

	bool IsZero() const
	{
		return SquareLength() < 0.00001f;
	}

	int ParallelTo(const TVector3<T>&v) const
	{
		T dp = this->Dot(v);
		T dp2 = SquareLength() * v.SquareLength();
		if ((T)10000 * std::abs(dp * dp - dp2) < 1)
		{
			return dp > 0 ? 1 : -1;
		}
		return 0;
	}

	int LargestAxis() const
	{
		if (y > x)
		{
			return z > y ? 2 : 1;
		}
		return z > x ? 2 : 0;
	}

	TVector3<T> Abs() const
	{
		return TVector3<T>(std::abs(x), std::abs(y), std::abs(z));
	}

	TVector3<T> Min(const TVector3<T>& rhs) const
	{
		return TVector3<T>(std::min(x, rhs.x), std::min(y, rhs.y), std::min(z, rhs.z));
	}

	TVector3<T> Max(const TVector3<T>& rhs) const
	{
		return TVector3<T>(std::max(x, rhs.x), std::max(y, rhs.y), std::max(z, rhs.z));
	}

	static TVector3<T> Lerp(const TVector3<T>& start, const TVector3<T>& end, float t)
	{
		return TVector3<T>(
			start.x * (1.0f - t) + end.x * t,
			start.y * (1.0f - t) + end.y * t,
			start.z * (1.0f - t) + end.z * t);
	}

	static TVector3<T> UnitLerp(const TVector3<T>& start, const TVector3<T>& end, float t)
	{
		return Lerp(start, end, t).Unit();
	}

	static TVector3<T> Slerp(const TVector3<T>& start, const TVector3<T>& end, float t)
	{
		float innerp = start.Dot(end);
		innerp = std::min(std::max(-1.0f, innerp), 1.0f);
		float angle = acosf(innerp) * t;
		TVector3<T> base = (end - start * innerp).Unit();
		return start * cosf(angle) + base * sinf(angle);
	}

	static TVector3<T> InfMin()
	{
		static TVector3<T> inf(-std::numeric_limits<T>::max, -std::numeric_limits<T>::max, -std::numeric_limits<T>::max);
		return inf;
	}

	static TVector3<T> InfMax()
	{
		static TVector3<T> inf(std::numeric_limits<T>::max, std::numeric_limits<T>::max, std::numeric_limits<T>::max);
		return inf;
	}

	static const TVector3<T>& Zero()
	{
		static TVector3<T> zero(0, 0, 0);
		return zero;
	}

	static const TVector3<T>& One()
	{
		static TVector3<T> One((T)1, (T)1, (T)1);
		return One;
	}

	static const TVector3<T>& UnitX()
	{
		static TVector3<T> unitX((T)1, 0, 0);
		return unitX;
	}

	static const TVector3<T>& UnitY()
	{
		static TVector3<T> unitY(0, (T)1, 0);
		return unitY;
	}

	static const TVector3<T>& UnitZ()
	{
		static TVector3<T> unitZ(0, 0, (T)1);
		return unitZ;
	}

	static TVector3<T> Random()
	{
		return TVector3<T>((T)rand() / RAND_MAX, (T)rand() / RAND_MAX, (T)rand() / RAND_MAX);
	}
};

template <typename T>
inline TVector3<T> operator* (float s, const TVector3<T>& vv)
{
	return vv * s;
}

template <typename T>
inline T DotProduct(const TVector3<T> &lhs, const TVector3<T> &rhs)
{
	return lhs.Dot(rhs);
}

template <typename T>
inline TVector3<T> CrossProduct(const TVector3<T>& lhs, const TVector3<T>& rhs)
{
	return lhs.Cross(rhs);
}

template <typename T>
inline T Determinant(const TVector3<T>& a, const TVector3<T>& b, const TVector3<T>& c)
{
	return (a.y * b.z * c.x + a.z * b.x * c.y -
			a.x * b.z * c.y - a.y * b.x * c.z +
			a.x * b.y * c.z - a.z * b.y * c.x);
}


typedef TVector3<float> Vector3d;

static_assert(sizeof(Vector3d) == 12, "sizeof Vector3d is not valid");


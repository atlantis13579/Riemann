#pragma once

#define _USE_MATH_DEFINES

#undef max
#undef min 

#include <math.h>
#include <algorithm>

class Vector3d
{
public:
	union
	{
		struct { float x, y, z; };
		struct { float coords[3]; };
	};

	Vector3d(float _x, float _y, float _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Vector3d(float v[3])
	{
		x = v[0];
		y = v[1];
		z = v[2];
	}

	Vector3d(const Vector3d& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	Vector3d(float v)
	{
		x = v;
		y = v;
		z = v;
	}

	Vector3d()
	{
	}

	inline float    Dot(const Vector3d& v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}

	inline Vector3d Cross(const Vector3d& v) const
	{
		return Vector3d(y * v.z - v.y * z, z * v.x - v.z * x, x * v.y - v.x * y);
	}

	Vector3d Unit() const
	{
		float m = Length();
		return Vector3d(x / m, y / m, z / m);
	}

	void Normalize()
	{
		float m = Length();
		x /= m;
		y /= m;
		z /= m;
	}

	float    Length() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	float    SquareLength() const
	{
		return x * x + y * y + z * z;
	}

	Vector3d& operator=(const Vector3d& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;

	}

	Vector3d operator*(float k) const
	{
		return Vector3d(x * k, y * k, z * k);
	}

	Vector3d operator*(const Vector3d& v) const
	{
		return Vector3d(x * v.x, y * v.y, z * v.z);
	}

	Vector3d operator/(float k) const
	{
		return Vector3d(x / k, y / k, z / k);
	}

	Vector3d operator+(const Vector3d& v) const
	{
		return Vector3d(x + v.x, y + v.y, z + v.z);
	}

	Vector3d operator-(const Vector3d& v) const
	{
		return Vector3d(x - v.x, y - v.y, z - v.z);
	}

	Vector3d operator-() const
	{
		return Vector3d(-x, -y, -z);
	}

	void	 operator+= (const Vector3d& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}

	void	 operator-= (const Vector3d& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	void	 operator*= (const Vector3d& v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	void	 operator/= (const Vector3d& v)
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
	}

	void	 operator*= (float k)
	{
		x *= k;
		y *= k;
		z *= k;
	}

	void	 operator/= (float k)
	{
		x /= k;
		y /= k;
		z /= k;
	}

	bool operator==(const Vector3d& v) const
	{
		return x == v.x && y == v.y && z == v.z;
	}

	bool operator!=(const Vector3d& v) const
	{
		return x != v.x || y != v.y || z != v.z;
	}

	float operator[](int i) const
	{
		return coords[i];
	}

	float& operator[](int i)
	{
		return coords[i];
	}

	int LargestAxis() const
	{
		int i = y > x ? 1 : 0;
		return z > coords[i] ? 2 : i;
	}

	Vector3d Abs() const
	{
		return Vector3d(fabsf(x), fabsf(y), fabsf(z));
	}

	Vector3d Min(const Vector3d& rhs) const
	{
		return Vector3d(std::min(x, rhs.x), std::min(y, rhs.y), std::min(z, rhs.z));
	}

	Vector3d Max(const Vector3d& rhs) const
	{
		return Vector3d(std::max(x, rhs.x), std::max(y, rhs.y), std::max(z, rhs.z));
	}

	static Vector3d Lerp(Vector3d& start, Vector3d& end, float t)
	{
		return (start*(1 - t) + end * t);
	}

	static Vector3d UnitLerp(Vector3d& start, Vector3d& end, float t)
	{
		return Lerp(start, end, t).Unit();
	}

	static Vector3d Slerp(Vector3d& start, Vector3d& end, float t)
	{
		float innerp = start.Dot(end);
		innerp = std::min(std::max(-1.0f, innerp), 1.0f);
		float angle = acosf(innerp) * t;
		Vector3d base = (end - start * innerp).Unit();
		return start * cosf(angle) + base * sinf(angle);
	}

	static Vector3d InfMin()
	{
		static Vector3d inf(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		return inf;
	}

	static Vector3d InfMax()
	{
		static Vector3d inf(FLT_MAX, FLT_MAX, FLT_MAX);
		return inf;
	}

	static const Vector3d& Zero()
	{
		static Vector3d zero(0.0f, 0.0f, 0.0f);
		return zero;
	}

	static const Vector3d& One()
	{
		static Vector3d One(1.0f, 1.0f, 1.0f);
		return One;
	}

	static const Vector3d& UnitX()
	{
		static Vector3d inf(1.0f, 0.0f, 0.0f);
		return inf;
	}

	static const Vector3d& UnitY()
	{
		static Vector3d inf(0.0f, 1.0f, 0.0f);
		return inf;
	}

	static const Vector3d& UnitZ()
	{
		static Vector3d inf(0.0f, 0.0f, 1.0f);
		return inf;
	}

	static Vector3d Random()
	{
		return Vector3d((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
	}
};

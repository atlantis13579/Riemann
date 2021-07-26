#pragma once

#include "Vector3d.h"
#include "Matrix4d.h"
#include "Matrix3d.h"

class Quaternion
{
public:
	union
	{
		struct { float w, x, y, z; };
		struct { float s; Vector3d v; };
	};

	Quaternion()
	{
	}

	Quaternion(const Quaternion& q)
	{
		s = q.s;
		v = q.v;
	}

	Quaternion(float _s, const Vector3d &_v)
	{
		s = _s;
		v = _v;
	}

	Quaternion(float _w, float _x, float _y, float _z)
	{
		w = _w;
		x = _x;
		y = _y;
		z = _z;
	}

	Quaternion(const Vector3d &q, float angle)
	{
		s = cosf(angle / 2);
		v = q * sinf(angle / 2);
	}

	Quaternion(float x, float y, float z)
	{
		Quaternion xrot = Quaternion(Vector3d(1, 0, 0), x);
		Quaternion yrot = Quaternion(Vector3d(0, 1, 0), y);
		Quaternion zrot = Quaternion(Vector3d(0, 0, 1), z);

		*this = zrot * yrot * xrot;
	}

	Quaternion& operator=(const Quaternion& q)
	{
		s = q.s;
		v = q.v;
		return *this;
	}

	Quaternion Conjugate() const
	{
		return Quaternion(s, -v.x, -v.y, -v.z);
	}

	Quaternion Inverse() const
	{
		float m = Magnitude();
		return Quaternion(s / m, Vector3d(-v.x / m, -v.y / m, -v.z / m));
	}

	Quaternion Unit() const
	{
		float m = Magnitude();
		return Quaternion(s / m, Vector3d(v.x / m, v.y / m, v.z / m));
	}

	float Magnitude() const
	{
		return sqrtf(s*s + v.x*v.x + v.y*v.y + v.z*v.z);
	}

	Vector3d ToEuler() const
	{
		return Vector3d(
			atan2f(2 * v.x*s - 2 * v.y*v.z, 1 - 2 * v.x*v.x - 2 * v.z*v.z),
			atan2f(2 * v.y*s - 2 * v.x*v.z, 1 - 2 * v.y*v.y - 2 * v.z*v.z),
			asinf(2 * v.x*v.y + 2 * v.z*s));
	}

	Quaternion operator+(const Quaternion& q)
	{
		return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
	}

	Quaternion operator-(const Quaternion& q)
	{
		return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
	}

	Quaternion operator*(float k) const
	{
		return Quaternion(w * k, v.x * k, v.y * k, v.z * k);
	}

	Quaternion operator*(const Quaternion& q)
	{
		return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,
						y * q.z - z * q.y + x * q.w + w * q.x,
						z * q.x - x * q.z + y * q.w + w * q.y,
						x * q.y - y * q.x + z * q.w + w * q.z);
	}
	
	Quaternion operator/(const Quaternion& q)
	{
		return *this * q.Inverse();
	}

	Vector3d operator*(Vector3d& vec)
	{
		Vector3d t = v.Cross(vec) * 2;
		return vec + t * s + v.Cross(t);
	}

	Quaternion& operator+=(const Quaternion& q)
	{
		w += q.w;
		x += q.x;
		y += q.y;
		z += q.z;
		return *this;
	}

	Quaternion& operator-=(const Quaternion& q)
	{
		w -= q.w;
		x -= q.x;
		y -= q.y;
		z -= q.z;
		return *this;
	}

	Quaternion& operator*=(const Quaternion& q)
	{
		*this = *this * q;
		return *this;
	}

	Quaternion& operator/=(const Quaternion& q)
	{
		*this *= q.Inverse();
		return *this;
	}

	Quaternion Lerp(Quaternion& end, float t) const
	{
		return ((*this) * (1.0f - t) + end * t).Unit();
	}

	Matrix4d ToRotationMatrix4d() const
	{
		float x2 = v.x * v.x;
		float y2 = v.y * v.y;
		float z2 = v.z * v.z;
		float xy = v.x * v.y;
		float xz = v.x * v.z;
		float yz = v.y * v.z;
		float wx = s * v.x;
		float wy = s * v.y;
		float wz = s * v.z;

		return Matrix4d(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}

	Matrix3d ToRotationMatrix() const
	{
		float x2 = v.x * v.x;
		float y2 = v.y * v.y;
		float z2 = v.z * v.z;
		float xy = v.x * v.y;
		float xz = v.x * v.z;
		float yz = v.y * v.z;
		float wx = s * v.x;
		float wy = s * v.y;
		float wz = s * v.z;

		return Matrix3d(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy),
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx),
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2));
	}

	static const Quaternion& Zero()
	{
		static Quaternion zero(0, 0, 0, 0);
		return zero;
	}

	static const Quaternion& UnitW()
	{
		static Quaternion unitW(1, 0, 0, 0);
		return unitW;
	}

	static const Quaternion & UnitX()
	{
		static Quaternion unitX(0, 1, 0, 0);
		return unitX;
	}

	static const Quaternion& UnitY()
	{
		static Quaternion unitY(0, 0, 1, 0);
		return unitY;
	}

	static const Quaternion& UnitZ()
	{
		static Quaternion unitZ(0, 0, 0, 1);
		return unitZ;
	}
};

inline Quaternion operator* (float v, const Quaternion& q)
{
	return q * v;
}

static_assert(sizeof(Quaternion) == 16, "sizeof Quaternion is not valid");
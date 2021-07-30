#pragma once

#include "Vector3d.h"
#include "Matrix4d.h"
#include "Matrix3d.h"

class Quaternion
{
public:
	union
	{
		struct { float x, y, z, w; };
		struct { Vector3d v; float s; };
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

	Quaternion(float _x, float _y, float _z, float _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	Quaternion& operator=(const Quaternion& q)
	{
		s = q.s;
		v = q.v;
		return *this;
	}

	void	FromRotationAxis(const Vector3d& q, float angle)
	{
		s = cosf(angle / 2);
		v = q * sinf(angle / 2);
	}

	Quaternion Conjugate() const
	{
		return Quaternion(-v.x, -v.y, -v.z, s);
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

	void FromEuler(float x, float y, float z)
	{
		Quaternion xrot, yrot, zrot;
		xrot.FromRotationAxis(Vector3d(1, 0, 0), x);
		yrot.FromRotationAxis(Vector3d(0, 1, 0), y);
		zrot.FromRotationAxis(Vector3d(0, 0, 1), z);
		*this = zrot * yrot * xrot;
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
		return Quaternion(x + q.x, y + q.y, z + q.z, w + q.w);
	}

	Quaternion operator-(const Quaternion& q)
	{
		return Quaternion(x - q.x, y - q.y, z - q.z, w - q.w);
	}

	Quaternion operator*(float k) const
	{
		return Quaternion(v.x * k, v.y * k, v.z * k, w * k);
	}

	Quaternion operator*(const Quaternion& q)
	{
		return Quaternion(
						y * q.z - z * q.y + x * q.w + w * q.x,
						z * q.x - x * q.z + y * q.w + w * q.y,
						x * q.y - y * q.x + z * q.w + w * q.z,
						w * q.w - x * q.x - y * q.y - z * q.z);
	}
	
	Quaternion operator/(const Quaternion& q)
	{
		return *this * q.Inverse();
	}

	Vector3d operator*(const Vector3d& vec) const
	{
		Vector3d t = v.Cross(vec) * 2;
		return vec + t * s + v.Cross(t);
	}

	Quaternion& operator+=(const Quaternion& q)
	{
		x += q.x;
		y += q.y;
		z += q.z;
		w += q.w;
		return *this;
	}

	Quaternion& operator-=(const Quaternion& q)
	{
		x -= q.x;
		y -= q.y;
		z -= q.z;
		w -= q.w;
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

	Quaternion Lerp(const Quaternion& end, float t) const
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

	static const Quaternion& One()
	{
		static Quaternion one(0, 0, 0, 1);
		return one;
	}

	static const Quaternion & UnitX()
	{
		static Quaternion unitX(1, 0, 0, 0);
		return unitX;
	}

	static const Quaternion& UnitY()
	{
		static Quaternion unitY(0, 1, 0, 0);
		return unitY;
	}

	static const Quaternion& UnitZ()
	{
		static Quaternion unitZ(0, 0, 1, 0);
		return unitZ;
	}
};

inline Quaternion operator* (float v, const Quaternion& q)
{
	return q * v;
}

static_assert(sizeof(Quaternion) == 16, "sizeof Quaternion is not valid");
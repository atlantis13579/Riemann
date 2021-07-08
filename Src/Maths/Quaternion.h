#pragma once

#include "Vector3d.h"
#include "Matrix4d.h"
#include "Matrix3d.h"

class Quaternion
{
public:
	float s;
	Vector3d v;
	Quaternion()
	{
		s = 1;
		v.x = v.y = v.z = 0;
	}
	Quaternion(float _s, const Vector3d &_v)
	{
		s = _s;
		v = _v;
	}

	Quaternion(float _s, float _x, float _y, float _z)
	{
		s = _s;
		v.x = _x;
		v.y = _y;
		v.z = _z;
	}

	Quaternion(Vector3d q, float angle)
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

	Quaternion Conjugate() const
	{
		return Quaternion(s, -v.x, -v.y, -v.z);
	}

	Quaternion Inverse() const
	{
		float m = Mag();
		return Quaternion(s / m, Vector3d(-v.x / m, -v.y / m, -v.z / m));
	}

	Quaternion Unit() const
	{
		float m = Mag();
		return Quaternion(s / m, Vector3d(v.x / m, v.y / m, v.z / m));
	}

	float Mag() const
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
		return Quaternion(s + q.s, Vector3d(v.x + q.v.x, v.y + q.v.y, v.z + q.v.z));
	}

	Quaternion operator-(Quaternion& q)
	{
		return Quaternion(s - q.s, Vector3d(v.x - q.v.x, v.y - q.v.y, v.z - q.v.z));
	}

	Quaternion operator*(Quaternion& q)
	{
		return Quaternion(s*q.s - v.Dot(q.v), Vector3d(q.v*s + v * q.s + v.Cross(q.v)));
	}

	Vector3d operator*(Vector3d& vec)
	{
		Vector3d t = v.Cross(vec) * 2;
		return vec + t * s + v.Cross(t);
	}

	Quaternion operator*(float k) const
	{
		return Quaternion(s*k, Vector3d(v.x*k, v.y*k, v.z*k));
	}

	Quaternion& operator*=(const Quaternion& q)
	{
		s = q.s;
		v = q.v;
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
};

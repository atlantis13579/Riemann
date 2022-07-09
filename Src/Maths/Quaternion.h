#pragma once

#include <math.h>
#include "Vector3d.h"
#include "Matrix4d.h"
#include "Matrix3d.h"

class Quaternion
{
public:
	float x, y, z, w;

	Quaternion()
	{
	}

	Quaternion(const Quaternion& q)
	{
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
	}

	Quaternion(float _w, const Vector3d &_v)
	{
		w = _w;
		x = _v.x;
		y = _v.y;
		z = _v.z;
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
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
		return *this;
	}

	Quaternion Conjugate() const
	{
		return Quaternion(-x, -y, -z, w);
	}

	Quaternion Inverse() const
	{
		float m = Length();
		return Quaternion(w / m, Vector3d(-x / m, -y / m, -z / m));
	}

	Quaternion Unit() const
	{
		float m = Length();
		return Quaternion(w / m, Vector3d(x / m, y / m, z / m));
	}

	float Dot(const Quaternion& rhs) const
	{
		return w * w + x * x + y * y + z * z;
	}

	float Length() const
	{
		return sqrtf(w*w + x*x + y*y + z*z);
	}

	float SquareLength() const
	{
		return w * w + x * x + y * y + z * z;
	}

	void FromRotationAxis(const Vector3d& axis, float radian)
	{
		*this = Quaternion(cosf(radian / 2), axis * sinf(radian / 2));
	}
	
	void FromTwoAxis(const Vector3d& AxisFrom, const Vector3d& AxisTo)
	{
		Vector3d UnitAxisFrom = AxisFrom.Unit();
		Vector3d UnitAxisTo = AxisTo.Unit();
		Vector3d Axis = UnitAxisFrom.Cross(UnitAxisTo);
		float Angle = acosf(UnitAxisFrom.Dot(UnitAxisTo));
		FromRotationAxis(Axis, Angle);
	}

	void FromEuler(float x, float y, float z)
	{
		Quaternion xrot, yrot, zrot;
		xrot.FromRotationAxis(Vector3d(1, 0, 0), x);
		yrot.FromRotationAxis(Vector3d(0, 1, 0), y);
		zrot.FromRotationAxis(Vector3d(0, 0, 1), z);
		*this = zrot * yrot * xrot;
	}

	// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
	void FromRotationMatrix(const Matrix3d& mat)
	{
		float t;
		Quaternion q;
		if (mat[2][2] < 0)
		{
			if (mat[0][0] > mat[1][1])
			{
				t = 1.0f + mat[0][0] - mat[1][1] - mat[2][2];
				q = Quaternion(t, mat[0][1] + mat[1][0], mat[2][0] + mat[0][2], mat[2][1] - mat[1][2]);
			}
			else
			{
				t = 1.0f - mat[0][0] + mat[1][1] - mat[2][2];
				q = Quaternion(mat[0][1] + mat[1][0], t, mat[1][2] + mat[2][1], mat[0][2] - mat[2][0]);
			}
		}
		else
		{
			if (mat[0][0] < -mat[1][1])
			{
				t = 1.0f - mat[0][0] - mat[1][1] + mat[2][2];
				q = Quaternion(mat[2][0] + mat[0][2], mat[1][2] + mat[2][1], t, mat[1][0] - mat[0][1]);
			}
			else
			{
				t = 1.0f + mat[0][0] + mat[1][1] + mat[2][2];
				q = Quaternion(mat[2][1] - mat[1][2], mat[0][2] - mat[2][0], mat[1][0] - mat[0][1], t);
			}
		}
		q *= 0.5f / sqrtf(t);
		*this = q;
	}

	Vector3d ToEuler() const
	{
		return Vector3d(
			atan2f(2 * x*w - 2 * y*z, 1 - 2 * x*x - 2 * z*z),
			atan2f(2 * y*w - 2 * x*z, 1 - 2 * y*y - 2 * z*z),
			asinf(2 * x*y + 2 * z*w));
	}

	Quaternion operator+(const Quaternion& q) const
	{
		return Quaternion(x + q.x, y + q.y, z + q.z, w + q.w);
	}

	Quaternion operator-(const Quaternion& q) const
	{
		return Quaternion(x - q.x, y - q.y, z - q.z, w - q.w);
	}

	Quaternion operator*(float k) const
	{
		return Quaternion(x * k, y * k, z * k, w * k);
	}

	Quaternion operator*(const Quaternion& q) const
	{
		return Quaternion(
						y * q.z - z * q.y + x * q.w + w * q.x,
						z * q.x - x * q.z + y * q.w + w * q.y,
						x * q.y - y * q.x + z * q.w + w * q.z,
						w * q.w - x * q.x - y * q.y - z * q.z);
	}
	
	Quaternion operator/(const Quaternion& q) const
	{
		return *this * q.Inverse();
	}

	Vector3d operator*(const Vector3d& vec) const
	{
		Vector3d v(x, y, z);
		Vector3d t = v.Cross(vec) * 2;
		return vec + t * w + v.Cross(t);
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

	Quaternion& operator*=(float k)
	{
		x *= k;
		y *= k;
		z *= k;
		w *= k;
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

	static Quaternion Lerp(const Quaternion& start, const Quaternion& end, float t)
	{
		return start * (1.0f - t) + end * t;
	}
 
	static Quaternion Nlerp(const Quaternion& start, const Quaternion& end, float t)
	{
		return Lerp(start, end, t).Unit();
	}

	static Quaternion Slerp(const Quaternion& start, const Quaternion& end, float t)
	{
		auto sinx_over_x = [](float x)
		{
			if (x * x < 1e-12f)
			{
				return 1.0f;
			}
			else
			{
				return sinf(x) / x;
			}
		};
		
		Quaternion q1 = start - end;
		float lengthD = sqrtf(q1.Dot(q1));

		Quaternion q2 = start + end;
		float lengthS = sqrtf(q2.Dot(q2));

		float a = 2.0f * atan2f(lengthD, lengthS);
		float s = 1.0f - t;
		Quaternion q = start * (sinx_over_x(s * a) / sinx_over_x(a) * s) + end * (sinx_over_x(t * a) / sinx_over_x(a) * t);
		return q.Unit();
	}
 
    // Comnining the slerp and nlerp, use nlerp when the angle is small, otherwise slerp
 	static Quaternion NSlerp(const Quaternion& start, const Quaternion& end, float t)
    {
        float dp = start.Dot(end);
        if (dp >= 0.99f)
        {
            return Nlerp(start, end, t);
        }
        return Slerp(start, end, t);
    }

	Matrix4d ToRotationMatrix4d() const
	{
		float x2 = x * x;
		float y2 = y * y;
		float z2 = z * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;

		return Matrix4d(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}

	Matrix3d ToRotationMatrix() const
	{
		float x2 = x * x;
		float y2 = y * y;
		float z2 = z * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;

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

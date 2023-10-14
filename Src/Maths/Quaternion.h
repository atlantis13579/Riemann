#pragma once

#include <math.h>
#include "Maths.h"
#include "Vector3.h"
#include "Matrix4.h"
#include "Matrix3.h"

class Quaternion
{
public:
	float x, y, z, w;

	Quaternion()
	{
	}

	explicit Quaternion(float _w, const Vector3 &_v)
	{
		w = _w;
		x = _v.x;
		y = _v.y;
		z = _v.z;
	}

	explicit Quaternion(float _x, float _y, float _z, float _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}
	
	Quaternion(const Quaternion& q)
	{
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
	}

	inline Quaternion& operator=(const Quaternion& q)
	{
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
		return *this;
	}

	inline Quaternion Conjugate() const
	{
		return Quaternion(-x, -y, -z, w);
	}

	inline Quaternion Inverse() const
	{
		float m = Length();
		return Quaternion(w / m, Vector3(-x / m, -y / m, -z / m));
	}

	inline Quaternion Unit() const
	{
		float m = Length();
		return Quaternion(w / m, Vector3(x / m, y / m, z / m));
	}

	inline float Dot(const Quaternion& rhs) const
	{
		return w * w + x * x + y * y + z * z;
	}

	inline float Length() const
	{
		return sqrtf(w*w + x*x + y*y + z*z);
	}

	inline float SquareLength() const
	{
		return w * w + x * x + y * y + z * z;
	}

	Quaternion& FromRotateX(float radian)
	{
		FromRotationAxis(Vector3::UnitX(), radian);
		return *this;
	}

	Quaternion& FromRotateY(float radian)
	{
		FromRotationAxis(Vector3::UnitY(), radian);
		return *this;
	}

	Quaternion& FromRotateZ(float radian)
	{
		FromRotationAxis(Vector3::UnitZ(), radian);
		return *this;
	}

	Quaternion& FromRotationAxis(const Vector3& axis, float radian)
	{
		*this = Quaternion(cosf(radian / 2), axis * sinf(radian / 2));
		return *this;
	}
	
	Quaternion& FromTwoAxis(const Vector3& AxisFrom, const Vector3& AxisTo)
	{
		Vector3 UnitAxisFrom = AxisFrom.Unit();
		Vector3 UnitAxisTo = AxisTo.Unit();
		Vector3 Axis = UnitAxisFrom.Cross(UnitAxisTo);
		float Angle = acosf(UnitAxisFrom.Dot(UnitAxisTo));
		return FromRotationAxis(Axis, Angle);
	}

	Quaternion& FromEuler(float yaw, float roll, float pitch)
	{
		float sp = sinf(pitch * 0.5f);
		float sy = sinf(yaw * 0.5f);
		float sr = sinf(roll * 0.5f);

		float cp = cosf(pitch * 0.5f);
		float cy = cosf(yaw * 0.5f);
		float cr = cosf(roll * 0.5f);

		x = cy * cr * sp + sy * sr * cp;
		y = sy * cr * cp - cy * sr * sp;
		z = cy * sr * cp - sy * cr * sp;
		w = cy * cr * cp + sy * sr * sp;
		return *this;
	}

	Vector3 ToEuler(float& yaw, float& roll, float& pitch) const
	{
		float sqx = x * x;
		float sqy = y * y;
		float sqz = z * z;
		float sqw = w * w;

		float unit = sqx + sqy + sqz + sqw;
		float test = w * x - y * z;
		if (test > 0.4999f * unit)		//  0.4999f OR  0.5f - EPSILON
		{
			// Singularity at north pole
			yaw = 2.0f * atan2f(y, w);		// Yaw
			pitch = PI_OVER_2;				// Pitch
			roll = 0;						// Roll
		}
		else if (test < -0.4999f * unit)	// -0.4999f OR -0.5f + EPSILON
		{
			// Singularity at south pole
			yaw = 2.0f * atan2f(y, w);		// Yaw
			pitch = -PI_OVER_2;				// Pitch
			roll = 0;						// Roll
		}
		else
		{
			yaw = atan2f(2.0f * (x * z + w * y), sqz + sqw - sqx - sqy);	// Yaw
			pitch = asinf(2.0f * test / unit);								// Pitch
			roll = atan2f(2.0f * (x * y + w * z), sqy + sqw - sqx - sqz);	// Roll
		}
	}

	// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
	Quaternion& FromRotationMatrix(const Matrix3& mat)
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
		return *this;
	}

	// https://www.intel.com/content/dam/develop/external/us/en/documents/293748-142817.pdf
	Matrix4 ToRotationMatrix4() const
	{
		const float x2 = x * x;
		const float y2 = y * y;
		const float z2 = z * z;
		const float xy = x * y;
		const float xz = x * z;
		const float yz = y * z;
		const float wx = w * x;
		const float wy = w * y;
		const float wz = w * z;

		return Matrix4(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}

	// https://www.intel.com/content/dam/develop/external/us/en/documents/293748-142817.pdf
	Matrix3 ToRotationMatrix3() const
	{
		const float x2 = x * x;
		const float y2 = y * y;
		const float z2 = z * z;
		const float xy = x * y;
		const float xz = x * z;
		const float yz = y * z;
		const float wx = w * x;
		const float wy = w * y;
		const float wz = w * z;

		return Matrix3(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy),
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx),
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2));
	}

	inline Quaternion operator-() const
	{
		return Quaternion(-x, -y, -z, -w);
	}

	inline Quaternion operator+(const Quaternion& q) const
	{
		return Quaternion(x + q.x, y + q.y, z + q.z, w + q.w);
	}

	inline Quaternion operator-(const Quaternion& q) const
	{
		return Quaternion(x - q.x, y - q.y, z - q.z, w - q.w);
	}

	inline Quaternion operator*(float k) const
	{
		return Quaternion(x * k, y * k, z * k, w * k);
	}

	inline Quaternion operator*(const Quaternion& q) const
	{
		return Quaternion(
						y * q.z - z * q.y + x * q.w + w * q.x,
						z * q.x - x * q.z + y * q.w + w * q.y,
						x * q.y - y * q.x + z * q.w + w * q.z,
						w * q.w - x * q.x - y * q.y - z * q.z);
	}
	
	inline Quaternion operator/(const Quaternion& q) const
	{
		return *this * q.Inverse();
	}

	inline Vector3 operator*(const Vector3& vec) const
	{
		Vector3 v(x, y, z);
		Vector3 t = v.Cross(vec) * 2;
		return vec + t * w + v.Cross(t);
	}

	inline Quaternion& operator+=(const Quaternion& q)
	{
		x += q.x;
		y += q.y;
		z += q.z;
		w += q.w;
		return *this;
	}

	inline Quaternion& operator-=(const Quaternion& q)
	{
		x -= q.x;
		y -= q.y;
		z -= q.z;
		w -= q.w;
		return *this;
	}

	inline Quaternion& operator*=(float k)
	{
		x *= k;
		y *= k;
		z *= k;
		w *= k;
		return *this;
	}

	inline Quaternion& operator*=(const Quaternion& q)
	{
		*this = *this * q;
		return *this;
	}

	inline Quaternion& operator/=(const Quaternion& q)
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
 
    // Combining the slerp and nlerp, use nlerp when the angle is small, otherwise slerp
 	static Quaternion NSlerp(const Quaternion& start, const Quaternion& end, float t)
    {
        float dp = start.Dot(end);
        if (dp >= 0.99f)
        {
            return Nlerp(start, end, t);
        }
        return Slerp(start, end, t);
    }

	static Quaternion Zero()
	{
		return Quaternion(0, 0, 0, 0);
	}

	static Quaternion One()
	{
		return Quaternion(0, 0, 0, 1);
	}

	static Quaternion UnitX()
	{
		return Quaternion(1, 0, 0, 0);
	}

	static Quaternion UnitY()
	{
		return Quaternion(0, 1, 0, 0);
	}

	static Quaternion UnitZ()
	{
		return Quaternion(0, 0, 1, 0);
	}
};

inline Quaternion operator* (float v, const Quaternion& q)
{
	return q * v;
}

static_assert(sizeof(Quaternion) == 16, "sizeof Quaternion is not valid");

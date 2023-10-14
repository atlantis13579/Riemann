#pragma once

#include <string.h>
#include "Maths.h"
#include "Vector3.h"

class Matrix3
{
public:
	float mat[3][3];

	explicit Matrix3()
	{
	}

	explicit Matrix3(float m[3][3])
	{
		memcpy(mat, m, sizeof(mat));
	}

	explicit Matrix3(float a00, float a01, float a02,
					 float a10, float a11, float a12,
					 float a20, float a21, float a22)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22;
	}

	explicit Matrix3(float a00, float a11, float a22)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = a00; mat[1][1] = a11; mat[2][2] = a22;
	}
	
	explicit Matrix3(const Vector3& diag)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = diag.x; mat[1][1] = diag.y; mat[2][2] = diag.z;
	}

	explicit Matrix3(const Vector3& c0, const Vector3& c1, const Vector3& c2, bool row_wise)
	{
		if (row_wise)
		{
			mat[0][0] = c0.x; mat[0][1] = c0.y; mat[0][2] = c0.z;
			mat[1][0] = c1.x; mat[1][1] = c1.y; mat[1][2] = c1.z;
			mat[2][0] = c2.x; mat[2][1] = c2.y; mat[2][2] = c2.z;
		}
		else
		{
			mat[0][0] = c0.x; mat[0][1] = c1.x; mat[0][2] = c2.x;
			mat[1][0] = c0.y; mat[1][1] = c1.y; mat[1][2] = c2.y;
			mat[2][0] = c0.z; mat[2][1] = c1.z; mat[2][2] = c2.z;
		}
	}
	
	inline Matrix3(const Matrix3& m)
	{
		memcpy(mat, m.mat, sizeof(mat));
	}

	inline void LoadZero()
	{
		memset(mat, 0, sizeof(mat));
	}
	
	inline void LoadIdentiry()
	{
		mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0;
		mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0;
		mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1;
	}

	inline Matrix3 Transpose() const
	{
		return Matrix3(mat[0][0], mat[1][0], mat[2][0],
			mat[0][1], mat[1][1], mat[2][1],
			mat[0][2], mat[1][2], mat[2][2]);
	}

	inline float Determinant() const
	{
		return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
			- mat[0][1] * (mat[2][2] * mat[1][0] - mat[1][2] * mat[2][0])
			+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	}

	inline Matrix3 operator+(const Matrix3& mm) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] + mm.mat[i][j];
		return Matrix3(m);
	}

	inline Matrix3 operator-(const Matrix3& mm) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] - mm.mat[i][j];
		return Matrix3(m);
	}

	inline Matrix3 operator-() const
	{
		return *this * -1.0f;
	}

	inline Matrix3 operator*(const Matrix3& mm) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][0] * mm.mat[0][j] + mat[i][1] * mm.mat[1][j] + mat[i][2] * mm.mat[2][j];
		return Matrix3(m);
	}

	inline Matrix3 operator*(float k) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] * k;
		return Matrix3(m);
	}

	inline Vector3 operator*(const Vector3& vv) const
	{
		Vector3 v;
		v.x = mat[0][0] * vv.x + mat[0][1] * vv.y + mat[0][2] * vv.z;
		v.y = mat[1][0] * vv.x + mat[1][1] * vv.y + mat[1][2] * vv.z;
		v.z = mat[2][0] * vv.x + mat[2][1] * vv.y + mat[2][2] * vv.z;
		return v;
	}

	inline Matrix3& operator+=(const Matrix3& mm)
	{
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mat[i][j] += mm[i][j];
		return *this;
	}

	inline Matrix3& operator-=(const Matrix3& mm)
	{
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mat[i][j] -= mm[i][j];
		return *this;
	}

	inline Matrix3& operator*=(const Matrix3& mm)
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][0] * mm.mat[0][j] + mat[i][1] * mm.mat[1][j] + mat[i][2] * mm.mat[2][j];
		memcpy(mat, m, sizeof(mat));
	}
	
	inline Matrix3& operator*=(float k)
	{
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mat[i][j] *= k;
		return *this;
	}
	
	inline Matrix3& operator/=(float k)
	{
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mat[i][j] /= k;
		return *this;
	}

	inline Matrix3& operator=(const Matrix3& rhs)
	{
		memcpy(mat, rhs.mat, sizeof(mat));
		return *this;
	}

	inline float operator()(int i, int j) const
	{
		return mat[i][j];
	}

	inline float& operator()(int i, int j)
	{
		return mat[i][j];
	}

	inline const Vector3& operator[](int i) const
	{
		const Vector3 *row = static_cast<const Vector3*>((const void*)mat);
		return row[i];
	}

	inline Vector3& operator[](int i)
	{
		Vector3 *row = static_cast<Vector3*>((void*)mat);
		return row[i];
	}

	inline const float* Data() const
	{
		return mat[0];
	}

	inline float* Data()
	{
		return mat[0];
	}

	inline const Vector3& GetRow(int i) const
	{
		const Vector3 *row = static_cast<const Vector3*>((const void*)mat);
		return row[i];
	}

	inline Vector3& GetRow(int i)
	{
		Vector3 *row = static_cast<Vector3*>((void*)mat);
		return row[i];
	}

	inline Vector3 GetCol(int i) const
	{
		return Vector3(mat[0][i], mat[1][i], mat[2][i]);
	}

	inline float	Trace() const
	{
		return mat[0][0] + mat[1][1] + mat[2][2];
	}

	float		L1Norm() const
	{
		float l1_norm = 0.0f;
		const float* p = (const float *)this;
		for (int i = 0; i < 9; ++i)
			l1_norm += fabsf(p[i]);
		return l1_norm;
	}

	float		L2Norm() const
	{
		float l2_norm = 0.0f;
		const float* p = (const float*)this;
		for (int i = 0; i < 9; ++i)
			l2_norm += p[i] * p[i];
		return l2_norm;
	}

	bool		Invertible() const
	{
		float d = Determinant();
		return fabsf(d) > 1e-6;
	}

	Matrix3		Inverse() const
	{
		float d = Determinant();
		// Warning!!! Assume d != 0
		d = 1.0f / d;
		return Matrix3((mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) * d, -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) * d, (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * d,
			-(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) * d, (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * d, -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) * d,
			(mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) * d, -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) * d, (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) * d);
	}
	
	Matrix3		PseudoInverse() const
	{
		Vector3 S;
		Matrix3 U, V;
		SingularValueDecompose(U, S, V);
		return V * Matrix3(S) * U.Transpose();
	}

	// Solve AX = B
	bool		Solve(const Vector3& b, Vector3& x)
	{
		float d = Determinant();
		if (fabsf(d) < 1e-6)
		{
			return false;
		}
		const Vector3 *row = static_cast<const Vector3*>((const void*)mat);
		x.x = b.Dot(row[1].Cross(row[2])) / d;
		x.y = row[0].Dot(b.Cross(row[2])) / d;
		x.z = row[0].Dot(row[1].Cross(b)) / d;
		return true;
	}

	static Matrix3 Skew(Vector3& v)
	{
		return Matrix3(0, -v.z, v.y,
						v.z, 0, -v.x,
						-v.y, v.x, 0);
	}

	// Generate Rotate Matrix
	Matrix3& LoadRotateX(float angle)
	{
		float c, s;
		c = cosf(angle);
		s = sinf(angle);
		mat[0][0] = 1.0f;	mat[0][1] = 0.0f;	mat[0][2] = 0.0f;
		mat[1][0] = 1.0f;	mat[1][1] = c;		mat[1][2] = s;
		mat[2][0] = 1.0f;	mat[2][1] = -s; 	mat[2][2] = c;
		return *this;
	}
	Matrix3& LoadRotateY(float angle)
	{
		float c, s;
		c = cosf(angle);
		s = sinf(angle);
		mat[0][0] = c;		mat[0][1] = 0.0f;	mat[0][2] = -s;
		mat[1][0] = 0.0f;	mat[1][1] = 1.0f;	mat[1][2] = 0.0f;
		mat[2][0] = s;		mat[2][1] = 0.0f;	mat[2][2] = c;
		return *this;
	}
	Matrix3& LoadRotateZ(float angle)
	{
		float c, s;
		c = cosf(angle);
		s = sinf(angle);
		mat[0][0] = c;		mat[0][1] = s; 		mat[0][2] = 0.0f;
		mat[1][0] = -s; 	mat[1][1] = c;		mat[1][2] = 0.0f;
		mat[2][0] = 0.0f;	mat[2][1] = 0.0f;	mat[2][2] = 1.0f;
		return *this;
	}
	Matrix3& Load2DOrthogonalTransform(float dx, float dy, float angle) {
		float c, s;
		c = cosf(angle);
		s = sinf(angle);
		mat[0][0] = c;		mat[0][1] = -s; 	mat[0][2] = dx;
		mat[1][0] = s; 		mat[1][1] = c;		mat[1][2] = dy;
		mat[2][0] = 0.0f;	mat[2][1] = 0.0f;	mat[2][2] = 1.0f;
		return *this;
	}

	Matrix3& FromAxisAngle(const Vector3& Axis, float Radian)
	{
		float fCos = cosf(Radian);
		float fSin = sinf(Radian);
		float fOneMinusCos = 1.0f - fCos;
		float fX2 = Axis.x * Axis.x;
		float fY2 = Axis.y * Axis.y;
		float fZ2 = Axis.z * Axis.z;
		float fXYM = Axis.x * Axis.y * fOneMinusCos;
		float fXZM = Axis.x * Axis.z * fOneMinusCos;
		float fYZM = Axis.y * Axis.z * fOneMinusCos;
		float fXSin = Axis.x * fSin;
		float fYSin = Axis.y * fSin;
		float fZSin = Axis.z * fSin;

		mat[0][0] = fX2 * fOneMinusCos + fCos;
		mat[0][1] = fXYM - fZSin;
		mat[0][2] = fXZM + fYSin;
		mat[1][0] = fXYM + fZSin;
		mat[1][1] = fY2 * fOneMinusCos + fCos;
		mat[1][2] = fYZM - fXSin;
		mat[2][0] = fXZM - fYSin;
		mat[2][1] = fYZM + fXSin;
		mat[2][2] = fZ2 * fOneMinusCos + fCos;
		return *this;
	}

	Matrix3& FromTwoAxis(const Vector3& AxisFrom, const Vector3& AxisTo)
	{
		Vector3 UnitAxisFrom = AxisFrom.Unit();
		Vector3 UnitAxisTo = AxisTo.Unit();
		Vector3 Axis = UnitAxisFrom.Cross(UnitAxisTo);
		float Angle = acosf(UnitAxisFrom.Dot(UnitAxisTo));
		return FromAxisAngle(Axis, Angle);
	}

	float		ToAxisAngle(Vector3& Axis) const
	{
		// Let (x,y,z) be the unit-length axis and let A be an angle of rotation.
		// The rotation matrix is R = I + sin(A)*P + (1-cos(A))*P^2 where
		// I is the identity and
		//
		//       +-        -+
		//   P = |  0 -z +y |
		//       | +z  0 -x |
		//       | -y +x  0 |
		//       +-        -+
		//
		// If A > 0, R represents a counterclockwise rotation about the axis in
		// the sense of looking from the tip of the axis vector towards the
		// origin.  Some algebra will show that
		//
		//   cos(A) = (trace(R)-1)/2  and  R - R^t = 2*sin(A)*P
		//
		// In the event that A = pi, R-R^t = 0 which prevents us from extracting
		// the axis through P.  Instead note that R = I+2*P^2 when A = pi, so
		// P^2 = (R-I)/2.  The diagonal entries of P^2 are x^2-1, y^2-1, and
		// z^2-1.  We can solve these for axis (x,y,z).  Because the angle is pi,
		// it does not matter which sign you choose on the square roots.

		float tr = mat[0][0] + mat[1][1] + mat[2][2];
		float c = 0.5f * (tr - 1.0f);
		float Angle = acosf(c);  // in [0,PI]

		if (Angle > 0.0f)
		{
			if (Angle < PI)
			{
				Axis.x = mat[2][1] - mat[1][2];
				Axis.y = mat[0][2] - mat[2][0];
				Axis.z = mat[1][0] - mat[0][1];
				Axis.Normalize();
			}
			else
			{
				if (mat[0][0] >= mat[1][1])
				{
					// r00 >= r11
					if (mat[0][0] >= mat[2][2])
					{
						// r00 is maximum diagonal term
						Axis.x = 0.5f * sqrtf(mat[0][0] - mat[1][1] - mat[2][2] + 1.0f);
						float inv = 0.5f / Axis.x;
						Axis.y = inv * mat[0][1];
						Axis.z = inv * mat[0][2];
					}
					else
					{
						// r22 is maximum diagonal term
						Axis.z = 0.5f * sqrtf(mat[2][2] - mat[0][0] - mat[1][1] + 1.0f);
						float inv = 0.5f / Axis.z;
						Axis.x = inv * mat[0][2];
						Axis.y = inv * mat[1][2];
					}
				}
				else
				{
					// r11 > r00
					if (mat[1][1] >= mat[2][2])
					{
						// r11 is maximum diagonal term
						Axis.y = 0.5f * sqrtf(mat[1][1] - mat[0][0] - mat[2][2] + 1.0f);
						float inv = 0.5f / Axis.y;
						Axis.x = inv * mat[0][1];
						Axis.z = inv * mat[1][2];
					}
					else
					{
						// r22 is maximum diagonal term
						Axis.z = 0.5f * sqrtf(mat[2][2] - mat[0][0] - mat[1][1] + 1.0f);
						float inv = 0.5f / Axis.z;
						Axis.x = inv * mat[0][2];
						Axis.y = inv * mat[1][2];
					}
				}
			}
		}
		else
		{
			// The angle is 0 and the matrix is the identity.  Any axis will
			// work, so just use the x-axis.
			Axis.x = 1.0f;
			Axis.y = 0.0f;
			Axis.z = 0.0f;
		}

		return Angle;
	}
	void		FromEulerAngles(float Yaw, float Pitch, float Roll)
	{
		const float sy = sinf(Yaw);
		const float cy = cosf(Yaw);
		const float sp = sinf(Pitch);
		const float cp = cosf(Pitch);
		const float sr = sinf(Roll);
		const float cr = cosf(Roll);

		mat[0][0] = (sr * sp * sy + cr * cy);
		mat[0][1] = (sr * cp);
		mat[0][2] = (sr * sp * cy - cr * sy);
		mat[1][0] = (cr * sp * sy - sr * cy);
		mat[1][1] = (cr * cp);
		mat[1][2] = (cr * sp * cy + sr * sy);
		mat[2][0] = (cp * sy);
		mat[2][1] = (-sp);
		mat[2][2] = (cp * cy);
	}
	bool		ToEulerAngles(float& Yaw, float& Pitch, float& Roll) const
	{
		// rot =  cy*cz          -cy*sz           sy
		//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
		//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

		Pitch = asinf(mat[0][2]);
		if (Pitch < PI_OVER_2)
		{
			if (Pitch > -PI_OVER_2)
			{
				Yaw = atan2f(-mat[1][2], mat[2][2]);
				Roll = atan2f(-mat[0][1], mat[0][0]);
				return true;
			}
			else
			{
				// Not a unique solution.
				float fRmY = atan2f(mat[1][0], mat[1][1]);
				Roll = float(0.0);  // any angle works
				Yaw = Roll - fRmY;
				return false;
			}
		}
		else
		{
			// Not a unique solution.
			float fRpY = atan2f(mat[1][0], mat[1][1]);
			Roll = float(0.0);  // any angle works
			Yaw = fRpY - Roll;
			return false;
		}
	}

	// static
	static Matrix3	ComputeCovarianceMatrix(const Vector3* v, int n)
	{
		Vector3 mean = Vector3::Zero();
		for (int i = 0; i < n; ++i)
		{
			mean += v[i];
		}
		mean *= (1.0f / n);

		float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
		for (int i = 0; i < n; ++i)
		{
			c00 += (v[i].x - mean.x) * (v[i].x - mean.x);
			c01 += (v[i].x - mean.x) * (v[i].y - mean.y);
			c02 += (v[i].x - mean.x) * (v[i].z - mean.z);
			c11 += (v[i].y - mean.y) * (v[i].y - mean.y);
			c12 += (v[i].y - mean.y) * (v[i].z - mean.z);
			c22 += (v[i].z - mean.z) * (v[i].z - mean.z);
		}

		Matrix3 covariance_matrix = Matrix3(c00, c01, c11, c01, c11, c12, c02, c12, c22);
		covariance_matrix *= (1.0f / n);
		return covariance_matrix;
	}

	float OneNorm() const
	{
		float norm = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			float sum = fabsf(mat[0][i]) + fabsf(mat[1][i]) + fabsf(mat[2][i]);
			if (sum > norm)
				norm = sum;
		}
		return norm;
	}

	float InfNorm() const
	{
		float norm = 0.0;
		for (int i = 0; i < 3; i++)
		{
			float sum = fabsf(mat[i][0]) + fabsf(mat[i][1]) + fabsf(mat[i][2]);
			if (sum > norm)
				norm = sum;
		}
		return norm;
	}
	
	// A = U * S * V^T

	void		Bidiagonalize(Matrix3& L, Matrix3& R)
	{
		float v[3], w[3];
		float sign, t1, t2;
		bool bIdentity;

		// map first column to (*,0,0)
		float l = sqrtf(mat[0][0] * mat[0][0] + mat[1][0] * mat[1][0] + mat[2][0] * mat[2][0]);
		if (l > 0.0)
		{
			sign = (mat[0][0] > 0.0f ? 1.0f : -1.0f);
			t1 = mat[0][0] + sign * l;
			v[1] = mat[1][0] / t1;
			v[2] = mat[2][0] / t1;

			t2 = -2.0f / (1.0f + v[1] * v[1] + v[2] * v[2]);
			w[0] = t2 * (mat[0][0] + mat[1][0] * v[1] + mat[2][0] * v[2]);
			w[1] = t2 * (mat[0][1] + mat[1][1] * v[1] + mat[2][1] * v[2]);
			w[2] = t2 * (mat[0][2] + mat[1][2] * v[1] + mat[2][2] * v[2]);
			mat[0][0] += w[0];
			mat[0][1] += w[1];
			mat[0][2] += w[2];
			mat[1][1] += v[1] * w[1];
			mat[1][2] += v[1] * w[2];
			mat[2][1] += v[2] * w[1];
			mat[2][2] += v[2] * w[2];

			L[0][0] = 1.0f + t2;
			L[0][1] = L[1][0] = t2 * v[1];
			L[0][2] = L[2][0] = t2 * v[2];
			L[1][1] = 1.0f + t2 * v[1] * v[1];
			L[1][2] = L[2][1] = t2 * v[1] * v[2];
			L[2][2] = 1.0f + t2 * v[2] * v[2];
			bIdentity = false;
		}
		else
		{
			L = Matrix3::Identity();
			bIdentity = true;
		}

		// map first row to (*,*,0)
		l = sqrtf(mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2]);
		if (l > 0.0f)
		{
			sign = (mat[0][1] > 0.0f ? 1.0f : -1.0f);
			t1 = mat[0][1] + sign * l;
			v[2] = mat[0][2] / t1;

			t2 = -2.0f / (1.0f + v[2] * v[2]);
			w[0] = t2 * (mat[0][1] + mat[0][2] * v[2]);
			w[1] = t2 * (mat[1][1] + mat[1][2] * v[2]);
			w[2] = t2 * (mat[2][1] + mat[2][2] * v[2]);
			mat[0][1] += w[0];
			mat[1][1] += w[1];
			mat[1][2] += w[1] * v[2];
			mat[2][1] += w[2];
			mat[2][2] += w[2] * v[2];

			R[0][0] = 1.0f;
			R[0][1] = R[1][0] = 0.0f;
			R[0][2] = R[2][0] = 0.0f;
			R[1][1] = 1.0f + t2;
			R[1][2] = R[2][1] = t2 * v[2];
			R[2][2] = 1.0f + t2 * v[2] * v[2];
		}
		else
		{
			R = Matrix3::Identity();
		}

		// map second column to (*,*,0)
		l = sqrtf(mat[1][1] * mat[1][1] + mat[2][1] * mat[2][1]);
		if (l > 0.0f)
		{
			sign = (mat[1][1] > 0.0f ? 1.0f : -1.0f);
			t1 = mat[1][1] + sign * l;
			v[2] = mat[2][1] / t1;

			t2 = -2.0f / (1.0f + v[2] * v[2]);
			w[1] = t2 * (mat[1][1] + mat[2][1] * v[2]);
			w[2] = t2 * (mat[1][2] + mat[2][2] * v[2]);
			mat[1][1] += w[1];
			mat[1][2] += w[2];
			mat[2][2] += v[2] * w[2];

			float fA = 1.0f + t2;
			float fB = t2 * v[2];
			float fC = 1.0f + fB * v[2];

			if (bIdentity)
			{
				L[0][0] = 1.0f;
				L[0][1] = L[1][0] = 0.0;
				L[0][2] = L[2][0] = 0.0;
				L[1][1] = fA;
				L[1][2] = L[2][1] = fB;
				L[2][2] = fC;
			}
			else
			{
				for (int iRow = 0; iRow < 3; ++iRow)
				{
					float fTmp0 = L[iRow][1];
					float fTmp1 = L[iRow][2];
					L[iRow][1] = fA * fTmp0 + fB * fTmp1;
					L[iRow][2] = fB * fTmp0 + fC * fTmp1;
				}
			}
		}
	}

	void GolubKahanStep(Matrix3& L, Matrix3& R)
	{
		float t11 = mat[0][1] * mat[0][1] + mat[1][1] * mat[1][1];
		float t22 = mat[1][2] * mat[1][2] + mat[2][2] * mat[2][2];
		float t12 = mat[1][1] * mat[1][2];
		float tr = t11 + t22;
		float diff = t11 - t22;
		float discriminant = sqrtf(diff * diff + 4.0f * t12 * t12);
		float root1 = 0.5f * (tr + discriminant);
		float root2 = 0.5f * (tr - discriminant);

		// adjust right
		float y = mat[0][0] - (fabsf(root1 - t22) <= fabsf(root2 - t22) ? root1 : root2);
		float z = mat[0][1];
		float inv = InvSqrt(y * y + z * z);
		float s = z * inv;
		float c = -y * inv;

		float t0 = mat[0][0];
		float t1 = mat[0][1];
		mat[0][0] = c * t0 - s * t1;
		mat[0][1] = s * t0 + c * t1;
		mat[1][0] = -s * mat[1][1];
		mat[1][1] *= c;

		for (int i = 0; i < 3; ++i)
		{
			t0 = R[0][i];
			t1 = R[1][i];
			R[0][i] = c * t0 - s * t1;
			R[1][i] = s * t0 + c * t1;
		}

		// adjust left
		y = mat[0][0];
		z = mat[1][0];
		inv = InvSqrt(y * y + z * z);
		s = z * inv;
		c = -y * inv;

		mat[0][0] = c * mat[0][0] - s * mat[1][0];
		t0 = mat[0][1];
		t1 = mat[1][1];
		mat[0][1] = c * t0 - s * t1;
		mat[1][1] = s * t0 + c * t1;
		mat[0][2] = -s * mat[1][2];
		mat[1][2] *= c;

		for (int i = 0; i < 3; ++i)
		{
			t0 = L[i][0];
			t1 = L[i][1];
			L[i][0] = c * t0 - s * t1;
			L[i][1] = s * t0 + c * t1;
		}

		// adjust right
		y = mat[0][1];
		z = mat[0][2];
		inv = InvSqrt(y * y + z * z);
		s = z * inv;
		c = -y * inv;

		mat[0][1] = c * mat[0][1] - s * mat[0][2];
		t0 = mat[1][1];
		t1 = mat[1][2];
		mat[1][1] = c * t0 - s * t1;
		mat[1][2] = s * t0 + c * t1;
		mat[2][1] = -s * mat[2][2];
		mat[2][2] *= c;

		for (int i = 0; i < 3; ++i)
		{
			t0 = R[1][i];
			t1 = R[2][i];
			R[1][i] = c * t0 - s * t1;
			R[2][i] = s * t0 + c * t1;
		}

		// adjust left
		y = mat[1][1];
		z = mat[2][1];
		inv = InvSqrt(y * y + z * z);
		s = z * inv;
		c = -y * inv;

		mat[1][1] = c * mat[1][1] - s * mat[2][1];
		t0 = mat[1][2];
		t1 = mat[2][2];
		mat[1][2] = c * t0 - s * t1;
		mat[2][2] = s * t0 + c * t1;

		for (int i = 0; i < 3; ++i)
		{
			t0 = L[i][1];
			t1 = L[i][2];
			L[i][1] = c * t0 - s * t1;
			L[i][2] = s * t0 + c * t1;
		}
	}

	void		SingularValueDecompose(Matrix3& U, Vector3& S, Matrix3& V) const
	{
		int i, j;
		const float epsilon = 1e-04f;
		const int   max_iterations = 32;

		Matrix3 A = *this;
		A.Bidiagonalize(U, V);

		for (uint32_t i = 0; i < max_iterations; i++)
		{
			bool bTest1 = fabsf(A[0][1]) <= epsilon * (fabsf(A[0][0]) + fabsf(A[1][1]));
			bool bTest2 = fabsf(A[1][2]) <= epsilon * (fabsf(A[1][1]) + fabsf(A[2][2]));
			if (bTest1)
			{
				if (bTest2)
				{
					S[0] = A[0][0];
					S[1] = A[1][1];
					S[2] = A[2][2];
					break;
				}
				else
				{
					// 2x2 closed form factorization
					float t0 = (A[1][1] * A[1][1] - A[2][2] * A[2][2] + A[1][2] * A[1][2]) / (A[1][2] * A[2][2]);
					float tan0 = 0.5f * (t0 + sqrtf(t0 * t0 + 4.0f));
					float c0 = InvSqrt(1.0f + tan0 * tan0);
					float s0 = tan0 * c0;

					for (j = 0; j < 3; ++j)
					{
						U[j][1] = c0 * U[j][1] - s0 * U[j][2];
						U[j][2] = s0 * U[j][1] + c0 * U[j][2];
					}

					float tan1 = (A[1][2] - A[2][2] * tan0) / A[1][1];
					float c1 = InvSqrt(1.0f + tan1 * tan1);
					float s1 = -tan1 * c1;

					for (i = 0; i < 3; ++i)
					{
						V[1][i] = c1 * V[1][i] - s1 * V[2][i];
						V[2][i] = s1 * V[1][i] + c1 * V[2][i];
					}

					S[0] = A[0][0];
					S[1] = c0 * c1 * A[1][1] -
						s1 * (c0 * A[1][2] - s0 * A[2][2]);
					S[2] = s0 * s1 * A[1][1] +
						c1 * (s0 * A[1][2] + c0 * A[2][2]);
					break;
				}
			}
			else
			{
				if (bTest2)
				{
					// 2x2 closed form factorization
					float t0 = (A[0][0] * A[0][0] + A[1][1] * A[1][1] -
						A[0][1] * A[0][1]) / (A[0][1] * A[1][1]);
					float tan0 = 0.5f * (-t0 + sqrtf(t0 * t0 + 4.0f));
					float c0 = InvSqrt(1.0f + tan0 * tan0);
					float s0 = tan0 * c0;

					for (j = 0; j < 3; ++j)
					{
						U[j][0] = c0 * U[j][0] - s0 * U[j][1];
						U[j][1] = s0 * U[j][0] + c0 * U[j][1];
					}

					float tan1 = (A[0][1] - A[1][1] * tan0) / A[0][0];
					float c1 = InvSqrt(1.0f + tan1 * tan1);
					float s1 = -tan1 * c1;

					for (i = 0; i < 3; ++i)
					{
						V[0][i] = c1 * V[0][i] - s1 * V[1][i];
						V[1][i] = s1 * V[0][i] + c1 * V[1][i];
					}

					S[0] = c0 * c1 * A[0][0] -
						s1 * (c0 * A[0][1] - s0 * A[1][1]);
					S[1] = s0 * s1 * A[0][0] +
						c1 * (s0 * A[0][1] + c0 * A[1][1]);
					S[2] = A[2][2];
					break;
				}
				else
				{
					A.GolubKahanStep(U, V);
				}
			}
		}

		// positize diagonal
		for (i = 0; i < 3; ++i)
		{
			if (S[i] < 0.0)
			{
				S[i] = -S[i];
				for (j = 0; j < 3; ++j)
					V[i][j] = -V[i][j];
			}
		}

		V = V.Transpose();
	}

	void		SingularValueCompose(const Vector3& S, const Matrix3& U, const Matrix3& V)
	{
		*this = U * Matrix3(S) * V.Transpose();
	}

	void		Orthonormalize()
	{
		// Algorithm uses Gram-Schmidt orthogonalization.  If 'this' matrix is
		//	   [ m0 ]		[ q0 ]
		// M = | m1 | ==>	| q1 |
		//	   [ m2 ]		[ q2 ]
		//	 
		//   q0 = m0/|m0|
		//   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
		//   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
		//
		// where |V| indicates length of vector V and A*B indicates dot
		// product of vectors A and B.

		// compute q0
		float inv = InvSqrt(mat[0][0] * mat[0][0] + mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2]);

		mat[0][0] *= inv;
		mat[0][1] *= inv;
		mat[0][2] *= inv;

		// compute q1
		float dp0 = mat[0][0] * mat[1][0] + mat[0][1] * mat[1][1] + mat[0][2] * mat[1][2];
		mat[1][0] -= dp0 * mat[0][0];
		mat[1][1] -= dp0 * mat[0][1];
		mat[1][2] -= dp0 * mat[0][2];

		inv = InvSqrt(mat[1][0] * mat[1][0] + mat[1][1] * mat[1][1] + mat[1][2] * mat[1][2]);
		mat[0][1] *= inv;
		mat[1][1] *= inv;
		mat[2][1] *= inv;

		// q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
		float dp1 = mat[1][0] * mat[2][0] + mat[1][1] * mat[2][1] + mat[1][2] * mat[2][2];
		dp0 = mat[0][0] * mat[2][0] + mat[0][1] * mat[2][1] + mat[0][2] * mat[2][2];
		mat[2][0] -= dp0 * mat[0][0] + dp1 * mat[1][0];
		mat[2][1] -= dp0 * mat[0][1] + dp1 * mat[1][1];
		mat[2][2] -= dp0 * mat[0][2] + dp1 * mat[1][2];

		inv = InvSqrt(mat[2][0] * mat[2][0] + mat[2][1] * mat[2][1] + mat[2][2] * mat[2][2]);
		mat[2][0] *= inv;
		mat[2][1] *= inv;
		mat[2][2] *= inv;
	}

	bool		PolarDecomposeUP(Matrix3& U, Matrix3& P) const
	{
		const float POLAR_TOLERANCE = 0.01f;

		Matrix3 Mk;
		Matrix3 Ek;
		float det, oneNorm, infNorm, oneNorm2;

		Mk = Transpose();

		oneNorm = Mk.OneNorm();
		infNorm = Mk.InfNorm();

		do
		{
			Vector3 v1 = Vector3(Mk(2, 0), Mk(2, 1), Mk(2, 2));
			Vector3 v2 = Vector3(Mk(0, 0), Mk(0, 1), Mk(0, 2));
			Vector3 v3 = Vector3(Mk(1, 0), Mk(1, 1), Mk(1, 2));

			Vector3 r1 = Vector3(Mk(1, 0), Mk(1, 1), Mk(1, 2)).Cross(v1);
			Vector3 r2 = Vector3(Mk(2, 0), Mk(2, 1), Mk(2, 2)).Cross(v2);
			Vector3 r3 = Vector3(Mk(0, 0), Mk(0, 1), Mk(0, 2)).Cross(v3);
			Matrix3 MadjTk(r1.x, r1.y, r1.z,
				r2.x, r2.y, r2.z,
				r3.x, r3.y, r3.z);

			det = Mk(0, 0) * MadjTk(0, 0) + Mk(0, 1) * MadjTk(0, 1) + Mk(0, 2) * MadjTk(0, 2);
			if (fabsf(det) < 1e-6)
			{
				return false;
			}

			float one = MadjTk.OneNorm();
			float inf = MadjTk.InfNorm();
			float gamma = sqrtf(sqrtf((one * inf) / (oneNorm * infNorm)) / fabsf(det));
			float g1 = gamma * 0.5f;
			float g2 = 0.5f / (gamma * det);

			for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
			{
				Ek(i, j) = Mk(i, j);
				Mk(i, j) = g1 * Mk(i, j) + g2 * MadjTk(i, j);
				Ek(i, j) -= Mk(i, j);
			}

			oneNorm2 = Ek.OneNorm();
			oneNorm = Mk.OneNorm();
			infNorm = Mk.InfNorm();
		} while (oneNorm2 > oneNorm * POLAR_TOLERANCE);

		// Q = Mk^T

		U = Mk.Transpose();

		for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			P(i, j) = 0.0;
			for (int k = 0; k < 3; ++k)
				P(i, j) += Mk(i, k) * mat[k][j];
		}

		// S must be symmetric; enforce the symmetry
		for (int i = 0; i < 3; i++)
		for (int j = i; j < 3; j++)
			P(i, j) = P(j, i) = 0.5f * (P(i, j) + P(j, i));

		return true;
	}

	bool		PolarDecomposeU(Matrix3& U) const
	{
		Matrix3 Mk;
		Matrix3 Ek;
		float det, M_oneNorm, M_infNorm, E_oneNorm;
		const float POLAR_TOLERANCE = 0.01f;

		Mk = Transpose();

		M_oneNorm = Mk.OneNorm();
		M_infNorm = Mk.InfNorm();

		do
		{
			Vector3 v1 = Vector3(Mk(2, 0), Mk(2, 1), Mk(2, 2));
			Vector3 v2 = Vector3(Mk(0, 0), Mk(0, 1), Mk(0, 2));
			Vector3 v3 = Vector3(Mk(1, 0), Mk(1, 1), Mk(1, 2));

			Vector3 r1 = Vector3(Mk(1, 0), Mk(1, 1), Mk(1, 2)).Cross(v1);
			Vector3 r2 = Vector3(Mk(2, 0), Mk(2, 1), Mk(2, 2)).Cross(v2);
			Vector3 r3 = Vector3(Mk(0, 0), Mk(0, 1), Mk(0, 2)).Cross(v3);
			Matrix3 MadjTk(r1.x, r1.y, r1.z,
				r2.x, r2.y, r2.z,
				r3.x, r3.y, r3.z);

			det = Mk(0, 0) * MadjTk(0, 0) + Mk(0, 1) * MadjTk(0, 1) + Mk(0, 2) * MadjTk(0, 2);
			if (fabsf(det) < 1e-6)
			{
				return false;
			}

			float MadjT_one = MadjTk.OneNorm();
			float MadjT_inf = MadjTk.InfNorm();

			float gamma = sqrtf(sqrtf((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / fabsf(det));
			float g1 = gamma * 0.5f;
			float g2 = 0.5f / (gamma * det);

			for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				Ek(i, j) = Mk(i, j);
				Mk(i, j) = g1 * Mk(i, j) + g2 * MadjTk(i, j);
				Ek(i, j) -= Mk(i, j);
			}

			E_oneNorm = Ek.OneNorm();
			M_oneNorm = Mk.OneNorm();
			M_infNorm = Mk.InfNorm();
		} while (E_oneNorm > M_oneNorm * POLAR_TOLERANCE);

		// Q = Mk^T

		U = Mk.Transpose();
		return true;
	}

	void		Diagonalize(float Diag[3], float SubDiag[3])
	{
		float fA = mat[0][0];
		float fB = mat[0][1];
		float fC = mat[0][2];
		float fD = mat[1][1];
		float fE = mat[1][2];
		float fF = mat[2][2];

		Diag[0] = fA;
		SubDiag[2] = 0.0;
		if (fabsf(fC) >= Epsilon(1.0f))
		{
			float fLength = sqrtf(fB * fB + fC * fC);
			float fInvLength = 1.0f / fLength;
			fB *= fInvLength;
			fC *= fInvLength;
			float fQ = 2.0f * fB * fE + fC * (fF - fD);
			Diag[1] = fD + fC * fQ;
			Diag[2] = fF - fC * fQ;
			SubDiag[0] = fLength;
			SubDiag[1] = fE - fB * fQ;
			mat[0][0] = 1.0f;
			mat[0][1] = 0.0f;
			mat[0][2] = 0.0f;
			mat[1][0] = 0.0f;
			mat[1][1] = fB;
			mat[1][2] = fC;
			mat[2][0] = 0.0f;
			mat[2][1] = fC;
			mat[2][2] = -fB;
		}
		else
		{
			Diag[1] = fD;
			Diag[2] = fF;
			SubDiag[0] = fB;
			SubDiag[1] = fE;
			LoadIdentiry();
		}
	}

	bool		QRIteration(float diag[3], float subdiag[3])
	{
		for (int i = 0; i < 3; i++)
		{
			const int max_iterations = 32;
			int it = 0;
			while (it++ < max_iterations)
			{
				int j;
				for (j = i; j <= 1; ++j)
				{
					float sum = fabsf(diag[j]) + fabsf(diag[j + 1]);
					if (fabsf(subdiag[j]) + sum == sum)
						break;
				}
				if (j == i)
					break;

				float t0 = (diag[i + 1] - diag[i]) / (2.0f * subdiag[i]);
				float t1 = sqrtf(t0 * t0 + 1.0f);
				if (t0 < 0.0)
					t0 = diag[j] - diag[i] + subdiag[i] / (t0 - t1);
				else
					t0 = diag[j] - diag[i] + subdiag[i] / (t0 + t1);
				float s = 1.0;
				float c = 1.0;
				float t2 = 0.0;
				for (int k = j - 1; k >= i; --k)
				{
					float t3 = s * subdiag[k];
					float t4 = c * subdiag[k];
					if (fabsf(t3) >= fabsf(t0))
					{
						c = t0 / t3;
						t1 = sqrtf(c * c + 1.0f);
						subdiag[k + 1] = t3 * t1;
						s = 1.0f / t1;
						c *= s;
					}
					else
					{
						s = t3 / t0;
						t1 = sqrtf(s * s + 1.0f);
						subdiag[k + 1] = t0 * t1;
						c = 1.0f / t1;
						s *= c;
					}
					t0 = diag[k + 1] - t2;
					t1 = (diag[k] - t0) * s + 2.0f * t4 * c;
					t2 = s * t1;
					diag[k + 1] = t0 + t2;
					t0 = c * t1 - t4;

					for (int r = 0; r < 3; ++r)
					{
						t3 = mat[r][k + 1];
						mat[r][k + 1] = s * mat[r][k] + c * t3;
						mat[r][k] = c * mat[r][k] - s * t3;
					}
				}
				diag[i] -= t2;
				subdiag[i] = t0;
				subdiag[j] = 0.0;
			}

			if (it == max_iterations)
			{
				// should not get here
				return false;
			}
		}

		return true;
	}

	bool		SolveEigenQRIteration(float EigenValue[3], Vector3 EigenVector[3]) const
	{
		Matrix3 m = *this;
		float SubDiag[3];
		m.Diagonalize(EigenValue, SubDiag);
		if (m.QRIteration(EigenValue, SubDiag))
		{
			for (int i = 0; i < 3; ++i)
			{
				EigenVector[i][0] = m[0][i];
				EigenVector[i][1] = m[1][i];
				EigenVector[i][2] = m[2][i];
			}
			return true;
		}

		return false;
	}

	bool		SolveEigenSymmetric(float EigenValue[3], Vector3 EigenVector[3]) const
	{
		return SolveEigenQRIteration(EigenValue, EigenVector);
	}
	
	static Matrix3 Zero()
	{
		return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	}

	static Matrix3 Identity()
	{
		return Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);
	}
};

inline Matrix3 operator* (float f, const Matrix3& mm)
{
	return mm * f;
}

inline Vector3 operator* (const Vector3 &v, const Matrix3& mm)
{
	return Vector3(v.x * mm[0][0] + v.y * mm[1][0] + v.z * mm[2][0],
				   v.x * mm[0][1] + v.y * mm[1][1] + v.z * mm[2][1],
				   v.x * mm[0][2] + v.y * mm[1][2] + v.z * mm[2][2]);
}

static_assert(sizeof(Matrix3) == 36, "sizeof Matrix3 is not valid");

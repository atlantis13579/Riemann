#pragma once

#include <string.h>
#include "Vector3.h"

class Matrix3
{
public:
	float mat[3][3];

	Matrix3()
	{

	}

	Matrix3(float m[3][3])
	{
		memcpy(mat, m, sizeof(mat));
	}

	Matrix3(const Matrix3& m)
	{
		memcpy(mat, m.mat, sizeof(mat));
	}

	Matrix3(float a00, float a01, float a02,
		float a10, float a11, float a12,
		float a20, float a21, float a22)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22;
	}

	Matrix3(float a00, float a11, float a22)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = a00; mat[1][1] = a11; mat[2][2] = a22;
	}
	
	Matrix3(const Vector3& diag)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = diag.x; mat[1][1] = diag.y; mat[2][2] = diag.z;
	}

	Matrix3(const Vector3& c0, const Vector3& c1, const Vector3& c2)
	{
		mat[0][0] = c0.x; mat[0][1] = c1.x; mat[0][2] = c2.x;
		mat[1][0] = c0.y; mat[1][1] = c1.y; mat[1][2] = c2.y;
		mat[2][0] = c0.z; mat[2][1] = c1.z; mat[2][2] = c2.z;
	}
	
	void LoadIdentiry()
	{
		mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0;
		mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0;
		mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1;
	}

	// Generate Rotate Matrix
	void LoadRotateX(float rfloatAngle);
	void LoadRotateY(float rfloatAngle);
	void LoadRotateZ(float rfloatAngle);
	void Load2DOrthogonalTransform(float dx, float dy, float dAngle);

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

	inline const Vector3& Row(int i) const
	{
		const Vector3 *row = static_cast<const Vector3*>((const void*)mat);
		return row[i];
	}

	inline Vector3& Row(int i)
	{
		Vector3 *row = static_cast<Vector3*>((void*)mat);
		return row[i];
	}

	inline Vector3 Column(int i) const
	{
		return Vector3(mat[0][i], mat[1][i], mat[2][i]);
	}

	float Trace() const
	{
		return mat[0][0] + mat[1][1] + mat[2][2];
	}

	float L1Norm() const
	{
		float l1_norm = 0.0f;
		const float* p = (const float *)this;
		for (int i = 0; i < 9; ++i)
			l1_norm += fabs(p[i]);
		return l1_norm;
	}

	float L2Norm() const
	{
		float l2_norm = 0.0f;
		const float* p = (const float*)this;
		for (int i = 0; i < 9; ++i)
			l2_norm += p[i] * p[i];
		return l2_norm;
	}

	bool Invertible() const
	{
		float d = Determinant();
		return fabsf(d) > 1e-6;
	}

	Matrix3 Inverse() const
	{
		float d = Determinant();
		// Warning!!! Assume d != 0
		d = 1.0f / d;
		return Matrix3((mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) * d, -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) * d, (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * d,
			-(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) * d, (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * d, -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) * d,
			(mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) * d, -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) * d, (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) * d);
	}

	// Solve AX = B
	bool Solve(const Vector3& b, Vector3& x)
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

	void		FromAxisAngle(const Vector3& Axis, float Radian)
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
	}

	void		FromTwoAxis(const Vector3& AxisFrom, const Vector3& AxisTo)
	{
		Vector3 UnitAxisFrom = AxisFrom.Unit();
		Vector3 UnitAxisTo = AxisTo.Unit();
		Vector3 Axis = UnitAxisFrom.Cross(UnitAxisTo);
		float Angle = acosf(UnitAxisFrom.Dot(UnitAxisTo));
		FromAxisAngle(Axis, Angle);
	}

	void		FromEulerAnglesXYZ(float Yaw, float Pitch, float Roll);

	float		ToAxisAngle(Vector3& Axis) const;
	bool		ToEulerAnglesXYZ(float& Yaw, float& Pitch, float& Roll) const;

	// A = U * S * V^T
	void		SingularValueDecomposition(Vector3& S, Matrix3& U, Matrix3& V) const;
	void		SingularValueComposition(const Vector3& S, const Matrix3& U, const Matrix3& V);
	void		Orthonormalize();
	void		PolarDecomposition(Matrix3& rU, Matrix3& rP) const;
	void		QDUDecomposition(Matrix3& rQ, Vector3& rD, Vector3& rU) const;
	float		SpectralNorm() const;

	void		SolveEigenSymmetric(float EigenValue[3], Vector3 EigenVector[3]) const;
	void		TriDiagonal(float Diag[3], float SubDiag[3]);
	bool		QRIteration(float Diag[3], float SubDiag[3]);

	static void Bidiagonalize(Matrix3& kA, Matrix3& kL, Matrix3& kR);
	static void GolubKahanStep(Matrix3& kA, Matrix3& kL, Matrix3& kR);
	static float MaxCubicRoot(float afCoeff[3]);

	static Matrix3 Zero()
	{
		return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	}

	static Matrix3 Identity()
	{
		return Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);;
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

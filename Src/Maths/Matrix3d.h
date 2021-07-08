#pragma once

#include "Vector3d.h"

class Matrix3d
{
public:
	union
	{
		struct { Vector3d row[3]; };
		struct { float mat[3][3]; };
	};

	Matrix3d()
	{

	}

	Matrix3d(float m[3][3])
	{
		memcpy(mat, m, sizeof(mat));
	}

	Matrix3d(const Matrix3d& m)
	{
		memcpy(mat, m.mat, sizeof(mat));
	}

	Matrix3d(float a00, float a01, float a02,
			float a10, float a11, float a12,
			float a20, float a21, float a22)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22;
	}

	Matrix3d(float a00, float a11, float a22)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = a00; mat[1][1] = a11; mat[2][2] = a22;
	}

	Matrix3d(const Vector3d& c0, const Vector3d& c1, const Vector3d& c2)
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

	Matrix3d Transpose() const
	{
		return Matrix3d(mat[0][0], mat[1][0], mat[2][0],
						mat[0][1], mat[1][1], mat[2][1],
						mat[0][2], mat[1][2], mat[2][2]);
	}

	float Determinant() const
	{
		return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
			- mat[0][1] * (mat[2][2] * mat[1][0] - mat[1][2] * mat[2][0])
			+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	}

	Matrix3d operator*(const Matrix3d& mm) const
	{
		float m[3][3] = { 0 };
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		for (int k = 0; k < 3; k++)
			m[i][j] += mat[i][k] * mm.mat[k][j];
		return Matrix3d(m);
	}

	Matrix3d operator*(float k) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] * k;
		return Matrix3d(m);
	}

	Vector3d operator*(const Vector3d& vv) const
	{
		float v[3] = { 0 };
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			v[i] += mat[i][j] * vv.coords[j];
		return Vector3d(v);
	}

	Matrix3d operator+(const Matrix3d& mm) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] + mm.mat[i][j];
		return Matrix3d(m);
	}

	Matrix3d operator-(const Matrix3d& mm) const
	{
		float m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] - mm.mat[i][j];
		return Matrix3d(m);
	}

	inline float operator()(int i, int j) const
	{
		return mat[i][j];
	}

	inline float& operator()(int i, int j)
	{
		return mat[i][j];
	}

	inline const Vector3d& operator[](int i) const
	{
		return row[i];
	}

	inline Vector3d& operator[](int i)
	{
		return row[i];
	}

	inline const Vector3d& Row(int i) const
	{
		return row[i];
	}

	inline Vector3d& Row(int i)
	{
		return row[i];
	}

	inline Vector3d Column(int i) const
	{
		return Vector3d(mat[0][i], mat[1][i], mat[2][i]);
	}

	float Trace() const
	{
		return mat[0][0] + mat[1][1] + mat[2][2];
	}

	inline Matrix3d& operator=(const Matrix3d& rhs)
	{
		memcpy(mat, rhs.mat, sizeof(mat));
		return *this;
	}

	bool Invertible() const
	{
		float d = Determinant();
		return fabsf(d) > 0.0000001f;
	}

	Matrix3d Inverse() const
	{
		float d = Determinant();
		// Assume d != 0
		return Matrix3d((mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) * d, -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) * d, (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * d,
			-(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) * d, (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * d, -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) * d,
			(mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) * d, -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) * d, (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) * d);
	}

	// Solve AX = B
	Vector3d Solve(const Vector3d &b)
	{
		float d = Determinant();
		// Assume d != 0
		Vector3d x;
		x.x = b.Dot(row[1].Cross(row[2])) / d;
		x.y = row[0].Dot(b.Cross(row[2])) / d;
		x.z = row[0].Dot(row[1].Cross(b)) / d;
		return x;
	}

	static Matrix3d Skew(Vector3d& v)
	{
		return Matrix3d(0, -v.z, v.y,
						v.z, 0, -v.x,
						-v.y, v.x, 0);
	}
};

static_assert(sizeof(Matrix3d) == 36, "sizeof Matrix3d is not valid");

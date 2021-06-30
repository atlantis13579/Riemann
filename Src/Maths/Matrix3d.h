#pragma once

#include "Vector3d.h"

class Matrix3d
{
public:
	float mat[3][3];
	Matrix3d()
	{
		mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0;
		mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0;
		mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1;
	}
	Matrix3d(float m[3][3])
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = m[i][j];
	}
	Matrix3d(const Matrix3d& m)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = m.mat[i][j];
	}
	Matrix3d(float a00, float a01, float a02,
		float a10, float a11, float a12,
		float a20, float a21, float a22)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22;

	}
	Matrix3d(Vector3d& c0, Vector3d& c1, Vector3d& c2)
	{
		mat[0][0] = c0.x; mat[0][1] = c1.x; mat[0][2] = c2.x;
		mat[1][0] = c0.y; mat[1][1] = c1.y; mat[1][2] = c2.y;
		mat[2][0] = c0.z; mat[2][1] = c1.z; mat[2][2] = c2.z;
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
		float m[3][3] = { 0 };
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
	float operator()(int i, int j) const
	{
		return mat[i][j];
	}
	float& operator()(int i, int j)
	{
		return mat[i][j];
	}
	float trace() const
	{
		return mat[0][0] + mat[1][1] + mat[2][2];
	}

	inline Matrix3d& operator=(const Matrix3d& rhs)
	{
		memcpy(mat, rhs.mat, 9 * sizeof(float));
		return *this;
	}

	Matrix3d Inverse() const
	{
		float d = Determinant();

		return Matrix3d((mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / d, -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) / d, (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / d,
						-(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) / d, (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / d, -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) / d,
						(mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / d, -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) / d, (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / d);
	}
	static Matrix3d Skew(Vector3d& v)
	{
		return Matrix3d(0, -v.z, v.y,
						v.z, 0, -v.x,
						-v.y, v.x, 0);
	}
};

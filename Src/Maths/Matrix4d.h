
#pragma once

#include "Vector3d.h"
#include "Vector4d.h"

class Matrix4d
{
public:
	union
	{
		float mat[4][4];
		float arr[16];
	};
	Matrix4d()
	{
		mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0; mat[0][3] = 0;
		mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0; mat[1][3] = 0;
		mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1; mat[2][3] = 0;
		mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
	}
	Matrix4d(float m[4][4])
	{
		for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			mat[i][j] = m[i][j];
	}
	Matrix4d(float a00, float a01, float a02, float a03,
			float a10, float a11, float a12, float a13,
			float a20, float a21, float a22, float a23,
			float a30, float a31, float a32, float a33)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02; mat[0][3] = a03;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12; mat[1][3] = a13;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22; mat[2][3] = a23;
		mat[3][0] = a30; mat[3][1] = a31; mat[3][2] = a32; mat[3][3] = a33;
	}

	Matrix4d Transpose()
	{
		return Matrix4d(mat[0][0], mat[1][0], mat[2][0], mat[3][0],
						mat[0][1], mat[1][1], mat[2][1], mat[3][1],
						mat[0][2], mat[1][2], mat[2][2], mat[3][2],
						mat[0][3], mat[1][3], mat[2][3], mat[3][3]);
	}

	Matrix4d Inverse()
	{
		float result[4][4];
		result[0][0] = -mat[2][3] * mat[3][2] * mat[1][1] + mat[2][2] * mat[3][3] * mat[1][1] + mat[2][3] * mat[3][1] * mat[1][2] - mat[2][2] * mat[3][1] * mat[1][3] - mat[3][3] * mat[1][2] * mat[2][1] + mat[3][2] * mat[1][3] * mat[2][1];
		result[0][1] = mat[0][1] * mat[2][3] * mat[3][2] - mat[0][1] * mat[2][2] * mat[3][3] - mat[2][3] * mat[3][1] * mat[0][2] + mat[2][2] * mat[3][1] * mat[0][3] + mat[3][3] * mat[0][2] * mat[2][1] - mat[3][2] * mat[0][3] * mat[2][1];
		result[0][2] = -mat[3][3] * mat[0][2] * mat[1][1] + mat[3][2] * mat[0][3] * mat[1][1] + mat[0][1] * mat[3][3] * mat[1][2] - mat[3][1] * mat[0][3] * mat[1][2] - mat[0][1] * mat[3][2] * mat[1][3] + mat[3][1] * mat[0][2] * mat[1][3];
		result[0][3] = mat[2][3] * mat[0][2] * mat[1][1] - mat[2][2] * mat[0][3] * mat[1][1] - mat[0][1] * mat[2][3] * mat[1][2] + mat[0][1] * mat[2][2] * mat[1][3] + mat[0][3] * mat[1][2] * mat[2][1] - mat[0][2] * mat[1][3] * mat[2][1];
		result[1][0] = mat[2][3] * mat[3][2] * mat[1][0] - mat[2][2] * mat[3][3] * mat[1][0] - mat[2][3] * mat[3][0] * mat[1][2] + mat[2][2] * mat[3][0] * mat[1][3] + mat[3][3] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][3] * mat[2][0];
		result[1][1] = -mat[0][0] * mat[2][3] * mat[3][2] + mat[0][0] * mat[2][2] * mat[3][3] + mat[2][3] * mat[3][0] * mat[0][2] - mat[2][2] * mat[3][0] * mat[0][3] - mat[3][3] * mat[0][2] * mat[2][0] + mat[3][2] * mat[0][3] * mat[2][0];
		result[1][2] = mat[3][3] * mat[0][2] * mat[1][0] - mat[3][2] * mat[0][3] * mat[1][0] - mat[0][0] * mat[3][3] * mat[1][2] + mat[3][0] * mat[0][3] * mat[1][2] + mat[0][0] * mat[3][2] * mat[1][3] - mat[3][0] * mat[0][2] * mat[1][3];
		result[1][3] = -mat[2][3] * mat[0][2] * mat[1][0] + mat[2][2] * mat[0][3] * mat[1][0] + mat[0][0] * mat[2][3] * mat[1][2] - mat[0][0] * mat[2][2] * mat[1][3] - mat[0][3] * mat[1][2] * mat[2][0] + mat[0][2] * mat[1][3] * mat[2][0];
		result[2][0] = -mat[2][3] * mat[3][1] * mat[1][0] + mat[2][3] * mat[3][0] * mat[1][1] - mat[3][3] * mat[1][1] * mat[2][0] + mat[3][1] * mat[1][3] * mat[2][0] + mat[3][3] * mat[1][0] * mat[2][1] - mat[3][0] * mat[1][3] * mat[2][1];
		result[2][1] = -mat[0][1] * mat[2][3] * mat[3][0] + mat[0][0] * mat[2][3] * mat[3][1] + mat[0][1] * mat[3][3] * mat[2][0] - mat[3][1] * mat[0][3] * mat[2][0] - mat[0][0] * mat[3][3] * mat[2][1] + mat[3][0] * mat[0][3] * mat[2][1];
		result[2][2] = -mat[0][1] * mat[3][3] * mat[1][0] + mat[3][1] * mat[0][3] * mat[1][0] + mat[0][0] * mat[3][3] * mat[1][1] - mat[3][0] * mat[0][3] * mat[1][1] + mat[0][1] * mat[3][0] * mat[1][3] - mat[0][0] * mat[3][1] * mat[1][3];
		result[2][3] = mat[0][1] * mat[2][3] * mat[1][0] - mat[0][0] * mat[2][3] * mat[1][1] + mat[0][3] * mat[1][1] * mat[2][0] - mat[0][1] * mat[1][3] * mat[2][0] - mat[0][3] * mat[1][0] * mat[2][1] + mat[0][0] * mat[1][3] * mat[2][1];
		result[3][0] = mat[2][2] * mat[3][1] * mat[1][0] - mat[2][2] * mat[3][0] * mat[1][1] + mat[3][2] * mat[1][1] * mat[2][0] - mat[3][1] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][0] * mat[2][1] + mat[3][0] * mat[1][2] * mat[2][1];
		result[3][1] = mat[0][1] * mat[2][2] * mat[3][0] - mat[0][0] * mat[2][2] * mat[3][1] - mat[0][1] * mat[3][2] * mat[2][0] + mat[3][1] * mat[0][2] * mat[2][0] + mat[0][0] * mat[3][2] * mat[2][1] - mat[3][0] * mat[0][2] * mat[2][1];
		result[3][2] = mat[0][1] * mat[3][2] * mat[1][0] - mat[3][1] * mat[0][2] * mat[1][0] - mat[0][0] * mat[3][2] * mat[1][1] + mat[3][0] * mat[0][2] * mat[1][1] - mat[0][1] * mat[3][0] * mat[1][2] + mat[0][0] * mat[3][1] * mat[1][2];
		result[3][3] = -mat[0][1] * mat[2][2] * mat[1][0] + mat[0][0] * mat[2][2] * mat[1][1] - mat[0][2] * mat[1][1] * mat[2][0] + mat[0][1] * mat[1][2] * mat[2][0] + mat[0][2] * mat[1][0] * mat[2][1] - mat[0][0] * mat[1][2] * mat[2][1];
		float d = 1.0f / (mat[0][0] * result[0][0] + mat[0][1] * result[1][0] + mat[0][2] * result[2][0] + mat[0][3] * result[3][0]);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result[i][j] *= d;
		return Matrix4d(result);
	}
	
	float Determinant() const
	{
		return (mat[0][0] * (-mat[2][3] * mat[3][2] * mat[1][1] + mat[2][2] * mat[3][3] * mat[1][1] + mat[2][3] * mat[3][1] * mat[1][2] - mat[2][2] * mat[3][1] * mat[1][3] - mat[3][3] * mat[1][2] * mat[2][1] + mat[3][2] * mat[1][3] * mat[2][1]) +
			mat[0][1] * (mat[2][3] * mat[3][2] * mat[1][0] - mat[2][2] * mat[3][3] * mat[1][0] - mat[2][3] * mat[3][0] * mat[1][2] + mat[2][2] * mat[3][0] * mat[1][3] + mat[3][3] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][3] * mat[2][0]) +
			mat[0][2] * (-mat[2][3] * mat[3][1] * mat[1][0] + mat[2][3] * mat[3][0] * mat[1][1] - mat[3][3] * mat[1][1] * mat[2][0] + mat[3][1] * mat[1][3] * mat[2][0] + mat[3][3] * mat[1][0] * mat[2][1] - mat[3][0] * mat[1][3] * mat[2][1]) +
			mat[0][3] * (mat[2][2] * mat[3][1] * mat[1][0] - mat[2][2] * mat[3][0] * mat[1][1] + mat[3][2] * mat[1][1] * mat[2][0] - mat[3][1] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][0] * mat[2][1] + mat[3][0] * mat[1][2] * mat[2][1]));

	}

	Matrix4d operator*(const Matrix4d& mm)
	{
		float m[4][4] = { 0 };
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				for (int k = 0; k < 4; k++)
					m[i][j] += mat[i][k] * mm.mat[k][j];
		return Matrix4d(m);
	}
	Vector3d operator*(const Vector3d& vv)
	{
		float in[4] = { vv.x, vv.y, vv.z, 1 };
		float out[4] = { 0 };
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				out[i] += mat[i][j] * in[j];
		return Vector3d(out);
	}
	Matrix4d operator+(const Matrix4d& mm)
	{
		float m[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				m[i][j] = mat[i][j] + mm.mat[i][j];
		return Matrix4d(m);
	}
	Matrix4d operator-(const Matrix4d& mm)
	{
		float m[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				m[i][j] = mat[i][j] - mm.mat[i][j];
		return Matrix4d(m);
	}

	Vector4d Row(int i) const
	{
		return Vector4d(mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
	}
};

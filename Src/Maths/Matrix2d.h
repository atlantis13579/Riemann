#pragma once

#include "Vector2d.h"

class Matrix2d
{
public:
	union
	{
		struct { Vector2d row[2]; };
		struct { float arr[4]; };
		struct { float mat[2][2]; };
	};

	Matrix2d()
	{

	}

	Matrix2d(float mm[2][2])
	{
		memcpy(mat, mm, sizeof(mat));
	}

	Matrix2d(const Matrix2d& mm)
	{
		memcpy(mat, mm.mat, sizeof(mat));
	}

	Matrix2d(float a00, float a01,
			float a10, float a11)
	{
		mat[0][0] = a00; mat[0][1] = a01;
		mat[1][0] = a10; mat[1][1] = a11;
	}

	Matrix2d(float a00, float a11)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = a00; mat[1][1] = a11;
	}

	Matrix2d(const Vector2d& c0, const Vector2d& c1)
	{
		mat[0][0] = c0.x; mat[0][1] = c1.x;
		mat[1][0] = c0.y; mat[1][1] = c1.y;
	}

	void LoadZero()
	{
		memset(mat, 0, sizeof(mat));
	}

	void LoadIdentiry()
	{
		mat[0][0] = 1; mat[0][1] = 0;
		mat[1][0] = 0; mat[1][1] = 1;
	}

	void LoadRotation(float angle)
	{
		mat[0][1] = cosf(angle);
		mat[1][0] = sinf(angle);
		mat[0][1] = -mat[1][0];
		mat[1][1] = mat[0][0];
	}

	Matrix2d Transpose() const
	{
		return Matrix2d(mat[0][0], mat[1][0],
						mat[0][1], mat[1][1]);
	}

	float Determinant() const
	{
		return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
	}

	Matrix2d operator*(const Matrix2d& mm) const
	{
		return Matrix2d(
			arr[0] * mm.arr[0] + arr[1] * mm.arr[2],
			arr[0] * mm.arr[1] + arr[1] * mm.arr[3],
			arr[2] * mm.arr[0] + arr[3] * mm.arr[2],
			arr[2] * mm.arr[1] + arr[3] * mm.arr[3]);
	}

	Matrix2d operator*(float k) const
	{
		return Matrix2d(
			arr[0] * k, arr[1] * k,
			arr[2] * k, arr[3] * k);
	}

	Vector2d operator*(const Vector2d& vec) const
	{
		return Vector2d(
			arr[0] * vec.x + arr[1] * vec.y,
			arr[2] * vec.x + arr[3] * vec.y);
	}

	Matrix2d operator+(const Matrix2d& mm) const
	{
		return Matrix2d(
			arr[0] + mm.arr[0], arr[1] + mm.arr[1],
			arr[2] + mm.arr[2], arr[3] + mm.arr[3]);
	}

	Matrix2d operator-(const Matrix2d& mm) const
	{
		return Matrix2d(
			arr[0] - mm.arr[0], arr[1] - mm.arr[1],
			arr[2] - mm.arr[2], arr[3] - mm.arr[3]);
	}

	inline Matrix2d& operator=(const Matrix2d& rhs)
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

	inline const Vector2d& operator[](int i) const
	{
		return row[i];
	}

	inline Vector2d& operator[](int i)
	{
		return row[i];
	}

	inline const Vector2d& Row(int i) const
	{
		return row[i];
	}

	inline Vector2d& Row(int i)
	{
		return row[i];
	}

	inline Vector2d Column(int i) const
	{
		return Vector2d(mat[0][i], mat[1][i]);
	}

	float Trace() const
	{
		return mat[0][0] + mat[1][1];
	}

	bool Invertible() const
	{
		float d = Determinant();
		return fabsf(d) > 1e-6;
	}

	Matrix2d Inverse() const
	{
		float det = Determinant();
		// Warning!!! Assume d != 0
		return Matrix2d(
			mat[0][0] / det, mat[0][1] / det,
			mat[1][0] / det, mat[1][1] / det);
	}

	// Solve AX = B
	Vector2d Solve(const Vector2d& b)
	{
		float det = Determinant();
		// Warning!!! Assume d != 0
		Vector2d x;
		x.x = (mat[1][1] * b.x - mat[0][1] * b.y) / det;
		x.y = (mat[0][0] * b.y - mat[1][0] * b.x) / det;
		return x;
	}

	static const Matrix2d& Identity()
	{
		static Matrix2d Identity(1, 0, 0, 1);
		return Identity;
	}
};

static_assert(sizeof(Matrix2d) == 16, "sizeof Matrix2d is not valid");
#pragma once

#include "Vector2.h"

class Matrix2
{
public:
	float mat[2][2];

	Matrix2()
	{

	}

	explicit Matrix2(float mm[2][2])
	{
		memcpy(mat, mm, sizeof(mat));
	}

	explicit Matrix2(float a00, float a01,
			float a10, float a11)
	{
		mat[0][0] = a00; mat[0][1] = a01;
		mat[1][0] = a10; mat[1][1] = a11;
	}

	explicit Matrix2(float a00, float a11)
	{
		mat[0][0] = a00; mat[0][1] = 0.0f;
		mat[1][0] = 0.0f; mat[1][1] = a11;
	}

	explicit Matrix2(const Vector2& c0, const Vector2& c1)
	{
		mat[0][0] = c0.x; mat[0][1] = c1.x;
		mat[1][0] = c0.y; mat[1][1] = c1.y;
	}
	
	Matrix2(const Matrix2& mm)
	{
		memcpy(mat, mm.mat, sizeof(mat));
	}

	void		LoadZero()
	{
		memset(mat, 0, sizeof(mat));
	}

	void		LoadIdentiry()
	{
		mat[0][0] = 1; mat[0][1] = 0;
		mat[1][0] = 0; mat[1][1] = 1;
	}

	void		LoadRotation(float angle)
	{
		mat[0][1] = cosf(angle);
		mat[1][0] = sinf(angle);
		mat[0][1] = -mat[1][0];
		mat[1][1] = mat[0][0];
	}

	Matrix2	operator*(const Matrix2& mm) const
	{
		return Matrix2(
			mat[0][0] * mm.mat[0][0] + mat[0][1] * mm.mat[1][0],
			mat[0][0] * mm.mat[0][1] + mat[0][1] * mm.mat[1][1],
			mat[1][0] * mm.mat[0][0] + mat[1][1] * mm.mat[1][0],
			mat[1][0] * mm.mat[0][1] + mat[1][1] * mm.mat[1][1]);
	}

	Matrix2	operator*(float k) const
	{
		return Matrix2(
			mat[0][0] * k, mat[0][1] * k,
			mat[1][0] * k, mat[1][1] * k);
	}

	Vector2	operator*(const Vector2& vec) const
	{
		return Vector2(
			mat[0][0] * vec.x + mat[0][1] * vec.y,
			mat[1][0] * vec.x + mat[1][1] * vec.y);
	}

	Matrix2	operator+(const Matrix2& mm) const
	{
		return Matrix2(
			mat[0][0] + mm.mat[0][0], mat[0][1] + mm.mat[0][1],
			mat[1][0] + mm.mat[1][0], mat[1][1] + mm.mat[1][1]);
	}

	Matrix2	operator-(const Matrix2& mm) const
	{
		return Matrix2(
			mat[0][0] - mm.mat[0][0], mat[0][1] - mm.mat[0][1],
			mat[1][0] - mm.mat[1][0], mat[1][1] - mm.mat[1][1]);
	}

	inline		Matrix2& operator=(const Matrix2& rhs)
	{
		memcpy(mat, rhs.mat, sizeof(mat));
		return *this;
	}

	inline const float* Data() const
	{
		return mat[0];
	}

	inline float* Data()
	{
		return mat[0];
	}

	inline float operator()(int i, int j) const
	{
		return mat[i][j];
	}

	inline float& operator()(int i, int j)
	{
		return mat[i][j];
	}

	inline const Vector2& operator[](int i) const
	{
		const Vector2 *row = static_cast<const Vector2*>((const void*)mat);
		return row[i];
	}

	inline Vector2& operator[](int i)
	{
		Vector2 *row = static_cast<Vector2*>((void*)mat);
		return row[i];
	}

	inline const Vector2& Row(int i) const
	{
		const Vector2 *row = static_cast<const Vector2*>((const void*)mat);
		return row[i];
	}

	inline Vector2& Row(int i)
	{
		Vector2 *row = static_cast<Vector2*>((void*)mat);
		return row[i];
	}

	inline Vector2 Column(int i) const
	{
		return Vector2(mat[0][i], mat[1][i]);
	}

	inline float	Trace() const
	{
		return mat[0][0] + mat[1][1];
	}

	inline float	Determinant() const
	{
		return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
	}

	float			L1Norm() const
	{
		float l1_norm = 0.0f;
		const float* p = (const float*)this;
		for (int i = 0; i < 4; ++i)
			l1_norm += fabsf(p[i]);
		return l1_norm;
	}

	float			L2Norm() const
	{
		float l2_norm = 0.0f;
		const float* p = (const float*)this;
		for (int i = 0; i < 4; ++i)
			l2_norm += p[i] * p[i];
		return l2_norm;
	}

	bool			Invertible() const
	{
		float d = Determinant();
		return fabsf(d) > 1e-6;
	}

	Matrix2		Inverse() const
	{
		float d = Determinant();
		// Warning!!! Assume d != 0
		d = 1.0f / d;
		return Matrix2(
			mat[0][0] * d, mat[0][1] * d,
			mat[1][0] * d, mat[1][1] * d);
	}

	Matrix2		Transpose() const
	{
		return Matrix2(mat[0][0], mat[1][0],
						mat[0][1], mat[1][1]);
	}

	// Solve AX = B
	bool			Solve(const Vector2& b, Vector2& x)
	{
		float det = Determinant();
		if (fabsf(det) < 1e-6)
		{
			return false;
		}
		x.x = (mat[1][1] * b.x - mat[0][1] * b.y) / det;
		x.y = (mat[0][0] * b.y - mat[1][0] * b.x) / det;
		return true;
	}

	void			SolveEigenSymmetric(float EigenValue[2], Vector2 EigenVector[3]) const
	{
		float sum = fabsf(mat[0][0]) + fabsf(mat[1][1]);

		// The matrix is diagonal within numerical round-off
		if (fabsf(mat[0][1]) + sum == sum)
		{
			EigenVector[0] = Vector2(1.0f, 0.0f);
			EigenVector[1] = Vector2(0.0f, 1.0f);
			EigenValue[0] = mat[0][0];
			EigenValue[1] = mat[1][1];
			return;
		}

		float trace = Trace();
		float diff = mat[0][0] - mat[1][1];
		float discr = sqrtf(diff * diff + 4.0f * mat[1][1] * mat[1][1]);
		EigenValue[0] = (trace - discr) * 0.5f;
		EigenValue[1] = (trace + discr) * 0.5f;

		float cs, sn;
		if (diff >= 0.0f)
		{
			cs = mat[0][1];
			sn = EigenValue[0] - mat[0][0];
		}
		else
		{
			cs = EigenValue[0] - mat[0][0];
			sn = mat[0][1];
		}
		float invLength = 1.0f / sqrtf(cs * cs + sn * sn);
		cs *= invLength;
		sn *= invLength;

		EigenVector[0] = Vector2(cs, -sn);
		EigenVector[1] = Vector2(sn, cs);
	}

	static Matrix2 Zero()
	{
		return  Matrix2(0, 0, 0, 0);
	}

	static Matrix2 Identity()
	{
		return  Matrix2(1, 0, 0, 1);
	}
};

static_assert(sizeof(Matrix2) == 16, "sizeof Matrix2 is not valid");

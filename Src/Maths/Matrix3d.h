#pragma once

#include "Vector3d.h"

template <typename T>
class TMatrix3
{
public:
	union
	{
		struct { TVector3<T> row[3]; };
		struct { T mat[3][3]; };
	};

	TMatrix3<T>()
	{
		mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0;
		mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0;
		mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1;
	}

	TMatrix3<T>(T m[3][3])
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = m[i][j];
	}

	TMatrix3<T>(const TMatrix3<T>& m)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = m.mat[i][j];
	}

	TMatrix3<T>(T a00, T a01, T a02,
		T a10, T a11, T a12,
		T a20, T a21, T a22)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22;
	}

	TMatrix3<T>(T a00, T a11, T a22)
	{
		memset(mat, 0, sizeof(mat));
		mat[0][0] = a00; mat[1][1] = a11; mat[2][2] = a22;
	}

	TMatrix3<T>(TVector3<T>& c0, TVector3<T>& c1, TVector3<T>& c2)
	{
		mat[0][0] = c0.x; mat[0][1] = c1.x; mat[0][2] = c2.x;
		mat[1][0] = c0.y; mat[1][1] = c1.y; mat[1][2] = c2.y;
		mat[2][0] = c0.z; mat[2][1] = c1.z; mat[2][2] = c2.z;
	}

	TMatrix3<T> Transpose() const
	{
		return TMatrix3<T>(mat[0][0], mat[1][0], mat[2][0],
			mat[0][1], mat[1][1], mat[2][1],
			mat[0][2], mat[1][2], mat[2][2]);
	}

	T Determinant() const
	{
		return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
			- mat[0][1] * (mat[2][2] * mat[1][0] - mat[1][2] * mat[2][0])
			+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	}

	TMatrix3<T> operator*(const TMatrix3<T>& mm) const
	{
		T m[3][3] = { 0 };
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		for (int k = 0; k < 3; k++)
			m[i][j] += mat[i][k] * mm.mat[k][j];
		return TMatrix3<T>(m);
	}

	TMatrix3<T> operator*(T k) const
	{
		T m[3][3] = { 0 };
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] * k;
		return TMatrix3<T>(m);
	}

	TVector3<T> operator*(const TVector3<T>& vv) const
	{
		T v[3] = { 0 };
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			v[i] += mat[i][j] * vv.coords[j];
		return TVector3<T>(v);
	}

	TMatrix3<T> operator+(const TMatrix3<T>& mm) const
	{
		T m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] + mm.mat[i][j];
		return TMatrix3<T>(m);
	}

	TMatrix3<T> operator-(const TMatrix3<T>& mm) const
	{
		T m[3][3];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = mat[i][j] - mm.mat[i][j];
		return TMatrix3<T>(m);
	}

	T operator()(int i, int j) const
	{
		return mat[i][j];
	}

	T& operator()(int i, int j)
	{
		return mat[i][j];
	}

	const TVector3<T>& operator[](int i) const
	{
		return row[i];
	}

	TVector3<T>& operator[](int i)
	{
		return row[i];
	}

	const TVector3<T>& Row(int i) const
	{
		return row[i];
	}

	TVector3<T>& Row(int i)
	{
		return row[i];
	}

	TVector3<T> Column(int i) const
	{
		return TVector3<T>(mat[0][i], mat[1][i], mat[2][i]);
	}

	T Trace() const
	{
		return mat[0][0] + mat[1][1] + mat[2][2];
	}

	inline TMatrix3<T>& operator=(const TMatrix3<T>& rhs)
	{
		memcpy(mat, rhs.mat, sizeof(mat));
		return *this;
	}

	TMatrix3<T> Inverse() const
	{
		T d = Determinant();

		return TMatrix3<T>((mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / d, -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) / d, (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / d,
						-(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) / d, (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / d, -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) / d,
						(mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / d, -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) / d, (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / d);
	}

	static TMatrix3<T> Skew(TVector3<T>& v)
	{
		return TMatrix3<T>(0, -v.z, v.y,
						v.z, 0, -v.x,
						-v.y, v.x, 0);
	}
};

typedef TMatrix3<float> Matrix3d;

static_assert(sizeof(Matrix3d) == 36, "sizeof Matrix3d is not valid");

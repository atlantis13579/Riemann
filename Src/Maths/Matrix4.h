#pragma once

#include <string.h>
#include "Vector3.h"
#include "Vector4.h"

namespace Maths
{
	class Matrix4
	{
	public:
		float mat[4][4];

		Matrix4()
		{
		}

		explicit Matrix4(float m[4][4])
		{
			memcpy(mat, m, sizeof(mat));
		}

		explicit Matrix4(float a00, float a01, float a02, float a03,
			float a10, float a11, float a12, float a13,
			float a20, float a21, float a22, float a23,
			float a30, float a31, float a32, float a33)
		{
			mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02; mat[0][3] = a03;
			mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12; mat[1][3] = a13;
			mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22; mat[2][3] = a23;
			mat[3][0] = a30; mat[3][1] = a31; mat[3][2] = a32; mat[3][3] = a33;
		}

		Matrix4(const Matrix4& m)
		{
			memcpy(mat, m.mat, sizeof(mat));
		}

		Matrix4(const Vector4& v0, const Vector4& v1, const Vector4& v2, const Vector4& v3, bool row_wise)
		{
			if (row_wise)
			{
				mat[0][0] = v0.x; mat[0][1] = v0.y; mat[0][2] = v0.z; mat[0][3] = v0.w;
				mat[1][0] = v1.x; mat[1][1] = v1.y; mat[1][2] = v1.z; mat[1][3] = v1.w;
				mat[2][0] = v2.x; mat[2][1] = v2.y; mat[2][2] = v2.z; mat[2][3] = v2.w;
				mat[3][0] = v3.x; mat[3][1] = v3.y; mat[3][2] = v3.z; mat[3][3] = v3.w;
			}
			else
			{
				mat[0][0] = v0.x; mat[0][1] = v1.x; mat[0][2] = v2.x; mat[0][3] = v3.x;
				mat[1][0] = v0.y; mat[1][1] = v1.y; mat[1][2] = v2.y; mat[1][3] = v3.y;
				mat[2][0] = v0.z; mat[2][1] = v1.z; mat[2][2] = v2.z; mat[2][3] = v3.z;
				mat[3][0] = v0.w; mat[3][1] = v1.w; mat[3][2] = v2.w; mat[3][3] = v3.w;
			}
		}

		void LoadIdentity()
		{
			mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0; mat[0][3] = 0;
			mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0; mat[1][3] = 0;
			mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1; mat[2][3] = 0;
			mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
		}

		void LoadZero()
		{
			memset(mat, 0, sizeof(mat));
		}

		inline Matrix4& operator=(const Matrix4& rhs)
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

		inline const Vector4& operator[](int i) const
		{
			const Vector4* row = static_cast<const Vector4*>((const void*)mat);
			return row[i];
		}

		inline Vector4& operator[](int i)
		{
			Vector4* row = static_cast<Vector4*>((void*)mat);
			return row[i];
		}

		inline const Vector4& Row(int i) const
		{
			const Vector4* row = static_cast<const Vector4*>((const void*)mat);
			return row[i];
		}

		inline Vector4& Row(int i)
		{
			Vector4* row = static_cast<Vector4*>((void*)mat);
			return row[i];
		}

		class ColumnProxy
		{
		public:
			operator Vector4() const
			{
				const Matrix4& mat = *m_mat;
				return Vector4(mat[0][m_index], mat[1][m_index], mat[2][m_index], mat[3][m_index]);
			}

			const Vector4& operator=(const Vector4& c)
			{
				Matrix4& mat = *m_mat;
				mat[0][m_index] = c.x;
				mat[1][m_index] = c.y;
				mat[2][m_index] = c.z;
				mat[3][m_index] = c.w;
				return c;
			}

		private:
			friend class Matrix4;

			ColumnProxy(Matrix4* _owner, int idx) : m_mat(_owner), m_index(idx) {}

			Matrix4*	m_mat{ nullptr };
			int			m_index;
		};

		inline ColumnProxy Column(int i)
		{
			return ColumnProxy(this, i);
		}

		inline Vector4 Column(int i) const
		{
			return Vector4(mat[0][i], mat[1][i], mat[2][i], mat[3][i]);
		}

		inline void SetColumn(int i, const Vector4& c)
		{
			mat[0][i] = c.x;
			mat[1][i] = c.y;
			mat[2][i] = c.z;
			mat[3][i] = c.w;
		}

		Matrix4 Transpose() const
		{
			return Matrix4(	mat[0][0], mat[1][0], mat[2][0], mat[3][0],
							mat[0][1], mat[1][1], mat[2][1], mat[3][1],
							mat[0][2], mat[1][2], mat[2][2], mat[3][2],
							mat[0][3], mat[1][3], mat[2][3], mat[3][3]);
		}

		bool Invertible() const
		{
			float d = Determinant();
			return fabsf(d) > 1e-6;
		}

		Matrix4 Inverse()
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
			return Matrix4(result);
		}

		float Trace() const
		{
			return mat[0][0] + mat[1][1] + mat[2][2] + mat[3][3];
		}

		float Determinant() const
		{
			return (mat[0][0] * (-mat[2][3] * mat[3][2] * mat[1][1] + mat[2][2] * mat[3][3] * mat[1][1] + mat[2][3] * mat[3][1] * mat[1][2] - mat[2][2] * mat[3][1] * mat[1][3] - mat[3][3] * mat[1][2] * mat[2][1] + mat[3][2] * mat[1][3] * mat[2][1]) +
				mat[0][1] * (mat[2][3] * mat[3][2] * mat[1][0] - mat[2][2] * mat[3][3] * mat[1][0] - mat[2][3] * mat[3][0] * mat[1][2] + mat[2][2] * mat[3][0] * mat[1][3] + mat[3][3] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][3] * mat[2][0]) +
				mat[0][2] * (-mat[2][3] * mat[3][1] * mat[1][0] + mat[2][3] * mat[3][0] * mat[1][1] - mat[3][3] * mat[1][1] * mat[2][0] + mat[3][1] * mat[1][3] * mat[2][0] + mat[3][3] * mat[1][0] * mat[2][1] - mat[3][0] * mat[1][3] * mat[2][1]) +
				mat[0][3] * (mat[2][2] * mat[3][1] * mat[1][0] - mat[2][2] * mat[3][0] * mat[1][1] + mat[3][2] * mat[1][1] * mat[2][0] - mat[3][1] * mat[1][2] * mat[2][0] - mat[3][2] * mat[1][0] * mat[2][1] + mat[3][0] * mat[1][2] * mat[2][1]));

		}

		float L1Norm() const
		{
			float l1_norm = 0.0f;
			const float* p = (const float*)this;
			for (int i = 0; i < 4; ++i)
				l1_norm += fabsf(p[i]);
			return l1_norm;
		}

		float L2Norm() const
		{
			float l2_norm = 0.0f;
			const float* p = (const float*)this;
			for (int i = 0; i < 4; ++i)
				l2_norm += p[i] * p[i];
			return l2_norm;
		}

		Matrix4 operator*(const Matrix4& mm) const
		{
			float m[4][4] = { 0 };
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					for (int k = 0; k < 4; k++)
						m[i][j] += mat[i][k] * mm.mat[k][j];
			return Matrix4(m);
		}

		Vector4 operator*(const Vector4& vv) const
		{
			Vector4 In(vv.x, vv.y, vv.z, vv.w);
			Vector4 out(0, 0, 0, 0);
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					out[i] += mat[i][j] * In[j];
			return out;
		}

		Matrix4 operator+(const Matrix4& mm) const
		{
			float m[4][4];
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					m[i][j] = mat[i][j] + mm.mat[i][j];
			return Matrix4(m);
		}

		Matrix4 operator-(const Matrix4& mm) const
		{
			float m[4][4];
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					m[i][j] = mat[i][j] - mm.mat[i][j];
			return Matrix4(m);
		}

		const Matrix4& operator+=(const Matrix4& mm)
		{
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mat[i][j] += mm.mat[i][j];
			return *this;
		}

		const Matrix4& operator-=(const Matrix4& mm)
		{
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mat[i][j] -= mm.mat[i][j];
			return *this;
		}

		const Matrix4& operator*=(float s)
		{
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mat[i][j] *= s;
			return *this;
		}

		const Matrix4& operator/=(float s)
		{
			float invs = 1.0f / s;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mat[i][j] *= invs;
			return *this;
		}

		static const Matrix4& Zero()
		{
			static Matrix4 mat(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			return mat;
		}

		static const Matrix4& Identity()
		{
			static Matrix4 mat(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
			return mat;
		}
	};

	template <typename T>
	inline TVector4<T> operator* (const TVector4<T>& vec, const Matrix4& mm) {
		return TVector4<T>(
			mm.mat[0][0] * vec.x + mm.mat[1][0] * vec.y + mm.mat[2][0] * vec.z + mm.mat[3][0] * vec.w,
			mm.mat[0][1] * vec.x + mm.mat[1][1] * vec.y + mm.mat[2][1] * vec.z + mm.mat[3][1] * vec.w,
			mm.mat[0][2] * vec.x + mm.mat[1][2] * vec.y + mm.mat[2][2] * vec.z + mm.mat[3][2] * vec.w,
			mm.mat[0][3] * vec.x + mm.mat[1][3] * vec.y + mm.mat[2][3] * vec.z + mm.mat[3][3] * vec.w);
	}

	template <typename T>
	inline TVector4<T> operator* (const Matrix4& mm, const TVector4<T>& vec) {
		return TVector4<T>(
			mm.mat[0][0] * vec.x + mm.mat[0][1] * vec.y + mm.mat[0][2] * vec.z + mm.mat[0][3] * vec.w,
			mm.mat[1][0] * vec.x + mm.mat[1][1] * vec.y + mm.mat[1][2] * vec.z + mm.mat[1][3] * vec.w,
			mm.mat[2][0] * vec.x + mm.mat[2][1] * vec.y + mm.mat[2][2] * vec.z + mm.mat[2][3] * vec.w,
			mm.mat[3][0] * vec.x + mm.mat[3][1] * vec.y + mm.mat[3][2] * vec.z + mm.mat[3][3] * vec.w);
	}

	static_assert(sizeof(Matrix4) == 64, "sizeof Matrix4 is not valid");
}

using Matrix4 = Maths::Matrix4;
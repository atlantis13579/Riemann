#pragma once

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <vector>

#undef max
#undef min 

#include "ShapeType.h"
#include "Sphere3d.h"
#include "../Maths/Matrix3.h"

class Capsule3d
{
public:
	Vector3		X0;
	Vector3		X1;
	float		Length;
	float		Radius;

public:
	Capsule3d() {}

	Capsule3d(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		Init(_X0, _X1, _Radius);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::CAPSULE;
	}

	void		Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		X0 = _X0;
		X1 = _X1;
		Length = (_X1 - _X0).Length();
		Radius = _Radius;
	}

public:
	inline Vector3		GetAxis() const
	{
		return X1 - X0;
	}

	inline Vector3		GetUnitAxis() const
	{
		return (X1 - X0).Unit();
	}

	inline Vector3		GetCenter() const
	{
		return (X0 + X1) * 0.5f;
	}

	inline float		GetHeight() const
	{
		return Length;
	}

	inline float		GetHalfHeight() const
	{
		return Length * 0.5f;
	}

	Box3d				GetBoundingVolume() const
	{
		Box3d box(X0, X0);
		box.Encapsulate(X1);
		box.Thicken(Radius);
		return box;
	}

	bool				IsYAxisAligned() const
	{
		return (X1 - X0).ParallelTo(Vector3::UnitY()) == 0;
	}

	bool				IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
	bool				IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool				IntersectSphere(const Vector3& rCenter, float rRadius) const;
	bool				IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const;
	bool				IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

	float				GetVolume() const
	{
		return GetVolume(Radius, GetHeight());
	}

	static float		GetVolume(float Radius, float Height)
	{
		return (float)M_PI * Radius * Radius * (Height + 4.0f * Radius / 3.0f);
	}

	Matrix3				GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, GetHeight(), Mass);
	}

	static Matrix3		GetInertiaTensor(float Radius, float Height, float Mass)
	{
		// https://www.wolframalpha.com/input/?i=capsule&assumption=%7B%22C%22,+%22capsule%22%7D+-%3E+%7B%22Solid%22%7D
		float R = Radius;
		float H = Height;
		float RR = R * R;
		float HH = H * H;

		// (5H^3 + 20*H^2R + 45HR^2 + 32R^3) / (60H + 80R)
		float Diag12 = Mass * (5.0f * HH * H + 20.0f * HH * R + 45.0f * H * RR + 32.0f * RR * R) / (60.0f * H + 80.0f * R);
		// (R^2 * (15H + 16R) / (30H +40R))
		float Diag3 = Mass * (RR * (15.0f * H + 16.0f * R)) / (30.0f * H + 40.0f * R);

		return Matrix3(Diag12, Diag12, Diag3);
	}

	Vector3 GetSupport(const Vector3& Direction) const
	{
		return GetSupport(X0, X1, Radius, Direction);
	}

	static Vector3		GetSupport(const Vector3& X0, const Vector3& X1, float Radius, const Vector3& Direction)
	{
		Vector3 Axis = X1 - X0;
		float dp = DotProduct(Direction, Axis);
		Vector3 cap = dp >= 0 ? X1 : X0;
		return cap + Direction.Unit() * Radius;
	}

	int					GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);

		// hit  top/bottom
		float len = DirectionXZ.Length();
		if (len == 0.0f)
		{
			FacePoints[0] = GetSupport(Direction);
			return 1;
		}

		Vector3 support = (Radius / len) * DirectionXZ;
		Vector3 support_top = Vector3(0, Length * 0.5f, 0) - support;
		Vector3 support_bottom = Vector3(0, -Length * 0.5f, 0) - support;

		float proj_top = support_top.Dot(Direction);
		float proj_bottom = support_bottom.Dot(Direction);

		// near parallel, hit edge
		if (fabsf(proj_top - proj_bottom) < 0.02f * Direction.Length())
		{
			FacePoints[0] = support_top;
			FacePoints[1] = support_bottom;
			return 2;
		}

		FacePoints[0] = GetSupport(Direction);
		return 1;
	}

	void				GetVertices(int stackCount, int sliceCount, std::vector<Vector3>& Vertices, std::vector<Vector3>* Normals)
	{
		const float mPI = 2.0f * asinf(1.0f);

		float phiStep = mPI / stackCount;
		float thetaStep = 2.0f * mPI / sliceCount;
		float Length = (X1 - X0).Length();

		Vertices.push_back(Vector3(0, Length * 0.5f + Radius, 0));
		if (Normals) Normals->push_back(Vector3::UnitY());

		for (int i = 1; i < stackCount; i++)
		{
			float phi = i * phiStep;
			float height = i <= stackCount / 2 ? Length * 0.5f : -Length * 0.5f;
			for (int j = 0; j <= sliceCount; j++)
			{
				float theta = j * thetaStep;
				Vector3 p = Vector3(Radius * sinf(phi) * cosf(theta), height + Radius * cosf(phi), Radius * sinf(phi) * sinf(theta));
				Vertices.push_back(p);
				if (Normals) Normals->push_back(p);
			}
		}
		Vertices.push_back(Vector3(0, -Length * 0.5f -Radius, 0));
		if (Normals) Normals->push_back(-Vector3::UnitY());

		if (!IsYAxisAligned())
		{
			Matrix3 Rot;
			Rot.FromTwoAxis(Vector3::UnitY(), X1 - X0);

			Vector3* pV = Vertices.data();
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				pV[i] = Rot * pV[i];
			}

			if (Normals)
			{
				Vector3* pN = Normals->data();
				for (size_t i = 0; i < Normals->size(); ++i)
				{
					pN[i] = Rot * pN[i];
				}
			}
		}

		Vector3 Center = GetCenter();
		if (Center.SquareLength() > 0.001f)
		{
			Vector3* pV = Vertices.data();
			for (size_t i = 0; i < Vertices.size(); ++i)
			{
				pV[i] = pV[i] + Center;
			}
		}

	}

	void				GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		const int stackCount = 5;
		const int sliceCount = 8;

		GetVertices(stackCount, sliceCount, Vertices, &Normals);

		for (int i = 1; i <= sliceCount; i++)
		{
			Indices.push_back(0);
			Indices.push_back(i + 1);
			Indices.push_back(i);
		}

		int baseIndex = 1;
		int Count = sliceCount + 1;
		for (int i = 0; i < stackCount - 2; i++)
		{
			for (int j = 0; j < sliceCount; j++)
			{
				Indices.push_back(baseIndex + i * Count + j);
				Indices.push_back(baseIndex + i * Count + j + 1);
				Indices.push_back(baseIndex + (i + 1) * Count + j);

				Indices.push_back(baseIndex + (i + 1) * Count + j);
				Indices.push_back(baseIndex + i * Count + j + 1);
				Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
			}
		}
		int PoleIndex = (int)Vertices.size() - 1;
		baseIndex = PoleIndex - Count;
		for (int i = 0; i < sliceCount; i++)
		{
			Indices.push_back(PoleIndex);
			Indices.push_back(baseIndex + i);
			Indices.push_back(baseIndex + i + 1);
		}
	}

	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		const int stackCount = 5;
		const int sliceCount = 8;

		GetVertices(stackCount, sliceCount, Vertices, nullptr);

		for (int i = 1; i <= sliceCount; i++)
		{
			Indices.push_back(0);
			Indices.push_back(i);

			Indices.push_back(i);
			Indices.push_back(i + 1);
		}

		int baseIndex = 1;
		int Count = sliceCount + 1;
		for (int i = 0; i < stackCount - 2; i++)
		{
			for (int j = 0; j < sliceCount; j++)
			{
				Indices.push_back(baseIndex + i * Count + j);
				Indices.push_back(baseIndex + i * Count + j + 1);

				Indices.push_back(baseIndex + i * Count + j + 1);
				Indices.push_back(baseIndex + (i + 1) * Count + j + 1);

				Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
				Indices.push_back(baseIndex + i * Count + j + 1);

				Indices.push_back(baseIndex + i * Count + j + 1);
				Indices.push_back(baseIndex + (i + 1) * Count + j + 1);
			}
		}
		int PoleIndex = (int)Vertices.size() - 1;
		baseIndex = PoleIndex - Count;
		for (int i = 0; i < sliceCount; i++)
		{
			Indices.push_back(PoleIndex);
			Indices.push_back(baseIndex + i);
		}
	}
};

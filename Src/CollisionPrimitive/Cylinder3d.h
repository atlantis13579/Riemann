#pragma once

#include <math.h>
#include <stdint.h>

#include "ShapeType.h"
#include "../Maths/Maths.h"
#include "../Maths/Matrix3.h"

// Approximation of top faces
static const Vector3 CylinderFaces[] =
{
	Vector3(0.0f,		1.0f,	1.0f),
	Vector3(PI_OVER_4,	1.0f,	PI_OVER_4),
	Vector3(1.0f,		1.0f,	0.0f),
	Vector3(PI_OVER_4,	1.0f,	-PI_OVER_4),
	Vector3(-0.0f,		1.0f,	-1.0f),
	Vector3(-PI_OVER_4,	1.0f,	-PI_OVER_4),
	Vector3(-1.0f,		1.0f,	0.0f),
	Vector3(-PI_OVER_4,	1.0f,	PI_OVER_4)
};

class Cylinder3d
{
public:
	float		Radius;
	float		Height;
	Vector3		X0, X1;

public:
	Cylinder3d() {}

	Cylinder3d(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		Init(_X0, _X1, _Radius);
	}

	void Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
	{
		Radius = _Radius;
		Height = (_X0 - _X1).Length();
		X0 = _X0;
		X1 = _X1;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::CYLINDER;
	}

public:

	inline Vector3 GetCenter() const
	{
		return (X0 + X1) * 0.5f;
	}

	inline Vector3 GetAxis() const
	{
		return X1 - X0;
	}

	inline Vector3	GetUnitAxis() const
	{
		return (X1 - X0).Unit();
	}

	inline float GetHeight() const
	{
		return Height;
	}

	inline float GetHalfHeight() const
	{
		return Height * 0.5f;
	}

	float GetVolume() const
	{
		return GetVolume(Radius, Height);
	}

	static float GetVolume(float Radius, float Height)
	{
		return (float)M_PI * Radius * Radius * Height;
	}

	Box3d		GetBoundingVolume() const
	{
		Box3d box(Vector3(-Radius, -Height * 0.5f, -Radius), Vector3(Radius, Height * 0.5f, Radius));
		Vector3 center = GetCenter();
		Vector3 axis = GetAxis();
		if (!axis.ParallelTo(Vector3::UnitY()) || center.SquareLength() > 1e-6)
		{
			Quaternion quat;
			quat.FromTwoAxis(Vector3::UnitY(), axis);
			box.Transform(center, quat);
		}
		return box;
	}

	Matrix3 GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Height, Mass);
	}

	static Matrix3 GetInertiaTensor(float Radius, float Height, float Mass)
	{
		// https://www.wolframalpha.com/input/?i=cylinder
		float RR = Radius * Radius;
		float Diag12 = Mass / 12.0f * (3.0f * RR + Height * Height);
		float Diag3 = Mass / 2.0f * RR;
		return Matrix3(Diag12, Diag12, Diag3);
	}

	// A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen, page 8
	Vector3		GetSupport(const Vector3& Direction) const
	{
		const float signy = Direction.y > 0 ? 1.0f : -1.0f;
		const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);
		if (o > 0)
			return Vector3(Radius * Direction.x / o, signy * Height * 0.5f, Radius * Direction.z / o);
		return Vector3(0.0f, signy * Height * 0.5f, 0.0f);
	}

	int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		const float HalfHeight = Height * 0.5f;
		const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);

		// side
		if (o * HalfHeight > Radius * fabsf(Direction.y))
		{
			FacePoints[0] = Vector3(-Direction.x * Radius / o, HalfHeight, -Direction.z * Radius / o);
			FacePoints[1] = Vector3(-Direction.x * Radius / o, -HalfHeight, -Direction.z * Radius / o);
			return 2;
		}

		// top or bottom
		const float signy = Direction.y > 0 ? 1.0f : -1.0f;
		Vector3 s(Radius, HalfHeight * signy, Radius);
		int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
		for (int i = 0; nPts; ++i)
		{
			FacePoints[i] = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
		}
		return nPts;
	}

	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		if (IntersectRayInfinityLength(Origin, Direction, Radius, t))
		{
			return false;
		}

		if (fabsf(Origin.y + (*t) * Direction.y) <= Height * 0.5f)
		{
			return true;
		}

		if (fabsf(Direction.y) > 1e-6f)
		{
			float h = Direction.y < 0.0f ? (Height * 0.5f - Origin.y) / Direction.y : -(Height * 0.5f + Origin.y) / Direction.y;

			if (h >= 0.0f)
			{
				Vector3 pos = Origin + h * Direction;
				float sq = pos.x * pos.x + pos.z * pos.z;
				if (sq <= Radius * Radius)
				{
					*t = h;
					return true;
				}
			}
		}

		return false;
	}

	static bool IntersectRayInfinityLength(const Vector3& Origin, const Vector3& Direction, float Radius, float *t)
	{
		Vector3 OriginXZ = Vector3(Origin.x, 0.0f, Origin.z);
		float sqr = OriginXZ.SquareLength();
		if (sqr > Radius * Radius)
		{
			Vector3 DirectionXZ = Vector3(Direction.x, 0.0f, Direction.z);
			float a = DirectionXZ.SquareLength();
			float b = 2.0f * OriginXZ.Dot(DirectionXZ);
			float c = sqr - Radius * Radius;

			float roots[2];
			if (SolveQuadratic(a, b, c) == 0)
			{
				return false;
			}
			float h = std::min(roots[0], roots[1]);
			if (h >= 0.0f)
			{
				*t = h;
				return true;
			}
		}
		*t = 0.0f;
		return true;
	}

	void		GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
		Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
		Vertices.resize(nPts * 2 + 2);
		Normals.resize(nPts * 2 + 2);

		Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
		Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
		Normals[2 * nPts] = Vector3::UnitY();
		Normals[2 * nPts + 1] = -Vector3::UnitY();
		for (int i = 0; i < nPts; ++i)
		{
			Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
			Vertices[2*i] = p;
			Vertices[2*i+1] = Vector3(p.x, -p.y, p.z);
			Normals[2*i] = (Vertices[2*i] - Normals[2*nPts]).Unit();
			Normals[2*i+1] = (Vertices[2*i+1] - Normals[2*nPts+1]).Unit();
		}

		Indices.reserve(3 * 4 * nPts);
		for (int i = 0; i < nPts; ++i)
		{
			int j = (i + 1) % nPts;
			Indices[12 * i + 0] = j, Indices[12 * i + 1] = i, Indices[12 * i + 2] = i + nPts;
			Indices[12 * i + 3] = j + nPts, Indices[12 * i + 4] = j, Indices[12 * i + 5] = i + nPts;
			Indices[12 * i + 6] = i, Indices[12 * i + 7] = j, Indices[12 * i + 8] = 2 * nPts;
			Indices[12 * i + 3] = i + nPts, Indices[12 * i + 4] = j + nPts, Indices[12 * i + 5] = 2 * nPts + 1;
		}
	}

	void		GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		int nPts = sizeof(CylinderFaces) / sizeof(CylinderFaces[0]);
		Vector3 s = Vector3(Radius, Height * 0.5f, Radius);
		Vertices.resize(nPts * 2 + 2);
		Vertices[2 * nPts] = Vector3(0.0f, Height * 0.5f, 0.0f);
		Vertices[2 * nPts + 1] = Vector3(0.0f, -Height * 0.5f, 0.0f);
		for (int i = 0; i < nPts; ++i)
		{
			Vector3 p = Vector3(CylinderFaces[i].x * s.x, CylinderFaces[i].y * s.y, CylinderFaces[i].z * s.z);
			Vertices[2 * i] = p;
			Vertices[2 * i + 1] = Vector3(p.x, -p.y, p.z);
		}

		Indices.reserve(2 * 3 * nPts);
		for (int i = 0; i < nPts; ++i)
		{
			int j = (i + 1) % nPts;
			Indices[6 * i + 0] = j, Indices[6 * i + 1] = i;
			Indices[6 * i + 2] = j + nPts, Indices[6 * i + 3] = i + nPts;
			Indices[6 * i + 4] = i, Indices[6 * i + 5] = i + nPts;
		}
	}
};

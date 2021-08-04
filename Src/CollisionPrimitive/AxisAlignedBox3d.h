
#pragma once

#include "ShapeType.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/Box3d.h"

class AxisAlignedBox3d
{
public:
	Vector3d Min;
	Vector3d Max;

public:
	AxisAlignedBox3d()
	{
	}

	AxisAlignedBox3d(const Vector3d& Bmin, const Vector3d& Bmax)
	{
		Min = Bmin;
		Max = Bmax;
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::BOX;
	}

public:
	bool			IntersectPoint(const Vector3d& Point) const
	{
		if (Point.x >= Min.x && Point.x <= Max.x &&
			Point.y >= Min.y && Point.y <= Max.y &&
			Point.z >= Min.z && Point.z <= Max.z)
		{
			return true;
		}
		return false;
	}

	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
	{
		if (Min.x > Bmax.x || Bmin.x > Max.x)
		{
			return false;
		}

		if (Min.y > Bmax.y || Bmin.y > Max.y)
		{
			return false;
		}

		if (Min.z > Bmax.z || Bmin.z > Max.z)
		{
			return false;
		}

		return true;
	}

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		const Vector3d b0 = Min - Origin;
		const Vector3d b1 = Max - Origin;

		float tMin = 0;
		float tMax = FLT_MAX;
		Vector3d Normal(0.0f);

		for (int i = 0; i < 3; ++i)
		{
			float t0, t1;
			if (fabsf(Dir[i]) < 0.00001f)
			{
				if (b0[i] > 0 || b1[i] < 0)
				{
					return false;
				}
				else
				{
					t0 = 0;
					t1 = FLT_MAX;
				}
			}
			else
			{
				const float InvDir = 1.0f / Dir[i];
				t0 = b0[i] * InvDir;
				t1 = b1[i] * InvDir;
			}

			Vector3d CurNormal = Vector3d(0.0f);
			CurNormal[i] = 1.0f;

			if (t0 > t1)
			{
				std::swap(t0, t1);
			}
			else
			{
				CurNormal[i] = -1.0f;
			}

			if (t0 > tMin)
			{
				Normal = CurNormal;
			}
			tMin = std::max(tMin, t0);
			tMax = std::min(tMax, t1);

			if (tMin > tMax)
			{
				return false;
			}
		}

		if (tMax < 0)
		{
			return false;
		}

		*t = tMin;
		return true;
	}

	Box3d			GetBoundingVolume() const
	{
		return Box3d(Min, Max);
	}

	float			GetVolume() const
	{
		return GetVolume(Min, Max);
	}

	static float	GetVolume(const Vector3d& Bmin, const Vector3d& Bmax)
	{
		return ((Bmax.x - Bmin.x) * (Bmax.y - Bmin.y) * (Bmax.z - Bmin.z));
	}

	Vector3d		GetCenterOfMass() const
	{
		return (Max + Min) * 0.5f;
	}

	Matrix3d		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Min, Max, Mass);
	}

	static Matrix3d GetInertiaTensor(const Vector3d& Bmin, const Vector3d& Bmax, float Mass)
	{
		Vector3d Dim = Bmax - Bmin;

		// https://www.wolframalpha.com/input/?i=cuboid
		const float M = Mass / 12;
		const float WW = Dim.x * Dim.x;
		const float HH = Dim.y * Dim.y;
		const float DD = Dim.z * Dim.z;
		return Matrix3d(M * (HH + DD), M * (WW + DD), M * (WW + HH));
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		return GetSupport(Min, Max, dir);
	}

	static Vector3d GetSupport(const Vector3d& Bmin, const Vector3d& Bmax, const Vector3d& dir)
	{
		return Vector3d(
			dir.x > 0 ? Bmax.x : Bmin.x,
			dir.y > 0 ? Bmax.y : Bmin.y,
			dir.z > 0 ? Bmax.z : Bmin.z
		);
	}

	void			GetMesh(std::vector<Vector3d> &Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		Vertices.resize(8);
		Box3d::GetVertices(Min, Max, &Vertices[0]);
		
		Vector3d Center = GetCenterOfMass();

		Normals.resize(8);
		for (int i = 0; i < 8; ++i)
		{
			Normals[i] = Vertices[i] - Center;
		}

		Indices = {
			0, 1, 2,
			1, 3, 2,
			4, 5, 6,
			5, 7, 6,
			0, 1, 4,
			5, 4, 1,
			1, 3, 5,
			7, 5, 3,
			2, 4, 0,
			6, 4, 2,
			3, 2, 6,
			6, 7, 3 };
	}

	void			GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices.resize(8);
		Box3d::GetVertices(Min, Max, &Vertices[0]);
		Indices = {
			0, 1, 1, 3, 3, 2, 2, 0,
			0, 4, 1, 5, 3, 7, 2, 6,
			4, 5, 5, 7, 7, 6, 6, 4 };
	}
};
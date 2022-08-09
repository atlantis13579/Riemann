
#pragma once

#include <float.h>
#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

class AxisAlignedBox3d
{
public:
	Vector3 Min;
	Vector3 Max;

public:
	AxisAlignedBox3d()
	{
	}

	AxisAlignedBox3d(const Vector3& Bmin, const Vector3& Bmax)
	{
		Min = Bmin;
		Max = Bmax;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::BOX;
	}

public:
	bool			IntersectPoint(const Vector3& Point) const
	{
		if (Point.x >= Min.x && Point.x <= Max.x &&
			Point.y >= Min.y && Point.y <= Max.y &&
			Point.z >= Min.z && Point.z <= Max.z)
		{
			return true;
		}
		return false;
	}

	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
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

	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		const Vector3 b0 = Min - Origin;
		const Vector3 b1 = Max - Origin;

		float tMin = 0;
		float tMax = FLT_MAX;
		Vector3 Normal(0.0f);

		for (int i = 0; i < 3; ++i)
		{
			float t0, t1;
			if (fabsf(Direction[i]) < 0.00001f)
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
				const float InvDir = 1.0f / Direction[i];
				t0 = b0[i] * InvDir;
				t1 = b1[i] * InvDir;
			}

			Vector3 CurNormal = Vector3(0.0f);
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

	bool			IntersectSphere(const Vector3& Center, float Radius) const;
	bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const;
	bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

	Vector3			ClosestPointTo(const Vector3& Point) const;
	float			SqrDistanceToPoint(const Vector3& Point) const;
	float			SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const;
	float			SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;

	Vector3			GetCenter() const
	{
		return (Max + Min) * 0.5f;
	}

	Vector3			GetExtent() const
	{
		return (Max - Min) * 0.5f;
	}

	Box3d			GetBoundingVolume() const
	{
		return Box3d(Min, Max);
	}

	float			GetVolume() const
	{
		return GetVolume(Min, Max);
	}

	static float	GetVolume(const Vector3& Bmin, const Vector3& Bmax)
	{
		return ((Bmax.x - Bmin.x) * (Bmax.y - Bmin.y) * (Bmax.z - Bmin.z));
	}

	Vector3		GetCenterOfMass() const
	{
		return (Max + Min) * 0.5f;
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Min, Max, Mass);
	}

	static Matrix3 GetInertiaTensor(const Vector3& Bmin, const Vector3& Bmax, float Mass)
	{
		Vector3 Dim = Bmax - Bmin;

		// https://www.wolframalpha.com/input/?i=cuboid
		const float M = Mass / 12;
		const float WW = Dim.x * Dim.x;
		const float HH = Dim.y * Dim.y;
		const float DD = Dim.z * Dim.z;
		return Matrix3(M * (HH + DD), M * (WW + DD), M * (WW + HH));
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		return GetSupport(Min, Max, Direction);
	}

	static Vector3 GetSupport(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction)
	{
		return Vector3(
			Direction.x > 0 ? Bmax.x : Bmin.x,
			Direction.y > 0 ? Bmax.y : Bmin.y,
			Direction.z > 0 ? Bmax.z : Bmin.z
		);
	}

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		return AxisAlignedBox3d::GetSupportFace(Min, Max, Direction, FacePoints);
	}

	static int		GetSupportFace(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction, Vector3* FacePoints)
	{
		int axis = Direction.Abs().LargestAxis();
		if (Direction[axis] < 0.0f)
		{
			switch (axis)
			{
			case 0:
				FacePoints[0] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				break;

			case 1:
				FacePoints[0] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				break;

			case 2:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				FacePoints[1] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				break;
			}
		}
		else
		{
			switch (axis)
			{
			case 0:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				FacePoints[2] = Vector3(Bmin.x, Bmax.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				break;

			case 1:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmin.y, Bmax.z);
				FacePoints[3] = Vector3(Bmin.x, Bmin.y, Bmax.z);
				break;

			case 2:
				FacePoints[0] = Vector3(Bmin.x, Bmin.y, Bmin.z);
				FacePoints[1] = Vector3(Bmin.x, Bmax.y, Bmin.z);
				FacePoints[2] = Vector3(Bmax.x, Bmax.y, Bmin.z);
				FacePoints[3] = Vector3(Bmax.x, Bmin.y, Bmin.z);
				break;
			}
		}

		return 4;
	}

	void			GetMesh2(std::vector<Vector3> &Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Vertices.resize(8);
		Box3d::GetVertices(Min, Max, &Vertices[0]);
		
		Vector3 Center = GetCenterOfMass();

		Normals.resize(8);
		for (int i = 0; i < 8; ++i)
		{
			Normals[i] = Vertices[i] - Center;
		}

		Indices = std::vector<uint16_t>({
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
			6, 7, 3 });
	}

	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Vertices.resize(36);
		Indices.resize(36);
		Normals.resize(36);

		Vector3 BV[] = { Min, Max };

#define SET_VERTICES(_idx, _x, _y, _z)	\
		Vertices[_idx] = Vector3(BV[_x].x, BV[_y].y, BV[_z].z);	\
		Indices[_idx] = (_idx);	\
		Normals[_idx] = (_z == 0) ? -Vector3::UnitZ() : Vector3::UnitZ();	\
		Vertices[_idx + 12] = Vector3(BV[_y].x, BV[_z].y, BV[_x].z);	\
		Indices[_idx + 12] = (_idx + 12);	\
		Normals[_idx + 12] = (_z == 0) ? -Vector3::UnitY() : Vector3::UnitY();	\
		Vertices[_idx + 24] = Vector3(BV[_z].x, BV[_x].y, BV[_y].z);	\
		Indices[_idx + 24] = (_idx + 24);	\
		Normals[_idx + 24] = (_z == 0) ? -Vector3::UnitX() : Vector3::UnitX();	\

		SET_VERTICES(0, 0, 0, 0);
		SET_VERTICES(1, 1, 0, 0);
		SET_VERTICES(2, 0, 1, 0);
		SET_VERTICES(3, 1, 0, 0);
		SET_VERTICES(4, 0, 1, 0);
		SET_VERTICES(5, 1, 1, 0);
		SET_VERTICES(6, 0, 0, 1);
		SET_VERTICES(7, 1, 0, 1);
		SET_VERTICES(8, 0, 1, 1);
		SET_VERTICES(9, 1, 0, 1);
		SET_VERTICES(10, 0, 1, 1);
		SET_VERTICES(11, 1, 1, 1);

#undef SET_VERTICES
	}

	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices.resize(8);
		Box3d::GetVertices(Min, Max, &Vertices[0]);
		Indices = std::vector<uint16_t>({
			0, 1, 1, 3, 3, 2, 2, 0,
			0, 4, 1, 5, 3, 7, 2, 6,
			4, 5, 5, 7, 7, 6, 6, 4 });
	}
};

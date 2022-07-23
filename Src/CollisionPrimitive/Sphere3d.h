
#pragma once

#include <math.h>
#include <stdint.h>

#include "ShapeType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

class Sphere3d
{
public:
	Vector3d Center;
	float Radius;

public:
	Sphere3d()
	{
	}

	Sphere3d(const Vector3d& InCenter, float InRadius)
	{
		Center = InCenter;
		Radius = InRadius;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::SPHERE;
	}

public:
	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		return RayIntersectSphere(Origin, Dir, Center, Radius, t);
	}

	static bool		RayIntersectSphere(const Vector3d& Origin, const Vector3d& Dir, const Vector3d& Center, float Radius, float* t)
	{
		Vector3d oc = Origin - Center;
		float a = Dir.SquareLength();
		float b = 2.0f * oc.Dot(Dir);
		float c = oc.SquareLength() - Radius * Radius;
		float discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
		{
			return false;
		}
		float h = (-b - sqrtf(discriminant)) / (2.0f * a);
		if (h >= 0)
		{
			*t = h;
			return true;
		}
		return false;
	}

	bool			IntersectPoint(const Vector3d& Point) const
	{
		float sqr_dist = (Point - Center).SquareLength();
		if (sqr_dist <= Radius * Radius)
		{
			return true;
		}
		return false;
	}

	// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
	{
		float dmin = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			if (Center[i] < Bmin[i])
			{
				dmin += (Center[i] - Bmin[i]) * (Center[i] - Bmin[i]);
			}
			else if (Center[i] > Bmax[i])
			{
				dmin += (Center[i] - Bmax[i]) * (Center[i] - Bmax[i]);
			}
		}

		if (dmin <= Radius * Radius)
		{
			return true;
		}
		return false;
	}

	bool			IntersectSphere(const Vector3d &rCenter, float rRadius) const
	{
		return SphereIntersectSphere(Center, Radius, rCenter, rRadius);
	}

	static bool		SphereIntersectSphere(const Vector3d& Center, float Radius, const Vector3d& rCenter, float rRadius)
	{
		float SqrDist = (Center - rCenter).SquareLength();
		return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
	}

	Box3d			GetBoundingVolume() const
	{
		Box3d Box;
		Box.BuildFromCenterAndExtent(Center, Vector3d(Radius));
		return Box;
	}

	float			GetVolume() const
	{
		const float FourThirdsPI = (4.0f / 3.0f) * (float)M_PI;
		return FourThirdsPI * Radius * Radius * Radius;
	}

	Vector3d		GetCenterOfMass() const
	{
		return Center;
	}

	Matrix3d		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Mass);
	}

	static Matrix3d GetInertiaTensor(float Radius, float Mass)
	{
		const float Diagonal = 2.0f * Mass * Radius * Radius / 5.0f;
		return Matrix3d(Diagonal, Diagonal, Diagonal);
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		return GetSupport(Center, Radius, dir);
	}

	static Vector3d GetSupport(const Vector3d& Center, float Radius, const Vector3d& dir)
	{
		float distSqr = dir.SquareLength();
		if (distSqr <= 1e-6)
		{
			return Center;
		}
		Vector3d Normalized = dir / sqrtf(distSqr);
		return Center + Normalized * Radius;
	}

	int				GetSupportFace(const Vector3d& dir, Vector3d* FacePoints) const
	{
		*FacePoints = Sphere3d::GetSupport(Center, Radius, dir);
		return 1;
	}

	void			GetVertices(int stackCount, int sliceCount, std::vector<Vector3d>* Vertices, std::vector<Vector3d>* Normals)
	{
		const float mPI = 2.0f * asinf(1.0f);

		float phiStep = mPI / stackCount;
		float thetaStep = 2.0f * mPI / sliceCount;

		Vertices->push_back(Vector3d(0, Radius, 0));
		if (Normals) Normals->push_back(Vector3d::UnitY());

		for (int i = 1; i < stackCount; i++)
		{
			float phi = i * phiStep;
			for (int j = 0; j <= sliceCount; j++)
			{
				float theta = j * thetaStep;
				Vector3d p = Vector3d(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta)) * Radius;
				Vertices->push_back(p);
				if (Normals) Normals->push_back(p - Center);
			}
		}
		Vertices->push_back(Vector3d(0, -Radius, 0));
		if (Normals) Normals->push_back(-Vector3d::UnitY());
	}

	void			GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		const int stackCount = 8;
		const int sliceCount = 12;

		GetVertices(stackCount, sliceCount, &Vertices, &Normals);

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

	void			GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{
		const int stackCount = 6;
		const int sliceCount = 8;

		GetVertices(stackCount, sliceCount, &Vertices, nullptr);

		for (int i = 1; i <= sliceCount; i++)
		{
			Indices.push_back(0);
			Indices.push_back(i);

			Indices.push_back(i);
			Indices.push_back(i+1);
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

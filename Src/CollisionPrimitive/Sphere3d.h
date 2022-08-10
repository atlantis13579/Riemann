
#pragma once

#include <math.h>
#include <stdint.h>

#include "ShapeType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3d.h"

class Sphere3d
{
public:
	Vector3 Center;
	float Radius;

public:
	Sphere3d()
	{
	}

	Sphere3d(const Vector3& _Center, float _Radius)
	{
		Center = _Center;
		Radius = _Radius;
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::SPHERE;
	}

public:
	bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		return RayIntersectSphere(Origin, Direction, Center, Radius, t);
	}

	static bool		RayIntersectSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& Center, float Radius, float* t)
	{
		Vector3 oc = Origin - Center;
		float a = Direction.SquareLength();
		float b = 2.0f * oc.Dot(Direction);
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

	bool			IntersectPoint(const Vector3& Point) const
	{
		float sqr_dist = (Point - Center).SquareLength();
		if (sqr_dist <= Radius * Radius)
		{
			return true;
		}
		return false;
	}

	// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
	bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
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

	bool			IntersectSphere(const Vector3 &rCenter, float rRadius) const
	{
		return SphereIntersectSphere(Center, Radius, rCenter, rRadius);
	}

	static bool		SphereIntersectSphere(const Vector3& Center, float Radius, const Vector3& rCenter, float rRadius)
	{
		float SqrDist = (Center - rCenter).SquareLength();
		return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
	}

	Box3d			GetBoundingVolume() const
	{
		Box3d Box;
		Box.BuildFromCenterAndExtent(Center, Vector3(Radius));
		return Box;
	}

	float			GetVolume() const
	{
		const float FourThirdsPI = (4.0f / 3.0f) * (float)M_PI;
		return FourThirdsPI * Radius * Radius * Radius;
	}

	Vector3		GetCenterOfMass() const
	{
		return Center;
	}

	Matrix3		GetInertiaTensor(float Mass) const
	{
		return GetInertiaTensor(Radius, Mass);
	}

	static Matrix3 GetInertiaTensor(float Radius, float Mass)
	{
		const float Diagonal = 2.0f * Mass * Radius * Radius / 5.0f;
		return Matrix3(Diagonal, Diagonal, Diagonal);
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		return GetSupport(Center, Radius, Direction);
	}

	static Vector3 GetSupport(const Vector3& Center, float Radius, const Vector3& Direction)
	{
		float distSqr = Direction.SquareLength();
		if (distSqr <= 1e-6)
		{
			return Center;
		}
		Vector3 Normalized = Direction / sqrtf(distSqr);
		return Center + Normalized * Radius;
	}

	int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		*FacePoints = Sphere3d::GetSupport(Center, Radius, Direction);
		return 1;
	}

	void			GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals)
	{
		const float mPI = 2.0f * asinf(1.0f);

		float phiStep = mPI / stackCount;
		float thetaStep = 2.0f * mPI / sliceCount;

		Vertices->push_back(Vector3(0, Radius, 0));
		if (Normals) Normals->push_back(Vector3::UnitY());

		for (int i = 1; i < stackCount; i++)
		{
			float phi = i * phiStep;
			for (int j = 0; j <= sliceCount; j++)
			{
				float theta = j * thetaStep;
				Vector3 p = Vector3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta)) * Radius;
				Vertices->push_back(p);
				if (Normals) Normals->push_back(p - Center);
			}
		}
		Vertices->push_back(Vector3(0, -Radius, 0));
		if (Normals) Normals->push_back(-Vector3::UnitY());
	}

	void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
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

	void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
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

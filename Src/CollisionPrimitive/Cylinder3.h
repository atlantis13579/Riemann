#pragma once

#include <math.h>
#include <vector>

#include "PrimitiveType.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class ConvexMesh;
	class HeightField3;
	class TriangleMesh;

	class Cylinder3
	{
	public:
		float		Radius;
		float		Height;
		Vector3		X0, X1;

	public:
		Cylinder3()
		{
			Init(Vector3(0.0f, -1.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f), 1.0f);
		}

		Cylinder3(const Vector3& _X0, const Vector3& _X1, float _Radius)
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

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::CYLINDER;
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

		bool	CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->BoundingVolume = CalculateBoundingVolume();
			p->CenterOfMass = p->BoundingVolume.GetCenter();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		float CalculateVolume() const
		{
			return CalculateVolume(Radius, Height);
		}

		static float CalculateVolume(float Radius, float Height)
		{
			return (float)M_PI * Radius * Radius * Height;
		}

		Box3 CalculateBoundingVolume() const
		{
			Box3 box(Vector3(-Radius, -Height * 0.5f, -Radius), Vector3(Radius, Height * 0.5f, Radius));
			Vector3 center = GetCenter();
			Vector3 axis = GetAxis();
			if (!axis.ParallelTo(Vector3::UnitY()) || center.SquareLength() > 1e-6)
			{
				Quaternion quat;
				quat.FromTwoAxis(Vector3::UnitY(), axis);
				box = Box3::Transform(box, center, quat);
			}
			return box;
		}

		Matrix3 CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Radius, Height, Mass);
		}

		static Matrix3 CalculateInertiaTensor(float Radius, float Height, float Mass)
		{
			// https://www.wolframalpha.com/input/?i=cylinder
			float RR = Radius * Radius;
			float Diag12 = Mass / 12.0f * (3.0f * RR + Height * Height);
			float Diag3 = Mass / 2.0f * RR;
			return Matrix3(Diag12, Diag12, Diag3);
		}

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		static bool IntersectRayInfinityLength(const Vector3& Origin, const Vector3& Direction, float Radius, float* t);
		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const;

		bool SweepAABB(const Vector3& Origin, const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const;
		bool SweepSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const;
		bool SweepPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const;
		bool SweepCapsule(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const;
		bool SweepConvex(const Vector3& Origin, const Vector3& Direction, const ConvexMesh* convex, Vector3* n, float* t) const;
		bool SweepTriangle(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool SweepHeightField(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool SweepTriangleMesh(const Vector3& Origin, const Vector3& Direction, const TriangleMesh* trimesh, Vector3* n, float* t) const;

		// A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen, page 8
		Vector3 GetSupport(const Vector3& Direction) const
		{
			const float sy = Direction.y > 0 ? 1.0f : -1.0f;
			const float o = sqrtf(Direction.x * Direction.x + Direction.z * Direction.z);
			if (o > 1e-6f)
			{
				Vector3 support(Radius * Direction.x / o, sy * Height * 0.5f, Radius * Direction.z / o);
				return support;
			}
			return Vector3(0.0f, sy * Height * 0.5f, 0.0f);
		}

		static const Vector3* GetCylinderFaces()
		{
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
			return CylinderFaces;
		}

		int GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
	};

	static_assert(sizeof(Cylinder3) == 32, "sizeof(Cylinder3) not right");
}
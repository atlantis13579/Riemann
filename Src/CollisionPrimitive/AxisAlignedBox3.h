#pragma once

#include <vector>

#include "PrimitiveType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class ConvexMesh;
	class HeightField3;
	class TriangleMesh;

	class AxisAlignedBox3
	{
	public:
		Vector3 Min;
		Vector3 Max;

	public:
		AxisAlignedBox3()
		{
			Min = -Vector3::One();
			Max = Vector3::One();
		}

		AxisAlignedBox3(const Vector3& Bmin, const Vector3& Bmax)
		{
			Min = Bmin;
			Max = Bmax;
		}

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::BOX;
		}

	public:
		bool			IntersectPoint(const Vector3& Point) const;
		bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool			IntersectSegment(const Vector3& P0, const Vector3& P1) const;
		bool			IntersectSphere(const Vector3& Center, float Radius) const;
		bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const;
		bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

		Vector3			ClosestPointToPoint(const Vector3& Point) const;
		float			SqrDistanceToPoint(const Vector3& Point) const;
		float			SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const;
		float			SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;

		bool			SweepAABB(const Vector3& Origin, const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const;
		bool			SweepSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const;
		bool			SweepPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const;
        bool        SweepCylinder(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const;
		bool			SweepCapsule(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const;
		bool			SweepConvex(const Vector3& Origin, const Vector3& Direction, const ConvexMesh* convex, Vector3* n, float* t) const;
		bool			SweepTriangle(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool			SweepHeightField(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool			SweepTriangleMesh(const Vector3& Origin, const Vector3& Direction, const TriangleMesh* trimesh, Vector3* n, float* t) const;

		Vector3			GetCenter() const
		{
			return (Max + Min) * 0.5f;
		}

		Vector3			GetExtent() const
		{
			return (Max - Min) * 0.5f;
		}

		Box3			CalculateBoundingVolume() const
		{
			return Box3(Min, Max);
		}

		float			CalculateVolume() const
		{
			return CalculateVolume(Min, Max);
		}

		static float	CalculateVolume(const Vector3& Bmin, const Vector3& Bmax)
		{
			return ((Bmax.x - Bmin.x) * (Bmax.y - Bmin.y) * (Bmax.z - Bmin.z));
		}

		bool			CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->BoundingVolume = CalculateBoundingVolume();
			p->CenterOfMass = p->BoundingVolume.GetCenter();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		Vector3		GetCenterOfMass() const
		{
			return (Max + Min) * 0.5f;
		}

		Matrix3		CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Min, Max, Mass);
		}

		static Matrix3 CalculateInertiaTensor(const Vector3& Bmin, const Vector3& Bmax, float Mass)
		{
			Vector3 Dim = Bmax - Bmin;

			// https://www.wolframalpha.com/input/?i=cuboid
			const float M = Mass / 12;
			const float WW = Dim.x * Dim.x;
			const float HH = Dim.y * Dim.y;
			const float DD = Dim.z * Dim.z;
			return Matrix3(M * (HH + DD), M * (WW + DD), M * (WW + HH));
		}

		Vector3			GetSupport(const Vector3& Direction) const;
		static Vector3	GetSupport(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction);
		int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;
		static int		GetSupportFace(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction, Vector3* FacePoints);

		void			GetMesh2(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
		void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
		void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
	};
}

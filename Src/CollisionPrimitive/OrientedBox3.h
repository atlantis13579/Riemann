#pragma once

#include "PrimitiveType.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
	class OrientedBox3
	{
	public:
		Vector3	Center;
		Vector3	Extent;
		Matrix3	Rotation;

		OrientedBox3()
		{
		}

		OrientedBox3(const Vector3& iCenter, const Vector3& iExtent, const Matrix3& _Rot)
		{
			Center = iCenter;
			Extent = iExtent;
			Rotation = _Rot;
		}

		inline Vector3		GetAxis(int i) const
		{
			return Rotation.Column(i);
		}

		void GetVertices(Vector3 v[8]) const
		{
			v[0] = Center + Rotation * Vector3(Extent.x, Extent.y, Extent.z);
			v[1] = Center + Rotation * Vector3(-Extent.x, Extent.y, Extent.z);
			v[2] = Center + Rotation * Vector3(Extent.x, -Extent.y, Extent.z);
			v[3] = Center + Rotation * Vector3(-Extent.x, -Extent.y, Extent.z);
			v[4] = Center + Rotation * Vector3(Extent.x, Extent.y, -Extent.z);
			v[5] = Center + Rotation * Vector3(-Extent.x, Extent.y, -Extent.z);
			v[6] = Center + Rotation * Vector3(Extent.x, -Extent.y, -Extent.z);
			v[7] = Center + Rotation * Vector3(-Extent.x, -Extent.y, -Extent.z);
		}

		static OrientedBox3 ComputeBoundingOBB_PCA(const Vector3* points, int n);

		static Box3 ComputeBoundingVolume(const Vector3& Center, const Vector3& Extent, const Matrix3& Rot)
		{
			Box3 box;
			box.SetEmpty();
			box.Encapsulate(Rot * (Center + Vector3(Extent.x, Extent.y, Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(Extent.x, Extent.y, -Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(Extent.x, -Extent.y, Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(Extent.x, -Extent.y, -Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(-Extent.x, Extent.y, Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(-Extent.x, Extent.y, -Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(-Extent.x, -Extent.y, Extent.z)));
			box.Encapsulate(Rot * (Center + Vector3(-Extent.x, -Extent.y, -Extent.z)));
			return box;
		}

		Box3 CalculateBoundingVolume() const
		{
			return OrientedBox3::ComputeBoundingVolume(Center, Extent, Rotation);
		}

		bool IntersectPoint(const Vector3& point) const;
		bool IntersectPlane(const Vector3& normal, const float D) const;
		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool IntersectOBB(const OrientedBox3& obb) const;
		bool IntersectOBB(const Vector3& iCenter, const Vector3& iExtent, const Matrix3& iRot) const;
		bool IntersectSphere(const Vector3& iCenter, float iRadius) const;
		bool IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const;
		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

		bool PenetrateOBB(const OrientedBox3& obb, Vector3* Normal, float* Depth) const;
		bool PenetrateOBB(const Vector3& iCenter, const Vector3& iExtent, const Matrix3& iRot, Vector3* Normal, float* Depth) const;
		bool PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const;
		bool PenetratePlane(const Vector3& pNormal, float D, Vector3* Normal, float* Depth) const;

		void ProjectAxis(const Vector3& Axis, float* t0, float* t1) const
		{
			const float c = Center.Dot(Axis);
			const float e = fabsf(Rotation.Column(0).Dot(Axis)) * Extent.x
							+ fabsf(Rotation.Column(1).Dot(Axis)) * Extent.y
							+ fabsf(Rotation.Column(2).Dot(Axis)) * Extent.z;
			*t0 = c - e;
			*t1 = c + e;
		}

		Vector3 ClosestPointToPoint(const Vector3& Point) const;
		float SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const;
		float SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const;
		float SqrDistanceToPoint(const Vector3& Point) const;

	private:

	};
}
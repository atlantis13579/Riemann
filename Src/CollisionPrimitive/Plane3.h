#pragma once

#include <vector>

#include "PrimitiveType.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class Plane3
	{
	public:
		Vector3 Normal;    //  P * Normal + D = 0
		float 	D;

	public:
		Plane3()
		{
			Normal = Vector3::UnitY();
			D = 0.0f;
		}

		Plane3(const Vector3& iNormal, Vector3& iOrigin)
		{
			Normal = iNormal.Unit();
			D = -Normal.Dot(iOrigin);
		}

		Plane3(const Vector3& InNormal, float InD)
		{
			Normal = InNormal.Unit();
			D = InD;
		}

		Plane3(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			const bool bHighPrecisionNormal = true;
			if (bHighPrecisionNormal)
			{
				Vector3 CA = (C - A).Unit();
				Vector3 BA = (B - A).Unit();
				Normal = CA.Cross(BA).Unit();
			}
			else
			{
				Vector3 CA = C - A;
				Vector3 BA = B - A;
				Normal = CA.Cross(BA).Unit();
			}
			D = -Normal.Dot(A);
		}

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::PLANE;
		}

	public:
		inline Vector3	GetOrigin() const
		{
			return -Normal * D;
		}

		inline float	MinusD() const
		{
			return -D;
		}

		inline Vector3	GetCenter() const
		{
			return -Normal * D;
		}

		void			Shift(float margin)
		{
			Vector3 p = GetOrigin();
			D = -margin - p.Dot(Normal);
		}

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
		{
			return RayIntersectPlane(Origin, Direction, Normal, D, t);
		}

		bool IntersectSegment(const Vector3& P0, const Vector3& P1) const
		{
			Vector3 e = P1 - P0;
			float t = (-D - Normal.Dot(P0)) / Normal.Dot(e);
			if (t >= 0.0f && t <= 1.0f)
			{
				// q = a + t * e;	intersect point
				return true;
			}
			return false;
		}

		static bool RayIntersectPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, float* t)
		{
			const float det = Direction.Dot(Normal);
			if (det > -kEpsilonPlane && det < kEpsilonPlane) {
				return false;
			}
			*t = -(Origin.Dot(Normal) + D) / det;
			return *t >= 0.0;
		}

		bool IntersectPoint(const Vector3& Point) const
		{
			const float det = Point.Dot(Normal) + D;

			if (fabsf(det) < kEpsilonPlane)
			{
				return true;
			}
			return false;
		}

		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
		{
			Vector3 v[8];
			Box3::GetVertices(Bmin, Bmax, v);
			int p0 = 0, p1 = 0, p2 = 0;
			for (int i = 0; i < 8; ++i)
			{
				const float det = v[i].Dot(Normal) + D;
				if (det > kEpsilonPlane)
					p2++;
				else if (det < -kEpsilonPlane)
					p1++;
				else
					p0++;
			}

			if (p1 > 0 && p2 > 0)
			{
				return true;
			}

			if (p0 == 0)
			{
				return false;
			}

			return true;
		}

		bool IntersectSphere(const Vector3& rCenter, float rRadius) const
		{
			float Dist = DistanceToPoint(rCenter);
			return Dist <= rRadius;
		}

		bool IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const
		{
			float Dist = DistanceToSegment(P0, P1);
			return Dist <= Radius;
		}
        
        bool IntersectDisk(const Vector3& P0, const Vector3& DiskNormal, float Radius) const
        {
            const float cos_theta = DiskNormal.Dot(Normal);
            const float dist = DistanceToPoint(P0);
            const float proj_radius = sqrtf(1.0f - cos_theta * cos_theta) * Radius;
            return dist <= proj_radius;
        }

		bool IntersectCylinder(const Vector3& P0, const Vector3& P1, float Radius) const
		{
            const Vector3 DiskNormal = (P1 - P0).Unit();
            const float dp = DiskNormal.Dot(Normal);
            if (fabsf(dp) < 1e-6f)
            {
                const float dist = DistanceToPoint(P0);
                return dist <= Radius;
            }
            else if (fabsf(1.0f - fabsf(dp)) < 1e-6f )
            {
                const float sign0 = SignedDistanceToPoint(P0);
                const float sign1 = SignedDistanceToPoint(P1);
                return sign0 * sign1 <= 0.0f;
            }
            else
            {
                return IntersectDisk(P0, DiskNormal, Radius) || IntersectDisk(P1, DiskNormal, Radius);
            }
			return false;
		}

		bool IntersectPlane(const Vector3& iNormal, float iD) const
		{
			const bool Parallel = Normal.ParallelTo(iNormal);
			if (Parallel)
			{
				return D == iD;
			}
			return true;
		}

		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
		{
			float Dist = DistanceToTriangle(A, B, C);
			return Dist > 0.0f;
		}

		bool PenetrateSphere(const Vector3& Center, float Radius, Vector3* normal, float* depth) const
		{
			const float d = SignedDistanceToPoint(Center);
			if (d > Radius)
			{
				return false;
			}

			*normal = -Normal;
			*depth = Radius - d;
			return true;
		}

		bool PenetrateOBB(const Vector3& rCenter, const Vector3& rExtent, const Matrix3& rRot, Vector3* normal, float* depth) const
		{
			Vector3 v[8];

			const Vector3 base0 = rRot.Column(0);
			const Vector3 base1 = rRot.Column(1);
			const Vector3 base2 = rRot.Column(2);

			const Vector3 axis0 = base0 * rExtent.x;
			const Vector3 axis1 = base1 * rExtent.y;
			const Vector3 axis2 = base2 * rExtent.z;

			//     7+------+6			0 = ---
			//     /|     /|			1 = +--
			//    / |    / |			2 = ++-
			//   / 4+---/--+5			3 = -+-
			// 3+------+2 /    y   z	4 = --+
			//  | /    | /     |  /		5 = +-+
			//  |/     |/      |/		6 = +++
			// 0+------+1      *---x	7 = -++

			v[0] = v[3] = v[4] = v[7] = rCenter - axis0;
			v[1] = v[2] = v[5] = v[6] = rCenter + axis0;

			Vector3 tmp = axis1 + axis2;
			v[0] -= tmp;
			v[1] -= tmp;
			v[6] += tmp;
			v[7] += tmp;

			tmp = axis1 - axis2;
			v[2] += tmp;
			v[3] += tmp;
			v[4] -= tmp;
			v[5] -= tmp;

			float dmin = FLT_MAX;
			for (int i = 0; i < 8; ++i)
			{
				const float d = Normal.Dot(v[i]) + D;
				if (d < dmin)
				{
					dmin = d;
				}
			}
			if (dmin > 0.0f)
			{
				return false;
			}

			*normal = -Normal;
			*depth = -dmin;
			return true;
		}

		static bool 	GetIntersection(const Plane3& p1, const Plane3& p2, Vector3& Origin, Vector3& Dir)
		{
			Dir = p1.Normal.Cross(p2.Normal);
			if (Dir.Dot(Dir) < 1e-6f)
				return false;

			float d11 = p1.Normal.Dot(p1.Normal);
			float d12 = p1.Normal.Dot(p2.Normal);
			float d22 = p2.Normal.Dot(p2.Normal);

			const float denom = d11 * d22 - d12 * d12;
			float k1 = (-p1.D * d22 + p2.D * d12) / denom;
			float k2 = (-p2.D * d11 + p1.D * d12) / denom;
			Origin = k1 * p1.Normal + k2 * p2.Normal;
			return true;
		}

		static bool 	GetIntersection(const Plane3& p1, const Plane3& p2, const Plane3& p3, Vector3& p)
		{
			Vector3 u = p2.Normal.Cross(p3.Normal);
			float denom = p1.Normal.Dot(u);
			if (fabsf(denom) < 1e-6f)
				return false;
			p = (-p1.D * u + p1.Normal.Cross(-p3.D * p2.Normal + p2.D * p3.Normal)) / denom;
			return true;
		}

		// static
		static float SignedDistanceToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
		{
			float signedDist = (Point - Origin).Dot(Normal);
			return signedDist;
		}

		// static
		static Vector3 ProjectToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
		{
			float signedDist = (Point - Origin).Dot(Normal);
			return Point - signedDist * Normal;
		}

		// static
		static bool IntersectPlanes(const Plane3& plane0, const Plane3& plane1, const Plane3& plane2, Vector3* intersects)
		{
			const Vector3 u = plane1.Normal.Cross(plane2.Normal);
			const float d = plane0.Normal.Dot(u);
			if (fabsf(d) < 1e-6f)
			{
				return false;
			}
			const Vector3 t = plane2.Normal * plane1.D - plane1.Normal * plane2.D;
			const Vector3 p = plane0.Normal.Cross(t) - u * plane0.D;
			*intersects = p / d;
			return true;
		}

		int CalculateRelationsToPoint(const Vector3& Point) const
		{
			Vector3 Origin = GetOrigin();
			const float dist = SignedDistanceToPlane(Point, Normal, Origin);
			if (dist > kEpsilonPlane)
			{
				return 1;
			}
			else if (dist < kEpsilonPlane)
			{
				return -1;
			}
			return 0;
		}

		float SignedDistanceToPoint(const Vector3& Point) const
		{
			Vector3 Origin = GetOrigin();
			return SignedDistanceToPlane(Point, Normal, Origin);
		}

		float DistanceToPoint(const Vector3& Point) const
		{
			float Dist = SignedDistanceToPoint(Point);
			return fabsf(Dist);
		}

		Vector3 ClosestPointTo(const Vector3& Point) const
		{
			float SignedDist = SignedDistanceToPoint(Point);
			return Point - SignedDist * Normal;
		}

		float DistanceToSegment(const Vector3& P0, const Vector3& P1) const
		{
			float SignedDist0 = SignedDistanceToPoint(P0);
			float SignedDist1 = SignedDistanceToPoint(P1);
			if (SignedDist0 * SignedDist1 <= 0.0f)
			{
				return 0.0f;
			}
			return std::min(fabsf(SignedDist0), fabsf(SignedDist1));
		}

		float DistanceToTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
		{
			float sA = SignedDistanceToPoint(A);
			float sB = SignedDistanceToPoint(B);
			float sC = SignedDistanceToPoint(C);
			if ((sA > 0.0f && sB > 0.0f && sC > 0.0f) || (sA < 0.0f && sB < 0.0f && sC < 0.0f))
			{
				return std::min(fabsf(sA), std::min(fabsf(sB), fabsf(sC)));
			}
			return 0.0f;
		}

		float			CalculateVolume() const { return 0.0f; }

		Box3 CalculateBoundingVolume() const
		{
			const float kMaxBV = kPlaneRadius;
			Box3 Box(-kMaxBV, kMaxBV);

			if (ParallelToXY())
			{
				if (Normal.z > kEpsilonPlane)
				{
					Box.Min.z = -D / Normal.z - kHalfThickness;
					Box.Max.z = -D / Normal.z + kPlaneSmallThickness;
				}
			}

			if (ParallelToYZ())
			{
				if (Normal.x > kEpsilonPlane)
				{
					Box.Min.x = -D / Normal.x - kHalfThickness;
					Box.Max.x = -D / Normal.x + kPlaneSmallThickness;
				}
			}

			if (ParallelToXZ())
			{
				if (Normal.y > kEpsilonPlane)
				{
					Box.Min.y = -D / Normal.y - kHalfThickness;
					Box.Max.y = -D / Normal.y + kPlaneSmallThickness;
				}
			}

			return Box;
		}

		bool PerpendicularTo(const Vector3& Axis) const
		{
			float det = Normal.Cross(Axis).SquareLength();
			if (det < kEpsilonPlane)
			{
				return true;
			}
			return false;
		}

		bool ParallelToXY() const
		{
			return PerpendicularTo(Vector3::UnitZ());
		}

		bool ParallelToXZ() const
		{
			return PerpendicularTo(Vector3::UnitY());
		}

		bool ParallelToYZ() const
		{
			return PerpendicularTo(Vector3::UnitX());
		}

		bool CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = 0.0f;
			p->Mass = 0.0f;
			p->CenterOfMass = GetOrigin();
			p->BoundingVolume = CalculateBoundingVolume();
			p->InertiaMat = Matrix3::Identity();
			return true;
		}

		Vector3 GetSupport(const Vector3& Direction) const
		{
			Box3 box = CalculateBoundingVolume();
			return Vector3(
				Direction.x > 0 ? box.Max.x : box.Min.x,
				Direction.y > 0 ? box.Max.y : box.Min.y,
				Direction.z > 0 ? box.Max.z : box.Min.z
			);
		}

		int GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
		{
			Box3 box = CalculateBoundingVolume();
			const Vector3& Bmin = box.Min;
			const Vector3& Bmax = box.Max;

			const int axis = Direction.Abs().LargestAxis();
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

		void GetVerties(Vector3* v, float Radius)
		{
			Vector3 v0 = Vector3::UnitY();
			if (Normal.ParallelTo(v0) != 0)
			{
				v0 = Vector3::UnitX();
			}

			const Vector3 v1 = Normal.Cross(v0);
			const Vector3 v2 = Normal.Cross(v1);

			const Vector3 Origin = GetOrigin();
			v[0] = Origin + v1 * Radius;
			v[1] = Origin + v2 * Radius;
			v[2] = Origin + v1 * -Radius;
			v[3] = Origin + v2 * -Radius;
			return;
		}

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
		{
			Vertices.resize(4);
			GetVerties(&Vertices[0], 100.0f);
			Normals = { Normal , Normal , Normal , Normal };
			Indices = { 2,1,0, 2,3,0 };
		}

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
		{
			Vertices.resize(4);
			GetVerties(&Vertices[0], 100.0f);
			Indices = { 0,1, 1,2, 2,3, 3,0 };
		}

	private:
		static constexpr float kEpsilonPlane = 0.000001f;
		static constexpr float kHalfThickness = 1.0f;
		static constexpr float kPlaneSmallThickness = 0.0001f;
		static constexpr float kPlaneRadius = 1000.0f;
	};

	static_assert(sizeof(Plane3) == 16, "sizeof(Plane3d) not right");
}

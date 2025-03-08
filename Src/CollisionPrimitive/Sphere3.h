#pragma once

#include <math.h>
#include <vector>
#include "PrimitiveType.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class ConvexMesh;
	class HeightField3;
	class TriangleMesh;

	class Sphere3
	{
	public:
		Vector3 Center;
		float Radius;

	public:
		Sphere3()
		{
			Center = Vector3::Zero();
			Radius = 1.0f;
		}

		Sphere3(const Vector3& _Center, float _Radius)
		{
			Center = _Center;
			Radius = _Radius;
		}

		Sphere3(const Vector3& A)
		{
			Center = A;
			Radius = 0.0f;
		}

		Sphere3(const Vector3& A, const Vector3& B)
		{
			Center = (A + B) * 0.5f;
			Radius = (A - B).Length() * 0.5f + kSphereEnlargeFactor;
		}

		// https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
		Sphere3(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			Vector3 BA = B - A, CA = C - A;
			float a = BA.Dot(BA), b = BA.Dot(CA), c = CA.Dot(CA);
			float d = a * c - b * b;
			if (fabsf(d) < 1e-6f)
			{
				Vector3 p[3] = { A, B, C };
				int max_i = -1;
				float max_dist = FLT_MAX;
				for (int i = 0; i < 3; ++i)
				{
					const float d = (p[i] - p[(i + 1) % 3]).SquareLength();
					if (d > max_dist)
					{
						max_dist = d;
						max_i = i;
					}
				}
				// assert(max_i != -1);

				Sphere3 s(p[max_i], p[(max_i + 1) % 3]);
				Center = s.Center;
				Radius = s.Radius;
				return;
			}
			float s = (a - b) * c / (2.0f * d), t = (c - b) * a / (2.0f * d);
			Center = A + s * BA + t * CA;
			Radius = (A - Center).Length() + kSphereEnlargeFactor;
		}

		Sphere3(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D)
		{
			Vector3 BA = B - A, CA = C - A, DA = D - A;
			const float n = BA.Dot(CA.Cross(DA));
			if (fabsf(n) < 1e-6f)
			{
				Sphere3 min(Vector3::Zero(), FLT_MAX);
				Vector3 p[4] = { A, B, C, D };
				for (int i = 0; i < 4; ++i)
				{
					Sphere3 s(p[i], p[(i + 1) % 4], p[(i + 2) % 4]);
					s.Encapsulate(p[(i + 3) % 4]);
					if (s.Radius < min.Radius)
					{
						min = s;
					}
				}
				Center = min.Center;
				Radius = min.Radius;
				return;
			}
			float d1 = BA.SquareLength(), d2 = CA.SquareLength(), d3 = DA.SquareLength();
			Center.x = (A.x + ((CA.y * DA.z - DA.y * CA.z) * d1 - (BA.y * DA.z - DA.y * BA.z) * d2 + (BA.y * CA.z - CA.y * BA.z) * d3) / (n * 2.0f));
			Center.y = (A.y + (-(CA.x * DA.z - DA.x * CA.z) * d1 + (BA.x * DA.z - DA.x * BA.z) * d2 - (BA.x * CA.z - CA.x * BA.z) * d3) / (n * 2.0f));
			Center.z = (A.z + ((CA.x * DA.y - DA.x * CA.y) * d1 - (BA.x * DA.y - DA.x * BA.y) * d2 + (BA.x * CA.y - CA.x * BA.y) * d3) / (n * 2.0f));
			Radius = (Center - A).Length() + kSphereEnlargeFactor;
		}

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::SPHERE;
		}

	public:

		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool IntersectPoint(const Vector3& Point) const;
		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool IntersectSphere(const Vector3& _Center, float _Radius) const;
		bool IntersectCapsule(const Vector3& X0, const Vector3& X1, float rRadius) const;
		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;
		bool IntersectConvex(const ConvexMesh *convex) const;
		bool IntersectHeightField(const HeightField3* hf) const;
		bool IntersectTriangleMesh(const TriangleMesh* trimesh) const;

		bool PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const;
		bool PenetratePlane(const Vector3& pNormal, float D, Vector3* Normal, float* Depth) const;
		bool PenetrateCapsule(const Vector3& X0, const Vector3& X1, float rRadius, Vector3* Normal, float* Depth) const;
		bool PenetrateOBB(const Vector3& rCenter, const Vector3& rExtent, const Matrix3& rRot, Vector3* Normal, float* Depth) const;

		bool SweepAABB(const Vector3& Origin, const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const;
		bool SweepSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const;
		bool SweepPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const;
        bool SweepCylinder(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const;
        bool SweepCapsule(const Vector3& Origin, const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* n, float* t) const;
		bool SweepConvex(const Vector3& Origin, const Vector3& Direction, const ConvexMesh* convex, Vector3* n, float* t) const;
		bool SweepTriangle(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool SweepHeightField(const Vector3& Origin, const Vector3& Direction, const HeightField3* hf, Vector3* n, float* t) const;
		bool SweepTriangleMesh(const Vector3& Origin, const Vector3& Direction, const TriangleMesh* trimesh, Vector3* n, float* t) const;

		static Box3	CalcBoundingVolume(const Vector3& Center, float Radius)
		{
			Box3 Box;
			Box.BuildFromCenterAndExtent(Center, Vector3(Radius));
			return Box;
		}

		Box3			CalculateBoundingVolume() const
		{
			return Sphere3::CalcBoundingVolume(Center, Radius);
		}

		void Enlarge(float d)
		{
			Radius += d;
		}

		Sphere3& Encapsulate(const Vector3& p)
		{
			Vector3 d = p - Center;
			float sqrdist = d.SquareLength();
			if (sqrdist > Radius * Radius)
			{
				float dist = sqrtf(sqrdist);
				float newRadius = (Radius + dist) * 0.5f;
				float k = (newRadius - Radius) / dist;
				Radius = newRadius;
				Center = Center + d * k;
			}
			return *this;
		}

		Sphere3& Encapsulate(const Sphere3& s1)
		{
			Vector3 d = s1.Center - Center;
			float dist2 = d.Dot(d);

			if ((s1.Radius - Radius) * (s1.Radius - Radius) >= dist2)
			{
				if (s1.Radius >= Radius)
				{
					Radius = s1.Radius;
					Center = s1.Center;
				}
			}
			else
			{
				float dist = sqrtf(dist2);
				float r = (dist + Radius + s1.Radius) * 0.5f;
				if (dist > 1e-6f)
					Center += ((r - Radius) / dist) * d;
				Radius = r;
			}
			return *this;
		}

		static Sphere3 ComputeBoundingSphere_MostSeparated(const Vector3* points, int n);
		static Sphere3 ComputeBoundingSphere_Eigen(const Vector3* points, int n);
		static Sphere3 ComputeBoundingSphere_RitterIteration(const Vector3* _points, int n);
		static Sphere3 ComputeBoundingSphere_Welzl(const Vector3* _points, int n);

		bool CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->CenterOfMass = Center;
			p->BoundingVolume = CalculateBoundingVolume();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		float			CalculateVolume() const
		{
			const float FourThirdsPI = (4.0f / 3.0f) * (float)M_PI;
			return FourThirdsPI * Radius * Radius * Radius;
		}

		Vector3			GetCenterOfMass() const
		{
			return Center;
		}

		Matrix3		CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Radius, Mass);
		}

		static Matrix3 CalculateInertiaTensor(float Radius, float Mass)
		{
			const float Diagonal = 2.0f * Mass * Radius * Radius / 5.0f;
			return Matrix3(Diagonal, Diagonal, Diagonal);
		}

		Vector3			GetCenter() const
		{
			return Center;
		}

		Vector3			GetSupport(const Vector3& Direction) const
		{
			return GetSupport(Center, Radius, Direction);
		}

		static Vector3	GetSupport(const Vector3& Center, float Radius, const Vector3& Direction)
		{
			float distSqr = Direction.SquareLength();
			if (distSqr <= 1e-6)
			{
				return Center;
			}
			Vector3 Normalized = Direction / sqrtf(distSqr);
			return Center + Normalized * Radius;
		}

		int GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
		{
			return 0;
		}

		void GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals);

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);

	private:

		// weather a if sphere with radius r moving from a to b intersects with a plane
		static bool TestMovingSpherePlane(const Vector3& a, const Vector3& b, float r, const Vector3& Normal, float D)
		{
			float adist = a.Dot(Normal) + D;
			float bdist = b.Dot(Normal) + D;
			if (adist * bdist < 0.0f)
				return true;
			if (fabsf(adist) <= r || fabsf(bdist) <= r)
				return true;
			return false;
		}

		static constexpr float kSphereEnlargeFactor = 1e-6f;		// avoid floating error
	};

	static_assert(sizeof(Sphere3) == 16, "sizeof(Sphere3) not right");
}

#pragma once

#include <math.h>
#include <random>
#include <vector>
#include "PrimitiveType.h"
#include "Segment3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class Sphere3
	{
	public:
		Vector3 Center;
		float Radius;

	public:
		Sphere3()
		{
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
		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
		{
			return RayIntersectSphere(Origin, Direction, Center, Radius, t);
		}

		// static
		static bool RayIntersectSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& Center, float Radius, float* t)
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

		bool IntersectPoint(const Vector3& Point) const
		{
			float sqr_dist = (Point - Center).SquareLength();
			if (sqr_dist <= Radius * Radius)
			{
				return true;
			}
			return false;
		}

		// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
		bool IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
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

		bool IntersectSphere(const Vector3& _Center, float _Radius) const
		{
			return SphereIntersectSphere(Center, Radius, _Center, _Radius);
		}

		bool IntersectCapsule(const Vector3& X0, const Vector3& X1, float rRadius) const
		{
			float SqrDist = Segment3::SqrDistancePointToSegment(Center, X0, X1);
			return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
		}

		bool IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
		{
			float sqrDist = (A - Center).SquareLength();
			if (sqrDist <= Radius * Radius)
			{
				return true;
			}

			const Vector3 cp = ClosestPtPointTriangle(Center, A, B, C);
			sqrDist = (cp - Center).SquareLength();
			if (sqrDist <= Radius * Radius)
			{
				return true;
			}
			return false;
		}

		static bool SphereIntersectSphere(const Vector3& Center, float Radius, const Vector3& rCenter, float rRadius)
		{
			float SqrDist = (Center - rCenter).SquareLength();
			return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
		}

		bool PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const
		{
			float SqrDist = (Center - rCenter).SquareLength();
			if (SqrDist > (Radius + rRadius) * (Radius + rRadius))
			{
				return false;
			}

			*Normal = Center - rCenter;
			float len = Normal->Length();
			if (len < 1e-6f)
			{
				*Normal = Vector3::UnitY();
			}
			else
			{
				*Normal = *Normal / len;
			}
			*Depth = Radius + rRadius - len;
			return true;
		}

		bool PenetratePlane(const Vector3& pNormal, float D, Vector3* Normal, float* Depth) const
		{
			const float d = (Center + pNormal * D).Dot(pNormal);
			if (d > Radius)
			{
				return false;
			}

			*Normal = pNormal;
			*Depth = Radius - d;
			return true;
		}

		bool PenetrateCapsule(const Vector3& X0, const Vector3& X1, float rRadius, Vector3* Normal, float* Depth) const
		{
			const float s = rRadius + Radius;
			Vector3 Closest = Segment3::ClosestPointOnSegmentToPoint(Center, X0, X1);
			if ((Center - Closest).SquareLength() > s * s)
			{
				return false;
			}

			*Normal = Center - Closest;

			const float len = Normal->Length();
			if (len < 1e-6f)
			{
				*Normal = Vector3::UnitY();
			}
			else
			{
				*Normal /= len;
			}
			*Depth = s - len;
			return true;
		}

		bool PenetrateOBB(const Vector3& rCenter, const Vector3& rExtent, const Matrix3& rRot, Vector3* Normal, float* Depth) const
		{
			const Vector3 Delta = Center - rCenter;
			Vector3 localDelta = Delta * rRot;

			bool outside = false;

			if (localDelta.x < -rExtent.x)
			{
				outside = true;
				localDelta.x = -rExtent.x;
			}
			else if (localDelta.x > rExtent.x)
			{
				outside = true;
				localDelta.x = rExtent.x;
			}

			if (localDelta.y < -rExtent.y)
			{
				outside = true;
				localDelta.y = -rExtent.y;
			}
			else if (localDelta.y > rExtent.y)
			{
				outside = true;
				localDelta.y = rExtent.y;
			}

			if (localDelta.z < -rExtent.z)
			{
				outside = true;
				localDelta.z = -rExtent.z;
			}
			else if (localDelta.z > rExtent.z)
			{
				outside = true;
				localDelta.z = rExtent.z;
			}

			if (outside)
			{
				*Normal = Delta - rRot * localDelta;
				const float sqr = Normal->SquareLength();
				if (sqr > Radius * Radius)
					return false;
				const float len = sqrtf(sqr);

				*Depth = len - Radius;
				*Normal /= len;
				return true;
			}

			Vector3 localNormal;
			Vector3 d = rExtent - localDelta.Abs();

			if (d.y < d.x)
			{
				if (d.y < d.z)
				{
					localNormal = Vector3(0.0f, localDelta.y > 0.0f ? 1.0f : -1.0f, 0.0f);
					*Depth = -d.y;
				}
				else
				{
					localNormal = Vector3(0.0f, 0.0f, localDelta.z > 0.0f ? 1.0f : -1.0f);
					*Depth = -d.z;
				}
			}
			else
			{
				if (d.x < d.z)
				{
					localNormal = Vector3(localDelta.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
					*Depth = -d.x;
				}
				else
				{
					localNormal = Vector3(0.0f, 0.0f, localDelta.z > 0.0f ? 1.0f : -1.0f);
					*Depth = -d.z;
				}
			}

			*Normal = rRot * localNormal;
			*Depth -= Radius;
			return true;
		}

		bool SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* n, float* t) const
		{
			// TODO
			return false;
		}

		bool SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* n, float* t) const
		{
			Sphere3 s1(rCenter, rRadius + Radius);
			if (s1.IntersectRay(Center, Direction, t))
			{
				Vector3 p = Center + Direction * (*t);
				Vector3 dir = p - s1.Center;
				if (dir.SquareLength() > 1e-6f)
				{
					*n = dir;
				}
				else
				{
					*n = -Direction;
				}
				return true;
			}
			return false;
		}

		bool SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* n, float* t) const
		{
			float dist = Normal.Dot(Center) + D;
			if (fabsf(dist) <= Radius)
			{
				*t = 0.0f;
				*n = Normal;
				return true;
			}
			else
			{
				float denom = Normal.Dot(Direction);
				if (denom * dist >= 0.0f)
				{
					return false;
				}
				else
				{
					float r = dist > 0.0f ? Radius : -Radius;
					*t = (r - dist) / denom;
					*n = Normal;
					return true;
				}
			}
		}

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

		static Sphere3	ComputeBoundingSphere_MostSeparated(const Vector3* points, int n)
		{
			Sphere3 sphere = SphereFromDistantPoints(points, n);
			for (int i = 0; i < n; ++i)
			{
				sphere.Encapsulate(points[i]);
			}
			return sphere;
		}

		static Sphere3 ComputeBoundingSphere_Eigen(const Vector3* points, int n)
		{
			float eigens[3];
			Vector3 v[3];
			Matrix3 covariance_matrix = Matrix3::ComputeCovarianceMatrix(points, n);
			covariance_matrix.SolveEigenSymmetric(eigens, v);

			// Find the component with largest abs eigenvalue (largest axis)
			Vector3 dir = v[0];
			float maxe = fabsf(eigens[0]);
			if (fabsf(eigens[1]) > maxe)
			{
				dir = v[1];
				maxe = fabsf(eigens[1]);
			}
			if (fabsf(eigens[2]) > maxe)
			{
				dir = v[2];
				maxe = fabsf(eigens[2]);
			}

			int imin = -1, imax = -1;
			float minproj = FLT_MAX, maxproj = -FLT_MAX;
			for (int i = 0; i < n; i++)
			{
				float proj = points[i].Dot(dir);
				if (proj < minproj)
				{
					minproj = proj;
					imin = i;
				}
				if (proj > maxproj)
				{
					maxproj = proj;
					imax = i;
				}
			}
			Vector3 minpt = points[imin];
			Vector3 maxpt = points[imax];

			float dist = sqrtf(DotProduct(maxpt - minpt, maxpt - minpt));

			Sphere3 sphere;
			sphere.Radius = dist * 0.5f;
			sphere.Center = (minpt + maxpt) * 0.5f;

			for (int i = 0; i < n; ++i)
			{
				sphere.Encapsulate(points[i]);
			}
			return sphere;
		}

		// Real Time Collision Detection - Christer Ericson
		// Chapter 4.3.4, page 98-99
		static Sphere3 ComputeBoundingSphere_RitterIteration(const Vector3* _points, int n)
		{
			std::vector<Vector3> points;
			points.resize(n);
			memcpy(points.data(), _points, n * sizeof(_points[0]));

			Sphere3 s = ComputeBoundingSphere_MostSeparated(points.data(), n);
			Sphere3 s2 = s;

			const int maxIterations = 8;
			for (int k = 0; k < maxIterations; k++)
			{
				s2.Radius = s2.Radius * 0.95f;

				for (int i = 0; i < n; i++)
				{
					int j = Maths::RandomInt(i + 1, n - 1);
					std::swap(points[i], points[j]);
					s2.Encapsulate(points[i]);
				}

				// found a tighter sphere
				if (s2.Radius < s.Radius)
					s = s2;
			}
			return s;
		}

		// static
		static Sphere3 ComputeBoundingSphere_Welzl(const Vector3* _points, int n)
		{
			std::vector<Vector3> points;
			points.resize(n);
			memcpy(points.data(), _points, n * sizeof(_points[0]));

			std::random_device rd;
			std::mt19937 g(rd());
			std::shuffle(points.begin(), points.end(), g);

			Vector3 support[4];
			Sphere3 s = WelzlAlgorithm_Recursive(points.data(), n, support, 0);
			s.Radius += kSphereEnlargeFactor;
			return s;
		}

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

		void GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals)
		{
			const float mPI = 2.0f * asinf(1.0f);

			float phiStep = mPI / stackCount;
			float thetaStep = 2.0f * mPI / sliceCount;

			Vertices->push_back(Center + Vector3(0, Radius, 0));
			if (Normals) Normals->push_back(Vector3::UnitY());

			for (int i = 1; i < stackCount; i++)
			{
				float phi = i * phiStep;
				for (int j = 0; j <= sliceCount; j++)
				{
					float theta = j * thetaStep;
					Vector3 p = Vector3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta)) * Radius;
					Vertices->push_back(Center + p);
					if (Normals) Normals->push_back(p);
				}
			}
			Vertices->push_back(Center + Vector3(0, -Radius, 0));
			if (Normals) Normals->push_back(-Vector3::UnitY());
		}

		void GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
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

		void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
		{
			const int stackCount = 6;
			const int sliceCount = 8;

			GetVertices(stackCount, sliceCount, &Vertices, nullptr);

			for (int i = 1; i <= sliceCount; i++)
			{
				Indices.push_back(0);
				Indices.push_back(i);

				Indices.push_back(i);
				Indices.push_back(i + 1);
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

	private:
		// Welzl, E. (1991). Smallest enclosing disks (balls and ellipsoids) (pp. 359-370). Springer Berlin Heidelberg.
		static Sphere3 WelzlAlgorithm_Recursive(Vector3* points, int n, Vector3* support, int n_support)
		{
			if (n == 0)
			{
				switch (n_support)
				{
				case 0: return Sphere3();
				case 1: return Sphere3(support[0]);
				case 2: return Sphere3(support[0], support[1]);
				case 3: return Sphere3(support[0], support[1], support[2]);
				case 4: return Sphere3(support[0], support[1], support[2], support[3]);
				}
			}

			int i = rand() % n;
			std::swap(points[i], points[n - 1]);

			Sphere3 min = WelzlAlgorithm_Recursive(points, n - 1, support, n_support);
			if ((points[n - 1] - min.Center).SquareLength() <= min.Radius * min.Radius)
			{
				return min;
			}
			support[n_support] = points[n - 1];
			return WelzlAlgorithm_Recursive(points, n - 1, support, n_support + 1);
		}

		static Vector3 ClosestPtPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
		{
			// Check if P in vertex region outside A
			Vector3 ab = b - a;
			Vector3 ac = c - a;
			Vector3 ap = p - a;
			float d1 = DotProduct(ab, ap);
			float d2 = DotProduct(ac, ap);
			if (d1 <= 0.0f && d2 <= 0.0f) return a; // barycentric coordinates (1,0,0)

			// Check if P in vertex region outside B
			Vector3 bp = p - b;
			float d3 = DotProduct(ab, bp);
			float d4 = DotProduct(ac, bp);
			if (d3 >= 0.0f && d4 <= d3) return b; // barycentric coordinates (0,1,0)

			// Check if P in edge region of AB, if so return projection of P onto AB
			float vc = d1 * d4 - d3 * d2;
			if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
				float v = d1 / (d1 - d3);
				return a + v * ab; // barycentric coordinates (1-v,v,0)
			}

			// Check if P in vertex region outside C
			Vector3 cp = p - c;
			float d5 = DotProduct(ab, cp);
			float d6 = DotProduct(ac, cp);
			if (d6 >= 0.0f && d5 <= d6) return c; // barycentric coordinates (0,0,1)

			// Check if P in edge region of AC, if so return projection of P onto AC
			float vb = d5 * d2 - d1 * d6;
			if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
				float w = d2 / (d2 - d6);
				return a + w * ac; // barycentric coordinates (1-w,0,w)
			}

			// Check if P in edge region of BC, if so return projection of P onto BC
			float va = d3 * d6 - d5 * d4;
			if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
				float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				return b + w * (c - b); // barycentric coordinates (0,1-w,w)
			}

			// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
			float denom = 1.0f / (va + vb + vc);
			float v = vb * denom;
			float w = vc * denom;
			return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
		}

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

		static void MostSeparatedPointsOnAABB(const Vector3* points, int n, int& min, int& max)
		{
			int minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
			for (int i = 1; i < n; i++)
			{
				if (points[i].x < points[minx].x)
					minx = i;
				if (points[i].x > points[maxx].x)
					maxx = i;
				if (points[i].y < points[miny].y)
					miny = i;
				if (points[i].y > points[maxy].y)
					maxy = i;
				if (points[i].z < points[minz].z)
					minz = i;
				if (points[i].z > points[maxz].z)
					maxz = i;
			}

			float dist2x = DotProduct(points[maxx] - points[minx], points[maxx] - points[minx]);
			float dist2y = DotProduct(points[maxy] - points[miny], points[maxy] - points[miny]);
			float dist2z = DotProduct(points[maxz] - points[minz], points[maxz] - points[minz]);
			min = minx;
			max = maxx;
			if (dist2y > dist2x && dist2y > dist2z)
			{
				max = maxy;
				min = miny;
			}
			if (dist2z > dist2x && dist2z > dist2y)
			{
				max = maxz;
				min = minz;
			}
		}

		static Sphere3 SphereFromDistantPoints(const Vector3* points, int n)
		{
			int min, max;
			MostSeparatedPointsOnAABB(points, n, min, max);

			Sphere3 sphere;
			sphere.Center = (points[min] + points[max]) * 0.5f;
			sphere.Radius = sqrtf(DotProduct(points[max] - sphere.Center, points[max] - sphere.Center));
			return sphere;
		}

		static constexpr float kSphereEnlargeFactor = 1e-6f;		// avoid floating error
	};

	static_assert(sizeof(Sphere3) == 16, "sizeof(Sphere3) not right");
}
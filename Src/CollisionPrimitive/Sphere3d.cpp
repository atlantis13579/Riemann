
#include <assert.h>
#include <algorithm>
#include <random>
#include "../Maths/Maths.h"
#include "Ray3d.h"
#include "Triangle3d.h"
#include "Sphere3d.h"

static const float kSphereEnlargeFactor = 1e-6f;		// avoid floating error

Sphere3d::Sphere3d(const Vector3& A)
{
	Center = A;
	Radius = 0.0f;
}

Sphere3d::Sphere3d(const Vector3& A, const Vector3 &B)
{
	Center = (A + B) * 0.5f;
	Radius = (A - B).Length() * 0.5f + kSphereEnlargeFactor;
}

// https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
Sphere3d::Sphere3d(const Vector3& A, const Vector3 &B, const Vector3 &C)
{
	Vector3 BA = B - A, CA = C - A;
	float a = BA.Dot(BA), b = BA.Dot(CA), c = CA.Dot(CA);
	float d = a * c - b * b;
	if (fabsf(d) < 1e-6f)
	{
		Vector3 p[3] = {A, B, C};
		int max_i = -1;
		float max_dist = FLT_MAX;
		for (int i = 0; i < 3; ++i)
		{
			const float d = (p[i] - p[(i+1)%3]).SquareLength();
			if (d > max_dist)
			{
				max_dist = d;
				max_i = i;
			}
		}
		assert(max_i != -1);
		
		Sphere3d s(p[max_i], p[(max_i+1)%3]);
		Center = s.Center;
		Radius = s.Radius;
		return;
	}
	float s = (a - b) * c / (2.0f * d), t = (c - b) * a / (2.0f * d);
	Center = A + s * BA + t * CA;
	Radius = (A - Center).Length() + kSphereEnlargeFactor;
}

Sphere3d::Sphere3d(const Vector3& A, const Vector3 &B, const Vector3 &C, const Vector3 &D)
{
	Vector3 BA = B - A, CA = C - A, DA = D - A;
	const float n = BA.Dot(CA.Cross(DA));
	if (fabsf(n) < 1e-6f)
	{
		Sphere3d min(Vector3::Zero(), FLT_MAX);
		Vector3 p[4] = {A, B, C, D};
		for (int i = 0; i < 4; ++i)
		{
			Sphere3d s(p[i], p[(i+1)%4], p[(i+2)%4]);
			s.Encapsulate(p[(i+3)%4]);
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
	Center.x = (A.x + ((CA.y*DA.z - DA.y*CA.z)*d1 - (BA.y*DA.z - DA.y*BA.z)*d2 + (BA.y*CA.z - CA.y*BA.z)*d3) / (n * 2.0f));
	Center.y = (A.y + (-(CA.x*DA.z - DA.x*CA.z)*d1 + (BA.x*DA.z - DA.x*BA.z)*d2 - (BA.x*CA.z - CA.x*BA.z)*d3) / (n * 2.0f));
	Center.z = (A.z + ((CA.x*DA.y - DA.x*CA.y)*d1 - (BA.x*DA.y - DA.x*BA.y)*d2 + (BA.x*CA.y - CA.x*BA.y)*d3) / (n * 2.0f));
	Radius = (Center - A).Length() + kSphereEnlargeFactor;
}

bool Sphere3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	return RayIntersectSphere(Origin, Direction, Center, Radius, t);
}

// static
bool Sphere3d::RayIntersectSphere(const Vector3& Origin, const Vector3& Direction, const Vector3& Center, float Radius, float* t)
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

bool Sphere3d::IntersectPoint(const Vector3& Point) const
{
	float sqr_dist = (Point - Center).SquareLength();
	if (sqr_dist <= Radius * Radius)
	{
		return true;
	}
	return false;
}

// AABB intersects with a solid sphere or not by Jim Arvo, in "Graphics Gems":
bool Sphere3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
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

bool Sphere3d::IntersectSphere(const Vector3& _Center, float _Radius) const
{
	return SphereIntersectSphere(Center, Radius, _Center, _Radius);
}

// static
Vector3 ClosestPtPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
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

bool Sphere3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
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

bool Sphere3d::SphereIntersectSphere(const Vector3& Center, float Radius, const Vector3& rCenter, float rRadius)
{
	float SqrDist = (Center - rCenter).SquareLength();
	return SqrDist <= (Radius + rRadius) * (Radius + rRadius);
}

bool Sphere3d::SweepAABB(const Vector3 &Direction, const Vector3& bmin, const Vector3& bmax, Vector3 *n, float *t) const
{
	// TODO
	return false;
}

bool Sphere3d::SweepSphere(const Vector3 &Direction, const Vector3 &rCenter, float rRadius, Vector3 *n, float *t) const
{
	Sphere3d s1(rCenter, rRadius + Radius);
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

// weather a if sphere with radius r moving from a to b intersects with a plane
bool TestMovingSpherePlane(const Vector3& a, const Vector3& b, float r, const Vector3 &Normal, float D)
{
	float adist = a.Dot(Normal) + D;
	float bdist = b.Dot(Normal) + D;
	if (adist * bdist < 0.0f)
		return true;
	if (fabsf(adist) <= r || fabsf(bdist) <= r)
		return true;
	return false;
}

bool Sphere3d::SweepPlane(const Vector3 &Direction, const Vector3 &Normal, float D, Vector3 *n, float *t) const
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

static void MostSeparatedPointsOnAABB(const Vector3* points, int n, int &min, int &max)
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

static Sphere3d SphereFromDistantPoints(const Vector3 *points, int n)
{
	int min, max;
	MostSeparatedPointsOnAABB(points, n, min, max);

	Sphere3d sphere;
	sphere.Center = (points[min] + points[max]) * 0.5f;
	sphere.Radius = sqrtf(DotProduct(points[max] - sphere.Center, points[max] - sphere.Center));
	return sphere;
}

Sphere3d& Sphere3d::Encapsulate(const Vector3& p)
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

Sphere3d& Sphere3d::Encapsulate(const Sphere3d& s1)
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

// static
Sphere3d	Sphere3d::ComputeBoundingSphere_MostSeparated(const Vector3 *points, int n)
{
	Sphere3d sphere = SphereFromDistantPoints(points, n);
	for (int i = 0; i < n; ++i)
	{
		sphere.Encapsulate(points[i]);
	}
	return sphere;
}

static Sphere3d EigenSphere(const Vector3 *points, int n)
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
	
	Sphere3d sphere;
	sphere.Radius = dist * 0.5f;
	sphere.Center = (minpt + maxpt) * 0.5f;
	return sphere;
}

// static
Sphere3d Sphere3d::ComputeBoundingSphere_Eigen(const Vector3 *points, int n)
{
	Sphere3d sphere = EigenSphere(points, n);
	for (int i = 0; i < n; ++i)
	{
		sphere.Encapsulate(points[i]);
	}
	return sphere;
}

// Real Time Collision Detection - Christer Ericson
// Chapter 4.3.4, page 98-99
Sphere3d Sphere3d::ComputeBoundingSphere_RitterIteration(const Vector3 *_points, int n)
{
	std::vector<Vector3> points;
	points.resize(n);
	memcpy(points.data(), _points, n * sizeof(_points[0]));
	
	Sphere3d s = ComputeBoundingSphere_MostSeparated(points.data(), n);
	Sphere3d s2 = s;
	
	const int maxIterations = 8;
	for (int k = 0; k < maxIterations; k++)
	{
		s2.Radius = s2.Radius * 0.95f;
	
		for (int i = 0; i < n; i++)
		{
			int j = RandomInt(i + 1, n - 1);
			std::swap(points[i], points[j]);
			s2.Encapsulate(points[i]);
		}

		// found a tighter sphere
		if (s2.Radius < s.Radius)
			s = s2;
	}
	return s;
}

// Welzl, E. (1991). Smallest enclosing disks (balls and ellipsoids) (pp. 359-370). Springer Berlin Heidelberg.
Sphere3d WelzlAlgorithm_Recursive(Vector3* points, int n, Vector3* support, int n_support)
{
	if (n == 0)
	{
		switch (n_support)
		{
		case 0: return Sphere3d();
		case 1: return Sphere3d(support[0]);
		case 2: return Sphere3d(support[0], support[1]);
		case 3: return Sphere3d(support[0], support[1], support[2]);
		case 4: return Sphere3d(support[0], support[1], support[2], support[3]);
		}
	}
	
	int i = rand() % n;
	std::swap(points[i], points[n - 1]);
	
	Sphere3d min = WelzlAlgorithm_Recursive(points, n - 1, support, n_support);
	if((points[n-1] - min.Center).SquareLength() <= min.Radius * min.Radius)
	{
		return min;
	}
	support[n_support] = points[n-1];
	return WelzlAlgorithm_Recursive(points, n - 1, support, n_support + 1);
}

// static
Sphere3d Sphere3d::ComputeBoundingSphere_Welzl(const Vector3 *_points, int n)
{
	std::vector<Vector3> points;
	points.resize(n);
	memcpy(points.data(), _points, n * sizeof(_points[0]));
	
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(points.begin(), points.end(), g);
	
	Vector3 support[4];
	Sphere3d s = WelzlAlgorithm_Recursive(points.data(), n, support, 0);
	s.Radius += kSphereEnlargeFactor;
	return s;
}

Vector3 Sphere3d::GetSupport(const Vector3& Center, float Radius, const Vector3& Direction)
{
	float distSqr = Direction.SquareLength();
	if (distSqr <= 1e-6)
	{
		return Center;
	}
	Vector3 Normalized = Direction / sqrtf(distSqr);
	return Center + Normalized * Radius;
}

Vector3 Sphere3d::GetSupport(const Vector3& Direction) const
{
	return GetSupport(Center, Radius, Direction);
}

int Sphere3d::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	*FacePoints = Sphere3d::GetSupport(Center, Radius, Direction);
	return 1;
}

void Sphere3d::GetVertices(int stackCount, int sliceCount, std::vector<Vector3>* Vertices, std::vector<Vector3>* Normals)
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

void Sphere3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
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

void Sphere3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
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

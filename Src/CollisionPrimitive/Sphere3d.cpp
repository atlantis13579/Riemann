
#include "Triangle3d.h"
#include "Sphere3d.h"

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

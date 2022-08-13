
#include "Triangle3d.h"
#include "Capsule3d.h"
#include "Sphere3d.h"

bool Triangle3d::IntersectPoint(const Vector3& Point) const
{
	Vector3 bc = BaryCentric2D(Point);
	return bc.x >= 0 && bc.y >= 0.0f && bc.z >= 0.0f;
}

bool Triangle3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	return RayIntersectTriangle(Origin, Direction, A, B, C, t);
}

// By Tomas Akenine-Moller
// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
// -----
bool Triangle3d::IntersectAABB(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Bmin, const Vector3& Bmax)
{
	Vector3 Center = (Bmax + Bmin) * 0.5f;
	Vector3 Extent = Bmax - Center;

	float min, max, p0, p1, p2, rad, fex, fey, fez;

	Vector3 v0 = A - Center;
	Vector3 v1 = B - Center;
	Vector3 v2 = C - Center;

	Vector3 e0 = v1 - v0;
	Vector3 e1 = v2 - v1;
	Vector3 e2 = v0 - v2;

	fex = fabsf(e0.x);
	fey = fabsf(e0.y);
	fez = fabsf(e0.z);

	p0 = e0.z * v0.y - e0.y * v0.z;
	p2 = e0.z * v2.y - e0.y * v2.z;
	if (p0 < p2)
	{
		min = p0;
		max = p2;
	}
	else
	{
		min = p2;
		max = p0;
	}

	rad = fez * Extent.y + fey * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p0 = -e0.z * v0.x + e0.x * v0.z;
	p2 = -e0.z * v2.x + e0.x * v2.z;
	if (p0 < p2)
	{
		min = p0;
		max = p2;
	}
	else
	{
		min = p2;
		max = p0;
	}

	rad = fez * Extent.x + fex * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p1 = e0.y * v1.x - e0.x * v1.y;
	p2 = e0.y * v2.x - e0.x * v2.y;
	if (p2 < p1)
	{
		min = p2;
		max = p1;
	}
	else
	{
		min = p1;
		max = p2;
	}

	rad = fey * Extent.x + fex * Extent.y;
	if (min > rad || max < -rad)
		return false;

	fex = fabsf(e1.x);
	fey = fabsf(e1.y);
	fez = fabsf(e1.z);

	p0 = e1.z * v0.y - e1.y * v0.z;
	p2 = e1.z * v2.y - e1.y * v2.z;
	if (p0 < p2)
	{
		min = p0;
		max = p2;
	}
	else
	{
		min = p2;
		max = p0;
	}

	rad = fez * Extent.y + fey * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p0 = -e1.z * v0.x + e1.x * v0.z;
	p2 = -e1.z * v2.x + e1.x * v2.z;
	if (p0 < p2)
	{
		min = p0;
		max = p2;
	}
	else
	{
		min = p2;
		max = p0;
	}

	rad = fez * Extent.x + fex * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p0 = e1.y * v0.x - e1.x * v0.y;
	p1 = e1.y * v1.x - e1.x * v1.y;
	if (p0 < p1)
	{
		min = p0;
		max = p1;
	}
	else
	{
		min = p1;
		max = p0;
	}

	rad = fey * Extent.x + fex * Extent.y;
	if (min > rad || max < -rad)
		return false;

	fex = fabsf(e2.x);
	fey = fabsf(e2.y);
	fez = fabsf(e2.z);

	p0 = e2.z * v0.y - e2.y * v0.z;
	p1 = e2.z * v1.y - e2.y * v1.z;
	if (p0 < p1)
	{
		min = p0;
		max = p1;
	}
	else
	{
		min = p1;
		max = p0;
	}

	rad = fez * Extent.y + fey * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p0 = -e2.z * v0.x + e2.x * v0.z;
	p1 = -e2.z * v1.x + e2.x * v1.z;
	if (p0 < p1)
	{
		min = p0;
		max = p1;
	}
	else
	{
		min = p1;
		max = p0;
	}

	rad = fez * Extent.x + fex * Extent.z;
	if (min > rad || max < -rad)
		return false;

	p1 = e2.y * v1.x - e2.x * v1.y;
	p2 = e2.y * v2.x - e2.x * v2.y;
	if (p2 < p1)
	{
		min = p2;
		max = p1;
	}
	else
	{
		min = p1;
		max = p2;
	}

	rad = fey * Extent.x + fex * Extent.y;
	if (min > rad || max < -rad)
		return false;

	min = std::min(v0.x, std::min(v1.x, v2.x));
	max = std::max(v0.x, std::max(v1.x, v2.x));
	if (min > Extent.x || max < -Extent.x)
		return false;

	min = std::min(v0.y, std::min(v1.y, v2.y));
	max = std::max(v0.y, std::max(v1.y, v2.y));
	if (min > Extent.y || max < -Extent.y)
		return false;

	min = std::min(v0.z, std::min(v1.z, v2.z));
	max = std::max(v0.z, std::max(v1.z, v2.z));
	if (min > Extent.z || max < -Extent.z)
		return false;

	Vector3 Normal = e0.Cross(e1);

	Vector3 vmin, vmax;
	float v;
	for (int i = 0; i <= 2; i++)
	{
		v = v0[i];
		if (Normal[i] > 0.0f)
		{
			vmin[i] = -Extent[i] - v;
			vmax[i] = Extent[i] - v;
		}
		else
		{
			vmin[i] = Extent[i] - v;
			vmax[i] = -Extent[i] - v;
		}
	}

	if (Normal.Dot(vmin) > 0.0f)
		return false;

	if (Normal.Dot(vmax) < 0.0f)
		return false;

	return true;
}

bool Triangle3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	return IntersectAABB(A, B, C, Bmin, Bmax);
}

static Vector3 ClosestPointOnEdge(const Vector3& P0, const Vector3& P1, const Vector3& Point)
{
	const Vector3 V1 = P1 - P0;
	const Vector3 V2 = Point - P0;

	const float dp1 = V2.Dot(V1);
	if (dp1 <= 0)
	{
		return P0;
	}

	const float dp2 = V1.Dot(V1);
	if (dp2 <= dp1)
	{
		return P1;
	}

	return P0 + V1 * (dp2 / dp2);
}

// static
Vector3 Triangle3d::ClosestPointOnTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C, unsigned char& mask)
{
	const Vector3 BA = A - B;
	const Vector3 AC = C - A;
	const Vector3 CB = B - C;
	const Vector3 Normal = BA.Cross(CB);

	const Vector3 V[3] = { B, A, C };
	const Vector3 N[3] = { Normal.Cross(BA), Normal.Cross(AC), Normal.Cross(CB) };

	mask = 0;
	for (int i = 0; i < 3; ++i)
	{
		if ((Point - V[i]).Dot(N[i]) > 0.0f)
		{
			mask += (1 << i);
		}
	}

	if (mask == 0b0000)
	{
		float signedDist = (Point - A).Dot(Normal);
		return Point - signedDist * Normal;
	}
	else if (mask == 0b0001)
	{
		return ClosestPointOnEdge(B, A, Point);
	}
	else if (mask == 0b0010)
	{
		return ClosestPointOnEdge(A, C, Point);
	}
	else if (mask == 0b0011)
	{
		return A;
	}
	else if (mask == 0b0100)
	{
		return ClosestPointOnEdge(C, B, Point);
	}
	else if (mask == 0b0101)
	{
		return B;
	}
	else if (mask == 0b0110)
	{
		return C;
	}

	assert(false);		// Should never comes here
	return A;
}

float Triangle3d::SqrDistancePointToTriangle(const Vector3& Point, const Vector3& A, const Vector3& B, const Vector3& C)
{
	unsigned char mask;
	Vector3 Closest = ClosestPointOnTriangle(Point, A, B, C, mask);
	return (Closest - Point).SquareLength();
}

float Triangle3d::SqrDistanceToPoint(const Vector3& Point) const
{
	return Triangle3d::SqrDistancePointToTriangle(Point, A, B, C);
}

bool Triangle3d::IntersectSphere(const Vector3& Center, float Radius) const
{
	// Find point P on triangle ABC closest to sphere center
	unsigned char mask;
	Vector3 p = ClosestPointOnTriangle(Center, A, B, C, mask);

	// Sphere and triangle intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	Vector3 v = p - Center;
	return DotProduct(v, v) <= Radius * Radius;
}

bool Triangle3d::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
{
	Capsule3d capsule(X0, X1, Radius);
	return capsule.IntersectTriangle(A, B, C);
}

// Moller CTrumbore intersection algorithm
bool Triangle3d::RayIntersectTriangle(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t)
{
	const float kEpsilonTri = 0.0000001f;
	//Find vectors for two edges sharing V1
	const Vector3 e1 = B - A;
	const Vector3 e2 = C - A;
	//Begin calculating determinant - also used to calculate u parameter
	const Vector3 P = Direction.Cross(e2);
	//if determinant is near zero, ray lies in plane of triangle
	const float det = e1.Dot(P);
	//NOT CULLING
	if (det > -kEpsilonTri && det < kEpsilonTri)
	{
		return false;
	}
	const float inv_det = 1.f / det;

	//calculate distance from V1 to ray origin
	const Vector3 T = Origin - A;

	//Calculate u parameter and test bound
	const float u = T.Dot(P) * inv_det;
	//The intersection lies outside of the triangle
	if (u < 0.f || u > 1.f)
	{
		return false;
	}

	//Prepare to test v parameter
	const Vector3 Q = T.Cross(e1);

	//Calculate V parameter and test bound
	const float v = Direction.Dot(Q) * inv_det;
	//The intersection lies outside of the triangle
	if (v < 0.f || u + v  > 1.f)
	{
		return false;
	}

	const float out = e2.Dot(Q) * inv_det;

	if (out > kEpsilonTri) { //ray intersection
		*t = out;
		return true;
	}

	// No hit, no win
	return false;
}

Vector3 Triangle3d::BaryCentric2D(const Vector3& Point) const
{
	float a = ((B.y - C.y) * (Point.x - C.x) + (C.x - B.x) * (Point.y - C.y)) / ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
	float b = ((C.y - A.y) * (Point.x - C.x) + (A.x - C.x) * (Point.y - C.y)) / ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
	return Vector3(a, b, 1.0f - a - b);
}

inline float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
{
	return (x1-x2)*(y2-y3) - (x2-x3)*(y1-y2);
}

Vector3	Triangle3d::BaryCentric3D(const Vector3& Point) const
{
	Vector3 v0 = B - A, v1 = C - A, v2 = Point - A;
	float d00 = DotProduct(v0, v0);
	float d01 = DotProduct(v0, v1);
	float d11 = DotProduct(v1, v1);
	float d20 = DotProduct(v2, v0);
	float d21 = DotProduct(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	float v = (d11 * d20 - d01 * d21) / denom;
	float w = (d00 * d21 - d01 * d20) / denom;
	float u = 1.0f - v - w;
	return Vector3(u, v, w);
}

float Triangle3d::TriangleArea3D(const Vector3& A, const Vector3& B, const Vector3& C)
{
	float cx = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
	float cy = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
	float cz = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
	return 0.5f * sqrtf(cx * cx + cy * cy + cz * cz);
}

Vector3 Triangle3d::GetSupport(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& Direction)
{
	float dpa = DotProduct(A, Direction);
	float dpb = DotProduct(B, Direction);
	float dpc = DotProduct(C, Direction);

	if (dpa >= dpb && dpa >= dpc)
	{
		return A;
	}
	else if (dpb >= dpa && dpb >= dpc)
	{
		return B;
	}

	return C;
}

Vector3 Triangle3d::GetSupport(const Vector3& Direction) const
{
	return GetSupport(A, B, C, Direction);
}

int Triangle3d::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	*FacePoints = Triangle3d::GetSupport(A, B, C, Direction);
	return 1;
}

void Triangle3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices = std::vector<Vector3>({ A , B, C });
	Vector3 Nor = GetNormal();
	Normals = std::vector<Vector3>({ Nor, Nor , Nor });
	Indices = std::vector<uint16_t>({ 0,1,2 });
}

void Triangle3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	Vertices = std::vector<Vector3>({ A , B, C });
	Indices = std::vector<uint16_t>({ 0,1, 1,2, 2,0 });
}

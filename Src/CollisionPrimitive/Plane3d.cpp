
#include "Plane3d.h"

const float kEpsilonPlane = 0.000001f;
const float kHalfThickness = 1.0f;
const float kPlaneSmallThickness = 0.0001f;
const float kPlaneRadius = 1000.0f;

bool Plane3d::IntersectPoint(const Vector3& Point) const
{
	const float det = Point.Dot(Normal) + D;

	if (fabsf(det) < kEpsilonPlane)
	{
		return true;
	}
	return false;
}

bool Plane3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	return RayIntersectPlane(Origin, Direction, Normal, D, t);
}

bool Plane3d::IntersectSegment(const Vector3& P0, const Vector3& P1) const
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

bool Plane3d::RayIntersectPlane(const Vector3& Origin, const Vector3& Direction, const Vector3& Normal, float D, float* t)
{
	const float det = Direction.Dot(Normal);
	if (det > -kEpsilonPlane && det < kEpsilonPlane) {
		return false;
	}
	*t = -(Origin.Dot(Normal) + D) / det;
	return *t >= 0.0;
}

bool Plane3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	Vector3 v[8];
	Box3d::GetVertices(Bmin, Bmax, v);
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

bool Plane3d::GetIntersection(const Plane3d& p1, const Plane3d& p2, const Plane3d& p3, Vector3& p)
{
	Vector3 u = p2.Normal.Cross(p3.Normal);
	float denom = p1.Normal.Dot(u);
	if (fabsf(denom) < 1e-6f)
		return false;
	p = (-p1.D * u + p1.Normal.Cross(-p3.D * p2.Normal + p2.D * p3.Normal)) / denom;
	return true;
}

bool Plane3d::GetIntersection(const Plane3d& p1, const Plane3d& p2, Vector3& Origin, Vector3& Dir)
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

float Plane3d::SignedDistanceToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
{
	float signedDist = (Point - Origin).Dot(Normal);
	return signedDist;
}

Vector3 Plane3d::ProjectToPlane(const Vector3& Point, const Vector3& Normal, const Vector3& Origin)
{
	float signedDist = (Point - Origin).Dot(Normal);
	return Point - signedDist * Normal;
}

float Plane3d::SignedDistanceTo(const Vector3& Point) const
{
	Vector3 Origin = GetOrigin();
	return SignedDistanceToPlane(Point, Normal, Origin);
}

float Plane3d::DistanceToPoint(const Vector3& Point) const
{
	float Dist = SignedDistanceTo(Point);
	return fabsf(Dist);
}

Vector3 Plane3d::ClosestPointTo(const Vector3& Point) const
{
	float SignedDist = SignedDistanceTo(Point);
	return Point - SignedDist * Normal;
}

float Plane3d::DistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	float SignedDist0 = SignedDistanceTo(P0);
	float SignedDist1 = SignedDistanceTo(P1);
	if (SignedDist0 * SignedDist1 <= 0.0f)
	{
		return 0.0f;
	}
	return std::min(fabsf(SignedDist0), fabsf(SignedDist1));
}

float Plane3d::DistanceToTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	float sA = SignedDistanceTo(A);
	float sB = SignedDistanceTo(B);
	float sC = SignedDistanceTo(C);
	if ((sA > 0.0f && sB > 0.0f && sC > 0.0f) || (sA < 0.0f && sB < 0.0f && sC < 0.0f))
	{
		return std::min(fabsf(sA), std::min(fabsf(sB), fabsf(sC)));
	}
	return 0.0f;
}

bool Plane3d::IntersectSphere(const Vector3& rCenter, float rRadius) const
{
	float Dist = DistanceToPoint(rCenter);
	return Dist <= rRadius;
}

bool Plane3d::IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const
{
	float Dist = DistanceToSegment(P0, P1);
	return Dist <= Radius;
}

bool Plane3d::IntersectPlane(const Vector3& _Normal, float _D) const
{
	bool Parallel = Normal.ParallelTo(_Normal);
	if (Parallel)
	{
		return D == _D;
	}
	return true;
}

bool Plane3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	float Dist = DistanceToTriangle(A, B, C);
	return Dist > 0.0f;
}

bool Plane3d::PenetrateSphere(const Vector3 &Center, float Radius, Vector3 *normal, float *depth) const
{
	const float d = SignedDistanceTo(Center);
	if (d > Radius)
	{
		return false;
	}

	*normal	= -Normal;
	*depth = Radius - d;
	return true;
}

bool Plane3d::PenetrateOBB(const Vector3 &rCenter, const Vector3 &rExtent, const Matrix3& rRot, Vector3 *normal, float *depth) const
{
	Vector3 v[8];
	
	const Vector3 base0 = rRot.GetCol(0);
	const Vector3 base1 = rRot.GetCol(1);
	const Vector3 base2 = rRot.GetCol(2);
	
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
	for(int i = 0; i < 8; ++i)
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

	*normal	= -Normal;
	*depth = -dmin;
	return true;
}

Box3d Plane3d::GetBoundingVolume() const
{
	const float kMaxBV = kPlaneRadius;
	Box3d Box(-kMaxBV, kMaxBV);

	if (ParallelToXY())
	{
		if (Normal.z > kEpsilonPlane)
		{
			Box.mMin.z = -D / Normal.z - kHalfThickness;
			Box.mMax.z = -D / Normal.z + kPlaneSmallThickness;
		}
	}

	if (ParallelToYZ())
	{
		if (Normal.x > kEpsilonPlane)
		{
			Box.mMin.x = -D / Normal.x - kHalfThickness;
			Box.mMax.x = -D / Normal.x + kPlaneSmallThickness;
		}
	}

	if (ParallelToXZ())
	{
		if (Normal.y > kEpsilonPlane)
		{
			Box.mMin.y = -D / Normal.y - kHalfThickness;
			Box.mMax.y = -D / Normal.y + kPlaneSmallThickness;
		}
	}

	return Box;
}

bool Plane3d::PerpendicularTo(const Vector3& Axis) const
{
	float det = Normal.Cross(Axis).SquareLength();
	if (det < kEpsilonPlane)
	{
		return true;
	}
	return false;
}

bool Plane3d::ParallelToXY() const
{
	return PerpendicularTo(Vector3::UnitZ());
}

bool Plane3d::ParallelToXZ() const
{
	return PerpendicularTo(Vector3::UnitY());
}

bool Plane3d::ParallelToYZ() const
{
	return PerpendicularTo(Vector3::UnitX());
}

Vector3 Plane3d::GetSupport(const Vector3& Direction) const
{
	Box3d box = GetBoundingVolume();
	return Vector3(
		Direction.x > 0 ? box.mMax.x : box.mMin.x,
		Direction.y > 0 ? box.mMax.y : box.mMin.y,
		Direction.z > 0 ? box.mMax.z : box.mMin.z
	);
}

int Plane3d::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	Box3d box = GetBoundingVolume();
	const Vector3& Bmin = box.mMin;
	const Vector3& Bmax = box.mMax;

	int axis = Direction.Abs().LargestAxis();
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

void Plane3d::GetVerties(Vector3* v, float Radius)
{
	Vector3 v0 = Vector3::UnitY();
	if (Normal.ParallelTo(v0) != 0)
	{
		v0 = Vector3::UnitX();
	}

	Vector3 v1 = Normal.Cross(v0);
	Vector3 v2 = Normal.Cross(v1);

	Vector3 Origin = GetOrigin();
	v[0] = Origin + v1 * Radius;
	v[1] = Origin + v2 * Radius;
	v[2] = Origin + v1 * -Radius;
	v[3] = Origin + v2 * -Radius;
	return;
}

void Plane3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices.resize(4);
	GetVerties(&Vertices[0], 100.0f);
	Normals = { Normal , Normal , Normal , Normal };
	Indices = { 2,1,0, 2,3,0 };
}

void Plane3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	Vertices.resize(4);
	GetVerties(&Vertices[0], 100.0f);
	Indices = { 0,1, 1,2, 2,3, 3,0 };
}

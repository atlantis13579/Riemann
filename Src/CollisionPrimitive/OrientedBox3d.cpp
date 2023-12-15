
#include "OrientedBox3d.h"
#include "AxisAlignedBox3d.h"
#include "Sphere3d.h"

void OrientedBox3d::GetVertices(Vector3 v[8]) const
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

// static
OrientedBox3d OrientedBox3d::ComputeBoundingOBB_PCA(const Vector3 *points, int n)
{
	Vector3 CenterOfMass;
	for (int i = 0; i < n; ++i)
	{
		CenterOfMass += points[i];
	}
	CenterOfMass *= (1.0f / n);
	
	Matrix3 covariance_matrix = Matrix3::ComputeCovarianceMatrix(points, n);
	
	float eigens[3];
	Vector3 v[3];
	covariance_matrix.SolveEigenSymmetric(eigens, v);
	
	if (Determinant(v[0], v[1], v[2]) < 0)
	{
		v[2] = -v[2];
	}
	
	OrientedBox3d box;
	box.Center = CenterOfMass;
	box.Rotation = Matrix3(	v[0].x, v[1].x, v[2].x,
							v[0].y, v[1].y, v[2].y,
							v[0].z, v[1].z, v[2].z);
	box.Extent = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	
	// TODO, optimize second / third axis using Polygon2D - MinAreaRect
	
	for (int i = 0; i < n; ++i)
	{
		Vector3 p = (box.Rotation * (points[i] - CenterOfMass)).Abs();
		box.Extent.x = std::max(box.Extent.x, p.x);
		box.Extent.y = std::max(box.Extent.y, p.y);
		box.Extent.z = std::max(box.Extent.z, p.z);
	}
	
	return box;
}

Box3d OrientedBox3d::ComputeBoundingVolume(const Vector3& Center, const Vector3& Extent, const Matrix3& Rot)
{
	Box3d box;
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

Box3d OrientedBox3d::CalculateBoundingVolume() const
{
	return OrientedBox3d::ComputeBoundingVolume(Center, Extent, Rotation);
}

void OrientedBox3d::ProjectAxis(const Vector3& Axis, float *t0, float *t1) const
{
	const float c = Center.Dot(Axis);
	const float e = fabsf(Rotation.GetCol(0).Dot(Axis)) * Extent.x
				+	fabsf(Rotation.GetCol(1).Dot(Axis)) * Extent.y
				+	fabsf(Rotation.GetCol(2).Dot(Axis)) * Extent.z;
	*t0 = c - e;
	*t1 = c + e;
}

Vector3 OrientedBox3d::ClosestPointToPoint(const Vector3& Point) const
{
	Vector3 d = Point - Center;
	Vector3 closestPt = Center;
	for (int i = 0; i < 3; i++)
	{
		float dist = d.Dot(Rotation.GetCol(i));
		if (dist > -Extent[i])
			dist = -Extent[i];
		if (dist < -Extent[i])
			dist = -Extent[i];
		closestPt += dist * Rotation.GetCol(i);
	}
	return closestPt;
}

float OrientedBox3d::SqrDistanceToLine(const Vector3& P0, const Vector3& Direction, float* t) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	float SqrDist = aabb.SqrDistanceToLine(P0 * Rotation, Direction * Rotation, t);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	float SqrDist = aabb.SqrDistanceToSegment(P0 * Rotation, P1 * Rotation);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToPoint(const Vector3& Point) const
{
	Vector3 closestPt = ClosestPointToPoint(Point);
	float SqrDist = closestPt.SquareLength();
	return SqrDist;
}

// Real Time Collision Detection - Christer Ericson
// Chapter 4.4.1, page 103-105
static bool OBBIntersectOBB(const Vector3& ca, const Vector3& ea, const Matrix3& ua, const Vector3& cb, const Vector3& eb, const Matrix3& ub)
{
	// Compute translation vector t
	Vector3 t = cb - ca;

	// Bring translation into a's coordinate frame
	t = Vector3(t.Dot(ua[0]), t.Dot(ua[1]), t.Dot(ua[2]));

	Matrix3 R, AbsR;
	float ra, rb;

	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edges are parallel and
	// their cross product is (near) null (see text for details)
	const float EPLISON = 1e-6f;
	for (int i = 0; i < 3; i++)
	for (int j = 0; j < 3; j++)
	{
		R[i][j] = ua[i].Dot(ub[j]);
		AbsR[i][j] = fabsf(R[i][j]) + EPLISON;
	}

	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++)
	{
		ra = ea[i];
		rb = eb[0] * AbsR[i][0] + eb[1] * AbsR[i][1] + eb[2] * AbsR[i][2];
		if (fabsf(t[i]) > ra + rb)		return false;
	}

	// Test axes L = B0, L = B1, L = B2
	for (int j = 0; j < 3; j++)
	{
		ra = ea[0] * AbsR[0][j] + ea[1] * AbsR[1][j] + ea[2] * AbsR[2][j];
		rb = eb[j];
		if (fabsf(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ra + rb)	return false;
	}

	// Test Axis L = A0 x B0
	ra = ea[1] * AbsR[2][0] + ea[2] * AbsR[1][0];
	rb = eb[1] * AbsR[0][2] + eb[2] * AbsR[0][1];
	if (fabsf(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)	return false;

	// Test Axis L = A0 x B1
	ra = ea[1] * AbsR[2][1] + ea[2] * AbsR[1][1];
	rb = eb[0] * AbsR[0][2] + eb[2] * AbsR[0][0];
	if (fabsf(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)	return false;

	// Test Axis L = A0 x B2
	ra = ea[1] * AbsR[2][2] + ea[2] * AbsR[1][2];
	if (fabsf(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)	return false;

	// Test Axis L = A1 x B0
	ra = ea[0] * AbsR[2][0] + ea[2] * AbsR[0][0];
	rb = eb[1] * AbsR[1][2] + eb[2] * AbsR[1][1];
	if (fabsf(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)	return false;

	// Test Axis L = A1 x B1
	ra = ea[0] * AbsR[2][1] + ea[2] * AbsR[0][1];
	rb = eb[0] * AbsR[1][2] + eb[2] * AbsR[1][0];
	if (fabsf(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)	return false;

	// Test Axis L = A1 x B2
	ra = ea[0] * AbsR[2][2] + ea[2] * AbsR[0][2];
	rb = eb[0] * AbsR[1][1] + eb[1] * AbsR[1][0];
	if (fabsf(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)	return false;

	// Test Axis L = A2 x B0
	ra = ea[0] * AbsR[1][0] + ea[1] * AbsR[0][0];
	rb = eb[1] * AbsR[2][2] + eb[2] * AbsR[2][1];
	if (fabsf(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)	return false;

	// Test Axis L = A2 x B1
	ra = ea[0] * AbsR[1][1] + ea[1] * AbsR[0][1];
	rb = eb[0] * AbsR[2][2] + eb[2] * AbsR[2][0];
	if (fabsf(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)	return false;

	// Test Axis L = A2 x B2
	ra = ea[0] * AbsR[1][2] + ea[1] * AbsR[0][2];
	rb = eb[0] * AbsR[2][1] + eb[1] * AbsR[2][0];
	if (fabsf(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)	return false;

	return true;
}

bool OrientedBox3d::IntersectPoint(const Vector3& point) const
{
	AxisAlignedBox3d aabb(-Extent, Extent);
	return aabb.IntersectPoint((point- Center) * Rotation);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3d::IntersectPlane(const Vector3& normal, const float D) const
{
	float r =	Extent.x * fabsf(DotProduct(normal, Rotation.GetCol(0))) +
				Extent.y * fabsf(DotProduct(normal, Rotation.GetCol(1))) +
				Extent.z * fabsf(DotProduct(normal, Rotation.GetCol(2)));
	float s = normal.Dot(Center) + D;
	return fabsf(s) <= r;
}

bool OrientedBox3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	return aabb.IntersectRay(Origin * Rotation, Direction * Rotation, t);
}

bool OrientedBox3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	const OrientedBox3d obb((Bmax + Bmin) * 0.5f, (Bmax - Bmin) * 0.5f, Matrix3::Identity());
	return IntersectOBB(obb);
}

bool OrientedBox3d::IntersectOBB(const OrientedBox3d& obb) const
{
	return OBBIntersectOBB(Center, Extent, Rotation, obb.Center, obb.Extent, obb.Rotation);
}

bool OrientedBox3d::IntersectOBB(const Vector3& _Center, const Vector3& _Extent, const Matrix3& _Rot) const
{
	return OBBIntersectOBB(Center, Extent, Rotation, _Center, _Extent, _Rot);
}

bool OrientedBox3d::IntersectSphere(const Vector3& _Center, float _Radius) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	return aabb.IntersectSphere(_Center * Rotation, _Radius);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3d::IntersectCapsule(const Vector3& X0, const Vector3 &X1, float Radius) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	return aabb.IntersectCapsule(X0 * Rotation, X1 * Rotation, Radius);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

bool OrientedBox3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3 &C) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	return aabb.IntersectTriangle(A * Rotation, B * Rotation, C * Rotation);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
}

static bool testBoxBoxAxis(const OrientedBox3d& obb0, const OrientedBox3d& obb1, const Vector3& axis, Vector3* normal, float* depth)
{
	float min0, max0;
	obb0.ProjectAxis(axis, &min0, &max0);

	float min1, max1;
	obb1.ProjectAxis(axis, &min1, &max1);

	if (max0 < min1 || max1 < min0)
		return false;

	float d = std::min(max0 - min1, max1 - min0);
	if (d < *depth)
	{
		*depth = d;
		*normal = axis;
	}
	return true;
}

bool OrientedBox3d::PenetrateOBB(const OrientedBox3d& obb, Vector3 *Normal, float *Depth) const
{
	Vector3 n;
	float d = FLT_MAX;
	for (int i = 0; i < 3; ++i)
	{
		if(!testBoxBoxAxis(*this, obb, Rotation.GetCol(i), &n, &d))
			return false;
		if(!testBoxBoxAxis(*this, obb, obb.Rotation.GetCol(i), &n, &d))
			return false;
	}

	for (int j = 0; j < 3; ++j)
	for (int i = 0; i < 3; ++i)
	{
		Vector3 cross = Rotation.GetRow(i).Cross(obb.Rotation.GetRow(j));
		if (cross.SquareLength() > 1e-6f)
		{
			cross.Normalize();
			if (!testBoxBoxAxis(*this, obb, cross, &n, &d))
				return false;
		}
	}
	
	if (n.Dot(Center - obb.Center) < 0.0f)
		n = -n;
	
	*Normal	= -n;
	*Depth	= d;
	return true;
}

bool OrientedBox3d::PenetrateOBB(const Vector3& _Center, const Vector3& _Extent, const Matrix3& _Rot, Vector3 *Normal, float *Depth) const
{
	OrientedBox3d obb(_Center, _Extent, _Rot);
	return PenetrateOBB(obb, Normal, Depth);
}

bool OrientedBox3d::PenetrateSphere(const Vector3 &rCenter, float rRadius, Vector3 *Normal, float *Depth) const
{
	Sphere3d sphere(rCenter, rRadius);
	if (sphere.PenetrateOBB(Center, Extent, Rotation, Normal, Depth))
	{
		*Normal = -*Normal;
		return true;
	}
	return false;
}

bool OrientedBox3d::PenetratePlane(const Vector3 &pNormal, float D, Vector3 *Normal, float *Depth) const
{
	Vector3 v[8];
	GetVertices(v);

	float dmin = FLT_MAX;
	for(int i = 0; i < 8; ++i)
	{
		const float d = pNormal.Dot(v[i]) + D;
		if (d < dmin)
		{
			dmin = d;
		}
	}
	if (dmin > 0.0f)
	{
		return false;
	}

	*Normal	= pNormal;
	*Depth = -dmin;
	return true;
}

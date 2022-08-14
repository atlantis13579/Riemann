
#include "OrientedBox3d.h"
#include "AxisAlignedBox3d.h"

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

Box3d OrientedBox3d::GetBoundingVolume() const
{
	return OrientedBox3d::ComputeBoundingVolume(Center, Extent, Rotation);
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
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	float SqrDist = aabb.SqrDistanceToPoint(Point * Rotation);
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
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);
	return aabb.IntersectPoint(point * Rotation);	// inv(Rot) * v = transpose(Rot) * v = v^T * (Rot)
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

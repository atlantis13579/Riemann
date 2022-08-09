
#include "OrientedBox3d.h"
#include "AxisAlignedBox3d.h"

// static
OrientedBox3d OrientedBox3d::CalcBoundingOBB_PCA(const Vector3 *points, int n)
{
	Vector3 mean;
	for (int i = 0; i < n; ++i)
	{
		mean += points[i];
	}
	mean *= (1.0f / n);
	
	Matrix3 covMatrix;
	covMatrix.LoadZero();
	
	for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j)
	{
		for (int k = 0; k < n; ++k)
		{
			float cij = (points[k][i] - mean[i]) * (points[k][j] - mean[j]);
			covMatrix[i][j] += cij;
		}
		covMatrix[i][j] /= n;
	}
	
	float eigens[3];
	Vector3 v[3];
	covMatrix.SolveEigenSymmetric(eigens, v);
	
	if (Determinant(v[0], v[1], v[2]) < 0)
	{
		v[2] = -v[2];
	}
	
	OrientedBox3d box;
	box.Center = mean;
	box.Rot = Matrix3(v[0].x, v[1].x, v[2].x,
					  v[0].y, v[1].y, v[2].y,
					  v[0].z, v[1].z, v[2].z);
	box.Extent = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	
	for (int i = 0; i < n; ++i)
	{
		Vector3 p = (box.Rot * (points[i] - mean)).Abs();
		box.Extent.x = std::max(box.Extent.x, p.x);
		box.Extent.y = std::max(box.Extent.y, p.y);
		box.Extent.z = std::max(box.Extent.z, p.z);
	}
	
	return box;
}

float OrientedBox3d::SqrDistanceToLine(const Vector3& P0, const Vector3& Dir, float* t) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 P0_BoxSpace = (P0 - Center) * Rot;
	const Vector3 Dir_BoxSpace = Dir * Rot;

	float SqrDist = aabb.SqrDistanceToLine(P0_BoxSpace, Dir_BoxSpace, t);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 P0_BoxSpace = (P0 - Center) * Rot;
	const Vector3 P1_BoxSpace = (P1 - Center) * Rot;
	float SqrDist = aabb.SqrDistanceToSegment(P0_BoxSpace, P1_BoxSpace);
	return SqrDist;
}

float OrientedBox3d::SqrDistanceToPoint(const Vector3& Point) const
{
	AxisAlignedBox3d aabb(Center - Extent, Center + Extent);

	const Vector3 Point_BoxSpace = (Point - Center) * Rot;

	float SqrDist = aabb.SqrDistanceToPoint(Point_BoxSpace);
	return SqrDist;
}

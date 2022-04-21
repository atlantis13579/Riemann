#include "Polygon3D.h"

float TriangleArea3D(const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
	return 0.5f * (b - a).Cross(c - a).Length();
}

float PolygonArea3D(const Vector3d* polygon, int nvert)
{
	if (nvert < 3)
		return 0.0f;

	float area = 0.0f;
	for (int i = 1; i < nvert - 1; ++i)
	{
		area += TriangleArea3D(polygon[0], polygon[i], polygon[i+1]);
	}
	return 0.5f * fabsf(area);
}

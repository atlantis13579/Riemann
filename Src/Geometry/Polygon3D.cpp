#include "Polygon3D.h"

float TriangleArea3D(const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
	float cx = (b.y - a.y) * (c.z - a.z) - (c.y - a.y) * (b.z - a.z);
	float cy = (b.z - a.z) * (c.x - a.x) - (c.z - a.z) * (b.x - a.x);
	float cz = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
	return 0.5f * sqrtf(cx * cx + cy * cy + cz * cz);
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
	return area;
}

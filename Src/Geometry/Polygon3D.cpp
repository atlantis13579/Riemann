#include "Polygon3d.h"
#include "../CollisionPrimitive/Triangle3d.h"

float PolygonArea3D(const Vector3d* polygon, int nvert)
{
	if (nvert < 3)
		return 0.0f;

	float area = 0.0f;
	for (int i = 1; i < nvert - 1; ++i)
	{
		area += Triangle3d::TriangleArea3D(polygon[0], polygon[i], polygon[i+1]);
	}
	return area;
}

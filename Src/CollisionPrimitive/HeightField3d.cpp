
#include "HeightField3d.h"
#include "Triangle3d.h"
#include "../Maths/Maths.h"

bool HeightField3d::IntersectRayY(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	int i = (int)((Origin.x - BV.Min.x) * InvDX);
	int j = (int)((Origin.z - BV.Min.z) * InvDZ);
	if (i < 0 || i >= (int)nRows - 1 || j <= 0 || j >= (int)nCols - 1)
	{
		return false;
	}

	int i0 = j * nCols + i;
	int i1 = j * nCols + i + 1;
	int i2 = (j + 1) * nCols + i;
	int i3 = (j + 1) * nCols + i + 1;

	float MinY = Min(Heights[i0], Heights[i1], Heights[i2], Heights[i3]);
	float MaxY = Max(Heights[i0], Heights[i1], Heights[i2], Heights[i3]);

	if (Origin.y > MaxY && Dir.y > 0)
	{
		return false;
	}

	if (Origin.y < MinY && Dir.y < 0)
	{
		return false;
	}

	bool hit = false;
	float min_t = FLT_MAX;

	Vector3d Tris[6];
	int nT = GetCellTriangle(i, j, Tris);
	for (int k = 0; k < nT; k += 3)
	{
		float tt;
		if (Triangle3d::RayIntersectTriangle(Origin, Dir, Tris[k], Tris[k + 1], Tris[k + 2], &tt) && tt < min_t)
		{
			min_t = tt;
			*t = min_t;
			hit = true;
		}
	}

	return hit;
}

bool HeightField3d::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	int AxisY = Dir.ParallelTo(Vector3d::UnitY());
	if (AxisY != 0)
	{
		// Handle the most common case
		return IntersectRayY(Origin, Dir, t);
	}

	return false;
}

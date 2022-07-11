
#include "HeightField3d.h"
#include "Triangle3d.h"
#include "Ray3d.h"
#include "../Maths/Maths.h"

#define X_INDEX(_x)		((int)((_x - BV.Min.x) * InvDX))
#define Z_INDEX(_z)		((int)((_z - BV.Min.z) * InvDZ))

static const float HfCellThickness = 0.0001f;

bool HeightField3d::IntersectRayCell(const Vector3d& Origin, const Vector3d& Dir, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
{
	float minH, maxH;
	GetHeightRange(i, j, minH, maxH);

	Result->AddTestCount(1);

	if (fabsf(Dir.y) < 0.000001f)
	{
		if (Origin.y < minH || Origin.y > maxH)
		{
			return false;
		}
	}

	{
		Vector3d Bmin(BV.Min.x + i * DX, minH, BV.Min.z + j * DZ);
		Vector3d Bmax(BV.Min.x + (i + 1) * DX, maxH, BV.Min.z + (j + 1) * DZ);
		float t0, t1;
		if (!Ray3d::RayIntersectAABB2(Origin, Dir, Bmin, Bmax, HfCellThickness, Option.maxDist, &t0, &t1))
		{
			return false;
		}
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
			if (min_t < Option.maxDist)
			{
				Result->hitTime = min_t;
				Result->cellIndex = j * nX + i;
				hit = true;
			}
		}
	}

	Result->AddTestCount(nT / 3);

	return hit;
}

bool HeightField3d::IntersectRayY(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
{
	const int i = X_INDEX(Origin.x);
	const int j = Z_INDEX(Origin.z);
	if (i < 0 || i >= (int)nX - 1 || j < 0 || j >= (int)nZ - 1)
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
			if (min_t < Option.maxDist)
			{
				Result->hitTime = min_t;
				Result->cellIndex = j * nX + i;
				hit = true;
			}
		}
	}

	Result->AddTestCount(nT / 3);

	return hit;
}

bool HeightField3d::IntersectRayBruteForce(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
{
	float min_dist = FLT_MAX;
	for (uint32_t i = 0; i < nX - 1; ++i)
	for (uint32_t j = 0; j < nZ - 1; ++j)
	{
		if (IntersectRayCell(Origin, Dir, i, j, Option, Result))
		{
			if (Result->hitTime < min_dist)
			{
				min_dist = Result->hitTime;
			}
		}
	}
	Result->hitTime = min_dist;
	return min_dist != FLT_MAX;
}

bool HeightField3d::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
{
	if (Origin.y > BV.Max.y && Dir.y >= 0)
	{
		return false;
	}
	else if (Origin.y < BV.Min.y && Dir.y <= 0)
	{
		return false;
	}

	int AxisY = Dir.ParallelTo(Vector3d::UnitY());
	if (AxisY != 0)
	{
		// Handle the most common case
		return IntersectRayY(Origin, Dir, Option, Result);
	}

	Result->AddTestCount(1);

	float t0, t1;
	if (!Ray3d::RayIntersectAABB2(Origin, Dir, BV.Min, BV.Max, HfCellThickness, Option.maxDist, &t0, &t1))
	{
		return false;
	}

	Vector3d P0 = Origin + Dir * t0;
	Vector3d P1 = Origin + Dir * t1;

	const int istart = X_INDEX(P0.x);
	const int jstart = Z_INDEX(P0.z);
	const int iend = X_INDEX(P1.x);
	const int jend = Z_INDEX(P1.z);
	const int di = Dir.x > 0 ? 1 : (Dir.x < 0 ? -1 : 0);
	const int dj = Dir.z > 0 ? 1 : (Dir.z < 0 ? -1 : 0);

	if (istart < 0 || istart >= (int)nX - 1 || jstart < 0 || jstart >= (int)nZ - 1)
	{
		return false;
	}
	
	if (iend < 0 || iend >= (int)nX - 1 || jend < 0 || jend >= (int)nZ - 1)
	{
		return false;
	}

	if (istart == iend && jstart == jend)
	{
		return IntersectRayCell(Origin, Dir, istart, jstart, Option, Result);
	}

	int i = istart;
	int j = jstart;

	if (istart != iend && jstart != jend)
	{
		const float minx = BV.Min.x + DX * istart;
		const float maxx = minx + DX;
		const float minz = BV.Min.z + DZ * jstart;
		const float maxz = minz + DZ;

		const float dtx = DX / fabsf(P1.x - P0.x);
		const float dty = DZ / fabsf(P1.z - P0.z);

		float tx = ((P0.x > P1.x) ? (P0.x - minx) : (maxx - P0.x)) / fabsf(P1.x - P0.x);
		float ty = ((P0.z > P1.z) ? (P0.z - minz) : (maxz - P0.z)) / fabsf(P1.z - P0.z);

		while (i != iend || j != jend)
		{
			if (IntersectRayCell(Origin, Dir, i, j, Option, Result))
			{
				return true;
			}

			if (tx <= ty)
			{
				if (i == iend)
					break;
				tx += dtx;
				i += di;
			}
			else
			{
				if (j == jend)
					break;
				ty += dty;
				j += dj;
			}
		}
	}
	else if (istart == iend)
	{
		while (j != jend)
		{
			if (IntersectRayCell(Origin, Dir, i, j, Option, Result))
			{
				return true;
			}
			j += dj;
		}
	}
	else
	{
		// jstart == jend
		while (i != iend)
		{
			if (IntersectRayCell(Origin, Dir, i, j, Option, Result))
			{
				return true;
			}
			i += di;
		}
	}

	return IntersectRayCell(Origin, Dir, iend, jend, Option, Result);
}


bool HeightField3d::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	HeightFieldHitOption Option;
	Option.maxDist = FLT_MAX;
	
	HeightFieldHitResult Result = { 0 };
	if (IntersectRay(Origin, Dir, Option, &Result))
	{
		*t = Result.hitTime;
		return true;
	}
	return false;
}

bool HeightField3d::IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
{
	Box3d Intersect;
	if (!BV.GetIntersection(Box3d(Bmin, Bmax), Intersect))
	{
		return false;
	}

	float minH = Bmin.y;
	float maxH = Bmax.y;

	const int i0 = X_INDEX(Intersect.Min.x);
	const int j0 = Z_INDEX(Intersect.Min.z);
	const int i1 = X_INDEX(Intersect.Max.x);
	const int j1 = Z_INDEX(Intersect.Max.z);
	for (int i = i0; i <= i1; ++i)
	for (int j = j0; j <= j1; ++j)
	{
		uint16_t Index = j * nX + i;
		if (minH <= Heights[Index] && Heights[Index] <= maxH)
		{
			return true;
		}
	}

	return true;
}

bool HeightField3d::GetCellBV(int i, int j, Box3d &box) const
{
	float minH, maxH;
	if (!GetHeightRange(i, j, minH, maxH))
	{
		return false;
	}
	box.Min = Vector3d(BV.Min.x + DX * (i + 0), minH, BV.Min.z + DZ * (j + 0));
	box.Max = Vector3d(BV.Min.x + DX * (i + 1), maxH, BV.Min.z + DZ * (j + 1));
	return true;
}

bool HeightField3d::GetHeightRange(int i, int j, float& minH, float& maxH) const
{
	assert(0 <= i && i < (int)nX - 1);
	assert(0 <= j && j < (int)nZ - 1);
	
	bool tessFlag = Cells[i + j * nX].Tessellation0 & 0x80;
	uint16_t i0 = j * nX + i;
	uint16_t i1 = j * nX + i + 1;
	uint16_t i2 = (j + 1) * nX + i;
	uint16_t i3 = (j + 1) * nX + i + 1;
	// i2---i3
	// |    |
	// |    |
	// i0---i1
	uint8_t Hole0 = Cells[i + j * nX].Tessellation0;
	uint8_t Hole1 = Cells[i + j * nX].Tessellation1;

	minH = FLT_MAX;
	maxH = -FLT_MAX;

	if (Hole0 != 0x7F && Hole1 != 0x7F)
	{
		minH = Min(Heights[i0], Heights[i1], Heights[i2], Heights[i3]);
		maxH = Max(Heights[i0], Heights[i1], Heights[i2], Heights[i3]);
		return true;
	}
	else if (Hole0 != 0x7F)
 	{
		minH = Min(Min(Heights[i0], Heights[i2]), (tessFlag ? Heights[i3] : Heights[i1]));
		maxH = Max(Max(Heights[i0], Heights[i2]), (tessFlag ? Heights[i3] : Heights[i1]));
	}
	else if (Hole1 != 0x7F)
	{
		minH = Min(Min(Heights[i1], Heights[i3]), (tessFlag ? Heights[i0] : Heights[i2]));
		maxH = Max(Max(Heights[i1], Heights[i3]), (tessFlag ? Heights[i0] : Heights[i2]));
	}

	return false;
}

int HeightField3d::GetCellTriangle(int i, int j, Vector3d Tris[6]) const
{
	assert(i >= 0 && i < (int)nX - 1);
	assert(j >= 0 && j < (int)nZ - 1);
	
	bool tessFlag = Cells[i + j * nX].Tessellation0 & 0x80;
	uint16_t i0 = j * nX + i;
	uint16_t i1 = j * nX + i + 1;
	uint16_t i2 = (j + 1) * nX + i;
	uint16_t i3 = (j + 1) * nX + i + 1;
	// i2---i3
	// |    |
	// |    |
	// i0---i1
	uint8_t Hole0 = Cells[i + j * nX].Tessellation0;
	uint8_t Hole1 = Cells[i + j * nX].Tessellation1;

	Vector3d Base = Vector3d(BV.Min.x + DX * i, 0.0f, BV.Min.z + DZ * j);

	int nt = 0;
	if (Hole0 != 0x7F)
	{
		Tris[0] = Base + Vector3d(0.0f, Heights[i2], DZ);
		Tris[1] = Base + Vector3d(0.0f, Heights[i0], 0.0f);
		Tris[2] = tessFlag ? Base + Vector3d(DX, Heights[i3], DZ) : Base + Vector3d(DX, Heights[i1], 0.0f);
		nt += 3;
	}
	if (Hole1 != 0x7F)
	{
		Tris[nt + 0] = Base + Vector3d(DX, Heights[i3], DZ);
		Tris[nt + 1] = tessFlag ? Base + Vector3d(0.0f, Heights[i0], 0.0f) : Base + Vector3d(0.0f, Heights[i2], DZ);
		Tris[nt + 2] = Base + Vector3d(DX, Heights[i1], 0.0f);
		nt += 3;
	}

	return nt;
}

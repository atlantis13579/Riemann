#include "AxisAlignedBox3.h"
#include "Cylinder3.h"
#include "Capsule3.h"
#include "Plane3.h"
#include "Sphere3.h"
#include "ConvexMesh.h"
#include "HeightField3.h"
#include "TriangleMesh.h"
#include "Triangle3.h"
#include "GJK.h"

#include <algorithm>
#include <cfloat>

namespace Riemann
{
static void face(unsigned int i0, unsigned int i1, unsigned int i2, Vector3& rkPnt, const Vector3& rkDir, const Vector3& extents, const Vector3& rkPmE, float* t, float& rfSqrDistance)
{
	Vector3 kPpE;
	float fLSqr, fInv, fTmp, fParam, fT, fDelta;

	kPpE[i1] = rkPnt[i1] + extents[i1];
	kPpE[i2] = rkPnt[i2] + extents[i2];
	if (rkDir[i0] * kPpE[i1] >= rkDir[i1] * rkPmE[i0])
	{
		if (rkDir[i0] * kPpE[i2] >= rkDir[i2] * rkPmE[i0])
		{
			// v[i1] >= -e[i1], v[i2] >= -e[i2] (distance = 0)
			if (t)
			{
				rkPnt[i0] = extents[i0];
				fInv = 1.0f / rkDir[i0];
				rkPnt[i1] -= rkDir[i1] * rkPmE[i0] * fInv;
				rkPnt[i2] -= rkDir[i2] * rkPmE[i0] * fInv;
				*t = -rkPmE[i0] * fInv;
			}
		}
		else
		{
			// v[i1] >= -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0] * rkDir[i0] + rkDir[i2] * rkDir[i2];
			fTmp = fLSqr * kPpE[i1] - rkDir[i1] * (rkDir[i0] * rkPmE[i0] + rkDir[i2] * kPpE[i2]);
			if (fTmp <= 2.0f * fLSqr * extents[i1])
			{
				fT = fTmp / fLSqr;
				fLSqr += rkDir[i1] * rkDir[i1];
				fTmp = kPpE[i1] - fT;
				fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * fTmp + rkDir[i2] * kPpE[i2];
				fParam = -fDelta / fLSqr;
				rfSqrDistance += rkPmE[i0] * rkPmE[i0] + fTmp * fTmp + kPpE[i2] * kPpE[i2] + fDelta * fParam;

				if (t)
				{
					*t = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = fT - extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i1] * rkDir[i1];
				fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * rkPmE[i1] + rkDir[i2] * kPpE[i2];
				fParam = -fDelta / fLSqr;
				rfSqrDistance += rkPmE[i0] * rkPmE[i0] + rkPmE[i1] * rkPmE[i1] + kPpE[i2] * kPpE[i2] + fDelta * fParam;

				if (t)
				{
					*t = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
		}
	}
	else
	{
		if (rkDir[i0] * kPpE[i2] >= rkDir[i2] * rkPmE[i0])
		{
			// v[i1] < -e[i1], v[i2] >= -e[i2]
			fLSqr = rkDir[i0] * rkDir[i0] + rkDir[i1] * rkDir[i1];
			fTmp = fLSqr * kPpE[i2] - rkDir[i2] * (rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1]);
			if (fTmp <= 2.0f * fLSqr * extents[i2])
			{
				fT = fTmp / fLSqr;
				fLSqr += rkDir[i2] * rkDir[i2];
				fTmp = kPpE[i2] - fT;
				fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1] + rkDir[i2] * fTmp;
				fParam = -fDelta / fLSqr;
				rfSqrDistance += rkPmE[i0] * rkPmE[i0] + kPpE[i1] * kPpE[i1] + fTmp * fTmp + fDelta * fParam;

				if (t)
				{
					*t = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = fT - extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i2] * rkDir[i2];
				fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1] + rkDir[i2] * rkPmE[i2];
				fParam = -fDelta / fLSqr;
				rfSqrDistance += rkPmE[i0] * rkPmE[i0] + kPpE[i1] * kPpE[i1] + rkPmE[i2] * rkPmE[i2] + fDelta * fParam;

				if (t)
				{
					*t = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = extents[i2];
				}
			}
		}
		else
		{
			// v[i1] < -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0] * rkDir[i0] + rkDir[i2] * rkDir[i2];
			fTmp = fLSqr * kPpE[i1] - rkDir[i1] * (rkDir[i0] * rkPmE[i0] + rkDir[i2] * kPpE[i2]);
			if (fTmp >= 0.0f)
			{
				// v[i1]-edge is closest
				if (fTmp <= 2.0f * fLSqr * extents[i1])
				{
					fT = fTmp / fLSqr;
					fLSqr += rkDir[i1] * rkDir[i1];
					fTmp = kPpE[i1] - fT;
					fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * fTmp + rkDir[i2] * kPpE[i2];
					fParam = -fDelta / fLSqr;
					rfSqrDistance += rkPmE[i0] * rkPmE[i0] + fTmp * fTmp + kPpE[i2] * kPpE[i2] + fDelta * fParam;

					if (t)
					{
						*t = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = fT - extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i1] * rkDir[i1];
					fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * rkPmE[i1] + rkDir[i2] * kPpE[i2];
					fParam = -fDelta / fLSqr;
					rfSqrDistance += rkPmE[i0] * rkPmE[i0] + rkPmE[i1] * rkPmE[i1] + kPpE[i2] * kPpE[i2] + fDelta * fParam;

					if (t)
					{
						*t = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				return;
			}

			fLSqr = rkDir[i0] * rkDir[i0] + rkDir[i1] * rkDir[i1];
			fTmp = fLSqr * kPpE[i2] - rkDir[i2] * (rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1]);
			if (fTmp >= 0.0f)
			{
				// v[i2]-edge is closest
				if (fTmp <= 2.0f * fLSqr * extents[i2])
				{
					fT = fTmp / fLSqr;
					fLSqr += rkDir[i2] * rkDir[i2];
					fTmp = kPpE[i2] - fT;
					fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1] + rkDir[i2] * fTmp;
					fParam = -fDelta / fLSqr;
					rfSqrDistance += rkPmE[i0] * rkPmE[i0] + kPpE[i1] * kPpE[i1] + fTmp * fTmp + fDelta * fParam;

					if (t)
					{
						*t = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = fT - extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i2] * rkDir[i2];
					fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1] + rkDir[i2] * rkPmE[i2];
					fParam = -fDelta / fLSqr;
					rfSqrDistance += rkPmE[i0] * rkPmE[i0] + kPpE[i1] * kPpE[i1] + rkPmE[i2] * rkPmE[i2] + fDelta * fParam;

					if (t)
					{
						*t = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = extents[i2];
					}
				}
				return;
			}

			// (v[i1],v[i2])-corner is closest
			fLSqr += rkDir[i2] * rkDir[i2];
			fDelta = rkDir[i0] * rkPmE[i0] + rkDir[i1] * kPpE[i1] + rkDir[i2] * kPpE[i2];
			fParam = -fDelta / fLSqr;
			rfSqrDistance += rkPmE[i0] * rkPmE[i0] + kPpE[i1] * kPpE[i1] + kPpE[i2] * kPpE[i2] + fDelta * fParam;

			if (t)
			{
				*t = fParam;
				rkPnt[i0] = extents[i0];
				rkPnt[i1] = -extents[i1];
				rkPnt[i2] = -extents[i2];
			}
		}
	}
}

static void caseNoZeros(Vector3& rkPnt, const Vector3& rkDir, const Vector3& extents, float* t, float& rfSqrDistance)
{
	Vector3 kPmE(rkPnt.x - extents.x, rkPnt.y - extents.y, rkPnt.z - extents.z);

	float fProdDxPy, fProdDyPx, fProdDzPx, fProdDxPz, fProdDzPy, fProdDyPz;

	fProdDxPy = rkDir.x * kPmE.y;
	fProdDyPx = rkDir.y * kPmE.x;
	if (fProdDyPx >= fProdDxPy)
	{
		fProdDzPx = rkDir.z * kPmE.x;
		fProdDxPz = rkDir.x * kPmE.z;
		if (fProdDzPx >= fProdDxPz)
		{
			// line intersects x = e0
			face(0, 1, 2, rkPnt, rkDir, extents, kPmE, t, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, t, rfSqrDistance);
		}
	}
	else
	{
		fProdDzPy = rkDir.z * kPmE.y;
		fProdDyPz = rkDir.y * kPmE.z;
		if (fProdDzPy >= fProdDyPz)
		{
			// line intersects y = e1
			face(1, 2, 0, rkPnt, rkDir, extents, kPmE, t, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, t, rfSqrDistance);
		}
	}
}

static void case0(unsigned int i0, unsigned int i1, unsigned int i2, Vector3& rkPnt, const Vector3& rkDir, const Vector3& extents, float* t, float& rfSqrDistance)
{
	float fPmE0 = rkPnt[i0] - extents[i0];
	float fPmE1 = rkPnt[i1] - extents[i1];
	float fProd0 = rkDir[i1] * fPmE0;
	float fProd1 = rkDir[i0] * fPmE1;
	float fDelta, fInvLSqr, fInv;

	if (fProd0 >= fProd1)
	{
		// line intersects P[i0] = e[i0]
		rkPnt[i0] = extents[i0];

		float fPpE1 = rkPnt[i1] + extents[i1];
		fDelta = fProd0 - rkDir[i0] * fPpE1;
		if (fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f / (rkDir[i0] * rkDir[i0] + rkDir[i1] * rkDir[i1]);
			rfSqrDistance += fDelta * fDelta * fInvLSqr;
			if (t)
			{
				rkPnt[i1] = -extents[i1];
				*t = -(rkDir[i0] * fPmE0 + rkDir[i1] * fPpE1) * fInvLSqr;
			}
		}
		else
		{
			if (t)
			{
				fInv = 1.0f / rkDir[i0];
				rkPnt[i1] -= fProd0 * fInv;
				*t = -fPmE0 * fInv;
			}
		}
	}
	else
	{
		// line intersects P[i1] = e[i1]
		rkPnt[i1] = extents[i1];

		float fPpE0 = rkPnt[i0] + extents[i0];
		fDelta = fProd1 - rkDir[i1] * fPpE0;
		if (fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f / (rkDir[i0] * rkDir[i0] + rkDir[i1] * rkDir[i1]);
			rfSqrDistance += fDelta * fDelta * fInvLSqr;
			if (t)
			{
				rkPnt[i0] = -extents[i0];
				*t = -(rkDir[i0] * fPpE0 + rkDir[i1] * fPmE1) * fInvLSqr;
			}
		}
		else
		{
			if (t)
			{
				fInv = 1.0f / rkDir[i1];
				rkPnt[i0] -= fProd1 * fInv;
				*t = -fPmE1 * fInv;
			}
		}
	}

	if (rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if (rkPnt[i2] > extents[i2])
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case00(unsigned int i0, unsigned int i1, unsigned int i2, Vector3& rkPnt, const Vector3& rkDir, const Vector3& extents, float* t, float& rfSqrDistance)
{
	float fDelta;

	if (t)
		*t = (extents[i0] - rkPnt[i0]) / rkDir[i0];

	rkPnt[i0] = extents[i0];

	if (rkPnt[i1] < -extents[i1])
	{
		fDelta = rkPnt[i1] + extents[i1];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i1] = -extents[i1];
	}
	else if (rkPnt[i1] > extents[i1])
	{
		fDelta = rkPnt[i1] - extents[i1];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i1] = extents[i1];
	}

	if (rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if (rkPnt[i2] > extents[i2])
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta * fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case000(Vector3& rkPnt, const Vector3& extents, float& rfSqrDistance)
{
	float fDelta;

	if (rkPnt.x < -extents.x)
	{
		fDelta = rkPnt.x + extents.x;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.x = -extents.x;
	}
	else if (rkPnt.x > extents.x)
	{
		fDelta = rkPnt.x - extents.x;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.x = extents.x;
	}

	if (rkPnt.y < -extents.y)
	{
		fDelta = rkPnt.y + extents.y;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.y = -extents.y;
	}
	else if (rkPnt.y > extents.y)
	{
		fDelta = rkPnt.y - extents.y;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.y = extents.y;
	}

	if (rkPnt.z < -extents.z)
	{
		fDelta = rkPnt.z + extents.z;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.z = -extents.z;
	}
	else if (rkPnt.z > extents.z)
	{
		fDelta = rkPnt.z - extents.z;
		rfSqrDistance += fDelta * fDelta;
		rkPnt.z = extents.z;
	}
}

float AxisAlignedBox3::SqrDistanceToLine(const Vector3& P0, const Vector3& tDir, float* t) const
{
	Vector3 Center = GetCenter() - P0;
	Vector3 Extent = (Max - Min) * 0.5f;
	Vector3 Dir = tDir;

	bool reflect[3];
	for (int i = 0; i < 3; i++)
	{
		if (Dir[i] < 0.0f)
		{
			Center[i] = -Center[i];
			Dir[i] = -Dir[i];
			reflect[i] = true;
		}
		else
		{
			reflect[i] = false;
		}
	}

	float sqrDistance = 0.0f;

	if (Dir.x > 0.0f)
	{
		if (Dir.y > 0.0f)
		{
			if (Dir.z > 0.0f)
			{
				caseNoZeros(Center, Dir, Extent, t, sqrDistance);		// (+,+,+)
			}
			else
			{
				case0(0, 1, 2, Center, Dir, Extent, t, sqrDistance);	// (+,+,0)
			}
		}
		else
		{
			if (Dir.z > 0.0f)
			{
				case0(0, 2, 1, Center, Dir, Extent, t, sqrDistance);	// (+,0,+)
			}
			else
			{
				case00(0, 1, 2, Center, Dir, Extent, t, sqrDistance);	// (+,0,0)
			}
		}
	}
	else
	{
		if (Dir.y > 0.0f)
		{
			if (Dir.z > 0.0f)
			{
				case0(1, 2, 0, Center, Dir, Extent, t, sqrDistance);	// (0,+,+)
			}
			else
			{
				case00(1, 0, 2, Center, Dir, Extent, t, sqrDistance);	// (0,+,0)
			}
		}
		else
		{
			if (Dir.z > 0.0f)
			{
				case00(2, 0, 1, Center, Dir, Extent, t, sqrDistance);	// (0,0,+)
			}
			else
			{
				case000(Center, Extent, sqrDistance);										// (0,0,0)

				*t = 0.0f;
			}
		}
	}

	return sqrDistance;
}

float AxisAlignedBox3::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
{
	const Vector3 Dir = P1 - P0;
	if (Dir.SquareLength() < TINY_NUMBER)
	{
		return SqrDistanceToPoint(P0);
	}

	float ts[8];
	int numTs = 0;
	auto addT = [&ts, &numTs](float t)
	{
		if (t < 0.0f || t > 1.0f)
		{
			return;
		}
		for (int i = 0; i < numTs; ++i)
		{
			if (fabsf(ts[i] - t) < 1e-6f)
			{
				return;
			}
		}
		ts[numTs++] = t;
	};

	addT(0.0f);
	addT(1.0f);
	for (int axis = 0; axis < 3; ++axis)
	{
		if (fabsf(Dir[axis]) > TINY_NUMBER)
		{
			addT((Min[axis] - P0[axis]) / Dir[axis]);
			addT((Max[axis] - P0[axis]) / Dir[axis]);
		}
	}
	std::sort(ts, ts + numTs);

	auto sqrDistanceAt = [this, &P0, &Dir](float t)
	{
		return SqrDistanceToPoint(P0 + Dir * t);
	};

	float minSqrDistance = std::min(sqrDistanceAt(0.0f), sqrDistanceAt(1.0f));
	for (int i = 0; i + 1 < numTs; ++i)
	{
		const float t0 = ts[i];
		const float t1 = ts[i + 1];
		if (t1 - t0 < 1e-6f)
		{
			continue;
		}

		const float mid = (t0 + t1) * 0.5f;
		float a = 0.0f;
		float b = 0.0f;
		for (int axis = 0; axis < 3; ++axis)
		{
			const float value = P0[axis] + Dir[axis] * mid;
			if (value < Min[axis])
			{
				a += Dir[axis] * Dir[axis];
				b += Dir[axis] * (P0[axis] - Min[axis]);
			}
			else if (value > Max[axis])
			{
				a += Dir[axis] * Dir[axis];
				b += Dir[axis] * (P0[axis] - Max[axis]);
			}
		}

		float t = mid;
		if (a > TINY_NUMBER)
		{
			t = std::max(t0, std::min(t1, -b / a));
		}
		minSqrDistance = std::min(minSqrDistance, sqrDistanceAt(t));
	}
	for (int i = 0; i < numTs; ++i)
	{
		minSqrDistance = std::min(minSqrDistance, sqrDistanceAt(ts[i]));
	}

	return minSqrDistance;
}

Vector3 AxisAlignedBox3::GetSupport(const Vector3& Direction) const
{
	return GetSupport(Min, Max, Direction);
}

// static
Vector3 AxisAlignedBox3::GetSupport(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction)
{
	return Vector3(
		Direction.x > 0 ? Bmax.x : Bmin.x,
		Direction.y > 0 ? Bmax.y : Bmin.y,
		Direction.z > 0 ? Bmax.z : Bmin.z
	);
}

int	 AxisAlignedBox3::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	return AxisAlignedBox3::GetSupportFace(Min, Max, Direction, FacePoints);
}

int AxisAlignedBox3::GetSupportFace(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction, Vector3* FacePoints)
{
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

bool AxisAlignedBox3::IntersectPoint(const Vector3& Point) const
{
	if (Point.x >= Min.x && Point.x <= Max.x &&
		Point.y >= Min.y && Point.y <= Max.y &&
		Point.z >= Min.z && Point.z <= Max.z)
	{
		return true;
	}
	return false;
}

bool AxisAlignedBox3::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
{
	if (Min.x > Bmax.x || Bmin.x > Max.x)
	{
		return false;
	}

	if (Min.y > Bmax.y || Bmin.y > Max.y)
	{
		return false;
	}

	if (Min.z > Bmax.z || Bmin.z > Max.z)
	{
		return false;
	}

	return true;
}

bool AxisAlignedBox3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	const Vector3 b0 = Min - Origin;
	const Vector3 b1 = Max - Origin;

	float tMin = 0;
	float tMax = FLT_MAX;

	for (int i = 0; i < 3; ++i)
	{
		float t0, t1;
		if (fabsf(Direction[i]) < 0.00001f)
		{
			if (b0[i] > 0 || b1[i] < 0)
			{
				return false;
			}
			else
			{
				t0 = 0;
				t1 = FLT_MAX;
			}
		}
		else
		{
			const float InvDir = 1.0f / Direction[i];
			t0 = b0[i] * InvDir;
			t1 = b1[i] * InvDir;
		}

		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		tMin = std::max(tMin, t0);
		tMax = std::min(tMax, t1);

		if (tMin > tMax)
		{
			return false;
		}
	}

	if (tMax < 0)
	{
		return false;
	}

	*t = tMin;
	return true;
}

bool AxisAlignedBox3::IntersectSegment(const Vector3& p0, const Vector3& p1) const
{
	Vector3 c = GetCenter();
	Vector3 e = GetExtent();
	Vector3 m = (p0 + p1) * 0.5f;
	Vector3 d = p1 - m;
	m = m - c;

	float adx = fabsf(d.x);
	if (fabsf(m.x) > e.x + adx)
		return false;
	float ady = fabsf(d.y);
	if (fabsf(m.y) > e.y + ady)
		return false;
	float adz = fabsf(d.z);
	if (fabsf(m.z) > e.z + adz)
		return false;

	const float kEps = 1e-6f;
	adx += kEps;
	ady += kEps;
	adz += kEps;

	if (fabsf(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady)
		return false;
	if (fabsf(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx)
		return false;
	if (fabsf(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx)
		return false;
	return true;
}

bool AxisAlignedBox3::IntersectSphere(const Vector3& Center, float Radius) const
{
	float SqrDist = SqrDistanceToPoint(Center);
	return SqrDist <= Radius * Radius;
}

bool AxisAlignedBox3::IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const
{
	float SqrDist = SqrDistanceToSegment(P0, P1);
	return SqrDist <= Radius * Radius;
}


#define AXISTEST_X01(a, b, fa, fb)					\
p0 = a*v0.y - b*v0.z;							\
p2 = a*v2.y - b*v2.z;							\
minimum = std::min(p0, p2);						\
maximum = std::max(p0, p2);						\
rad = fa * extents.y + fb * extents.z;			\
if(minimum>rad || maximum<-rad) return false;

#define AXISTEST_X2(a, b, fa, fb)					\
p0 = a*v0.y - b*v0.z;							\
p1 = a*v1.y - b*v1.z;							\
minimum = std::min(p0, p1);						\
maximum = std::max(p0, p1);						\
rad = fa * extents.y + fb * extents.z;			\
if(minimum>rad || maximum<-rad) return false;

#define AXISTEST_Y02(a, b, fa, fb)					\
p0 = -a*v0.x + b*v0.z;							\
p2 = -a*v2.x + b*v2.z;							\
minimum = std::min(p0, p2);						\
maximum = std::max(p0, p2);						\
rad = fa * extents.x + fb * extents.z;			\
if(minimum>rad || maximum<-rad) return false;

#define AXISTEST_Y1(a, b, fa, fb)					\
p0 = -a*v0.x + b*v0.z;							\
p1 = -a*v1.x + b*v1.z;							\
minimum = std::min(p0, p1);						\
maximum = std::max(p0, p1);						\
rad = fa * extents.x + fb * extents.z;			\
if(minimum>rad || maximum<-rad) return false;

#define AXISTEST_Z12(a, b, fa, fb)					\
p1 = a*v1.x - b*v1.y;							\
p2 = a*v2.x - b*v2.y;							\
minimum = std::min(p1, p2);						\
maximum = std::max(p1, p2);						\
rad = fa * extents.x + fb * extents.y;			\
if(minimum>rad || maximum<-rad) return false;

#define AXISTEST_Z0(a, b, fa, fb)					\
p0 = a*v0.x - b*v0.y;							\
p1 = a*v1.x - b*v1.y;							\
minimum = std::min(p0, p1);						\
maximum = std::max(p0, p1);						\
rad = fa * extents.x + fb * extents.y;			\
if(minimum>rad || maximum<-rad) return false;

#define FINDMINMAX(x0, x1, x2, minimum, maximum)	\
minimum = std::min(x0, x1);						\
maximum = std::max(x0, x1);						\
minimum = std::min(minimum, x2);				\
maximum = std::max(maximum, x2);

// Real Time Collision Detection - Christer Ericson
// Chapter 5.2.9, page 169-172
bool AxisAlignedBox3::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
{
	Vector3 boxcenter = GetCenter();
	Vector3 extents = GetExtent();

	const Vector3 v0 = A - boxcenter;
	const Vector3 v1 = B - boxcenter;
	const Vector3 v2 = C - boxcenter;

	// compute triangle edges
	const Vector3 e0 = v1 - v0;	// tri edge 0
	const Vector3 e1 = v2 - v1;	// tri edge 1
	const Vector3 e2 = v0 - v2;	// tri edge 2

	float minimum, maximum, rad, p0, p1, p2;

	// Bullet 3: test the 9 tests first (this was faster)
	float fex = fabsf(e0.x);
	float fey = fabsf(e0.y);
	float fez = fabsf(e0.z);
	AXISTEST_X01(e0.z, e0.y, fez, fey);
	AXISTEST_Y02(e0.z, e0.x, fez, fex);
	AXISTEST_Z12(e0.y, e0.x, fey, fex);

	fex = fabsf(e1.x);
	fey = fabsf(e1.y);
	fez = fabsf(e1.z);
	AXISTEST_X01(e1.z, e1.y, fez, fey);
	AXISTEST_Y02(e1.z, e1.x, fez, fex);
	AXISTEST_Z0(e1.y, e1.x, fey, fex);

	fex = fabsf(e2.x);
	fey = fabsf(e2.y);
	fez = fabsf(e2.z);
	AXISTEST_X2(e2.z, e2.y, fez, fey);
	AXISTEST_Y1(e2.z, e2.x, fez, fex);
	AXISTEST_Z12(e2.y, e2.x, fey, fex);

	// Bullet 1:
	//  first test overlap in the {x,y,z}-directions
	//  find minimum, maximum of the triangle each direction, and test for overlap in
	//  that direction -- this is equivalent to testing a minimal AABB around
	//  the triangle against the AABB

	// test in X-direction
	FINDMINMAX(v0.x, v1.x, v2.x, minimum, maximum);
	if (minimum > extents.x || maximum < -extents.x)
		return false;

	// test in Y-direction
	FINDMINMAX(v0.y, v1.y, v2.y, minimum, maximum);
	if (minimum > extents.y || maximum < -extents.y)
		return false;

	// test in Z-direction
	FINDMINMAX(v0.z, v1.z, v2.z, minimum, maximum);
	if (minimum > extents.z || maximum < -extents.z)
		return false;

	// Bullet 2:
	//  test if the box intersects the plane of the triangle
	//  compute plane equation of triangle: normal*x+d=0
	Vector3 normal = e0.Cross(e1);
	const float D = -DotProduct(normal, v0);	// plane eq: normal.x+d=0
	Plane3 plane(normal, D);
	if (!plane.IntersectAABB(-extents, extents))
		return false;

	return true;
}

static Vector3 cross100(const Vector3& b)
{
	return Vector3(0.0f, -b.z, b.y);
}

static Vector3 cross010(const Vector3& b)
{
	return Vector3(b.z, 0.0f, -b.x);
}

static Vector3 cross001(const Vector3& b)
{
	return Vector3(-b.y, b.x, 0.0f);
}

static Vector3 safeUnitOrZero(const Vector3& v)
{
	Vector3 n = v;
	n.SafeNormalize();
	return n;
}

static bool testBoxTriSweepAxis(const Vector3 tri[3], const Vector3& extents, const Vector3& dir, const Vector3& axis, bool& validMTD, float& tfirst, float& tlast)
{
	const float d0t = tri[0].Dot(axis);
	const float d1t = tri[1].Dot(axis);
	const float d2t = tri[2].Dot(axis);

	float triMin = std::min(d0t, d1t);
	float triMax = std::max(d0t, d1t);
	triMin = std::min(triMin, d2t);
	triMax = std::max(triMax, d2t);

	const float boxExt = fabsf(axis.x) * extents.x + fabsf(axis.y) * extents.y + fabsf(axis.z) * extents.z;
	const float d0 = -boxExt - triMax;
	const float d1 = boxExt - triMin;
	const bool intersect = (d0 <= 0.0f && d1 >= 0.0f);
	validMTD &= intersect;

	const float v = dir.Dot(axis);
	if (fabsf(v) < 1.0e-6f)
	{
		return intersect;
	}

	const float oneOverV = -1.0f / v;
	const float t0_ = d0 * oneOverV;
	const float t1_ = d1 * oneOverV;
	const float t0 = std::min(t0_, t1_);
	const float t1 = std::max(t0_, t1_);

	if (t0 > tlast || t1 < tfirst)
	{
		return false;
	}

	tlast = std::min(t1, tlast);
	tfirst = std::max(t0, tfirst);
	return true;
}

static bool testBoxTriSweepAxisXYZ(const Vector3 tri[3], const Vector3& extents, const Vector3& dir, int axis, bool& validMTD, float& tfirst, float& tlast)
{
	const float d0t = tri[0][axis];
	const float d1t = tri[1][axis];
	const float d2t = tri[2][axis];

	float triMin = std::min(d0t, d1t);
	float triMax = std::max(d0t, d1t);
	triMin = std::min(triMin, d2t);
	triMax = std::max(triMax, d2t);

	const float boxExt = extents[axis];
	const float d0 = -boxExt - triMax;
	const float d1 = boxExt - triMin;
	const bool intersect = (d0 <= 0.0f && d1 >= 0.0f);
	validMTD &= intersect;

	const float v = dir[axis];
	if (fabsf(v) < 1.0e-6f)
	{
		return intersect;
	}

	const float oneOverV = -1.0f / v;
	const float t0_ = d0 * oneOverV;
	const float t1_ = d1 * oneOverV;
	const float t0 = std::min(t0_, t1_);
	const float t1 = std::max(t0_, t1_);

	if (t0 > tlast || t1 < tfirst)
	{
		return false;
	}

	tlast = std::min(t1, tlast);
	tfirst = std::max(t0, tfirst);
	return true;
}

static bool triBoxSweepTestBoxSpace(const Vector3 tri[3], const Vector3& extents, const Vector3& dir, float tmax, float& toi, bool doBackfaceCulling)
{
	const Vector3 normal = (tri[1] - tri[0]).Cross(tri[2] - tri[0]);
	if (doBackfaceCulling && normal.Dot(dir) >= 0.0f)
	{
		return false;
	}

	bool validMTD = true;
	float tfirst = -FLT_MAX;
	float tlast = FLT_MAX;

	if (!testBoxTriSweepAxis(tri, extents, dir, normal, validMTD, tfirst, tlast))
		return false;
	if (!testBoxTriSweepAxisXYZ(tri, extents, dir, 0, validMTD, tfirst, tlast))
		return false;
	if (!testBoxTriSweepAxisXYZ(tri, extents, dir, 1, validMTD, tfirst, tlast))
		return false;
	if (!testBoxTriSweepAxisXYZ(tri, extents, dir, 2, validMTD, tfirst, tlast))
		return false;

	for (int i = 0; i < 3; ++i)
	{
		const int j = (i + 1) % 3;
		const Vector3 triEdge = tri[j] - tri[i];

		const Vector3 sep0 = cross100(triEdge);
		if (sep0.SquareLength() >= 1.0e-6f && !testBoxTriSweepAxis(tri, extents, dir, sep0, validMTD, tfirst, tlast))
			return false;

		const Vector3 sep1 = cross010(triEdge);
		if (sep1.SquareLength() >= 1.0e-6f && !testBoxTriSweepAxis(tri, extents, dir, sep1, validMTD, tfirst, tlast))
			return false;

		const Vector3 sep2 = cross001(triEdge);
		if (sep2.SquareLength() >= 1.0e-6f && !testBoxTriSweepAxis(tri, extents, dir, sep2, validMTD, tfirst, tlast))
			return false;
	}

	if (tfirst > tmax || tlast < 0.0f)
	{
		return false;
	}

	if (tfirst <= 0.0f)
	{
		if (!validMTD)
		{
			return false;
		}
		toi = 0.0f;
	}
	else
	{
		toi = tfirst;
	}

	return true;
}

static void computeBoxPoints(const Vector3& minimum, const Vector3& maximum, Vector3 pts[8])
{
	pts[0] = Vector3(minimum.x, minimum.y, minimum.z);
	pts[1] = Vector3(maximum.x, minimum.y, minimum.z);
	pts[2] = Vector3(maximum.x, maximum.y, minimum.z);
	pts[3] = Vector3(minimum.x, maximum.y, minimum.z);
	pts[4] = Vector3(minimum.x, minimum.y, maximum.z);
	pts[5] = Vector3(maximum.x, minimum.y, maximum.z);
	pts[6] = Vector3(maximum.x, maximum.y, maximum.z);
	pts[7] = Vector3(minimum.x, maximum.y, maximum.z);
}

static const uint8_t* getBoxEdges()
{
	static const uint8_t indices[] =
	{
		0, 1,	1, 2,	2, 3,	3, 0,
		7, 6,	6, 5,	5, 4,	4, 7,
		1, 5,	6, 2,
		3, 7,	4, 0
	};
	return indices;
}

static const Vector3* getBoxVertexNormals()
{
	static const float invSqrt3 = 0.577350269189f;
	static const Vector3 vertexNormals[] =
	{
		Vector3(-invSqrt3, -invSqrt3, -invSqrt3),
		Vector3( invSqrt3, -invSqrt3, -invSqrt3),
		Vector3( invSqrt3,  invSqrt3, -invSqrt3),
		Vector3(-invSqrt3,  invSqrt3, -invSqrt3),
		Vector3(-invSqrt3, -invSqrt3,  invSqrt3),
		Vector3( invSqrt3, -invSqrt3,  invSqrt3),
		Vector3( invSqrt3,  invSqrt3,  invSqrt3),
		Vector3(-invSqrt3,  invSqrt3,  invSqrt3)
	};
	return vertexNormals;
}

static const Vector3* getBoxLocalEdgeNormals()
{
	static const float invSqrt2 = 0.707106781188f;
	static const Vector3 edgeNormals[] =
	{
		Vector3(0.0f, -invSqrt2, -invSqrt2),
		Vector3(invSqrt2, 0.0f, -invSqrt2),
		Vector3(0.0f, invSqrt2, -invSqrt2),
		Vector3(-invSqrt2, 0.0f, -invSqrt2),
		Vector3(0.0f, invSqrt2, invSqrt2),
		Vector3(invSqrt2, 0.0f, invSqrt2),
		Vector3(0.0f, -invSqrt2, invSqrt2),
		Vector3(-invSqrt2, 0.0f, invSqrt2),
		Vector3(invSqrt2, -invSqrt2, 0.0f),
		Vector3(invSqrt2, invSqrt2, 0.0f),
		Vector3(-invSqrt2, invSqrt2, 0.0f),
		Vector3(-invSqrt2, -invSqrt2, 0.0f)
	};
	return edgeNormals;
}

static const Vector3* getNearPlaneNormals()
{
	static const Vector3 planeNormals[] =
	{
		Vector3( 1.0f,  0.0f,  0.0f),
		Vector3( 0.0f,  1.0f,  0.0f),
		Vector3( 0.0f,  0.0f,  1.0f),
		Vector3(-1.0f,  0.0f,  0.0f),
		Vector3( 0.0f, -1.0f,  0.0f),
		Vector3( 0.0f,  0.0f, -1.0f)
	};
	return planeNormals;
}

static void makeFatEdge(Vector3& p0, Vector3& p1, float fatCoeff)
{
	Vector3 delta = p1 - p0;
	const float m = delta.Length();
	if (m > 0.0f)
	{
		delta *= fatCoeff / m;
		p0 -= delta;
		p1 += delta;
	}
}

static void inflateTriangle(const Vector3 tri[3], float fatCoeff, Vector3 outTri[3])
{
	const Vector3 center = (tri[0] + tri[1] + tri[2]) * 0.333333333f;
	for (int i = 0; i < 3; ++i)
	{
		const Vector3 v = tri[i] - center;
		outTri[i] = tri[i] + v * fatCoeff;
	}
}

static bool rayTriPrecaCull(const Vector3& orig, const Vector3& dir, const Vector3& vert0, const Vector3& edge1, const Vector3& edge2, const Vector3& pvec, float det, float oneOverDet, float& t)
{
	const Vector3 tvec = orig - vert0;
	const float u = tvec.Dot(pvec);
	if (u < 0.0f || u > det)
	{
		return false;
	}

	const Vector3 qvec = tvec.Cross(edge1);
	const float v = dir.Dot(qvec);
	if (v < 0.0f || u + v > det)
	{
		return false;
	}

	t = edge2.Dot(qvec) * oneOverDet;
	return true;
}

static bool rayTriPrecaNoCull(const Vector3& orig, const Vector3& dir, const Vector3& vert0, const Vector3& edge1, const Vector3& edge2, const Vector3& pvec, float oneOverDet, float& t)
{
	const Vector3 tvec = orig - vert0;
	const float u = tvec.Dot(pvec) * oneOverDet;
	if (u < 0.0f || u > 1.0f)
	{
		return false;
	}

	const Vector3 qvec = tvec.Cross(edge1);
	const float v = dir.Dot(qvec) * oneOverDet;
	if (v < 0.0f || u + v > 1.0f)
	{
		return false;
	}

	t = edge2.Dot(qvec) * oneOverDet;
	return true;
}

static int intersectRayAABB2(const Vector3& minimum, const Vector3& maximum, const Vector3& ro, const Vector3& oneOverDir, float& tnear, float& tfar, bool fbx, bool fby, bool fbz)
{
	if (fbx && (ro.x < minimum.x || ro.x > maximum.x))
		return -1;
	if (fby && (ro.y < minimum.y || ro.y > maximum.y))
		return -1;
	if (fbz && (ro.z < minimum.z || ro.z > maximum.z))
		return -1;

	float t1x = (minimum.x - ro.x) * oneOverDir.x;
	float t2x = (maximum.x - ro.x) * oneOverDir.x;
	float t1y = (minimum.y - ro.y) * oneOverDir.y;
	float t2y = (maximum.y - ro.y) * oneOverDir.y;
	float t1z = (minimum.z - ro.z) * oneOverDir.z;
	float t2z = (maximum.z - ro.z) * oneOverDir.z;

	int bx;
	int by;
	int bz;

	if (t1x > t2x)
	{
		std::swap(t1x, t2x);
		bx = 3;
	}
	else
	{
		bx = 0;
	}

	if (t1y > t2y)
	{
		std::swap(t1y, t2y);
		by = 4;
	}
	else
	{
		by = 1;
	}

	if (t1z > t2z)
	{
		std::swap(t1z, t2z);
		bz = 5;
	}
	else
	{
		bz = 2;
	}

	int ret;
	if (!fbx)
	{
		tnear = t1x;
		tfar = t2x;
		ret = bx;
	}
	else
	{
		tnear = -FLT_MAX;
		tfar = FLT_MAX;
		ret = -1;
	}

	if (!fby)
	{
		if (t1y > tnear)
		{
			tnear = t1y;
			ret = by;
		}
		tfar = std::min(tfar, t2y);
	}

	if (!fbz)
	{
		if (t1z > tnear)
		{
			tnear = t1z;
			ret = bz;
		}
		tfar = std::min(tfar, t2z);
	}

	if (tnear > tfar || tfar < FLT_EPSILON)
	{
		return -1;
	}

	return ret;
}

static void closestAxis2(const Vector3& v, uint32_t& j, uint32_t& k)
{
	const float absPx = fabsf(v.x);
	const float absPy = fabsf(v.y);
	const float absPz = fabsf(v.z);
	j = 1;
	k = 2;
	if (absPy > absPx && absPy > absPz)
	{
		j = 2;
		k = 0;
	}
	else if (absPz > absPx)
	{
		j = 0;
		k = 1;
	}
}

static bool intersectEdgeEdge3(const Vector3& planeNormal, float planeD, const Vector3& p1, const Vector3& p2, const Vector3& dir, const Vector3& v1, const Vector3& p3, const Vector3& p4, float& dist, Vector3& ip, uint32_t i, uint32_t j, float coeff)
{
	const float d3 = planeNormal.Dot(p3) + planeD;
	const float temp = d3 * (planeNormal.Dot(p4) + planeD);
	if (temp > 0.0f)
	{
		return false;
	}

	const Vector3 v2 = p4 - p3;
	const float temp2 = planeNormal.Dot(v2);
	if (temp2 == 0.0f)
	{
		return false;
	}

	ip = p3 - v2 * (d3 / temp2);
	dist = (v1[i] * (ip[j] - p1[j]) - v1[j] * (ip[i] - p1[i])) * coeff;
	if (dist < 0.0f)
	{
		return false;
	}

	ip -= dist * dir;
	const float temp3 = (p1.x - ip.x) * (p2.x - ip.x) + (p1.y - ip.y) * (p2.y - ip.y) + (p1.z - ip.z) * (p2.z - ip.z);
	return temp3 < 0.0f;
}

static void edgeEdgeDistNoZeroVector(Vector3& x, Vector3& y, const Vector3& p, const Vector3& a, const Vector3& q, const Vector3& b)
{
	const Vector3 T = q - p;
	const float ADotA = a.Dot(a);
	const float BDotB = b.Dot(b);
	if (ADotA <= 1.0e-12f || BDotB <= 1.0e-12f)
	{
		x = p;
		y = q;
		return;
	}

	const float ADotB = a.Dot(b);
	const float ADotT = a.Dot(T);
	const float BDotT = b.Dot(T);
	const float denom = ADotA * BDotB - ADotB * ADotB;

	float t;
	if (denom != 0.0f)
	{
		t = (ADotT * BDotB - BDotT * ADotB) / denom;
		t = std::max(0.0f, std::min(1.0f, t));
	}
	else
	{
		t = 0.0f;
	}

	float u = (t * ADotB - BDotT) / BDotB;
	if (u < 0.0f)
	{
		u = 0.0f;
		t = ADotT / ADotA;
		t = std::max(0.0f, std::min(1.0f, t));
	}
	else if (u > 1.0f)
	{
		u = 1.0f;
		t = (ADotB + ADotT) / ADotA;
		t = std::max(0.0f, std::min(1.0f, t));
	}

	x = p + a * t;
	y = q + b * u;
}

static void computeEdgeEdgeNormal(Vector3& normal, const Vector3& p1, const Vector3& p2_p1, const Vector3& p3, const Vector3& p4_p3, const Vector3& dir, float d)
{
	const float safeDistanceForNormalComputation = 0.1f;
	const Vector3 p1s = p1 + dir * (d - safeDistanceForNormalComputation);
	Vector3 x, y;
	edgeEdgeDistNoZeroVector(x, y, p1s, p2_p1, p3, p4_p3);
	normal = x - y;
}

static bool sweepBoxTriangleFeatureBased(const Vector3 tri[3], const Vector3& boxMin, const Vector3& boxMax, const Vector3& motion, const Vector3& oneOverMotion, Vector3& hit, Vector3& normal, float& d, bool isDoubleSided)
{
	const float localEpsilon = 0.00001f;
	const float fatTriangleCoeff = 0.02f;
	const float fatBoxEdgeCoeff = 0.01f;

	const Vector3 denormalizedTriNormal = (tri[1] - tri[0]).Cross(tri[2] - tri[0]);
	const bool doBackfaceCulling = !isDoubleSided;
	if (doBackfaceCulling && denormalizedTriNormal.Dot(motion) >= 0.0f)
	{
		return false;
	}

	Vector3 boxVertices[8];
	computeBoxPoints(boxMin, boxMax, boxVertices);

	Vector3 fatTri[3];
	inflateTriangle(tri, fatTriangleCoeff, fatTri);

	float minDist = d;
	int col = -1;

	{
		const Vector3 edge1 = fatTri[1] - fatTri[0];
		const Vector3 edge2 = fatTri[2] - fatTri[0];
		const Vector3 pvec = motion.Cross(edge2);
		const float det = edge1.Dot(pvec);
		const float oneOverDet = det != 0.0f ? 1.0f / det : 0.0f;
		const Vector3* vertexNormals = getBoxVertexNormals();

		uint32_t hitIndex = 0;
		if (doBackfaceCulling)
		{
			if (det >= localEpsilon)
			{
				for (uint32_t i = 0; i < 8; ++i)
				{
					if (vertexNormals[i].Dot(denormalizedTriNormal) >= 0.0f)
						continue;

					float tt;
					if (!rayTriPrecaCull(boxVertices[i], motion, fatTri[0], edge1, edge2, pvec, det, oneOverDet, tt))
						continue;
					if (tt < 0.0f || tt > minDist)
						continue;

					minDist = tt;
					col = 0;
					hitIndex = i;
				}
			}
		}
		else
		{
			if (det <= -localEpsilon || det >= localEpsilon)
			{
				for (uint32_t i = 0; i < 8; ++i)
				{
					float tt;
					if (!rayTriPrecaNoCull(boxVertices[i], motion, fatTri[0], edge1, edge2, pvec, oneOverDet, tt))
						continue;
					if (tt < 0.0f || tt > minDist)
						continue;

					minDist = tt;
					col = 0;
					hitIndex = i;
				}
			}
		}

		if (col == 0)
		{
			hit = boxVertices[hitIndex] + minDist * motion;
			normal = denormalizedTriNormal;
		}
	}

	{
		const Vector3 negMotion = -motion;
		const Vector3 negInvMotion = -oneOverMotion;
		const bool b0 = fabsf(negMotion.x) < FLT_EPSILON;
		const bool b1 = fabsf(negMotion.y) < FLT_EPSILON;
		const bool b2 = fabsf(negMotion.z) < FLT_EPSILON;
		const Vector3* boxNormals = getNearPlaneNormals();

		for (uint32_t i = 0; i < 3; ++i)
		{
			float tnear, tfar;
			const int plane = intersectRayAABB2(boxMin, boxMax, tri[i], negInvMotion, tnear, tfar, b0, b1, b2);
			if (plane == -1 || tnear < 0.0f)
				continue;

			if (tnear <= minDist)
			{
				minDist = tnear;
				normal = boxNormals[plane];
				col = 1;
				hit = tri[i];
			}
		}
	}

	uint32_t savedJ = 0xFFFFFFFF;
	uint32_t savedK = 0xFFFFFFFF;
	Vector3 p1s;
	Vector3 v1s;

	{
		const uint8_t* edges = getBoxEdges();
		const Vector3* edgeNormals = getBoxLocalEdgeNormals();
		for (uint32_t i = 0; i < 12; ++i)
		{
			Vector3 p1 = boxVertices[*edges++];
			Vector3 p2 = boxVertices[*edges++];
			makeFatEdge(p1, p2, fatBoxEdgeCoeff);

			if (edgeNormals[i].Dot(motion) < 0.0f)
				continue;

			const Vector3 v1 = p2 - p1;
			const Vector3 planeNormal = v1.Cross(motion);
			const float planeD = -planeNormal.Dot(p1);

			uint32_t closestI, closestJ;
			closestAxis2(planeNormal, closestI, closestJ);

			const float denom = v1[closestI] * motion[closestJ] - v1[closestJ] * motion[closestI];
			if (denom == 0.0f)
				continue;

			const float coeff = 1.0f / denom;
			for (uint32_t j = 0; j < 3; ++j)
			{
				const uint32_t k = (j + 1) % 3;
				float dist;
				Vector3 ip;
				if (intersectEdgeEdge3(planeNormal, planeD, p1, p2, motion, v1, tri[j], tri[k], dist, ip, closestI, closestJ, coeff))
				{
					if (dist <= minDist)
					{
						p1s = p1;
						v1s = v1;
						savedJ = j;
						savedK = k;
						col = 2;
						minDist = dist;
						hit = ip + motion * dist;
					}
				}
			}
		}
	}

	if (col == -1)
	{
		return false;
	}

	if (col == 2)
	{
		const Vector3& p3 = tri[savedJ];
		const Vector3& p4 = tri[savedK];
		computeEdgeEdgeNormal(normal, p1s, v1s, p3, p4 - p3, motion, minDist);
	}

	d = minDist;
	return true;
}

static void computeBoxTriImpactData(Vector3& hit, Vector3& normal, const Vector3& boxExtents, const Vector3& localDir, const Vector3 triInBoxSpace[3], float impactDist)
{
	const Vector3 boxMin = -boxExtents;
	const Vector3 boxMax = boxExtents;
	const Vector3 oneOverDir(
		localDir.x != 0.0f ? 1.0f / localDir.x : 0.0f,
		localDir.y != 0.0f ? 1.0f / localDir.y : 0.0f,
		localDir.z != 0.0f ? 1.0f / localDir.z : 0.0f);

	float featureDistance = FLT_MAX;
	if (sweepBoxTriangleFeatureBased(triInBoxSpace, boxMin, boxMax, localDir, oneOverDir, hit, normal, featureDistance, true))
	{
		if (normal.SafeNormalize() > 1.0e-6f)
		{
			if (normal.Dot(localDir) > 0.0f)
			{
				normal = -normal;
			}
			return;
		}
	}

	const Vector3 movedCenter = localDir * impactDist;
	hit = Triangle3::ClosestPointOnTriangleToPoint(movedCenter, triInBoxSpace[0], triInBoxSpace[1], triInBoxSpace[2]);
	normal = movedCenter - hit;
	if (normal.SafeNormalize() <= 1.0e-6f)
	{
		normal = -safeUnitOrZero(localDir);
	}
	else if (normal.Dot(localDir) > 0.0f)
	{
		normal = -normal;
	}
}

static bool sweepAABBToTriangleTOI(const Vector3& boxMin, const Vector3& boxMax, const Vector3& direction, const Vector3& A, const Vector3& B, const Vector3& C, float maxDist, float& toi)
{
	if (direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	const Vector3 center = (boxMin + boxMax) * 0.5f;
	const Vector3 extents = (boxMax - boxMin) * 0.5f;
	Vector3 tri[3] = { A - center, B - center, C - center };
	return triBoxSweepTestBoxSpace(tri, extents, direction, maxDist, toi, false);
}

static bool computeAABBTriangleSweepResult(const Vector3& boxMin, const Vector3& boxMax, const Vector3& direction, const Vector3& A, const Vector3& B, const Vector3& C, float toi, Vector3* p, Vector3* n, float* t)
{
	if (t)
	{
		*t = toi;
	}

	if (toi == 0.0f)
	{
		if (p)
		{
			const Vector3 center = (boxMin + boxMax) * 0.5f;
			*p = Triangle3::ClosestPointOnTriangleToPoint(center, A, B, C);
		}
		if (n)
		{
			*n = -safeUnitOrZero(direction);
		}
		return true;
	}

	if (p || n)
	{
		const Vector3 center = (boxMin + boxMax) * 0.5f;
		const Vector3 extents = (boxMax - boxMin) * 0.5f;
		const Vector3 tri[3] = { A - center, B - center, C - center };
		Vector3 localHit, localNormal;
		computeBoxTriImpactData(localHit, localNormal, extents, direction, tri, toi);
		if (p)
		{
			*p = localHit + center;
		}
		if (n)
		{
			*n = localNormal;
		}
	}
	return true;
}

static bool sweepMovingAABBAABBInterval(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& enter, float& exit)
{
	Box3 movingBox(movingMin, movingMax);
	if (movingBox.Intersect(staticMin, staticMax))
	{
		enter = 0.0f;
		exit = maxDist;
		return true;
	}

	const Vector3 center = (movingMin + movingMax) * 0.5f;
	const Vector3 extents = (movingMax - movingMin) * 0.5f;
	const Vector3 expandedMin = staticMin - extents;
	const Vector3 expandedMax = staticMax + extents;

	enter = 0.0f;
	exit = maxDist;
	for (int i = 0; i < 3; ++i)
	{
		if (fabsf(direction[i]) < 1.0e-8f)
		{
			if (center[i] < expandedMin[i] || center[i] > expandedMax[i])
			{
				return false;
			}
			continue;
		}

		const float invDir = 1.0f / direction[i];
		float t0 = (expandedMin[i] - center[i]) * invDir;
		float t1 = (expandedMax[i] - center[i]) * invDir;
		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		enter = std::max(enter, t0);
		exit = std::min(exit, t1);
		if (enter > exit)
		{
			return false;
		}
	}

	if (exit < 0.0f || enter > maxDist)
	{
		return false;
	}

	enter = std::max(enter, 0.0f);
	return true;
}

static bool sweepMovingAABBAABB(const Vector3& movingMin, const Vector3& movingMax, const Vector3& direction, const Vector3& staticMin, const Vector3& staticMax, float maxDist, float& toi)
{
	float enter, exit;
	if (!sweepMovingAABBAABBInterval(movingMin, movingMax, direction, staticMin, staticMax, maxDist, enter, exit))
	{
		return false;
	}
	toi = std::max(enter, 0.0f);
	return true;
}

static void testTriangleForBestBoxSweep(const Vector3& boxMin, const Vector3& boxMax, const Vector3& direction, const Vector3& unitDir, const Vector3& A, const Vector3& B, const Vector3& C, float& bestDistance, float& bestAlignment, Vector3 bestTri[3], bool& hit)
{
	float currentDistance;
	if (!sweepAABBToTriangleTOI(boxMin, boxMax, direction, A, B, C, bestDistance, currentDistance))
	{
		return;
	}

	Vector3 triNormal = (B - A).Cross(C - A);
	if (triNormal.SafeNormalize() <= 1.0e-6f)
	{
		triNormal = -unitDir;
	}

	const float alignment = Triangle3::ComputeAlignmentValue(triNormal, unitDir);
	if (!hit || Triangle3::IsBetterTriangle(currentDistance, alignment, bestDistance, bestAlignment))
	{
		bestDistance = currentDistance;
		bestAlignment = alignment;
		bestTri[0] = A;
		bestTri[1] = B;
		bestTri[2] = C;
		hit = true;
	}
}

static bool sweepTriangleMeshBruteForce(const AxisAlignedBox3& box, const Vector3& direction, const TriangleMesh* trimesh, Vector3 bestTri[3], float& bestDistance)
{
	bool hit = false;
	float bestAlignment = 2.0f;
	const Vector3 unitDir = safeUnitOrZero(direction);

	for (uint32_t i = 0; i < trimesh->NumTriangles; ++i)
	{
		const Vector3& A = trimesh->GetVertex(i, 0);
		const Vector3& B = trimesh->GetVertex(i, 1);
		const Vector3& C = trimesh->GetVertex(i, 2);
		testTriangleForBestBoxSweep(box.Min, box.Max, direction, unitDir, A, B, C, bestDistance, bestAlignment, bestTri, hit);
	}

	return hit;
}

static bool sweepTriangleMeshBVH(const AxisAlignedBox3& box, const Vector3& direction, const TriangleMesh* trimesh, const MeshBVH4* bvh, Vector3 bestTri[3], float& bestDistance)
{
	if (bvh == nullptr || bvh->BatchPtr == nullptr || bvh->NumRoots == 0)
	{
		return sweepTriangleMeshBruteForce(box, direction, trimesh, bestTri, bestDistance);
	}

	bool hit = false;
	float bestAlignment = 2.0f;
	const Vector3 unitDir = safeUnitOrZero(direction);

	const uint32_t maxStack = 128;
	uint32_t stack[maxStack];
	uint32_t stackPtr = 0;
	for (int j = int(bvh->NumRoots) - 1; j >= 0; --j)
	{
		stack[stackPtr++] = uint32_t(j) * sizeof(BVHNodeBatch);
	}

	const uint8_t* batchPtr = reinterpret_cast<const uint8_t*>(bvh->BatchPtr);
	while (stackPtr > 0)
	{
		uint32_t top = stack[--stackPtr];
		if (top & 1)
		{
			LeafNode leaf(top & ~1u);
			const uint32_t numLeafTriangles = leaf.GetNumTriangles();
			const uint32_t baseTriIndex = leaf.GetTriangleIndex();
			for (uint32_t i = 0; i < numLeafTriangles; ++i)
			{
				const uint32_t triIndex = baseTriIndex + i;
				const Vector3& A = trimesh->GetVertex(triIndex, 0);
				const Vector3& B = trimesh->GetVertex(triIndex, 1);
				const Vector3& C = trimesh->GetVertex(triIndex, 2);
				testTriangleForBestBoxSweep(box.Min, box.Max, direction, unitDir, A, B, C, bestDistance, bestAlignment, bestTri, hit);
			}
			continue;
		}

		const BVHNodeBatch* node = reinterpret_cast<const BVHNodeBatch*>(batchPtr + top);
		for (uint32_t i = 0; i < SIMD_WIDTH; ++i)
		{
			if (node->minx[i] > node->maxx[i])
			{
				continue;
			}

			float nodeToi;
			if (!sweepMovingAABBAABB(box.Min, box.Max, direction,
				Vector3(node->minx[i], node->miny[i], node->minz[i]),
				Vector3(node->maxx[i], node->maxy[i], node->maxz[i]), bestDistance, nodeToi))
			{
				continue;
			}

			if (stackPtr < maxStack)
			{
				stack[stackPtr++] = node->Data[i];
			}
			else
			{
				return sweepTriangleMeshBruteForce(box, direction, trimesh, bestTri, bestDistance);
			}
		}
	}

	return hit;
}

bool AxisAlignedBox3::SweepAABB(const Vector3& Direction, const Vector3& bmin, const Vector3& bmax, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectAABB(bmin, bmax))
	{
		if (p)
		{
			*p = GetSupport(Direction);
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	AxisAlignedBox3 box(bmin, bmax);
	GJKShapecast gjk;
    return gjk.Solve(Direction, this, &box, p, n, t);
}

bool AxisAlignedBox3::SweepSphere(const Vector3& Direction, const Vector3& rCenter, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectSphere(rCenter, rRadius))
	{
		if (p)
		{
			*p = GetSupport(Direction);
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	Sphere3 sp(rCenter, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &sp, p, n, t);
}

bool AxisAlignedBox3::SweepPlane(const Vector3& Direction, const Vector3& Normal, float D, Vector3* p, Vector3* n, float* t) const
{
	Plane3 plane(Normal, D);

    const Vector3 Origin = GetCenter();
	const float dp = Direction.Dot(Normal);
	if (fabsf(dp) < 1e-6f)
	{
		if (plane.IntersectAABB(Min, Max))
		{
			*n = -Direction;
			*t = 0.0f;
			return true;
		}
	}

	const Vector3 RelativeOrigin = GetSupport(Direction);
	if (plane.IntersectRay(RelativeOrigin, Direction, t))
	{
		*n = dp < 0.0f ? Normal : -Normal;
		return true;
	}
	return false;
}

bool AxisAlignedBox3::SweepCylinder(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
    Cylinder3 cylinder(X0, X1, rRadius);
	GJKIntersection gjkIntersect;
	if (gjkIntersect.Solve(this, &cylinder) == GJK_status::Intersect)
	{
		if (p)
		{
			*p = GetSupport(Direction);
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

    GJKShapecast gjk;
    return gjk.Solve(Direction, this, &cylinder, p, n, t);
}

bool AxisAlignedBox3::SweepCapsule(const Vector3& Direction, const Vector3& X0, const Vector3& X1, float rRadius, Vector3* p, Vector3* n, float* t) const
{
	if (IntersectCapsule(X0, X1, rRadius))
	{
		if (p)
		{
			*p = GetSupport(Direction);
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	Capsule3 capsule(X0, X1, rRadius);
	GJKShapecast gjk;
	return gjk.Solve(Direction, this, &capsule, p, n, t);
}

bool AxisAlignedBox3::SweepConvex(const Vector3& Direction, const ConvexMesh* convex, Vector3* p, Vector3* n, float* t) const
{
	if (convex && convex->IntersectAABB(Min, Max))
	{
		if (p)
		{
			*p = GetSupport(Direction);
		}
		if (n)
		{
			*n = -Direction;
			n->SafeNormalize();
		}
		if (t)
		{
			*t = 0.0f;
		}
		return true;
	}

	GJKShapecast gjk;
	return gjk.Solve(Direction, this, convex, p, n, t);
}

bool AxisAlignedBox3::SweepTriangle(const Vector3& Direction, const Vector3 &A, const Vector3 &B, const Vector3 &C, Vector3* p, Vector3* n, float* t) const
{
	float toi;
	if (!sweepAABBToTriangleTOI(Min, Max, Direction, A, B, C, FLT_MAX, toi))
	{
		return false;
	}

	return computeAABBTriangleSweepResult(Min, Max, Direction, A, B, C, toi, p, n, t);
}

bool AxisAlignedBox3::SweepHeightField(const Vector3& Direction, const HeightField3* hf, Vector3* p, Vector3* n, float* t) const
{
	if (hf == nullptr || hf->Cells == nullptr || hf->nX < 2 || hf->nZ < 2 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	float hfEnter, hfExit;
	if (!sweepMovingAABBAABBInterval(Min, Max, Direction, hf->BV.Min, hf->BV.Max, FLT_MAX, hfEnter, hfExit))
	{
		return false;
	}

	Box3 overlap;
	if (hfExit == FLT_MAX)
	{
		overlap = hf->BV;
	}
	else
	{
		Box3 sweptBox(Min + Direction * hfEnter, Max + Direction * hfEnter);
		sweptBox.Encapsulate(Min + Direction * hfExit, Max + Direction * hfExit);
		if (!sweptBox.GetIntersection(hf->BV, overlap))
		{
			return false;
		}
	}

	const int i0 = std::max(0, std::min((int)hf->nX - 2, (int)((overlap.Min.x - hf->BV.Min.x) * hf->InvDX)));
	const int j0 = std::max(0, std::min((int)hf->nZ - 2, (int)((overlap.Min.z - hf->BV.Min.z) * hf->InvDZ)));
	const int i1 = std::max(0, std::min((int)hf->nX - 2, (int)((overlap.Max.x - hf->BV.Min.x) * hf->InvDX)));
	const int j1 = std::max(0, std::min((int)hf->nZ - 2, (int)((overlap.Max.z - hf->BV.Min.z) * hf->InvDZ)));

	bool hit = false;
	float bestDistance = FLT_MAX;
	float bestAlignment = 2.0f;
	Vector3 bestTri[3];
	const Vector3 unitDir = safeUnitOrZero(Direction);

	for (int i = i0; i <= i1; ++i)
	{
		for (int j = j0; j <= j1; ++j)
		{
			Box3 cellBox;
			if (!hf->GetCellBV(i, j, cellBox))
			{
				continue;
			}

			float cellToi;
			if (!sweepMovingAABBAABB(Min, Max, Direction, cellBox.Min, cellBox.Max, bestDistance, cellToi))
			{
				continue;
			}

			Vector3 tris[6];
			const int numTriVerts = hf->GetCellTriangle(i, j, tris);
			for (int k = 0; k < numTriVerts; k += 3)
			{
				testTriangleForBestBoxSweep(Min, Max, Direction, unitDir, tris[k], tris[k + 1], tris[k + 2], bestDistance, bestAlignment, bestTri, hit);
			}
		}
	}

	if (!hit)
	{
		return false;
	}

	return computeAABBTriangleSweepResult(Min, Max, Direction, bestTri[0], bestTri[1], bestTri[2], bestDistance, p, n, t);
}

bool AxisAlignedBox3::SweepTriangleMesh(const Vector3& Direction, const TriangleMesh* trimesh, Vector3* p, Vector3* n, float* t) const
{
	if (trimesh == nullptr || trimesh->NumTriangles == 0 || Direction.SquareLength() <= 1.0e-12f)
	{
		return false;
	}

	float meshToi;
	if (!sweepMovingAABBAABB(Min, Max, Direction, trimesh->BoundingVolume.Min, trimesh->BoundingVolume.Max, FLT_MAX, meshToi))
	{
		return false;
	}

	Vector3 bestTri[3];
	float bestDistance = FLT_MAX;
	const bool hit = sweepTriangleMeshBVH(*this, Direction, trimesh, trimesh->GetBVH(), bestTri, bestDistance);
	if (!hit)
	{
		return false;
	}

	return computeAABBTriangleSweepResult(Min, Max, Direction, bestTri[0], bestTri[1], bestTri[2], bestDistance, p, n, t);
}

Vector3 AxisAlignedBox3::ClosestPointToPoint(const Vector3& Point) const
{
	Vector3 closest = Point;

	for (int i = 0; i < 3; i++)
	{
		if (closest[i] < Min[i])
		{
			closest[i] = Min[i];
		}
		else if (closest[i] > Max[i])
		{
			closest[i] = Max[i];
		}
	}
	return closest;
}

float AxisAlignedBox3::SqrDistanceToPoint(const Vector3& Point) const
{
	Vector3 closest = ClosestPointToPoint(Point);
	return (closest - Point).SquareLength();
}

void AxisAlignedBox3::GetMesh2(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices.resize(8);
	Box3::GetVertices(Min, Max, &Vertices[0]);

	Vector3 Center = GetCenterOfMass();

	Normals.resize(8);
	for (int i = 0; i < 8; ++i)
	{
		Normals[i] = Vertices[i] - Center;
		Normals[i].Normalize();
	}

	Indices = std::vector<uint16_t>({
		0, 1, 2,
		1, 3, 2,
		4, 6, 5,
		5, 6, 7,
		0, 4, 1,
		5, 1, 4,
		1, 5, 3,
		7, 3, 5,
		2, 4, 0,
		6, 4, 2,
		3, 6, 2,
		6, 3, 7 });

	return;
}

void AxisAlignedBox3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices.clear();
	Indices.clear();
	Normals.clear();
	Vertices.reserve(36);
	Indices.reserve(36);
	Normals.reserve(36);

	const Vector3 v000(Min.x, Min.y, Min.z);
	const Vector3 v100(Max.x, Min.y, Min.z);
	const Vector3 v010(Min.x, Max.y, Min.z);
	const Vector3 v110(Max.x, Max.y, Min.z);
	const Vector3 v001(Min.x, Min.y, Max.z);
	const Vector3 v101(Max.x, Min.y, Max.z);
	const Vector3 v011(Min.x, Max.y, Max.z);
	const Vector3 v111(Max.x, Max.y, Max.z);

	auto addTriangle = [&](const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& normal)
	{
		const uint16_t base = (uint16_t)Vertices.size();
		Vertices.push_back(a);
		Vertices.push_back(b);
		Vertices.push_back(c);
		Normals.push_back(normal);
		Normals.push_back(normal);
		Normals.push_back(normal);
		Indices.push_back(base);
		Indices.push_back((uint16_t)(base + 1));
		Indices.push_back((uint16_t)(base + 2));
	};

	auto addFace = [&](const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& normal)
	{
		addTriangle(v0, v1, v2, normal);
		addTriangle(v2, v1, v3, normal);
	};

	addFace(v000, v010, v100, v110, -Vector3::UnitZ());
	addFace(v001, v101, v011, v111, Vector3::UnitZ());
	addFace(v000, v100, v001, v101, -Vector3::UnitY());
	addFace(v010, v011, v110, v111, Vector3::UnitY());
	addFace(v000, v001, v010, v011, -Vector3::UnitX());
	addFace(v100, v110, v101, v111, Vector3::UnitX());
}

void AxisAlignedBox3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	Vertices.resize(8);
	Box3::GetVertices(Min, Max, &Vertices[0]);
	Indices = std::vector<uint16_t>({
		0, 1, 1, 3, 3, 2, 2, 0,
		0, 4, 1, 5, 3, 7, 2, 6,
		4, 5, 5, 7, 7, 6, 6, 4 });
}

}


#include "AxisAlignedBox3d.h"


static void face(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, const Vector3d& rkPmE, float* t, float& rfSqrDistance)
{
	Vector3d kPpE;
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

static void caseNoZeros(Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* t, float& rfSqrDistance)
{
	Vector3d kPmE(rkPnt.x - extents.x, rkPnt.y - extents.y, rkPnt.z - extents.z);

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

static void case0(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* t, float& rfSqrDistance)
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

static void case00(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* t, float& rfSqrDistance)
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

static void case000(Vector3d& rkPnt, const Vector3d& extents, float& rfSqrDistance)
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

float AxisAlignedBox3d::SqrDistanceToLine(const Vector3d& P0, const Vector3d& tDir, float* t) const
{
	Vector3d Center = GetCenter() - P0;
	Vector3d Extent = (Max - Min) * 0.5f;
	Vector3d Dir = tDir;

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

float AxisAlignedBox3d::SqrDistanceToSegment(const Vector3d& P0, const Vector3d& P1) const
{
	float t;
	float sqrDistance = SqrDistanceToLine(P0, P1 - P0, &t);
	if (t >= 0.0f)
	{
		if (t <= 1.0f)
		{
			return sqrDistance;
		}
		else
		{
			return SqrDistanceToPoint(P1);
		}
	}
	else
	{
		return SqrDistanceToPoint(P0);
	}
}

Vector3d AxisAlignedBox3d::ClosestPointTo(const Vector3d& Point) const
{
	Vector3d closest = GetCenter();

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

float AxisAlignedBox3d::SqrDistanceToPoint(const Vector3d& Point) const
{
	Vector3d closest = ClosestPointTo(Point);
	return (closest - Point).SquareLength();
}


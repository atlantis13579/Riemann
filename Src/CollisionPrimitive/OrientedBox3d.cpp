
#include "OrientedBox3d.h"


static void face(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, const Vector3d& rkPmE, float* pfLParam, float& rfSqrDistance)
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
			if (pfLParam)
			{
				rkPnt[i0] = extents[i0];
				fInv = 1.0f / rkDir[i0];
				rkPnt[i1] -= rkDir[i1] * rkPmE[i0] * fInv;
				rkPnt[i2] -= rkDir[i2] * rkPmE[i0] * fInv;
				*pfLParam = -rkPmE[i0] * fInv;
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

				if (pfLParam)
				{
					*pfLParam = fParam;
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

				if (pfLParam)
				{
					*pfLParam = fParam;
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

				if (pfLParam)
				{
					*pfLParam = fParam;
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

				if (pfLParam)
				{
					*pfLParam = fParam;
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

					if (pfLParam)
					{
						*pfLParam = fParam;
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

					if (pfLParam)
					{
						*pfLParam = fParam;
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

					if (pfLParam)
					{
						*pfLParam = fParam;
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

					if (pfLParam)
					{
						*pfLParam = fParam;
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

			if (pfLParam)
			{
				*pfLParam = fParam;
				rkPnt[i0] = extents[i0];
				rkPnt[i1] = -extents[i1];
				rkPnt[i2] = -extents[i2];
			}
		}
	}
}

static void caseNoZeros(Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* pfLParam, float& rfSqrDistance)
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
			face(0, 1, 2, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
	else
	{
		fProdDzPy = rkDir.z * kPmE.y;
		fProdDyPz = rkDir.y * kPmE.z;
		if (fProdDzPy >= fProdDyPz)
		{
			// line intersects y = e1
			face(1, 2, 0, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
}

static void case0(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* pfLParam, float& rfSqrDistance)
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
			if (pfLParam)
			{
				rkPnt[i1] = -extents[i1];
				*pfLParam = -(rkDir[i0] * fPmE0 + rkDir[i1] * fPpE1) * fInvLSqr;
			}
		}
		else
		{
			if (pfLParam)
			{
				fInv = 1.0f / rkDir[i0];
				rkPnt[i1] -= fProd0 * fInv;
				*pfLParam = -fPmE0 * fInv;
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
			if (pfLParam)
			{
				rkPnt[i0] = -extents[i0];
				*pfLParam = -(rkDir[i0] * fPpE0 + rkDir[i1] * fPmE1) * fInvLSqr;
			}
		}
		else
		{
			if (pfLParam)
			{
				fInv = 1.0f / rkDir[i1];
				rkPnt[i0] -= fProd1 * fInv;
				*pfLParam = -fPmE1 * fInv;
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

static void case00(unsigned int i0, unsigned int i1, unsigned int i2, Vector3d& rkPnt, const Vector3d& rkDir, const Vector3d& extents, float* pfLParam, float& rfSqrDistance)
{
	float fDelta;

	if (pfLParam)
		*pfLParam = (extents[i0] - rkPnt[i0]) / rkDir[i0];

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

float OrientedBox3d::SqrDistanceToLine(const Vector3d& lineOrigin, const Vector3d& lineDirection, float* lineParam, Vector3d* boxParam) const
{
	const Vector3d& axis0 = Rot.Column(0);
	const Vector3d& axis1 = Rot.Column(1);
	const Vector3d& axis2 = Rot.Column(2);

	const Vector3d diff = lineOrigin - Center;
	Vector3d pnt(diff.Dot(axis0), diff.Dot(axis1), diff.Dot(axis2));
	Vector3d dir(lineDirection.Dot(axis0), lineDirection.Dot(axis1), lineDirection.Dot(axis2));

	bool reflect[3];
	for (int i = 0; i < 3; i++)
	{
		if (dir[i] < 0.0f)
		{
			pnt[i] = -pnt[i];
			dir[i] = -dir[i];
			reflect[i] = true;
		}
		else
		{
			reflect[i] = false;
		}
	}

	float sqrDistance = 0.0f;

	if (dir.x > 0.0f)
	{
		if (dir.y > 0.0f)
		{
			if (dir.z > 0.0f)
			{
				caseNoZeros(pnt, dir, Extent, lineParam, sqrDistance);		// (+,+,+)
			}
			else
			{
				case0(0, 1, 2, pnt, dir, Extent, lineParam, sqrDistance);	// (+,+,0)
			}
		}
		else
		{
			if (dir.z > 0.0f)
			{
				case0(0, 2, 1, pnt, dir, Extent, lineParam, sqrDistance);	// (+,0,+)
			}
			else
			{
				case00(0, 1, 2, pnt, dir, Extent, lineParam, sqrDistance);	// (+,0,0)
			}
		}
	}
	else
	{
		if (dir.y > 0.0f)
		{
			if (dir.z > 0.0f)
			{
				case0(1, 2, 0, pnt, dir, Extent, lineParam, sqrDistance);	// (0,+,+)
			}
			else
			{
				case00(1, 0, 2, pnt, dir, Extent, lineParam, sqrDistance);	// (0,+,0)
			}
		}
		else
		{
			if (dir.z > 0.0f)
			{
				case00(2, 0, 1, pnt, dir, Extent, lineParam, sqrDistance);	// (0,0,+)
			}
			else
			{
				case000(pnt, Extent, sqrDistance);										// (0,0,0)
				if (lineParam)
					*lineParam = 0.0f;
			}
		}
	}

	if (boxParam)
	{
		for (int i = 0; i < 3; i++)
		{
			if (reflect[i])
				pnt[i] = -pnt[i];
		}

		*boxParam = pnt;
	}

	return sqrDistance;
}

float OrientedBox3d::SqrDistanceToSegment(const Vector3d& P0, const Vector3d& P1) const
{

	float lp;
	Vector3d bp;
	float sqrDistance = SqrDistanceToLine(P0, P1 - P0, &lp, &bp);
	if (lp >= 0.0f)
	{
		if (lp <= 1.0f)
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

float OrientedBox3d::SqrDistanceToPoint(const Vector3d& Point) const
{
	const Vector3d diff = Point - Center;

	Vector3d closest(Rot.Column(0).Dot(diff), Rot.Column(1).Dot(diff), Rot.Column(2).Dot(diff));

	float sqrDistance = 0.0f;
	for (int ax = 0; ax < 3; ax++)
	{
		if (closest[ax] < -Extent[ax])
		{
			const float delta = closest[ax] + Extent[ax];
			sqrDistance += delta * delta;
			closest[ax] = -Extent[ax];
		}
		else if (closest[ax] > Extent[ax])
		{
			const float delta = closest[ax] - Extent[ax];
			sqrDistance += delta * delta;
			closest[ax] = Extent[ax];
		}
	}

	return sqrDistance;
}
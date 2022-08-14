
#include "AxisAlignedBox3d.h"
#include "Plane3d.h"

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

float AxisAlignedBox3d::SqrDistanceToLine(const Vector3& P0, const Vector3& tDir, float* t) const
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

float AxisAlignedBox3d::SqrDistanceToSegment(const Vector3& P0, const Vector3& P1) const
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

Vector3 AxisAlignedBox3d::GetSupport(const Vector3& Direction) const
{
	return GetSupport(Min, Max, Direction);
}

// static
Vector3 AxisAlignedBox3d::GetSupport(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction)
{
	return Vector3(
		Direction.x > 0 ? Bmax.x : Bmin.x,
		Direction.y > 0 ? Bmax.y : Bmin.y,
		Direction.z > 0 ? Bmax.z : Bmin.z
	);
}

int	 AxisAlignedBox3d::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
{
	return AxisAlignedBox3d::GetSupportFace(Min, Max, Direction, FacePoints);
}

int AxisAlignedBox3d::GetSupportFace(const Vector3& Bmin, const Vector3& Bmax, const Vector3& Direction, Vector3* FacePoints)
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

void AxisAlignedBox3d::GetMesh2(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices.resize(8);
	Box3d::GetVertices(Min, Max, &Vertices[0]);

	Vector3 Center = GetCenterOfMass();

	Normals.resize(8);
	for (int i = 0; i < 8; ++i)
	{
		Normals[i] = Vertices[i] - Center;
	}

	Indices = std::vector<uint16_t>({
		0, 1, 2,
		1, 3, 2,
		4, 5, 6,
		5, 7, 6,
		0, 1, 4,
		5, 4, 1,
		1, 3, 5,
		7, 5, 3,
		2, 4, 0,
		6, 4, 2,
		3, 2, 6,
		6, 7, 3 });
}

void AxisAlignedBox3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
{
	Vertices.resize(36);
	Indices.resize(36);
	Normals.resize(36);

	Vector3 BV[] = { Min, Max };

#define SET_VERTICES(_idx, _x, _y, _z)	\
			Vertices[_idx] = Vector3(BV[_x].x, BV[_y].y, BV[_z].z);	\
			Indices[_idx] = (_idx);	\
			Normals[_idx] = (_z == 0) ? -Vector3::UnitZ() : Vector3::UnitZ();	\
			Vertices[_idx + 12] = Vector3(BV[_y].x, BV[_z].y, BV[_x].z);	\
			Indices[_idx + 12] = (_idx + 12);	\
			Normals[_idx + 12] = (_z == 0) ? -Vector3::UnitY() : Vector3::UnitY();	\
			Vertices[_idx + 24] = Vector3(BV[_z].x, BV[_x].y, BV[_y].z);	\
			Indices[_idx + 24] = (_idx + 24);	\
			Normals[_idx + 24] = (_z == 0) ? -Vector3::UnitX() : Vector3::UnitX();	\

	SET_VERTICES(0, 0, 0, 0);
	SET_VERTICES(1, 1, 0, 0);
	SET_VERTICES(2, 0, 1, 0);
	SET_VERTICES(3, 1, 0, 0);
	SET_VERTICES(4, 0, 1, 0);
	SET_VERTICES(5, 1, 1, 0);
	SET_VERTICES(6, 0, 0, 1);
	SET_VERTICES(7, 1, 0, 1);
	SET_VERTICES(8, 0, 1, 1);
	SET_VERTICES(9, 1, 0, 1);
	SET_VERTICES(10, 0, 1, 1);
	SET_VERTICES(11, 1, 1, 1);

#undef SET_VERTICES
}

void AxisAlignedBox3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
{
	Vertices.resize(8);
	Box3d::GetVertices(Min, Max, &Vertices[0]);
	Indices = std::vector<uint16_t>({
		0, 1, 1, 3, 3, 2, 2, 0,
		0, 4, 1, 5, 3, 7, 2, 6,
		4, 5, 5, 7, 7, 6, 6, 4 });
}

bool AxisAlignedBox3d::IntersectPoint(const Vector3& Point) const
{
	if (Point.x >= Min.x && Point.x <= Max.x &&
		Point.y >= Min.y && Point.y <= Max.y &&
		Point.z >= Min.z && Point.z <= Max.z)
	{
		return true;
	}
	return false;
}

bool AxisAlignedBox3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
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

bool AxisAlignedBox3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
{
	const Vector3 b0 = Min - Origin;
	const Vector3 b1 = Max - Origin;

	float tMin = 0;
	float tMax = FLT_MAX;
	Vector3 Normal(0.0f);

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

		Vector3 CurNormal = Vector3(0.0f);
		CurNormal[i] = 1.0f;

		if (t0 > t1)
		{
			std::swap(t0, t1);
		}
		else
		{
			CurNormal[i] = -1.0f;
		}

		if (t0 > tMin)
		{
			Normal = CurNormal;
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

bool AxisAlignedBox3d::IntersectSegment(const Vector3& p0, const Vector3& p1) const
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

bool AxisAlignedBox3d::IntersectSphere(const Vector3& Center, float Radius) const
{
	float SqrDist = SqrDistanceToPoint(Center);
	return SqrDist <= Radius * Radius;
}

bool AxisAlignedBox3d::IntersectCapsule(const Vector3& P0, const Vector3& P1, float Radius) const
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
bool AxisAlignedBox3d::IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const
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
	Plane3d plane(normal, D);
	if (!plane.IntersectAABB(-extents, extents))
		return false;

	return true;
}

Vector3 AxisAlignedBox3d::ClosestPointTo(const Vector3& Point) const
{
	Vector3 closest = GetCenter();

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

float AxisAlignedBox3d::SqrDistanceToPoint(const Vector3& Point) const
{
	Vector3 closest = ClosestPointTo(Point);
	return (closest - Point).SquareLength();
}


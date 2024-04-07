
#include "HeightField3d.h"
#include "Capsule3d.h"
#include "OrientedBox3d.h"
#include "Triangle3d.h"
#include "Ray3d.h"
#include "Sphere3d.h"
#include "../Maths/Maths.h"

namespace Riemann
{
	#define X_INDEX(_x)		((int)((_x - BV.Min.x) * InvDX))
	#define Z_INDEX(_z)		((int)((_z - BV.Min.z) * InvDZ))

	static const float HfCellThickness = 0.0001f;

	bool HeightField3d::IntersectRayCell(const Vector3& Origin, const Vector3& Direction, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		float minH, maxH;
		GetHeightRange(i, j, minH, maxH);

		Result->AddTestCount(1);

		if (fabsf(Direction.y) < 0.000001f)
		{
			if (Origin.y < minH || Origin.y > maxH)
			{
				return false;
			}
		}

		{
			Vector3 Bmin(BV.Min.x + i * DX, minH, BV.Min.z + j * DZ);
			Vector3 Bmax(BV.Min.x + (i + 1) * DX, maxH, BV.Min.z + (j + 1) * DZ);
			float t0, t1;
			if (!Ray3d::RayIntersectAABB2(Origin, Direction, Bmin, Bmax, HfCellThickness, Option.maxDist, &t0, &t1))
			{
				return false;
			}
		}

		bool hit = false;
		float min_t = FLT_MAX;

		Vector3 Tris[6];
		int nT = GetCellTriangle(i, j, Tris);
		for (int k = 0; k < nT; k += 3)
		{
			float tt;
			if (Triangle3d::RayIntersectTriangle(Origin, Direction, Tris[k], Tris[k + 1], Tris[k + 2], &tt) && tt < min_t)
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

	bool HeightField3d::IntersectRayY(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		const int i = X_INDEX(Origin.x);
		const int j = Z_INDEX(Origin.z);
		if (i < 0 || i >= (int)nX - 1 || j < 0 || j >= (int)nZ - 1)
		{
			return false;
		}

		bool hit = false;
		float min_t = FLT_MAX;

		Vector3 Tris[6];
		int nT = GetCellTriangle(i, j, Tris);
		for (int k = 0; k < nT; k += 3)
		{
			float tt;
			if (Triangle3d::RayIntersectTriangle(Origin, Direction, Tris[k], Tris[k + 1], Tris[k + 2], &tt) && tt < min_t)
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

	bool HeightField3d::IntersectRayBruteForce(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		float min_dist = FLT_MAX;
		for (uint32_t i = 0; i < nX - 1; ++i)
			for (uint32_t j = 0; j < nZ - 1; ++j)
			{
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
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

	bool HeightField3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		if (Origin.y > BV.Max.y && Direction.y >= 0)
		{
			return false;
		}
		else if (Origin.y < BV.Min.y && Direction.y <= 0)
		{
			return false;
		}

		if (Direction.ParallelTo(Vector3::UnitY()))
		{
			// Handle the most common case
			return IntersectRayY(Origin, Direction, Option, Result);
		}

		Result->AddTestCount(1);

		float t0, t1;
		if (!Ray3d::RayIntersectAABB2(Origin, Direction, BV.Min, BV.Max, HfCellThickness, Option.maxDist, &t0, &t1))
		{
			return false;
		}

		Vector3 P0 = Origin + Direction * t0;
		Vector3 P1 = Origin + Direction * t1;

		const int istart = X_INDEX(P0.x);
		const int jstart = Z_INDEX(P0.z);
		const int iend = X_INDEX(P1.x);
		const int jend = Z_INDEX(P1.z);
		const int di = Direction.x > 0 ? 1 : (Direction.x < 0 ? -1 : 0);
		const int dj = Direction.z > 0 ? 1 : (Direction.z < 0 ? -1 : 0);

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
			return IntersectRayCell(Origin, Direction, istart, jstart, Option, Result);
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
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
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
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
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
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
				{
					return true;
				}
				i += di;
			}
		}

		return IntersectRayCell(Origin, Direction, iend, jend, Option, Result);
	}


	bool HeightField3d::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		HeightFieldHitOption Option;
		Option.maxDist = FLT_MAX;

		HeightFieldHitResult Result = { 0 };
		if (IntersectRay(Origin, Direction, Option, &Result))
		{
			*t = Result.hitTime;
			return true;
		}
		return false;
	}

	bool HeightField3d::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		Box3 Intersect;
		if (!BV.GetIntersection(Box3(Bmin, Bmax), Intersect))
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
				float H = GetHeight(i * nZ + j);
				if (minH <= H && H <= maxH)
				{
					return true;
				}
			}

		return true;
	}

	template<class Shape>
	bool IntersectHF(const HeightField3d* hf, const Shape& shape)
	{
		const Box3 b = shape.CalculateBoundingVolume();
		const Box3& BV = hf->BV;
		const float InvDX = hf->InvDX;
		const float InvDZ = hf->InvDZ;

		Box3 Intersect;
		if (!BV.GetIntersection(b, Intersect))
		{
			return false;
		}

		const int i0 = X_INDEX(Intersect.Min.x);
		const int j0 = Z_INDEX(Intersect.Min.z);
		const int i1 = X_INDEX(Intersect.Max.x);
		const int j1 = Z_INDEX(Intersect.Max.z);
		for (int i = i0; i <= i1; ++i)
			for (int j = j0; j <= j1; ++j)
			{
				Vector3 Tris[6];
				int n = hf->GetCellTriangle(i, j, Tris);
				for (int k = 0; k < n; k += 3)
				{
					if (shape.IntersectTriangle(Tris[k], Tris[k + 1], Tris[k + 2]))
					{
						return true;
					}
				}
			}

		return false;
	}

	bool HeightField3d::IntersectOBB(const Vector3& Center, const Vector3& Extent, const Matrix3& rot) const
	{
		OrientedBox3d obb(Center, Extent, rot);
		return IntersectHF(this, obb);
	}

	bool HeightField3d::IntersectSphere(const Vector3& Center, float Radius) const
	{
		Sphere3d sphere(Center, Radius);
		return IntersectHF(this, sphere);
	}

	bool HeightField3d::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
	{
		Capsule3d capsule(X0, X1, Radius);
		return IntersectHF(this, capsule);
	}

	bool HeightField3d::CalculateVolumeProperties(MassParameters* p, float Density) const
	{
		p->Volume = 0.0f;
		p->Mass = 0.0f;
		p->CenterOfMass = BV.GetCenter();
		p->BoundingVolume = BV;
		p->InertiaMat = Matrix3::Identity();
		return true;
	}

	bool HeightField3d::GetCellBV(int i, int j, Box3& box) const
	{
		float minH, maxH;
		if (!GetHeightRange(i, j, minH, maxH))
		{
			return false;
		}
		box.Min = Vector3(BV.Min.x + DX * (i + 0), minH, BV.Min.z + DZ * (j + 0));
		box.Max = Vector3(BV.Min.x + DX * (i + 1), maxH, BV.Min.z + DZ * (j + 1));
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

		float Height0 = GetHeight(i0);
		float Height1 = GetHeight(i1);
		float Height2 = GetHeight(i2);
		float Height3 = GetHeight(i3);

		if (Hole0 != 0x7F && Hole1 != 0x7F)
		{
			minH = Maths::Min(Height0, Height1, Height2, Height3);
			maxH = Maths::Max(Height0, Height1, Height2, Height3);
			return true;
		}
		else if (Hole0 != 0x7F)
		{
			minH = Maths::Min(Maths::Min(Height0, Height2), (tessFlag ? Height3 : Height1));
			maxH = Maths::Max(Maths::Max(Height0, Height2), (tessFlag ? Height3 : Height1));
		}
		else if (Hole1 != 0x7F)
		{
			minH = Maths::Min(Maths::Min(Height1, Height3), (tessFlag ? Height0 : Height2));
			maxH = Maths::Max(Maths::Max(Height1, Height3), (tessFlag ? Height0 : Height2));
		}

		return false;
	}

	int HeightField3d::GetCellTriangle(int i, int j, Vector3 Tris[6]) const
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

		Vector3 Base = Vector3(BV.Min.x + DX * i, 0.0f, BV.Min.z + DZ * j);

		float Height0 = GetHeight(i0);
		float Height1 = GetHeight(i1);
		float Height2 = GetHeight(i2);
		float Height3 = GetHeight(i3);

		int nt = 0;
		if (Hole0 != 0x7F)
		{
			Tris[0] = Base + Vector3(0.0f, Height2, DZ);
			Tris[1] = Base + Vector3(0.0f, Height0, 0.0f);
			Tris[2] = tessFlag ? Base + Vector3(DX, Height3, DZ) : Base + Vector3(DX, Height1, 0.0f);
			nt += 3;
		}
		if (Hole1 != 0x7F)
		{
			Tris[nt + 0] = Base + Vector3(DX, Height3, DZ);
			Tris[nt + 1] = tessFlag ? Base + Vector3(0.0f, Height0, 0.0f) : Base + Vector3(0.0f, Height2, DZ);
			Tris[nt + 2] = Base + Vector3(DX, Height1, 0.0f);
			nt += 3;
		}

		return nt;
	}

	int HeightField3d::GetCellTriangle(int i, int j, uint32_t Tris[6]) const
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

		int nt = 0;
		if (Hole0 != 0x7F)
		{
			Tris[0] = i2;
			Tris[1] = i0;
			Tris[2] = tessFlag ? i3 : i1;
			nt += 3;
		}
		if (Hole1 != 0x7F)
		{
			Tris[nt + 0] = i3;
			Tris[nt + 1] = tessFlag ? i0 : i2;
			Tris[nt + 2] = i1;
			nt += 3;
		}

		return nt;
	}

	void HeightField3d::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		if (nZ * nZ == 0)
		{
			return;
		}

		Vertices.resize(nX * nZ);
		for (uint32_t i = 0; i < nX; i++)
			for (uint32_t j = 0; j < nZ; j++)
			{
				Vertices[i * nZ + j] = Vector3(BV.Min.x + DX * i, GetHeight(i * nZ + j), BV.Min.z + DZ * j);
			}

		assert(Vertices.size() < 65535);
		Indices.resize((nZ - 1) * (nX - 1) * 2 * 3);
		int nTris = 0;
		uint32_t Tris[6];

		for (uint32_t i = 0; i < (nZ - 1); ++i)
			for (uint32_t j = 0; j < (nX - 1); ++j)
			{
				int nT = GetCellTriangle(i, j, Tris);
				for (int k = 0; k < nT; k += 3)
				{
					Indices[3 * nTris + 0] = Tris[k + 0];
					Indices[3 * nTris + 1] = Tris[k + 1];
					Indices[3 * nTris + 2] = Tris[k + 2];
					nTris++;
				}
			}

		std::vector<int> Count;
		Count.resize(Vertices.size(), 0);
		Normals.resize(Vertices.size());
		memset(&Normals[0], 0, sizeof(Normals[0]) * Normals.size());
		for (int i = 0; i < nTris; ++i)
		{
			uint16_t i0 = Indices[3 * i + 0];
			uint16_t i1 = Indices[3 * i + 1];
			uint16_t i2 = Indices[3 * i + 2];
			const Vector3& v0 = Vertices[i0];
			const Vector3& v1 = Vertices[i1];
			const Vector3& v2 = Vertices[i2];
			Vector3 Nor = (v1 - v0).Cross(v2 - v0);
			Normals[i0] += Nor.Unit(); Count[i0]++;
			Normals[i1] += Nor.Unit(); Count[i1]++;
			Normals[i2] += Nor.Unit(); Count[i2]++;
		}

		for (size_t i = 0; i < Normals.size(); ++i)
		{
			Normals[i] *= 1.0f / Count[i];
			Normals[i].Normalize();
		}
	}

	void HeightField3d::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
}
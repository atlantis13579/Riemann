#pragma once

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Box3d.h"
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"

struct HeightFieldHitOption
{
	float	maxDist;
};

struct HeightFieldHitResult
{
	float		hitTime;
	Vector3d	hitNormal;
	uint32_t	cellIndex;
	int		    hitTestCount;
    
	void AddTestCount(int Count)
	{
		#ifdef _DEBUG
		hitTestCount += Count;
		#endif // _DEBUG
	}
};

class HeightField3d
{
public:
	struct CellInfo
	{
		uint8_t	Tessellation0;
		uint8_t	Tessellation1;
	};

	uint32_t 				nX;				// X
	uint32_t 				nZ;				// Z
	std::vector<float>		Heights;		// Y
	std::vector<CellInfo>	Cells;
	Box3d					BV;
	float					DX, DZ;
	float					InvDX, InvDZ;

	HeightField3d()
	{
	}

	HeightField3d(const Box3d& _Bv, int _nX, int _nZ)
	{
		Init(_Bv, _nX, _nZ);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::HEIGHTFIELD;
	}

	void    Init(const Box3d& _Bv, int _nX, int _nZ)
	{
		BV = _Bv;
		nX = _nX;
		nZ = _nZ;
		DX = BV.GetLengthX() / (nX - 1);
		DZ = BV.GetLengthZ() / (nZ - 1);
		InvDX = 1.0f / DX;
		InvDZ = 1.0f / DZ;
		Heights.resize(nX * nZ);
		memset(&Heights[0], 0, Heights.size() * sizeof(Heights[0]));
		Cells.resize(nX * nZ);
		memset(&Cells[0], 0, Cells.size() * sizeof(Cells[0]));
	}

	Box3d   GetBoundingVolume() const
	{
		return BV;
	}

    bool    IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const;
	bool    IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const;
    bool    IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult *Result) const;

	Vector3d    GetSupport(const Vector3d& dir) const
	{
		// TODO
		return Vector3d::UnitY();
	}

	Matrix3d	GetInertiaTensor(float Mass) const
	{
		return Matrix3d(Mass, Mass, Mass);
	}

	bool	GetCellBV(int i, int j, Box3d &box) const;
	bool	GetHeightRange(int i, int j, float &minH, float & maxH) const;

	int		GetCellTriangle(int i, int j, Vector3d Tris[6]) const;

	int		GetCellTriangle(int i, int j, uint32_t Tris[6]) const
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

	void GetMesh(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3d>& Normals)
	{
		if (nZ * nZ == 0)
		{
			return;
		}

		Vertices.resize(nX * nZ);
		for (uint32_t i = 0; i < nX; i++)
		for (uint32_t j = 0; j < nZ; j++)
		{
			Vertices[i * nZ + j] = Vector3d(BV.Min.x + DX * i, Heights[j + (i * nZ)], BV.Min.z + DX * j);
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
			const Vector3d& v0 = Vertices[i0];
			const Vector3d& v1 = Vertices[i1];
			const Vector3d& v2 = Vertices[i2];
			Vector3d Nor = (v1 - v0).Cross(v2 - v0);
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

	void GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
    
private:
	bool IntersectRayBruteForce(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
    bool IntersectRayCell(const Vector3d& Origin, const Vector3d& Dir, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
    bool IntersectRayY(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
};

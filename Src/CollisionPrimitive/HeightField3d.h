#pragma once

#include "assert.h"
#include <vector>

#include "../Maths/Box3d.h"
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "ShapeType.h"

struct HeightFieldHitOption
{
	float	maxDist;
};

struct HeightFieldHitResult
{
	float		hitTime;
	Vector3d	hitNormal;
	uint32_t	cellIndex;
	#ifdef _DEBUG
	int			TestCount;
	#endif // _DEBUG
};

class HeightField3d
{
public:
	struct CellInfo
	{
		uint8_t	Tessellation0;
		uint8_t	Tessellation1;
	};

	uint32_t 				nRows;				// X
	uint32_t 				nCols;				// Z
	std::vector<float>		Heights;			// Y
	std::vector<CellInfo>	Cells;
	Box3d					BV;
	float					DX, DZ;
	float					InvDX, InvDZ;

	HeightField3d()
	{
	}

	HeightField3d(const Box3d& _Bv, int _nRows, int _nCols)
	{
		Init(_Bv, _nRows, _nCols);
	}

	static constexpr ShapeType	StaticType()
	{
		return ShapeType::HEIGHTFIELD;
	}

	void Init(const Box3d& _Bv, int _nRows, int _nCols)
	{
		BV = _Bv;
		nRows = _nRows;
		nCols = _nCols;
		DX = BV.GetSizeX() / (nRows - 1);
		DZ = BV.GetSizeZ() / (nCols - 1);
		InvDX = (nRows - 1) / BV.GetSizeX();
		InvDZ = (nCols - 1) / BV.GetSizeZ();
		Heights.resize(nRows * nCols);
		memset(&Heights[0], 0, Heights.size() * sizeof(Heights[0]));
		Cells.resize(nRows * nCols);
		memset(&Cells[0], 0, Cells.size() * sizeof(Cells[0]));
	}

	Box3d			GetBoundingVolume() const
	{
		return BV;
	}

	bool			IntersectRayCell(const Vector3d& Origin, const Vector3d& Dir, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
	bool			IntersectRayY(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const HeightFieldHitOption& Option, HeightFieldHitResult *Result) const;
	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const;
	bool			IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const;

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		// TODO
		return Vector3d::UnitY();
	}

	Matrix3d		GetInertiaTensor(float Mass) const
	{
		return Matrix3d(Mass, Mass, Mass);
	}

	bool	GetCellBV(int i, int j, Box3d &box) const;
	bool	GetHeightRange(int i, int j, float &minH, float & maxH) const;

	int		GetCellTriangle(int i, int j, Vector3d Tris[6]) const;

	int		GetCellTriangle(int i, int j, uint32_t Tris[6]) const
	{
		bool tessFlag = Cells[i + j * nCols].Tessellation0 & 0x80;
		uint16_t i0 = j * nCols + i;
		uint16_t i1 = j * nCols + i + 1;
		uint16_t i2 = (j + 1) * nCols + i;
		uint16_t i3 = (j + 1) * nCols + i + 1;
		// i2---i3
		// |    |
		// |    |
		// i0---i1
		uint8_t Hole0 = Cells[i + j * nCols].Tessellation0;
		uint8_t Hole1 = Cells[i + j * nCols].Tessellation1;

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
		if (nCols * nCols == 0)
		{
			return;
		}

		Vertices.resize(nRows * nCols);
		for (uint32_t i = 0; i < nRows; i++)
		for (uint32_t j = 0; j < nCols; j++)
		{
			Vertices[i * nCols + j] = Vector3d(BV.Min.x + DX * i, Heights[j + (i * nCols)], BV.Min.z + DX * j);
		}

		assert(Vertices.size() < 65535);
		Indices.resize((nCols - 1) * (nRows - 1) * 2 * 3);
		int nTris = 0;
		uint32_t Tris[6];

		for (uint32_t i = 0; i < (nCols - 1); ++i)
		for (uint32_t j = 0; j < (nRows - 1); ++j)
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
			Normals[i0] += Nor.Unit(), Count[i0]++;
			Normals[i1] += Nor.Unit(), Count[i1]++;
			Normals[i2] += Nor.Unit(), Count[i2]++;
		}

		for (size_t i = 0; i < Normals.size(); ++i)
		{
			Normals[i] *= 1.0f / Count[i];
			Normals[i].Normalize();
		}
	}

	void			GetWireframe(std::vector<Vector3d>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
};

#pragma once

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Box3d.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

struct HeightFieldHitOption
{
	float	maxDist;
};

struct HeightFieldHitResult
{
	float		hitTime;
	Vector3	hitNormal;
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
		int16_t	HeightSample;
		uint8_t	Tessellation0;
		uint8_t	Tessellation1;
	};

	uint32_t 				nX;				// X
	uint32_t 				nZ;				// Z
	Box3d					BV;
	float					DX, DZ;
	float					InvDX, InvDZ;
	float 					HeightScale;
	std::vector<CellInfo>	mCells;
	CellInfo*				Cells;
	
	HeightField3d()
	{
		HeightScale = 1.0f;
		Cells = nullptr;
	}

	HeightField3d(const Box3d& _Bv, int _nX, int _nZ)
	{
		Init(_Bv, _nX, _nZ);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::HEIGHTFIELD;
	}

	void    	Init(const Box3d& _Bv, int _nX, int _nZ)
	{
		BV = _Bv;
		nX = _nX;
		nZ = _nZ;
		DX = BV.GetLengthX() / (nX - 1);
		DZ = BV.GetLengthZ() / (nZ - 1);
		InvDX = 1.0f / DX;
		InvDZ = 1.0f / DZ;
	}
	
	void	AllocMemory()
	{
		mCells.resize(nX * nZ * sizeof(CellInfo));
		Cells = &mCells[0];
		memset(Cells, 0, mCells.size() * sizeof(mCells[0]));
	}
	
	float 			GetHeight(int idx) const
	{
		return Cells[idx].HeightSample * HeightScale;
	}
	
	uint8_t			GetTessellationFlag0(int idx) const
	{
		return Cells[idx].Tessellation0;
	}
	
	uint8_t			GetTessellationFlag1(int idx) const
	{
		return Cells[idx].Tessellation1;
	}
	
	Box3d   		GetBoundingVolume() const
	{
		return BV;
	}

    bool		IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
	bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
    bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult *Result) const;

	Vector3		GetSupport(const Vector3& Direction) const
	{
		assert(false);
		return Vector3::Zero();
	}

	int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		assert(false);
		return 0;
	}

	Matrix3	GetInertiaTensor(float Mass) const
	{
		return Matrix3(Mass, Mass, Mass);
	}

	bool	GetCellBV(int i, int j, Box3d &box) const;
	bool	GetHeightRange(int i, int j, float &minH, float & maxH) const;

	int		GetCellTriangle(int i, int j, Vector3 Tris[6]) const;

	int		GetCellTriangle(int i, int j, uint32_t Tris[6]) const;

	void	GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		if (nZ * nZ == 0)
		{
			return;
		}

		Vertices.resize(nX * nZ);
		for (uint32_t i = 0; i < nX; i++)
		for (uint32_t j = 0; j < nZ; j++)
		{
			Vertices[i * nZ + j] = Vector3(BV.mMin.x + DX * i, GetHeight(i * nZ + j), BV.mMin.z + DX * j);
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

	void GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
    
private:
	bool IntersectRayBruteForce(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
    bool IntersectRayCell(const Vector3& Origin, const Vector3& Direction, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
    bool IntersectRayY(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
};

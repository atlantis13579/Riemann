#pragma once

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "ShapeType.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Geometry
{
	struct HeightFieldHitOption
	{
		float	maxDist;
	};

	struct HeightFieldHitResult
	{
		float		hitTime;
		Vector3		hitNormal;
		uint32_t	cellIndex;
		int		    hitTestCount;

		void AddTestCount(int Count)
		{
			hitTestCount += Count;
		}
	};

	class HeightField3
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
		Box3					BV;
		float					DX, DZ;
		float					InvDX, InvDZ;
		float 					HeightScale;
		std::vector<CellInfo>	mCells;
		CellInfo* Cells;

		HeightField3()
		{
			HeightScale = 1.0f;
			Cells = nullptr;
		}

		HeightField3(const Box3& _Bv, int _nX, int _nZ)
		{
			Init(_Bv, _nX, _nZ);
		}

		static constexpr ShapeType	StaticType()
		{
			return ShapeType::HEIGHTFIELD;
		}

		void    	Init(const Box3& _Bv, int _nX, int _nZ)
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

		bool		IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool		IntersectOBB(const Vector3& Center, const Vector3& Extent, const Matrix3& rot) const;
		bool		IntersectSphere(const Vector3& Center, float Radius) const;
		bool		IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const;
		bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;

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

		bool	CalculateVolumeProperties(MassParameters* p, float Density) const;

		inline Vector3	GetCellCenter2D(int i, int j) const
		{
			Vector3 v = Vector3(BV.Min.x + DX * (i + 0.0f), 0.0f, BV.Min.z + DZ * (j + 0.5f));
			return v;
		}

		bool	GetCellBV(int i, int j, Box3& box) const;
		bool	GetHeightRange(int i, int j, float& minH, float& maxH) const;
		int		GetCellTriangle(int i, int j, Vector3 Tris[6]) const;
		int		GetCellTriangle(int i, int j, uint32_t Tris[6]) const;

		void	GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
		void	GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);

	private:
		bool IntersectRayBruteForce(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
		bool IntersectRayCell(const Vector3& Origin, const Vector3& Direction, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
		bool IntersectRayY(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const;
	};
}
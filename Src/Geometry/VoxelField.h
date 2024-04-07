#pragma once

#include "../Maths/Box3.h"
#include "../Core/BatchList.h"

#include "stdint.h"
#include <functional>
#include <string>
#include <unordered_map>

namespace Geometry
{
	class Mesh;

	struct VoxelizationInfo
	{
		float	VoxelSize;
		float	VoxelHeight;
		float   YMergeThr;
		int		InterpMethod;
		Box3	BV;
	};

	struct Voxel
	{
		uint32_t		data;
		uint16_t		ymin;
		uint16_t		ymax;
		Voxel* next;
	};

	struct VoxelBatch
	{
		std::vector<Voxel>	Voxels;
		int					Current;
	};

	class VoxelField
	{
	public:
		VoxelField();
		~VoxelField();

	public:
		void			MakeEmpty(const Box3& Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);
		bool			VoxelizationTri(const Vector3& v0, const Vector3& v1, const Vector3& v2, const VoxelizationInfo& info);
		bool			VoxelizationYPlane(float world_y);
		bool			VoxelizationYPlane(float world_y_min, float world_y_max);
		bool			VoxelizationCube(const Box3& cube);
		bool			MakeComplementarySet();
		uint64_t		Separate(const Vector3& pos, uint32_t data, float IntersectThr);
		int				SolveTopology(float IntersectThr, std::unordered_map<int, uint64_t>* volumes = nullptr);
		bool			IntersectYPlane(float y_value, std::vector<int>& output, float Thr);
		int				Filter(std::function<bool(Voxel* v)> func);
		int				FilterByY(float world_y);
		int				FilterByData(uint32_t data);
		void			FilterTopNByVolume(const std::unordered_map<int, uint64_t>& volumes, int TopN);
		void			TraversalVoxel(std::function<bool(Voxel* v)> callback);
		void			TraversalField(std::function<void(int idx, Voxel* v)> callback);

		std::string		DebugString(int idx) const;
		Voxel* GetVoxel(const Vector3& pos);
		Voxel* GetVoxel(int idx);
		Voxel* GetVoxelByY(Voxel* Base, float y, float Thr);
		int				WorldSpaceToVoxelIndex(const Vector3& pos) const;
		int				WorldSpaceToVoxelSpaceY(float pos_y) const;
		float			VoxelSpaceToWorldSpaceY(uint16_t y) const;
		Box3			GetVoxelBox(const Vector3& pos) const;
		Box3			GetVoxelBox(int x, int y, int z) const;
		uint32_t		GetVoxelData(const Vector3& pos);

		bool			SerializeTo(const char* filename);
		bool			SerializeFrom(const char* filename);

		int				GetSizeX() const
		{
			return m_SizeX;
		}

		int				GetSizeZ() const
		{
			return m_SizeZ;
		}

		const Box3& GetBoundingVolume() const
		{
			return m_BV;
		}

		float			GetVoxelVolume() const
		{
			return m_VoxelSize * m_VoxelSize * m_VoxelHeight;
		}

		uint64_t		EstimateMemoryUseage() const;
		uint64_t		EstimateMemoryUseageEx() const;

		Mesh* CreateDebugMesh(int x1, int x2, int z1, int z2) const;
		bool			Verify();
		void			GenerateHeightMap(std::vector<float>& heightmap) const;
		void			GenerateBitmapByLevel(std::vector<int>& levels, int* level_max) const;
		void			GenerateBitmapByData(std::vector<int>& output, uint32_t data) const;
		bool			AddVoxel(int idx, uint16_t ymin, uint16_t ymax, float MergeThr);

	private:
		Voxel* AllocVoxel();
		void			FreeVoxel(Voxel* p);
		int				CalculateNumFields() const;

		uint64_t		SeparateImpl(int idx, Voxel* base, uint32_t data, uint16_t Thr);

	private:
		int			m_SizeX, m_SizeZ, m_SizeY;
		float		m_VoxelSize, m_VoxelHeight;
		float		m_InvVoxelSize, m_InvVoxelHeight;
		Box3		m_BV;

		std::vector<Voxel*>			m_Fields;
		BatchList<Voxel>		m_VoxelBatchs;
	};

	#define VOXEL_FILE_MAGIC	(0x32F0587A)

	struct VoxelFileHeader
	{
		uint32_t		nVoxels;
		uint32_t		nFields;
		int				SizeX;
		int				SizeZ;
		int				SizeY;
		float			VoxelSize;
		float			VoxelHeight;
		Box3			BV;
	};

	struct VoxelFileField
	{
		uint32_t		idx;
	};

	static_assert(sizeof(VoxelFileHeader) == 52, "sizeof VoxelFileHeader is not valid");
	static_assert(sizeof(VoxelFileField) == 4, "sizeof VoxelFileField is not valid");

	struct VoxelFast
	{
		uint32_t	data;
		uint16_t	ymin;
		uint16_t	ymax;
	};

	static_assert(sizeof(VoxelFast) == 8, "sizeof of VoxelFast is not correct");
}
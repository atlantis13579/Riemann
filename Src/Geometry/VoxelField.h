#pragma once

#include "../Maths/Box3d.h"

#include <functional>
#include <unordered_map>

class Mesh;

struct VoxelizationInfo
{
	float	VoxelSize;
	float	VoxelHeight;
	float   YMergeThr;
	Box3d	BV;
};

typedef unsigned long long	vx_uint64;
typedef unsigned int		vx_uint32;

struct Voxel
{
	vx_uint32		data;
	unsigned short	ymin;
	unsigned short	ymax;
	Voxel*			next;
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
	void			MakeEmpty(const Box3d& Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);
	bool			VoxelizationTrianglesSet(const VoxelizationInfo& info, Mesh* mesh);
	bool			VoxelizationTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info);
	bool			MakeComplementarySet();
	vx_uint64		Separate(const Vector3d& pos, vx_uint32 data, float IntersectThr);
	int				SolveTopology(float IntersectThr, std::unordered_map<int, vx_uint64>* volumes = nullptr);
	bool			IntersectYPlane(float y_value, std::vector<int>& output, float Thr);
	void			Filter(std::function<bool(vx_uint32 data)> func);
	void			FilterTopNByVolume(const std::unordered_map<int, vx_uint64>& volumes, int TopN);

	Voxel*			GetVoxel(const Vector3d& pos);
	int				WorldSpaceToVoxelIndex(const Vector3d& pos) const;
	int				WorldSpaceToVoxelSpaceY(float pos_y) const;
	float			VoxelSpaceToWorldSpaceY(unsigned short y) const;
	Box3d			GetVoxelBox(const Vector3d& pos) const;
	Box3d			GetVoxelBox(int x, int y, int z) const;
	vx_uint32		GetVoxelData(const Vector3d& pos);

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

	const Box3d&	GetBoundingVolume() const
	{
		return m_BV;
	}

	float			GetVoxelVolume() const
	{
		return m_VoxelSize * m_VoxelSize * m_VoxelHeight;
	}

	vx_uint64		EstimateMemoryUseage() const;
	vx_uint64		EstimateMemoryUseageEx() const;

	Mesh*	CreateDebugMesh(int x1, int x2, int z1, int z2) const;
	bool			Verify() const;
	void			ResetData();
	void			GenerateHeightMap(std::vector<float>& heightmap) const;
	void			GenerateBitmapByLevel(std::vector<int>& levels, int* level_max) const;
	void			GenerateBitmapByData(std::vector<int>& output, vx_uint32 data) const;
	bool			AddVoxel(int idx, unsigned short ymin, unsigned short ymax, float MergeThr);

private:
	Voxel*			AllocVoxel();
	void			FreeVoxel(Voxel* p);
	int				CalculateNumFields() const;

	vx_uint64		SeparateImpl(int idx, Voxel* base, vx_uint32 data, unsigned short Thr);
	Voxel*			GetVoxelByY(Voxel* Base, float y, float Thr);

private:
	int			m_SizeX, m_SizeZ, m_SizeY;
	float		m_VoxelSize, m_VoxelHeight;
	float		m_InvVoxelSize, m_InvVoxelHeight;
	Box3d		m_BV;

	std::vector<Voxel*>			m_Fields;
	std::vector<VoxelBatch>		m_VoxelBatchs;
	Voxel*						m_FreeVoxelList;
	int							m_NumVoxels;
};
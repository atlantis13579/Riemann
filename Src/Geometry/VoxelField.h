#pragma once

#include "../Maths/Box3d.h"

class TriangleMesh;

struct VoxelizationInfo
{
	float	VoxelSize;
	float	VoxelHeight;
	float   YMergeThr;
	Box3d	Boundry;
};

struct Voxel
{
	int		low;
	int		high;
	Voxel*	next;
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
	bool	VoxelizeTriangles(const VoxelizationInfo &info, TriangleMesh *mesh);
	bool	VoxelizeTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info);
	
	int		GetSizeX() const
	{
		return m_SizeX;
	}

	int		GetSizeZ() const
	{
		return m_SizeZ;
	}

	void	GenerateHeightMap(std::vector<int>& heightmap);
	void	CalculateYLimit(int *ymin, int *ymax);

private:
	bool	AddVoxel(int x, int y, int smin, int smax, float MergeThr);
	Voxel*	AllocVoxel();
	void    FreeVoxel(Voxel* p);

private:
	int		m_SizeX, m_SizeZ, m_SizeY;
	Box3d	m_WorldBox;

	std::vector<Voxel*>			m_Fields;
	std::vector<VoxelBatch>		m_VoxelBatchs;
	Voxel*						m_FreeVoxelList;
	int							m_NumVoxels;
};
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
	int				data;
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
	void	InitField(int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);
	bool	VoxelizeTriangles(const VoxelizationInfo &info, TriangleMesh *mesh);
	bool	VoxelizeTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info);
	bool	MakeComplement();
	int		SolveSpatialTopology();

	int		GetVoxelIdx(const Vector3d &pos) const;
	Box3d	GetVoxelBox() const;
	float	GetVoxelY(unsigned short y) const;

	int		GetSizeX() const
	{
		return m_SizeX;
	}

	int		GetSizeZ() const
	{
		return m_SizeZ;
	}

	void	GenerateHeightMap(std::vector<float>& heightmap) const;
	void	GenerateLevels(std::vector<int>& levels, int* level_max) const;
	void	CalculateYLimit(float *ymin, float *ymax) const;
	bool	AddVoxel(int x, int y, unsigned short ymin, unsigned short ymax, float MergeThr);

private:
	Voxel*	AllocVoxel();
	void    FreeVoxel(Voxel* p);

private:
	int			m_SizeX, m_SizeZ, m_SizeY;
	float		m_VoxelSize, m_VoxelHeight;
	float		m_InvVoxelSize, m_InvVoxelHeight;
	Box3d		m_WorldBox;

	std::vector<Voxel*>			m_Fields;
	std::vector<VoxelBatch>		m_VoxelBatchs;
	Voxel*						m_FreeVoxelList;
	int							m_NumVoxels;
};
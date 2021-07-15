#pragma once

#include "../Maths/Box3d.h"

#include <map>

class TriangleMesh;

struct VoxelizationInfo
{
	float	VoxelSize;
	float	VoxelHeight;
	float   YMergeThr;
	Box3d	BV;
};

struct Voxel
{
	unsigned int	data;
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
	void			MakeEmptySet(const Box3d& Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);
	bool			VoxelizationTrianglesSet(const VoxelizationInfo& info, TriangleMesh* mesh);
	bool			VoxelizationTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info);
	bool			MakeComplementarySet();
	int				Separate(std::map<int, unsigned long long> *volumes = nullptr);
	bool			IntersectYPlane(float y_value, std::vector<int>& output, bool take_next = false);
	void			FilterByData(unsigned int data);

	const Voxel*	GetVoxel(const Vector3d& pos) const;
	int				GetVoxelIdx(const Vector3d& pos) const;
	int				GetVoxelYCoordinate(float pos_y) const;
	float			GetVoxelY(unsigned short y) const;
	Box3d			GetVoxelBox(const Vector3d& pos) const;
	Box3d			GetVoxelBox(int x, int y, int z) const;
	unsigned int	GetVoxelData(const Vector3d& pos) const;

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

	unsigned long long	EstimateMemoryUseage() const;
	unsigned long long	EstimateMemoryUseageEx() const;

	TriangleMesh*	CreateDebugMesh(int x1, int x2, int z1, int z2) const;
	bool			Verify() const;
	void			GenerateHeightMap(std::vector<float>& heightmap) const;
	void			GenerateBitmapByLevel(std::vector<int>& levels, int* level_max) const;
	void			GenerateBitmapByData(std::vector<int>& output, unsigned int data) const;
	bool			AddVoxel(int idx, unsigned short ymin, unsigned short ymax, float MergeThr);

private:
	Voxel*			AllocVoxel();
	void			FreeVoxel(Voxel* p);
	int				CalculateNumFields() const;
	unsigned int	ExtractVoxelData(const Voxel* v, float y, bool take_next = false) const;

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
#pragma once

#include "../Maths/Box3d.h"

#define VOXEL_TOP			(0x80000000)
#define VOXEL_DATA(data)	((data) & ~VOXEL_TOP)
#define VOXEL_FILE_MAGIC	(0x32F0587A)

struct VoxelFileHeader
{
	int				nVoxels;
	int				nFields;
	int				SizeX;
	int				SizeZ;
	int				SizeY;
	float			VoxelSize;
	float			VoxelHeight;
	Box3d			BV;
};

struct VoxelFileField
{
	int				idx;
};

static_assert(sizeof(VoxelFileHeader) == 52, "sizeof VoxelFileHeader is not valid");
static_assert(sizeof(VoxelFileField) == 4, "sizeof VoxelFileHeader is not valid");

struct VoxelFast
{
	unsigned int	data;
	unsigned short	ymin;
	unsigned short	ymax;

	inline const VoxelFast* next() const
	{
		if (data & VOXEL_TOP)
			return nullptr;
		return this + 1;
	}
};

static_assert(sizeof(VoxelFast) == 8, "sizeof of VoxelFast is not correct");

class SparseVoxelFieldInference
{
public:
	SparseVoxelFieldInference();
	~SparseVoxelFieldInference();

	void		InitField(const Box3d& Bv, int nVoxels, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);

	int			GetVoxelIdx(const Vector3d& pos) const;
	int			GetVoxelY(float pos_y) const;
	Box3d		GetVoxelBox(const Vector3d& pos) const;
	Box3d		GetVoxelBox(int x, int y, int z) const;
	float		GetVoxelY(unsigned short y) const;

	bool		SerializeFrom(const char* filename);

private:
	int			m_SizeX, m_SizeZ, m_SizeY;
	float		m_VoxelSize, m_VoxelHeight;
	float		m_InvVoxelSize, m_InvVoxelHeight;
	Box3d		m_BV;

	std::vector<VoxelFast*>		m_Fields;
	std::vector<VoxelFast>		m_VoxelPool;
	int							m_NumVoxels;
};

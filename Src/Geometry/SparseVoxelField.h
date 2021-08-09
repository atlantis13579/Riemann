#pragma once

#include <stdint.h>
#include "../Maths/Box3d.h"

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
	Box3d			BV;
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

class SparseVoxelField
{
public:
	SparseVoxelField();
	~SparseVoxelField();

	void			InitField(const Box3d& Bv, int nVoxels, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight);

	int				GetVoxelIdx(const Vector3d& pos) const;
	int				VoxelSpaceToWorldSpaceY(float pos_y) const;
	float			VoxelSpaceToWorldSpaceY(uint16_t y) const;
	uint32_t		WorldSpaceToVoxelSpaceY(const Vector3d& pos) const;

	bool			SerializeFrom(const char* filename);
	int				GetVoxelCount(uint32_t idx) const;

private:
	int				m_SizeX, m_SizeZ, m_SizeY;
	float			m_VoxelSize, m_VoxelHeight;
	float			m_InvVoxelSize, m_InvVoxelHeight;
	Box3d			m_BV;

	std::vector<uint32_t>		m_Indices;
	std::vector<VoxelFast>		m_Voxels;
	int							m_NumVoxels;
};

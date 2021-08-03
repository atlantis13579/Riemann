
#include "SparseVoxelField.h"

#include "assert.h"
#include "../Maths/Maths.h"

SparseVoxelField::SparseVoxelField()
{
	m_SizeX = 0;
	m_SizeY = 0;
	m_SizeZ = 0;
	m_NumVoxels = 0;
}

SparseVoxelField::~SparseVoxelField()
{

}

void	SparseVoxelField::InitField(const Box3d& Bv, int nVoxels, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
{
	m_BV = Bv;
	m_SizeX = SizeX;
	m_SizeY = SizeY;
	m_SizeZ = SizeZ;
	m_VoxelSize = VoxelSize;
	m_VoxelHeight = VoxelHeight;
	m_InvVoxelSize = 1.0f / VoxelSize;
	m_InvVoxelHeight = 1.0f / VoxelHeight;

	m_Indices.resize(m_SizeX * m_SizeZ);
	memset(&m_Indices[0], 0, sizeof(m_Indices[0]) * m_SizeX * m_SizeZ);

	m_Voxels.resize(nVoxels);
}


int		SparseVoxelField::GetVoxelIdx(const Vector3d& pos) const
{
	const int x = Clamp((int)((pos.x - m_BV.Min.x) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z = Clamp((int)((pos.z - m_BV.Min.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}


int		SparseVoxelField::VoxelSpaceToWorldSpaceY(float pos_y) const
{
	const int y = Clamp((int)((pos_y - m_BV.Min.y) * m_InvVoxelHeight), 0, m_SizeY - 1);
	return y;
}

float	SparseVoxelField::VoxelSpaceToWorldSpaceY(uint16_t y) const
{
	return m_BV.Min.y + (y + 0.5f) * m_VoxelHeight;
}


uint32_t	SparseVoxelField::WorldSpaceToVoxelSpaceY(const Vector3d& pos) const
{
	int idx = GetVoxelIdx(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	uint32_t i0 = m_Indices[idx];
	uint32_t i1 = i0 + GetVoxelCount(idx);
	for (uint32_t i = i0; i < i1; ++i)
	{
		const VoxelFast* v = &m_Voxels[i];
		float ymin = m_BV.Min.y + m_VoxelHeight * v->ymin;
		float ymax = m_BV.Min.y + m_VoxelHeight * (v->ymax + 1);
		if (ymin <= pos.y && pos.y <= ymax)
		{
			return v->data;
		}
	}
	return 0;
}


bool	SparseVoxelField::SerializeFrom(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	if (!fp)
	{
		return false;
	}

	_fseeki64(fp, 0, SEEK_END);
	uint64_t fileSize = _ftelli64(fp);
	_fseeki64(fp, 0, SEEK_SET);

	if (fileSize < 4 + sizeof(VoxelFileHeader))
	{
		return false;
	}

	uint32_t Magic;
	fread(&Magic, sizeof(Magic), 1, fp);
	if (Magic != VOXEL_FILE_MAGIC)
	{
		return false;
	}

	VoxelFileHeader	header;
	fread(&header, sizeof(header), 1, fp);
	uint64_t nExpectedSize = 4 + sizeof(VoxelFileHeader);
	nExpectedSize += sizeof(VoxelFileField) * header.nFields;
	nExpectedSize += sizeof(VoxelFast) * header.nVoxels;
	if (fileSize != nExpectedSize)
	{
		return false;
	}

	InitField(header.BV, header.nVoxels, header.SizeX, header.SizeY, header.SizeZ, header.VoxelSize, header.VoxelHeight);

	assert((size_t)header.nVoxels == m_Voxels.size());
	fread(&m_Voxels[0], sizeof(VoxelFast), m_Voxels.size(), fp);

	m_Indices.resize(header.nFields);
	fread(&m_Indices[0], sizeof(VoxelFileField), m_Indices.size(), fp);
	return true;
}

int SparseVoxelField::GetVoxelCount(uint32_t idx) const
{
	return idx == (int)m_Indices.size() - 1 ? ((int)m_Voxels.size() - m_Indices[idx]) : (m_Indices[idx + 1] - m_Indices[idx]);
}

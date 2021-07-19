
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

static int CountVoxel(const VoxelFast* v)
{
	int Count = 0;
	while (v)
	{
		++Count;
		v = v->next();
	}
	return Count;
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

	m_Fields.resize(m_SizeX * m_SizeZ);
	memset(&m_Fields[0], 0, sizeof(m_Fields[0]) * m_SizeX * m_SizeZ);

	m_VoxelPool.resize(nVoxels);
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

float	SparseVoxelField::VoxelSpaceToWorldSpaceY(unsigned short y) const
{
	return m_BV.Min.y + (y + 0.5f) * m_VoxelHeight;
}


unsigned int	SparseVoxelField::WorldSpaceToVoxelSpaceY(const Vector3d& pos) const
{
	int idx = GetVoxelIdx(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	const VoxelFast* v = m_Fields[idx];
	while (v)
	{
		float ymin = m_BV.Min.y + m_VoxelHeight * v->ymin;
		float ymax = m_BV.Min.y + m_VoxelHeight * (v->ymax + 1);
		if (ymin <= pos.y && pos.y <= ymax)
		{
			return v->raw_data();
		}
		v = v->next();
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
	unsigned long long fileSize = _ftelli64(fp);
	_fseeki64(fp, 0, SEEK_SET);

	if (fileSize < 4 + sizeof(VoxelFileHeader))
	{
		return false;
	}

	unsigned int Magic;
	fread(&Magic, sizeof(Magic), 1, fp);
	if (Magic != VOXEL_FILE_MAGIC)
	{
		return false;
	}

	VoxelFileHeader	header;
	fread(&header, sizeof(header), 1, fp);
	unsigned long long nExpectedSize = 4 + sizeof(VoxelFileHeader);
	nExpectedSize += sizeof(VoxelFileField) * header.nFields;
	nExpectedSize += sizeof(VoxelFast) * header.nVoxels;
	if (fileSize != nExpectedSize)
	{
		return false;
	}

	InitField(header.BV, header.nVoxels, header.SizeX, header.SizeY, header.SizeZ, header.VoxelSize, header.VoxelHeight);

	assert((size_t)header.nVoxels == m_VoxelPool.size());
	fread(&m_VoxelPool[0], sizeof(VoxelFast), m_VoxelPool.size(), fp);

	std::vector<VoxelFileField> buffer_field;
	buffer_field.resize(header.nFields);
	fread(&buffer_field[0], sizeof(VoxelFileField), buffer_field.size(), fp);

	size_t curr = 0;
	for (int i = 0; i < header.nFields; ++i)
	{
		if (curr >= m_VoxelPool.size())
		{
			assert(false);
			return false;
		}

		int idx = buffer_field[i].idx;
		m_Fields[idx] = &m_VoxelPool[curr];
		curr += (size_t)CountVoxel(m_Fields[idx]);
	}

	return true;
}

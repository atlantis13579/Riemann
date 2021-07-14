
#include "SparseVoxelFieldInference.h"

#include "assert.h"
#include "../Maths/Maths.h"

SparseVoxelFieldInference::SparseVoxelFieldInference()
{
	m_SizeX = 0;
	m_SizeY = 0;
	m_SizeZ = 0;
	m_NumVoxels = 0;
}

SparseVoxelFieldInference::~SparseVoxelFieldInference()
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


void	SparseVoxelFieldInference::InitField(const Box3d& Bv, int nVoxels, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
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


int		SparseVoxelFieldInference::GetVoxelIdx(const Vector3d& pos) const
{
	const int x = Clamp((int)((pos.x - m_BV.Min.x) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z = Clamp((int)((pos.z - m_BV.Min.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}


int		SparseVoxelFieldInference::GetVoxelY(float pos_y) const
{
	const int y = Clamp((int)((pos_y - m_BV.Min.y) * m_InvVoxelHeight), 0, m_SizeY - 1);
	return y;
}


Box3d	SparseVoxelFieldInference::GetVoxelBox(const Vector3d& pos) const
{
	int idx = GetVoxelIdx(pos);
	int z = idx / m_SizeX;
	int x = idx - z * m_SizeX;
	int y = GetVoxelY(pos.y);
	return GetVoxelBox(x, y, z);
}


Box3d	SparseVoxelFieldInference::GetVoxelBox(int x, int y, int z) const
{
	if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ || y < 0 || y >= m_SizeY)
	{
		// Error
		return Box3d::Unit();
	}
	Box3d box;
	box.Min.x = m_BV.Min.x + m_VoxelSize * x;
	box.Min.y = m_BV.Min.y + m_VoxelHeight * y;
	box.Min.z = m_BV.Min.z + m_VoxelSize * x;
	box.Max = box.Min + Vector3d(m_VoxelSize, m_VoxelHeight, m_VoxelSize);
	return box;
}


float	SparseVoxelFieldInference::GetVoxelY(unsigned short y) const
{
	return m_BV.Min.y + (y + 0.5f) * m_VoxelHeight;
}


bool	SparseVoxelFieldInference::SerializeFrom(const char* filename)
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

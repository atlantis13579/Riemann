
#include "SparseVoxelField.h"
#include "SparseVoxelFieldInference.h"

#include <assert.h>
#include <map>
#include <queue>
#include "../Maths/Maths.h"
#include "../CollisionPrimitive/TriangleMesh.h"

SparseVoxelField::SparseVoxelField()
{
	m_SizeX = 0;
	m_SizeY = 0;
	m_SizeZ = 0;
	m_NumVoxels = 0;
	m_FreeVoxelList = nullptr;
}

SparseVoxelField::~SparseVoxelField()
{

}

void SparseVoxelField::InitField(const Box3d &Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
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

	m_VoxelBatchs.clear();
	m_FreeVoxelList = nullptr;
}


bool SparseVoxelField::VoxelizeTriangles(const VoxelizationInfo& info, TriangleMesh* _mesh)
{
	m_SizeX = (int)(info.BV.GetSizeX() / info.VoxelSize + 0.5f);
	m_SizeY = (int)(info.BV.GetSizeY() / info.VoxelHeight + 0.5f);
	m_SizeZ = (int)(info.BV.GetSizeZ() / info.VoxelSize + 0.5f);

	InitField(info.BV, m_SizeX, m_SizeY, m_SizeZ, info.VoxelSize, info.VoxelHeight);

	const TriangleMesh &mesh = *_mesh;
	for (unsigned int i = 0; i < mesh.GetNumTriangles(); ++i)
	{
		Vector3d v0 = mesh(i, 0);
		Vector3d v1 = mesh(i, 1);
		Vector3d v2 = mesh(i, 2);
		if (!VoxelizeTri(v0, v1, v2, info))
		{
			return false;
		}
	}
	return true;
}


int		SparseVoxelField::GetVoxelIdx(const Vector3d& pos) const
{
	const int x = Clamp((int)((pos.x - m_BV.Min.x) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z = Clamp((int)((pos.z - m_BV.Min.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}


int		SparseVoxelField::GetVoxelY(float pos_y) const
{
	const int y = Clamp((int)((pos_y - m_BV.Min.y) * m_InvVoxelHeight), 0, m_SizeY - 1);
	return y;
}


Box3d	SparseVoxelField::GetVoxelBox(const Vector3d& pos) const
{
	int idx = GetVoxelIdx(pos);
	int z = idx / m_SizeX;
	int x = idx - z * m_SizeX;
	int y = GetVoxelY(pos.y);
	return GetVoxelBox(x, y, z);
}


Box3d	SparseVoxelField::GetVoxelBox(int x, int y, int z) const
{
	if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ || y < 0 || y >= m_SizeY)
	{
		// Error
		return Box3d::Unit();
	}
	Box3d box;
	box.Min.x = m_BV.Min.x + m_VoxelSize * x;
	box.Min.y = m_BV.Min.y + m_VoxelHeight * y;
	box.Min.z = m_BV.Min.z + m_VoxelSize * z;
	box.Max = box.Min + Vector3d(m_VoxelSize, m_VoxelHeight, m_VoxelSize);
	return box;
}


float	SparseVoxelField::GetVoxelY(unsigned short y) const
{
	return m_BV.Min.y + (y + 0.5f) * m_VoxelHeight;
}

unsigned int	SparseVoxelField::GetVoxelData(const Vector3d& pos) const
{
	int idx = GetVoxelIdx(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	Voxel* v = m_Fields[idx];
	while (v)
	{
		float ymin = m_BV.Min.y + m_VoxelHeight * v->ymin;
		float ymax = m_BV.Min.y + m_VoxelHeight * (v->ymax + 1);
		if (ymin <= pos.y && pos.y <= ymax)
		{
			return v->data;
		}
		v = v->next;
	}
	return 0;
}


static void dividePoly(const Vector3d* In, int nn, Vector3d* out1, int* nout1, Vector3d* out2, int* nout2, float x, int axis)
{
	float d[12];
	for (int i = 0; i < nn; ++i)
		d[i] = x - In[i][axis];

	int m = 0, n = 0;
	for (int i = 0, j = nn - 1; i < nn; j = i, ++i)
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)
		{
			float s = d[j] / (d[j] - d[i]);
			out1[m] = In[j] + (In[i] - In[j]) * s;
			out2[n] = out1[m];

			m++;
			n++;
			if (d[i] > 0)
			{
				out1[m] = In[i];
				m++; 
			}
			else if (d[i] < 0)
			{
				out2[n] = In[i];
				n++;
			}
		}
		else
		{
			if (d[i] >= 0)
			{
				out1[m] = In[i];
				m++;
				if (d[i] != 0)
					continue;
			}
			out2[n] = In[i];
			n++;
		}
	}

	*nout1 = m;
	*nout2 = n;
}

bool SparseVoxelField::AddVoxel(int x, int y, unsigned short ymin, unsigned short ymax, float MergeThr)
{
	int idx = x + y * m_SizeX;

	Voxel* s = AllocVoxel();
	if (s == nullptr)
	{
		return false;
	}
	s->data = 0;
	s->ymin = ymin;
	s->ymax = ymax;
	s->next = nullptr;

	if (m_Fields[idx] == nullptr)
	{
		m_Fields[idx] = s;
		return true;
	}

	Voxel* prev = nullptr;
	Voxel* p = m_Fields[idx];

	while (p)
	{
		if (p->ymin > s->ymax)
		{
			break;
		}
		else if (p->ymax + 1 < s->ymin)
		{
			prev = p;
			p = p->next;
		}
		else
		{
			if (p->ymin < s->ymin)
				s->ymin = p->ymin;
			if (p->ymax > s->ymax)
				s->ymax = p->ymax;

			Voxel* next = p->next;
			FreeVoxel(p);
			if (prev)
				prev->next = next;
			else
				m_Fields[idx] = next;
			p = next;
		}
	}

	if (prev)
	{
		s->next = prev->next;
		prev->next = s;
	}
	else
	{
		s->next = m_Fields[idx];
		m_Fields[idx] = s;
	}

	return true;
}

Voxel*	SparseVoxelField::AllocVoxel()
{
	m_NumVoxels++;

	if (m_FreeVoxelList)
	{
		Voxel* p = m_FreeVoxelList;
		m_FreeVoxelList = m_FreeVoxelList->next;
		return p;
	}

	int kVoxelBatchSize = std::max(1024, m_SizeX * m_SizeZ / 2);
	if (m_VoxelBatchs.empty() || m_VoxelBatchs.back().Current >= m_VoxelBatchs.back().Voxels.size())
	{
		m_VoxelBatchs.resize(m_VoxelBatchs.size() + 1);
		m_VoxelBatchs.back().Voxels.resize(kVoxelBatchSize);
		m_VoxelBatchs.back().Current = 0;
	}
	VoxelBatch& Batch = m_VoxelBatchs.back();
	Voxel* p = &Batch.Voxels[Batch.Current++];
	return p;
}

void SparseVoxelField::FreeVoxel(Voxel* p)
{
	m_NumVoxels--;

	p->next = m_FreeVoxelList;
	m_FreeVoxelList = p;
}

static bool VoxelIntersects(const Voxel* v1, const Voxel *v2)
{
	return v2->ymin <= v1->ymax && v2->ymax >= v1->ymin;
}

static bool VoxelIntersects2(const Voxel* v1, const Voxel* v2, float Thr)
{
	return v2->ymin < v1->ymax - Thr && v2->ymax > v1->ymin + Thr;
}

static Voxel* FindVoxel(Voxel* v, unsigned int data)
{
	while (v)
	{
		if (v->data == data)
			return v;
		v = v->next;
	}
	return nullptr;
}

static int CountVoxel(const Voxel* v)
{
	int Count = 0;
	while (v)
	{
		++Count;
		v = v->next;
	}
	return Count;
}

static int VoxelMaxData(const Voxel* v)
{
	unsigned int Data = 0;
	while (v)
	{
		Data = std::max(Data, v->data);
		v = v->next;
	}
	return Data;
}


bool SparseVoxelField::VoxelizeTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info)
{
	const float dy = m_BV.Max.y - m_BV.Min.y;
	const float ics = 1.0f / m_VoxelSize;
	const float ich = 1.0f / m_VoxelHeight;

	Box3d tbox({ v0, v1, v2 });

	if (!tbox.Intersect(m_BV))
		return true;

	const int z0 = Clamp((int)((tbox.Min.z - m_BV.Min.z) * ics), 0, m_SizeZ - 1);
	const int z1 = Clamp((int)((tbox.Max.z - m_BV.Min.z) * ics), 0, m_SizeZ - 1);

	Vector3d buf[7 * 4];
	Vector3d *In = buf, *inrow = buf + 7, *p1 = buf + 7 * 2, *p2 = buf + 7 * 3;

	In[0] = v0;
	In[1] = v1;
	In[2] = v2;

	int nvrow, nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		const float cz = m_BV.Min.z + z * m_VoxelSize;
		dividePoly(In, nvIn, inrow, &nvrow, p1, &nvIn, cz + m_VoxelSize, 2);
		std::swap(In, p1);
		if (nvrow < 3)
			continue;

		float minX = inrow[0].x, maxX = inrow[0].x;
		for (int i = 1; i < nvrow; ++i)
		{
			minX = std::min(minX, inrow[i].x);
			maxX = std::max(maxX, inrow[i].x);
		}
		const int x0 = Clamp((int)((minX - m_BV.Min.x) * ics), 0, m_SizeX - 1);
		const int x1 = Clamp((int)((maxX - m_BV.Min.x) * ics), 0, m_SizeX - 1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			const float cx = m_BV.Min.x + x * m_VoxelSize;
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx + m_VoxelSize, 0);
			std::swap(inrow, p2);
			if (nv < 3)
				continue;

			float ymin = p1[0].y, ymax = p1[0].y;
			for (int i = 1; i < nv; ++i)
			{
				ymin = std::min(ymin, p1[i].y);
				ymax = std::max(ymax, p1[i].y);
			}

			if (ymax < m_BV.Min.y || ymin > m_BV.Max.y)
				continue;

			if (ymin < m_BV.Min.y)
				ymin = m_BV.Min.y;
			if (ymax > m_BV.Max.y)
				ymax = m_BV.Max.y;
			ymin -= m_BV.Min.y;
			ymax -= m_BV.Min.y;

			const unsigned short y0 = (unsigned short)Clamp((int)floorf(ymin * ich), 0, m_SizeY - 1);
			const unsigned short y1 = (unsigned short)Clamp((int)ceilf(ymax * ich), 0, m_SizeY - 1);

			if (!AddVoxel(x, z, y0, y1, info.YMergeThr))
				return false;
		}
	}

	return true;
}

bool SparseVoxelField::MakeComplement()
{
	unsigned short yhigh = m_SizeY;

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		// float ylow = m_WorldBox.Min.y;
		unsigned short ylow = 0;
		Voxel *p = m_Fields[idx], *prev = nullptr;
		while (p)
		{
			auto t = p->ymin;
			p->ymin = ylow;
			ylow = p->ymax + 1;
			p->ymax = t;
			prev = p;
			p = p->next;
		}

		// if (fabsf(m_WorldBox.Max.y - ylow) > 1e-3)
		if (yhigh != ylow)
		{
			p = AllocVoxel();
			if (p == nullptr)
			{
				return false;
			}
			p->data = 0;
			p->ymin = ylow;
			p->ymax = yhigh;
			p->next = nullptr;
		}

		if (prev)
			prev->next = p;
		else
			m_Fields[idx] = p;
	}

	return true;
}

// X 0 X
// 3 X 1 
// X 2 X
static int vx_neighbour4x_safe(int idx, int nx, int nz, int dir) {
	int z = idx / nx;
	int x = idx - z * nx;
	switch (dir) {
	case 0:
		return z < nz - 1 ? idx + nx : -1;
	case 1:
		return x < nx - 1 ? idx + 1 : -1;
	case 2:
		return z > 0 ? idx - nx : -1;
	case 3:
		return x > 0 ? idx - 1 : -1;
	}
	return -1;
}


int SparseVoxelField::SolveSpatialTopology()
{
	int SpaceFound = 0;
	// std::map<int, int> SpaceCount;

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		Voxel* curr = m_Fields[idx];
		while (curr)
		{
			if (curr->data != 0)
			{
				curr = curr->next;
				continue;
			}

			curr->data = ++SpaceFound;

			std::queue<std::pair<Voxel*, int>> voxel_qu;
			voxel_qu.push(std::make_pair(curr, idx));
			int count = 0;

			while (!voxel_qu.empty())
			{
				auto next = voxel_qu.front();
				voxel_qu.pop();
				++count;

				for (int d = 0; d < 4; ++d)
				{
					int nidx = vx_neighbour4x_safe(next.second, m_SizeX, m_SizeZ, d);
					if (nidx == -1)
						continue;
					Voxel* nv = m_Fields[nidx];
					while (nv)
					{
						if (nv->data == 0 && VoxelIntersects(next.first, nv))
						{
							nv->data = next.first->data;
							voxel_qu.push(std::make_pair(nv, nidx));
						}
						nv = nv->next;
					}
				}
			}

			// SpaceCount[SpaceFound] = count;

			curr = curr->next;
		}
	}

	return SpaceFound;
}

int		SparseVoxelField::CalculateNumFields() const
{
	int Count = 0;
	for (int i = 0; i < m_SizeZ * m_SizeX; ++i)
	{
		if (m_Fields[i] != nullptr) Count++;
	}
	return Count;
}

unsigned long long SparseVoxelField::EstimateMemoryUseage() const
{
	return m_NumVoxels * sizeof(Voxel) + m_SizeX * m_SizeZ * sizeof(Voxel*);
}

unsigned long long SparseVoxelField::EstimateMemoryUseageEx() const
{
	return m_NumVoxels * sizeof(VoxelFast) + m_SizeX * m_SizeZ * sizeof(void*);
}

void	SparseVoxelField::GenerateHeightMap(std::vector<float>& bitmap) const
{
	bitmap.resize(m_SizeX * m_SizeZ);
	memset(&bitmap[0], 0, sizeof(bitmap[0]) * m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		Voxel* v = m_Fields[idx];
		while (v)
		{
			if (v->next == nullptr)
			{
				bitmap[idx] = v->ymax;
				break;
			}
			v = v->next;
		}
	}
}

void	SparseVoxelField::GenerateLevels(std::vector<int>& levels, int * level_max) const
{
	levels.resize(m_SizeX * m_SizeZ);
	memset(&levels[0], 0, sizeof(levels[0]) * m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		levels[idx] = CountVoxel(m_Fields[idx]);
		*level_max = std::max(*level_max, levels[idx]);
	}
}

void	SparseVoxelField::GenerateData(std::vector<int>& output, unsigned int data) const
{
	output.resize(m_SizeX * m_SizeZ);
	memset(&output[0], 0, sizeof(output[0]) * m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		Voxel* v = FindVoxel(m_Fields[idx], data);
		output[idx] = v ? (int)(v->ymax - v->ymin) : 0;
	}
}

void	SparseVoxelField::CalculateYLimit(float* ymin, float* ymax) const
{
	unsigned short smin = 0;
	unsigned short smax = 65535;

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		Voxel* v = m_Fields[idx];
		if (v == nullptr)
			continue;
		smin = std::min(smin, v->ymin);

		while (v)
		{
			if (v->next == nullptr)
			{
				smax = std::max(smax, v->ymax);
				break;
			}
			v = v->next;
		}
	}

	*ymin = m_BV.Min.y + smin * m_VoxelHeight;
	*ymax = m_BV.Min.y + (smax+1) * m_VoxelHeight;
}

bool	SparseVoxelField::SerializeTo(const char* filename)
{

	VoxelFileHeader	header;
	header.nFields = CalculateNumFields();
	header.nVoxels = m_NumVoxels;
	header.SizeX = m_SizeX;
	header.SizeY = m_SizeY;
	header.SizeZ = m_SizeZ;
	header.VoxelSize = m_VoxelSize;
	header.VoxelHeight = m_VoxelHeight;
	header.BV = m_BV;

	std::vector<VoxelFileField> buffer_field;
	buffer_field.resize(header.nFields);

	std::vector<VoxelFast> buffer_vx;
	buffer_vx.resize(header.nVoxels);

	VoxelFileField* pf = &buffer_field[0];
	VoxelFast* pvx = &buffer_vx[0];

	int nFields = 0;
	int nVoxels = 0;
	for (int i = 0; i < m_SizeZ * m_SizeX; ++i)
	{
		Voxel* v = m_Fields[i];
		if (v == nullptr)
		{
			continue;
		}

		if (nFields == header.nFields)
		{
			assert(false);
			return false;
		}

		pf->idx = i;
		while (v)
		{
			if (nVoxels == m_NumVoxels)
			{
				assert(false);
				return false;
			}

			pvx->data = v->data;
			if (v->next == nullptr)
			{
				pvx->data |= VOXEL_TOP;
			}
			pvx->ymin = v->ymin;
			pvx->ymax = v->ymax;
			++pvx;
			++nVoxels;

			v = v->next;
		}

		++pf;
		++nFields;
	}

	if (nFields != header.nFields)
	{
		assert(false);
		return false;
	}

	if (nVoxels != header.nVoxels)
	{
		assert(false);
		return false;
	}

	FILE* fp = fopen(filename, "wb");
	if (!fp)
	{
		return false;
	}

	unsigned int Magic = VOXEL_FILE_MAGIC;
	fwrite(&Magic, sizeof(Magic), 1, fp);
	fwrite(&header, sizeof(header), 1, fp);
	fwrite(&buffer_vx[0], sizeof(VoxelFast), buffer_vx.size(), fp);
	fwrite(&buffer_field[0], sizeof(VoxelFileField), buffer_field.size(), fp);
	fclose(fp);

	return true;
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

	InitField(header.BV, header.SizeX, header.SizeY, header.SizeZ, header.VoxelSize, header.VoxelHeight);
	
	m_NumVoxels = header.nVoxels;

	m_VoxelBatchs.resize(1);
	m_VoxelBatchs[0].Current = header.nVoxels;
	m_VoxelBatchs[0].Voxels.resize(header.nVoxels);
	std::vector<Voxel>& Voxels = m_VoxelBatchs[0].Voxels;

	{
		std::vector<VoxelFast> buffer_vx;
		buffer_vx.resize(header.nVoxels);
		fread(&buffer_vx[0], sizeof(VoxelFast), buffer_vx.size(), fp);
		for (int i = 0; i < header.nVoxels; ++i)
		{
			Voxel* vx = &Voxels[i];
			const VoxelFast* vf = &buffer_vx[i];
			vx->data = VOXEL_DATA(vf->data);
			vx->ymax = vf->ymax;
			vx->ymin = vf->ymin;
			vx->next = (vf->data & VOXEL_TOP) ? nullptr : &Voxels[i+1];
		}
		buffer_vx.clear();
	}

	{
		std::vector<VoxelFileField> buffer_field;
		buffer_field.resize(header.nFields);
		fread(&buffer_field[0], sizeof(VoxelFileField), buffer_field.size(), fp);

		size_t curr = 0;
		for (int i = 0; i < header.nFields; ++i)
		{
			if (curr >= Voxels.size())
			{
				assert(false);
				return false;
			}

			int idx = buffer_field[i].idx;
			m_Fields[idx] = &Voxels[curr];
			curr += (size_t)CountVoxel(m_Fields[idx]);
		}
		buffer_field.clear();
	}

	return true;
}
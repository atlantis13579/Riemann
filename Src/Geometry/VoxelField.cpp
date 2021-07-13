
#include "VoxelField.h"

#include <assert.h>
#include <queue>
#include "../Maths/Maths.h"
#include "../CollisionPrimitive/TriangleMesh.h"

VoxelField::VoxelField()
{
	m_SizeX = 0;
	m_SizeZ = 0;
	m_SizeY = 0;
	m_NumVoxels = 0;
	m_FreeVoxelList = nullptr;
}

VoxelField::~VoxelField()
{

}

void VoxelField::InitField(int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
{
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

bool VoxelField::VoxelizeTriangles(const VoxelizationInfo& info, TriangleMesh* _mesh)
{
	m_WorldBox = info.Boundry;

	m_SizeX = (int)(info.Boundry.GetSizeX() / info.VoxelSize + 0.5f);
	m_SizeY = (int)(info.Boundry.GetSizeY() / info.VoxelHeight + 0.5f);
	m_SizeZ = (int)(info.Boundry.GetSizeZ() / info.VoxelSize + 0.5f);

	InitField(m_SizeX, m_SizeY, m_SizeZ, info.VoxelSize, info.VoxelHeight);

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

bool VoxelField::AddVoxel(int x, int y, unsigned short ymin, unsigned short ymax, float MergeThr)
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
		else if (p->ymax < s->ymin)
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

int		VoxelField::GetVoxelIdx(const Vector3d& pos) const
{
	const int x = Clamp((int)((pos.x - m_WorldBox.Min.x) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z = Clamp((int)((pos.z - m_WorldBox.Min.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}

float	VoxelField::GetVoxelY(unsigned short y) const
{
	return m_WorldBox.Min.y + (y + 0.5f) * m_VoxelHeight;
}

Voxel*	VoxelField::AllocVoxel()
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

void VoxelField::FreeVoxel(Voxel* p)
{
	m_NumVoxels--;

	p->next = m_FreeVoxelList;
	m_FreeVoxelList = p;
}

static bool VoxelIntersects(const Voxel* v1, const Voxel *v2)
{
	return v2->ymin < v1->ymax && v2->ymax > v1->ymin;
}

static bool VoxelIntersects2(const Voxel* v1, const Voxel* v2, float Thr)
{
	return v2->ymin < v1->ymax - Thr && v2->ymax > v1->ymin + Thr;
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
	int Data = -1;
	while (v)
	{
		Data = std::max(Data, v->data);
		v = v->next;
	}
	return Data;
}


bool VoxelField::VoxelizeTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info)
{
	const float dy = m_WorldBox.Max.y - m_WorldBox.Min.y;
	const float ics = 1.0f / m_VoxelSize;
	const float ich = 1.0f / m_VoxelHeight;

	Box3d tbox({ v0, v1, v2 });

	if (!tbox.Intersect(m_WorldBox))
		return true;

	const int z0 = Clamp((int)((tbox.Min.z - m_WorldBox.Min.z) * ics), 0, m_SizeZ - 1);
	const int z1 = Clamp((int)((tbox.Max.z - m_WorldBox.Min.z) * ics), 0, m_SizeZ - 1);

	Vector3d buf[7 * 4];
	Vector3d *In = buf, *inrow = buf + 7, *p1 = buf + 7 * 2, *p2 = buf + 7 * 3;

	In[0] = v0;
	In[1] = v1;
	In[2] = v2;

	int nvrow, nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		const float cz = m_WorldBox.Min.z + z * m_VoxelSize;
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
		const int x0 = Clamp((int)((minX - m_WorldBox.Min.x) * ics), 0, m_SizeX - 1);
		const int x1 = Clamp((int)((maxX - m_WorldBox.Min.x) * ics), 0, m_SizeX - 1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			const float cx = m_WorldBox.Min.x + x * m_VoxelSize;
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

			if (ymax < m_WorldBox.Min.y || ymin > m_WorldBox.Max.y)
				continue;

			if (ymin < m_WorldBox.Min.y)
				ymin = m_WorldBox.Min.y;
			if (ymax > m_WorldBox.Max.y)
				ymax = m_WorldBox.Max.y;

			const unsigned short y0 = Clamp((unsigned short)((ymin - m_WorldBox.Min.y) * ich), (unsigned short)0, (unsigned short)(m_SizeY - 1));
			const unsigned short y1 = Clamp((unsigned short)((ymax - m_WorldBox.Min.y) * ich), (unsigned short)0, (unsigned short)(m_SizeY - 1));

			if (!AddVoxel(x, z, y0, y1, info.YMergeThr))
				return false;
		}
	}

	return true;
}

bool VoxelField::MakeComplement()
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
			ylow = p->ymax;
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


int VoxelField::SolveSpatialTopology()
{
	int SpaceFound = 0;

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

			curr = curr->next;
		}
	}

	return SpaceFound;
}

void	VoxelField::GenerateHeightMap(std::vector<float>& bitmap) const
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

void	VoxelField::GenerateLevels(std::vector<int>& levels, int * level_max) const
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

void	VoxelField::CalculateYLimit(float* ymin, float* ymax) const
{
	*ymin = FLT_MAX;
	*ymax = -FLT_MAX;

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
}

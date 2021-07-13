
#include "VoxelField.h"

#include <assert.h>
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

bool VoxelField::VoxelizeTriangles(const VoxelizationInfo& info, TriangleMesh* _mesh)
{
	m_WorldBox = info.Boundry;

	m_SizeX = (int)(info.Boundry.GetSizeX() / info.VoxelSize + 0.5f);
	m_SizeZ = (int)(info.Boundry.GetSizeZ() / info.VoxelSize + 0.5f);
	m_Fields.resize(m_SizeX * m_SizeZ);
	memset(&m_Fields[0], 0, sizeof(m_Fields[0]) * m_SizeX * m_SizeZ);

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

bool VoxelField::AddVoxel(int x, int y, int smin, int smax, float MergeThr)
{
	int idx = x + y * m_SizeX;

	Voxel* s = AllocVoxel();
	if (s == nullptr)
	{
		return false;
	}
	s->low = smin;
	s->high = smax;
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
		if (p->low > s->high)
		{
			break;
		}
		else if (p->high < s->low)
		{
			prev = p;
			p = p->next;
		}
		else
		{
			if (p->low < s->low)
				s->low = p->low;
			if (p->high > s->high)
				s->high = p->high;

			// if (abs(s->max - p->max) <= (int)MergeThr)
			//	;

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


Voxel* VoxelField::AllocVoxel()
{
	m_NumVoxels++;

	if (m_FreeVoxelList)
	{
		Voxel* p = m_FreeVoxelList;
		m_FreeVoxelList = m_FreeVoxelList->next;
		return p;
	}

	int kVoxelBatchSize = m_SizeX * m_SizeZ / 2;
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

bool VoxelField::VoxelizeTri(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const VoxelizationInfo& info)
{
	const float dy = info.Boundry.Max.y - info.Boundry.Min.y;
	const float ics = 1.0f / info.VoxelSize;
	const float ich = 1.0f / info.VoxelHeight;

	Box3d tbox({ v0, v1, v2 });

	if (!tbox.Intersect(info.Boundry))
		return true;

	const int z0 = Clamp((int)((tbox.Min.z - info.Boundry.Min.z) * ics), 0, m_SizeZ - 1);
	const int z1 = Clamp((int)((tbox.Max.z - info.Boundry.Min.z) * ics), 0, m_SizeZ - 1);

	Vector3d buf[7 * 4];
	Vector3d *In = buf, *inrow = buf + 7, *p1 = buf + 7 * 2, *p2 = buf + 7 * 3;

	In[0] = v0;
	In[1] = v1;
	In[2] = v2;

	int nvrow, nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		const float cz = info.Boundry.Min.z + z * info.VoxelSize;
		dividePoly(In, nvIn, inrow, &nvrow, p1, &nvIn, cz + info.VoxelSize, 2);
		std::swap(In, p1);
		if (nvrow < 3)
			continue;

		float minX = inrow[0].x, maxX = inrow[0].x;
		for (int i = 1; i < nvrow; ++i)
		{
			minX = std::min(minX, inrow[i * 3].x);
			maxX = std::max(maxX, inrow[i * 3].x);
		}
		const int x0 = Clamp((int)((minX - info.Boundry.Min.x) * ics), 0, m_SizeX - 1);
		const int x1 = Clamp((int)((maxX - info.Boundry.Min.x) * ics), 0, m_SizeX - 1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			const float cx = info.Boundry.Min.x + x * info.VoxelSize;
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx + info.VoxelSize, 0);
			std::swap(inrow, p2);
			if (nv < 3)
				continue;

			float smin = p1[1].y, smax = p1[1].y;
			for (int i = 1; i < nv; ++i)
			{
				smin = std::min(smin, p1[i].y);
				smax = std::max(smax, p1[i].y);
			}
			smin -= info.Boundry.Min.y;
			smax -= info.Boundry.Min.y;

			if (smax < 0.0f || smin > dy)
				continue;

			if (smin < 0.0f)
				smin = 0;
			if (smax > dy)
				smax = dy;

			int ismin = (int)floorf(smin * ich);
			int ismax = (int)ceilf(smax * ich);
			if (ismin < 0)
				ismin = 0;
			if (ismax <= ismin)
				ismax = ismin + 1;

			if (!AddVoxel(x, z, ismin, ismax, info.YMergeThr))
				return false;
		}
	}

	return true;
}

void VoxelField::GenerateHeightMap(std::vector<int>& bitmap)
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
				bitmap[idx] = v->high;
				break;
			}
			v = v->next;
		}
	}
}

void VoxelField::CalculateYLimit(int* ymin, int* ymax)
{
	*ymin = INT_MAX;
	*ymax = -INT_MAX;

	for (int i = 0; i < m_SizeZ; ++i)
	for (int j = 0; j < m_SizeX; ++j)
	{
		int idx = i * m_SizeX + j;
		Voxel* v = m_Fields[idx];
		if (v == nullptr)
			continue;
		*ymin = std::min(*ymin, v->low);

		while (v)
		{
			if (v->next == nullptr)
			{
				*ymax = std::max(*ymax, v->high);
				break;
			}
			v = v->next;
		}
	}
}


#include <assert.h>
#include <queue>

#include "VoxelField.h"
#include "../Maths/Maths.h"
#include "../CollisionPrimitive/Mesh.h"

VoxelField::VoxelField()
{
	m_SizeX = 0;
	m_SizeY = 0;
	m_SizeZ = 0;
}

VoxelField::~VoxelField()
{
	m_Fields.clear();
}

template<typename T>
T vx_clamp(const T X, const T Min, const T Max)
{
	return X < Min ? Min : X < Max ? X : Max;
}

void VoxelField::MakeEmpty(const Box3d &Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
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

	int kVoxelBatchSize = vx_clamp(m_SizeX * m_SizeZ / 2, 1024, 1024 * 1024);
	m_VoxelBatchs.Init(1, kVoxelBatchSize);
}


bool VoxelField::VoxelizationTrianglesSet(const VoxelizationInfo& info, Mesh* _mesh)
{
	m_SizeX = (int)(info.BV.GetLengthX() / info.VoxelSize + 0.5f);
	m_SizeY = (int)(info.BV.GetLengthY() / info.VoxelHeight + 0.5f);
	m_SizeZ = (int)(info.BV.GetLengthZ() / info.VoxelSize + 0.5f);

	MakeEmpty(info.BV, m_SizeX, m_SizeY, m_SizeZ, info.VoxelSize, info.VoxelHeight);

	const Mesh &mesh = *_mesh;
	for (uint32_t i = 0; i < mesh.GetNumTriangles(); ++i)
	{
		Vector3 v0 = mesh(i, 0);
		Vector3 v1 = mesh(i, 1);
		Vector3 v2 = mesh(i, 2);
		if (!VoxelizationTri(v0, v1, v2, info))
		{
			return false;
		}
	}
	return true;
}


bool VoxelField::IntersectYPlane(float y_value, std::vector<int>& output, float Thr)
{
	output.resize(m_SizeX * m_SizeZ);
	for (int i = 0; i < m_SizeZ * m_SizeX; ++i)
	{
		Voxel* v = GetVoxelByY(m_Fields[i], y_value, Thr);
		output[i] = v ? v->data : 0;
	}

	return true;
}

Voxel* VoxelField::GetVoxelByY(Voxel* Base, float y, float Thr)
{
	Voxel* p = Base;
	Voxel* prev = nullptr;
	while (p)
	{
		float ymin = m_BV.mMin.y + m_VoxelHeight * p->ymin;
		float ymax = m_BV.mMin.y + m_VoxelHeight * (p->ymax + 1);
		if (ymin <= y && y <= ymax)
		{
			return p;
		}
		else if (y < ymin)
		{
			if (ymin - y < Thr)
			{
				return p;
			}
			break;
		}
		prev = p;
		p = p->next;
	}
	if (prev)
	{
		float ymax = m_BV.mMin.y + m_VoxelHeight * (prev->ymax + 1);
		if (y - ymax < Thr)
		{
			return prev;
		}
	}
	return nullptr;
}

Voxel*	VoxelField::GetVoxel(const Vector3& pos)
{
	int idx = WorldSpaceToVoxelIndex(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return nullptr;
	return GetVoxelByY(m_Fields[idx], pos.y, 1e-3f);
}

Voxel*	VoxelField::GetVoxel(int idx)
{
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return nullptr;
	return m_Fields[idx];
}

std::string VoxelField::DebugString(int idx) const
{
	std::string str = "";
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return str;
	Voxel* v = m_Fields[idx];
	while (v)
	{
		str += "[" + std::to_string(v->ymin) + "," + std::to_string(v->ymax) + "]";
		v = v->next;
	}
	return str;
}


int		VoxelField::WorldSpaceToVoxelIndex(const Vector3& pos) const
{
	const int x = vx_clamp((int)((pos.x - m_BV.mMin.x) * m_InvVoxelSize), 0, m_SizeX - 1);
	const int z = vx_clamp((int)((pos.z - m_BV.mMin.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}

int		VoxelField::WorldSpaceToVoxelSpaceY(float pos_y) const
{
	const int y = vx_clamp((int)((pos_y - m_BV.mMin.y) * m_InvVoxelHeight), 0, m_SizeY - 1);
	return y;
}

float	VoxelField::VoxelSpaceToWorldSpaceY(uint16_t y) const
{
	return m_BV.mMin.y + (y + 0.5f) * m_VoxelHeight;
}

Box3d	VoxelField::GetVoxelBox(const Vector3& pos) const
{
	int idx = WorldSpaceToVoxelIndex(pos);
	int z = idx / m_SizeX;
	int x = idx - z * m_SizeX;
	int y = WorldSpaceToVoxelSpaceY(pos.y);
	return GetVoxelBox(x, y, z);
}

Box3d	VoxelField::GetVoxelBox(int x, int y, int z) const
{
	if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ || y < 0 || y >= m_SizeY)
	{
		// Error
		return Box3d::Unit();
	}
	Box3d box;
	box.mMin.x = m_BV.mMin.x + m_VoxelSize * x;
	box.mMin.y = m_BV.mMin.y + m_VoxelHeight * y;
	box.mMin.z = m_BV.mMin.z + m_VoxelSize * z;
	box.mMax = box.mMin + Vector3(m_VoxelSize, m_VoxelHeight, m_VoxelSize);
	return box;
}



uint32_t	VoxelField::GetVoxelData(const Vector3& pos)
{
	int idx = WorldSpaceToVoxelIndex(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	Voxel* v = GetVoxelByY(m_Fields[idx], pos.y, 0.0f);
	return v ? v->data : 0;
}

bool VoxelField::AddVoxel(int idx, uint16_t ymin, uint16_t ymax, float MergeThr)
{
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

Voxel*	VoxelField::AllocVoxel()
{
	return m_VoxelBatchs.Alloc();
}

void VoxelField::FreeVoxel(Voxel* p)
{
	m_VoxelBatchs.Free(p);
}

static bool VoxelIntersects(const Voxel* v1, const Voxel* v2, uint16_t Thr)
{
	return v2->ymin <= v1->ymax - Thr && v2->ymax >= v1->ymin + Thr;
}

static Voxel* FindVoxel(Voxel* v, uint32_t data)
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


static void InterpolateTri(const Vector3* In, int nn, Vector3* out1, int* nout1, Vector3* out2, int* nout2, float x, int axis)
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
			float slope = d[j] / (d[j] - d[i]);
			out1[m] = In[j] + (In[i] - In[j]) * slope;
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

bool VoxelField::VoxelizationTri(const Vector3& v0, const Vector3& v1, const Vector3& v2, const VoxelizationInfo& info)
{
	// const float dy = m_BV.Max.y - m_BV.Min.y;
	const float ics = 1.0f / m_VoxelSize;
	const float ich = 1.0f / m_VoxelHeight;

	Box3d tbox(v0, v0);
	tbox.Encapsulate(v1, v2);

	if (!tbox.Intersect(m_BV))
		return true;

	const int z0 = vx_clamp((int)((tbox.mMin.z - m_BV.mMin.z) * ics), 0, m_SizeZ - 1);
	const int z1 = vx_clamp((int)((tbox.mMax.z - m_BV.mMin.z) * ics), 0, m_SizeZ - 1);

	Vector3 buf[7 * 4];
	Vector3 *In = buf, *inrow = buf + 7, *p1 = buf + 7 * 2, *p2 = buf + 7 * 3;

	In[0] = v0;
	In[1] = v1;
	In[2] = v2;

	int nvrow, nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		const float cz = m_BV.mMin.z + z * m_VoxelSize;
		InterpolateTri(In, nvIn, inrow, &nvrow, p1, &nvIn, cz + m_VoxelSize, 2);
		std::swap(In, p1);
		if (nvrow < 3)
			continue;

		float minX = inrow[0].x, maxX = inrow[0].x;
		for (int i = 1; i < nvrow; ++i)
		{
			minX = std::min(minX, inrow[i].x);
			maxX = std::max(maxX, inrow[i].x);
		}
		const int x0 = vx_clamp((int)((minX - m_BV.mMin.x) * ics), 0, m_SizeX - 1);
		const int x1 = vx_clamp((int)((maxX - m_BV.mMin.x) * ics), 0, m_SizeX - 1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			const float cx = m_BV.mMin.x + x * m_VoxelSize;
			InterpolateTri(inrow, nv2, p1, &nv, p2, &nv2, cx + m_VoxelSize, 0);
			std::swap(inrow, p2);
			if (nv < 3)
				continue;

			float ymin = p1[0].y, ymax = p1[0].y;
			for (int i = 1; i < nv; ++i)
			{
				ymin = std::min(ymin, p1[i].y);
				ymax = std::max(ymax, p1[i].y);
			}

			if (ymax < m_BV.mMin.y || ymin > m_BV.mMax.y)
				continue;

			if (ymin < m_BV.mMin.y)
				ymin = m_BV.mMin.y;
			if (ymax > m_BV.mMax.y)
				ymax = m_BV.mMax.y;
			ymin -= m_BV.mMin.y;
			ymax -= m_BV.mMin.y;

			const uint16_t y0 = (uint16_t)vx_clamp((int)(ymin * ich), 0, m_SizeY - 1);
			const uint16_t y1 = (uint16_t)vx_clamp((int)(ymax * ich), 0, m_SizeY - 1);

			if (!AddVoxel(z * m_SizeX + x, y0, y1, info.YMergeThr))
				return false;
		}
	}

	return true;
}

bool VoxelField::VoxelizationYPlane(float world_y)
{
	return VoxelizationYPlane(world_y, world_y);
}

bool VoxelField::VoxelizationYPlane(float world_y_min, float world_y_max)
{
	uint16_t ymin = (uint16_t)WorldSpaceToVoxelSpaceY(world_y_min);
	uint16_t ymax = (uint16_t)WorldSpaceToVoxelSpaceY(world_y_max);
	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		AddVoxel(i, ymin, ymax, 0.0f);
	}
	return true;
}

bool VoxelField::VoxelizationCube(const Box3d& cube)
{
	const int x0 = vx_clamp((int)((cube.mMin.x - m_BV.mMin.x) * m_InvVoxelSize), 0, m_SizeX - 1);
	const int x1 = vx_clamp((int)((cube.mMax.x - m_BV.mMin.x) * m_InvVoxelSize), 0, m_SizeX - 1);
	const int z0 = vx_clamp((int)((cube.mMin.z - m_BV.mMin.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z1 = vx_clamp((int)((cube.mMax.z - m_BV.mMin.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const uint16_t y0 = (uint16_t)WorldSpaceToVoxelSpaceY(cube.mMin.y);
	const uint16_t y1 = (uint16_t)WorldSpaceToVoxelSpaceY(cube.mMax.y);
	for (int z = z0; z <= z1; ++z)
	for (int x = x0; x <= x1; ++x)
	{
		AddVoxel(z * m_SizeX + x, y0, y1, 0.0f);
	}
	return true;
}

bool VoxelField::MakeComplementarySet()
{
	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		Voxel* old = m_Fields[i];

		m_Fields[i] = nullptr;

		Voxel* p = old;

		uint16_t first_y = p ? p->ymin : m_SizeY;
		if (first_y > 0)
		{
			AddVoxel(i, 0, first_y - 1, 0.0f);
		}

		while (p)
		{
			if (p->next)
			{
				AddVoxel(i, p->ymax + 1, p->next->ymin - 1, 0.0f);
			}
			else if (p->ymax < m_SizeY - 1)
			{
				AddVoxel(i, p->ymax + 1, m_SizeY - 1, 0.0f);
			}
			p = p->next;
		}

		p = old;
		while (p)
		{
			Voxel* next = p->next;
			FreeVoxel(p);
			p = next;
		}
	}

	return true;
}

int			VoxelField::Filter(std::function<bool(Voxel* v)> func)
{
	int Count = 0;
	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		Voxel* p = m_Fields[i], * prev = nullptr;
		while (p)
		{
			while (func(p))
			{
				++Count;
				if (prev)
					prev->next = p->next;
				else
					m_Fields[i] = p->next;
				if (p->next == nullptr)
					goto end_while;
				Voxel* next = p->next;
				FreeVoxel(p);
				p = next;
			}

			prev = p;
			p = p->next;
		}

end_while:
		;
	}
	return Count;
}

int			VoxelField::FilterByY(float world_y)
{
	const uint16_t ny = (uint16_t)WorldSpaceToVoxelSpaceY(world_y);
	return Filter([=](Voxel* v) -> bool
		{
			return v->ymin <= ny;
		});
}

int			VoxelField::FilterByData(uint32_t data)
{
	return Filter([=](Voxel* v) -> bool
		{
			return v->data != data;
		});
}

void		VoxelField::FilterTopNByVolume(const std::unordered_map<int, uint64_t>& volumes, int TopN)
{
	if ((int)volumes.size() > TopN)
	{
		std::vector<uint64_t> volume_list;
		for (auto it : volumes)
		{
			volume_list.push_back(it.second);
		}
		std::sort(volume_list.begin(), volume_list.end());

		uint64_t area_thr = volume_list[volume_list.size() - TopN];
		auto filter_func = [&volumes, area_thr](Voxel *v)
		{
			auto it = volumes.find(v->data);
			if (it != volumes.end() && it->second <= area_thr)
			{
				return true;
			}
			return false;
		};
		Filter(filter_func);
	}
}

void	VoxelField::TraversalVoxel(std::function<bool(Voxel* v)> callback)
{
	for (auto v : m_Fields)
	{
		while (v)
		{
			if (!callback(v))
			{
				return;
			}
			v = v->next;
		}
	}
}

void	VoxelField::TraversalField(std::function<void(int idx, Voxel* v)> callback)
{
	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		if (m_Fields[i])
		{
			callback(i, m_Fields[i]);
		}
	}
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

uint64_t	VoxelField::Separate(const Vector3& pos, uint32_t data, float IntersectThr)
{
	int idx = WorldSpaceToVoxelIndex(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	Voxel* p = GetVoxelByY(m_Fields[idx], pos.y, 0.0f);
	if (p == nullptr)
	{
		return 0;
	}
	uint16_t Thr = (uint16_t)ceilf(IntersectThr * m_InvVoxelHeight);
	return SeparateImpl(idx, p, data, Thr);
}


uint64_t	VoxelField::SeparateImpl(int idx, Voxel* base, uint32_t data, uint16_t Thr)
{
	base->data = data;

	std::queue<std::pair<Voxel*, int>> queue_vx;
	queue_vx.emplace(base, idx);
	uint64_t voxel_count = 0;

	while (!queue_vx.empty())
	{
		auto next = queue_vx.front();
		queue_vx.pop();
		voxel_count += next.first->ymax - next.first->ymin + 1;
		
		for (int d = 0; d < 4; ++d)
		{
			int nidx = vx_neighbour4x_safe(next.second, m_SizeX, m_SizeZ, d);
			if (nidx == -1)
				continue;
			Voxel* nv = m_Fields[nidx];
			while (nv)
			{
				if (nv->data == 0 && VoxelIntersects(next.first, nv, Thr))
				{
					nv->data = next.first->data;
					queue_vx.emplace(nv, nidx);
				}
				nv = nv->next;
			}
		}
	}
	return voxel_count;
}

int		VoxelField::SolveTopology(float IntersectThr, std::unordered_map<int, uint64_t>* volumes)
{
	uint16_t Thr = (uint16_t)std::max(1, (int)floorf(IntersectThr * m_InvVoxelHeight));
	int SpaceFound = 0;
	if (volumes) volumes->clear();

	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		Voxel* curr = m_Fields[i];
		while (curr)
		{
			if (curr->data == 0)
			{
				uint64_t voxel_count = SeparateImpl(i, curr, ++SpaceFound, Thr);
				if (volumes) volumes->emplace(SpaceFound, voxel_count);
			}

			curr = curr->next;
		}
	}

	return SpaceFound;
}

int		VoxelField::CalculateNumFields() const
{
	int Count = 0;
	for (int i = 0; i < m_SizeZ * m_SizeX; ++i)
	{
		if (m_Fields[i] != nullptr) Count++;
	}
	return Count;
}

uint64_t VoxelField::EstimateMemoryUseage() const
{
	return m_VoxelBatchs.GetCount() * sizeof(Voxel) + m_SizeX * m_SizeZ * sizeof(Voxel*);
}

uint64_t VoxelField::EstimateMemoryUseageEx() const
{
	return  m_VoxelBatchs.GetCount() * sizeof(VoxelFast) + m_SizeX * m_SizeZ * sizeof(uint32_t);
}

void	VoxelField::GenerateHeightMap(std::vector<float>& heightmap) const
{
	heightmap.resize(m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		heightmap[i] = m_BV.mMin.y;

		Voxel* v = m_Fields[i];
		while (v)
		{
			if (v->next == nullptr)
			{
				heightmap[i] = m_BV.mMin.y + (v->ymax + 1) * m_VoxelHeight;
				break;
			}
			v = v->next;
		}
	}
}

void	VoxelField::GenerateBitmapByLevel(std::vector<int>& levels, int * level_max) const
{
	levels.resize(m_SizeX * m_SizeZ);
	memset(&levels[0], 0, sizeof(levels[0]) * m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		levels[i] = CountVoxel(m_Fields[i]);
		*level_max = std::max(*level_max, levels[i]);
	}
}

void	VoxelField::GenerateBitmapByData(std::vector<int>& output, uint32_t data) const
{
	output.resize(m_SizeX * m_SizeZ);
	memset(&output[0], 0, sizeof(output[0]) * m_SizeX * m_SizeZ);

	for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
	{
		Voxel* v = FindVoxel(m_Fields[i], data);
		// output[i] = v ? (int)(v->ymax - v->ymin) : 0;
		output[i] = v ? 1 : 0;
	}
}

Mesh* VoxelField::CreateDebugMesh(int x1, int x2, int z1, int z2) const
{
	Mesh* mesh = new Mesh;

	z1 = std::max(z1, 0);
	z2 = std::min(z2, m_SizeZ - 1);
	x1 = std::max(x1, 0);
	x2 = std::min(x2, m_SizeX - 1);

	for (int i = z1; i <= z2; ++i)
	{
		Vector3 Bmin, Bmax;
		Bmin.z = m_BV.mMin.z + m_VoxelSize * i;
		Bmax.z = m_BV.mMin.z + m_VoxelSize * (i + 1);

		for (int j = x1; j <= x2; ++j)
		{
			Voxel* v = m_Fields[i * m_SizeX + j];

			Bmin.x = m_BV.mMin.x + m_VoxelSize * j;
			Bmax.x = m_BV.mMin.x + m_VoxelSize * (j+1);
			while (v)
			{
				Bmin.y = m_BV.mMin.y + m_VoxelHeight * v->ymin;
				Bmax.y = m_BV.mMin.y + m_VoxelHeight * (v->ymax + 1);
				mesh->AddAABB(Bmin, Bmax);

				v = v->next;
			}
		}
	}
	return mesh;
}

bool	VoxelField::Verify()
{
	TraversalVoxel([](Voxel* v) -> bool
		{
			if (v->ymin > v->ymax)
			{
				return false;
			}

			if (v->next && v->ymax >= v->next->ymin)
			{
				return false;
			}

			return true;
		});
	return true;
}

bool	VoxelField::SerializeTo(const char* filename)
{

	VoxelFileHeader	header;
	header.nFields = CalculateNumFields();
	header.nVoxels = m_VoxelBatchs.GetCount();
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
			if (nVoxels == m_VoxelBatchs.GetCount())
			{
				assert(false);
				return false;
			}

			pvx->data = v->data;
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

	uint32_t Magic = VOXEL_FILE_MAGIC;
	fwrite(&Magic, sizeof(Magic), 1, fp);
	fwrite(&header, sizeof(header), 1, fp);
	fwrite(&buffer_vx[0], sizeof(VoxelFast), buffer_vx.size(), fp);
	fwrite(&buffer_field[0], sizeof(VoxelFileField), buffer_field.size(), fp);
	fclose(fp);

	return true;
}

bool	VoxelField::SerializeFrom(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	if (!fp)
	{
		return false;
	}

	fseek(fp, 0, SEEK_END);
	size_t fileSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

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

	MakeEmpty(header.BV, header.SizeX, header.SizeY, header.SizeZ, header.VoxelSize, header.VoxelHeight);
	
	uint32_t NumVoxels = header.nVoxels;

	m_VoxelBatchs.Init(0, NumVoxels);
	Voxel* Voxels = m_VoxelBatchs.AllocOneBatch();

	{
		std::vector<VoxelFast> buffer_vx;
		buffer_vx.resize(header.nVoxels);
		fread(&buffer_vx[0], sizeof(VoxelFast), buffer_vx.size(), fp);
		for (uint32_t i = 0; i < header.nVoxels; ++i)
		{
			Voxel* vx = &Voxels[i];
			const VoxelFast* vf = &buffer_vx[i];
			vx->data = vf->data;
			vx->ymax = vf->ymax;
			vx->ymin = vf->ymin;
			// WARNING: vx->next is junk
		}
		buffer_vx.clear();
	}

	{
		std::vector<VoxelFileField> buffer_field;
		buffer_field.resize(header.nFields);
		fread(&buffer_field[0], sizeof(VoxelFileField), buffer_field.size(), fp);

		uint32_t curr = 0;
		for (uint32_t i = 0; i < header.nFields; ++i)
		{
			if (curr >= NumVoxels)
			{
				assert(false);
				return false;
			}

			int idx = buffer_field[i].idx;
			m_Fields[idx] = &Voxels[curr];
			int Count = (i < header.nFields - 1) ? (buffer_field[i + 1].idx - buffer_field[i].idx) : (NumVoxels - buffer_field[i + 1].idx);
			for (int j = 0; j < Count - 1; ++j)
			{
				Voxels[curr + j].next = &Voxels[curr + j + 1];
			}
			Voxels[curr + Count - 1].next = nullptr;
		}
		buffer_field.clear();
	}

	return true;
}


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


int		SparseVoxelField::GetVoxelIdx(const Vector3& pos) const
{
	const int x = Clamp((int)((pos.x - m_BV.mMin.x) * m_InvVoxelSize), 0, m_SizeZ - 1);
	const int z = Clamp((int)((pos.z - m_BV.mMin.z) * m_InvVoxelSize), 0, m_SizeZ - 1);
	return z * m_SizeX + x;
}


int		SparseVoxelField::VoxelSpaceToWorldSpaceY(float pos_y) const
{
	const int y = Clamp((int)((pos_y - m_BV.mMin.y) * m_InvVoxelHeight), 0, m_SizeY - 1);
	return y;
}

float	SparseVoxelField::VoxelSpaceToWorldSpaceY(uint16_t y) const
{
	return m_BV.mMin.y + (y + 0.5f) * m_VoxelHeight;
}


uint32_t	SparseVoxelField::WorldSpaceToVoxelSpaceY(const Vector3& pos) const
{
	int idx = GetVoxelIdx(pos);
	if (idx < 0 || idx >= m_SizeX * m_SizeZ)
		return 0;
	uint32_t i0 = m_Indices[idx];
	uint32_t i1 = i0 + GetVoxelCount(idx);
	for (uint32_t i = i0; i < i1; ++i)
	{
		const VoxelFast* v = &m_Voxels[i];
		float ymin = m_BV.mMin.y + m_VoxelHeight * v->ymin;
		float ymax = m_BV.mMin.y + m_VoxelHeight * (v->ymax + 1);
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

	fseek(fp, 0, SEEK_END);
	size_t fileSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

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

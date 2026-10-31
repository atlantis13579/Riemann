
#include <assert.h>
#include <cmath>
#include <limits>
#include <queue>

#include "VoxelField.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "../Core/File.h"
#include "../Maths/Maths.h"

namespace Riemann
{
	static bool VoxelFinite(const Vector3& V)
	{
		return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
	}

	static bool VoxelDimensionsValid(const Box3& Bv, int SizeX, int SizeY, int SizeZ, float Size, float Height)
	{
		if (SizeX <= 0 || SizeZ <= 0 || SizeY <= 0 || SizeY > 65536 ||
			int64_t(SizeX) * SizeZ > std::numeric_limits<int>::max() ||
			!std::isfinite(Size) || !std::isfinite(Height) || Size <= 0.0f || Height <= 0.0f ||
			!std::isfinite(1.0f / Size) || !std::isfinite(1.0f / Height) ||
			!VoxelFinite(Bv.Min) || !VoxelFinite(Bv.Max) ||
			Bv.Min.x > Bv.Max.x || Bv.Min.y > Bv.Max.y || Bv.Min.z > Bv.Max.z)
		{
			return false;
		}
		return VoxelFinite(Bv.Min + Vector3(SizeX * Size, SizeY * Height, SizeZ * Size));
	}

	VoxelField::VoxelField()
	{
		m_SizeX = 0;
		m_SizeY = 0;
		m_SizeZ = 0;
		m_VoxelSize = m_VoxelHeight = 0.0f;
		m_InvVoxelSize = m_InvVoxelHeight = 0.0f;
		m_BV = Box3(Vector3::Zero(), Vector3::Zero());
		m_VoxelBatchs.init(0, 1024);
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

	void VoxelField::MakeEmpty(const Box3 &Bv, int SizeX, int SizeY, int SizeZ, float VoxelSize, float VoxelHeight)
	{
		if (!VoxelDimensionsValid(Bv, SizeX, SizeY, SizeZ, VoxelSize, VoxelHeight))
		{
			VoxelField Empty;
			Swap(Empty);
			return;
		}
		m_BV = Bv;

		m_SizeX = SizeX;
		m_SizeY = SizeY;
		m_SizeZ = SizeZ;
		m_VoxelSize = VoxelSize;
		m_VoxelHeight = VoxelHeight;
		m_InvVoxelSize = 1.0f / VoxelSize;
		m_InvVoxelHeight = 1.0f / VoxelHeight;

		m_Fields.assign(size_t(m_SizeX) * m_SizeZ, nullptr);

		int kVoxelBatchSize = vx_clamp(m_SizeX * m_SizeZ / 2, 1024, 1024 * 1024);
		m_VoxelBatchs.init(0, kVoxelBatchSize);
	}

	void VoxelField::Swap(VoxelField& Other)
	{
		std::swap(m_SizeX, Other.m_SizeX);
		std::swap(m_SizeY, Other.m_SizeY);
		std::swap(m_SizeZ, Other.m_SizeZ);
		std::swap(m_VoxelSize, Other.m_VoxelSize);
		std::swap(m_VoxelHeight, Other.m_VoxelHeight);
		std::swap(m_InvVoxelSize, Other.m_InvVoxelSize);
		std::swap(m_InvVoxelHeight, Other.m_InvVoxelHeight);
		std::swap(m_BV, Other.m_BV);
		m_Fields.swap(Other.m_Fields);
		std::swap(m_VoxelBatchs, Other.m_VoxelBatchs);
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
			float ymin = m_BV.Min.y + m_VoxelHeight * p->ymin;
			float ymax = m_BV.Min.y + m_VoxelHeight * (p->ymax + 1);
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
			float ymax = m_BV.Min.y + m_VoxelHeight * (prev->ymax + 1);
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
		if (m_SizeX == 0 || !VoxelFinite(pos)) return -1;
		const double fx = (double(pos.x) - m_BV.Min.x) / m_VoxelSize;
		const double fz = (double(pos.z) - m_BV.Min.z) / m_VoxelSize;
		if (fx < 0.0 || fx >= m_SizeX || fz < 0.0 || fz >= m_SizeZ) return -1;
		const int x = (int)fx;
		const int z = (int)fz;
		return z * m_SizeX + x;
	}

	int		VoxelField::WorldSpaceToVoxelSpaceY(float pos_y) const
	{
		if (m_SizeY == 0 || !std::isfinite(pos_y)) return -1;
		const double y = (double(pos_y) - m_BV.Min.y) / m_VoxelHeight;
		return y >= 0.0 && y < m_SizeY ? (int)y : -1;
	}

	float	VoxelField::VoxelSpaceToWorldSpaceY(uint16_t y) const
	{
		return m_BV.Min.y + (y + 0.5f) * m_VoxelHeight;
	}

	Box3	VoxelField::GetVoxelBox(const Vector3& pos) const
	{
		int idx = WorldSpaceToVoxelIndex(pos);
		if (idx < 0) return Box3::Unit();
		int z = idx / m_SizeX;
		int x = idx - z * m_SizeX;
		int y = WorldSpaceToVoxelSpaceY(pos.y);
		return GetVoxelBox(x, y, z);
	}

	Box3	VoxelField::GetVoxelBox(int x, int y, int z) const
	{
		if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ || y < 0 || y >= m_SizeY)
		{
			// Error
			return Box3::Unit();
		}
		Box3 box;
		box.Min.x = m_BV.Min.x + m_VoxelSize * x;
		box.Min.y = m_BV.Min.y + m_VoxelHeight * y;
		box.Min.z = m_BV.Min.z + m_VoxelSize * z;
		box.Max = box.Min + Vector3(m_VoxelSize, m_VoxelHeight, m_VoxelSize);
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
		if (idx < 0 || size_t(idx) >= m_Fields.size() || ymin > ymax || ymax >= m_SizeY ||
			!std::isfinite(MergeThr) || MergeThr < 0.0f) return false;
		// MergeThr is the maximum empty gap in world-space Y; touching cells always merge.
		const int Gap = (int)std::min(65536.0, std::floor(double(MergeThr) / m_VoxelHeight));
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
			if (int(p->ymin) > int(s->ymax) + 1 + Gap)
			{
				break;
			}
			else if (int(p->ymax) + 1 + Gap < int(s->ymin))
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
		return m_VoxelBatchs.allocate();
	}

	void VoxelField::FreeVoxel(Voxel* p)
	{
		m_VoxelBatchs.free(p);
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
		if (m_SizeX == 0 || !VoxelFinite(v0) || !VoxelFinite(v1) || !VoxelFinite(v2) ||
			!std::isfinite(info.YMergeThr) || info.YMergeThr < 0.0f) return false;
		// const float dy = m_BV.Max.y - m_BV.Min.y;
		const float ics = 1.0f / m_VoxelSize;
		const float ich = 1.0f / m_VoxelHeight;

		Box3 tbox(v0, v0);
		tbox.Encapsulate(v1, v2);

		if (!tbox.Intersect(m_BV))
			return true;

		const int z0 = (int)vx_clamp((double(tbox.Min.z) - m_BV.Min.z) * ics, 0.0, double(m_SizeZ - 1));
		const int z1 = (int)vx_clamp((double(tbox.Max.z) - m_BV.Min.z) * ics, 0.0, double(m_SizeZ - 1));

		Vector3 buf[7 * 4];
		Vector3 *In = buf, *inrow = buf + 7, *p1 = buf + 7 * 2, *p2 = buf + 7 * 3;

		In[0] = v0;
		In[1] = v1;
		In[2] = v2;

		int nvrow, nvIn = 3;

		for (int z = z0; z <= z1; ++z)
		{
			const float cz = m_BV.Min.z + z * m_VoxelSize;
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
			const int x0 = (int)vx_clamp((double(minX) - m_BV.Min.x) * ics, 0.0, double(m_SizeX - 1));
			const int x1 = (int)vx_clamp((double(maxX) - m_BV.Min.x) * ics, 0.0, double(m_SizeX - 1));

			int nv, nv2 = nvrow;

			for (int x = x0; x <= x1; ++x)
			{
				const float cx = m_BV.Min.x + x * m_VoxelSize;
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

				if (ymax < m_BV.Min.y || ymin > m_BV.Max.y)
					continue;

				if (ymin < m_BV.Min.y)
					ymin = m_BV.Min.y;
				if (ymax > m_BV.Max.y)
					ymax = m_BV.Max.y;
				ymin -= m_BV.Min.y;
				ymax -= m_BV.Min.y;

				const uint16_t y0 = (uint16_t)vx_clamp(double(ymin) * ich, 0.0, double(m_SizeY - 1));
				const uint16_t y1 = (uint16_t)vx_clamp(double(ymax) * ich, 0.0, double(m_SizeY - 1));

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

	static bool VoxelInterval(float Min, float Max, float Origin, float Spacing, int Count, int& First, int& Last)
	{
		const double A = (double(Min) - Origin) / Spacing;
		const double B = (double(Max) - Origin) / Spacing;
		if (A >= Count || B < 0.0 || (A != B && B <= 0.0)) return false;
		First = (int)std::max(0.0, std::floor(A));
		// A nonzero interval is half-open, so an exact upper cell boundary adds no extra cell.
		Last = A == B ? First : (int)std::min(double(Count - 1), std::ceil(B) - 1.0);
		return First <= Last;
	}

	bool VoxelField::VoxelizationYPlane(float world_y_min, float world_y_max)
	{
		if (m_SizeY == 0 || !std::isfinite(world_y_min) || !std::isfinite(world_y_max) || world_y_min > world_y_max) return false;
		int ymin, ymax;
		if (!VoxelInterval(world_y_min, world_y_max, m_BV.Min.y, m_VoxelHeight, m_SizeY, ymin, ymax)) return true;
		for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
		{
			if (!AddVoxel(i, (uint16_t)ymin, (uint16_t)ymax, 0.0f)) return false;
		}
		return true;
	}

	bool VoxelField::VoxelizationCube(const Box3& cube)
	{
		if (m_SizeX == 0 || !VoxelFinite(cube.Min) || !VoxelFinite(cube.Max) ||
			cube.Min.x > cube.Max.x || cube.Min.y > cube.Max.y || cube.Min.z > cube.Max.z) return false;
		int x0, x1, y0, y1, z0, z1;
		if (!VoxelInterval(cube.Min.x, cube.Max.x, m_BV.Min.x, m_VoxelSize, m_SizeX, x0, x1) ||
			!VoxelInterval(cube.Min.y, cube.Max.y, m_BV.Min.y, m_VoxelHeight, m_SizeY, y0, y1) ||
			!VoxelInterval(cube.Min.z, cube.Max.z, m_BV.Min.z, m_VoxelSize, m_SizeZ, z0, z1)) return true;
		for (int z = z0; z <= z1; ++z)
		for (int x = x0; x <= x1; ++x)
		{
			if (!AddVoxel(z * m_SizeX + x, (uint16_t)y0, (uint16_t)y1, 0.0f)) return false;
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

			int first_y = p ? p->ymin : m_SizeY;
			if (first_y > 0)
			{
				AddVoxel(i, 0, first_y - 1, 0.0f);
			}

			while (p)
			{
				if (p->next && int(p->ymax) + 1 < p->next->ymin)
				{
					AddVoxel(i, p->ymax + 1, p->next->ymin - 1, 0.0f);
				}
				else if (!p->next && p->ymax < m_SizeY - 1)
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
			Voxel** Link = &m_Fields[i];
			while (*Link)
			{
				Voxel* p = *Link;
				if (func(p))
				{
					++Count;
					*Link = p->next;
					FreeVoxel(p);
				}
				else
				{
					Link = &p->next;
				}
			}
		}
		return Count;
	}

	int			VoxelField::FilterByY(float world_y)
	{
		if (m_SizeY == 0 || !std::isfinite(world_y) || world_y < m_BV.Min.y) return 0;
		const int ny = (int)std::min(double(m_SizeY - 1), (double(world_y) - m_BV.Min.y) / m_VoxelHeight);
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
		return m_VoxelBatchs.size() * sizeof(Voxel) + m_SizeX * m_SizeZ * sizeof(Voxel*);
	}

	uint64_t VoxelField::EstimateMemoryUseageEx() const
	{
		return  m_VoxelBatchs.size() * sizeof(VoxelFast) + m_SizeX * m_SizeZ * sizeof(uint32_t);
	}

	void	VoxelField::GenerateHeightMap(std::vector<float>& heightmap) const
	{
		heightmap.resize(m_SizeX * m_SizeZ);

		for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
		{
			heightmap[i] = m_BV.Min.y;

			Voxel* v = m_Fields[i];
			while (v)
			{
				if (v->next == nullptr)
				{
					heightmap[i] = m_BV.Min.y + (v->ymax + 1) * m_VoxelHeight;
					break;
				}
				v = v->next;
			}
		}
	}

	void	VoxelField::GenerateBitmapByLevel(std::vector<int>& levels, int * level_max) const
	{
		levels.assign(m_Fields.size(), 0);
		if (level_max) *level_max = 0;

		for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
		{
			levels[i] = CountVoxel(m_Fields[i]);
			if (level_max) *level_max = std::max(*level_max, levels[i]);
		}
	}

	void	VoxelField::GenerateBitmapByData(std::vector<int>& output, uint32_t data) const
	{
		output.assign(m_Fields.size(), 0);

		for (int i = 0; i < m_SizeX * m_SizeZ; ++i)
		{
			Voxel* v = FindVoxel(m_Fields[i], data);
			// output[i] = v ? (int)(v->ymax - v->ymin) : 0;
			output[i] = v ? 1 : 0;
		}
	}

	bool VoxelField::Verify() const
	{
		if (m_SizeX == 0 && m_SizeY == 0 && m_SizeZ == 0)
		{
			return m_Fields.empty() && m_VoxelBatchs.empty() && m_VoxelSize == 0.0f && m_VoxelHeight == 0.0f;
		}
		if (!VoxelDimensionsValid(m_BV, m_SizeX, m_SizeY, m_SizeZ, m_VoxelSize, m_VoxelHeight) ||
			m_Fields.size() != size_t(m_SizeX) * m_SizeZ) return false;
		size_t Count = 0;
		for (const Voxel* Head : m_Fields)
		{
			int PreviousMax = -1;
			for (const Voxel* V = Head; V; V = V->next)
			{
				if (V->ymin > V->ymax || V->ymax >= m_SizeY || V->ymin <= PreviousMax ||
					++Count > size_t(m_VoxelBatchs.size())) return false;
				PreviousMax = V->ymax;
			}
		}
		return Count == size_t(m_VoxelBatchs.size());
	}

	bool VoxelField::GenerateMesh(StaticMesh& Mesh) const
	{
		if (m_SizeX == 0)
		{
			Mesh.Clear();
			return Verify();
		}
		return GenerateMesh(Mesh, 0, m_SizeX - 1, 0, m_SizeZ - 1);
	}

	bool VoxelField::GenerateMesh(StaticMesh& Mesh, int x1, int x2, int z1, int z2) const
	{
		Mesh.Clear();
		if (x1 > x2 || z1 > z2 || !Verify()) return false;
		x1 = std::max(0, x1);
		x2 = std::min(m_SizeX - 1, x2);
		z1 = std::max(0, z1);
		z2 = std::min(m_SizeZ - 1, z2);
		if (x1 > x2 || z1 > z2) return true;

		// Each face owns four vertices so hard voxel edges retain their flat normals.
		static const int Corners[6][4][3] = {
			{{0,0,0}, {0,0,1}, {0,1,1}, {0,1,0}},
			{{1,0,0}, {1,1,0}, {1,1,1}, {1,0,1}},
			{{0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}},
			{{0,1,0}, {0,1,1}, {1,1,1}, {1,1,0}},
			{{0,0,0}, {0,1,0}, {1,1,0}, {1,0,0}},
			{{0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}}
		};
		static const Vector3 Normals[6] = {
			Vector3(-1,0,0), Vector3(1,0,0), Vector3(0,-1,0),
			Vector3(0,1,0), Vector3(0,0,-1), Vector3(0,0,1)
		};
		std::vector<Vector3> Vertices, VertexNormals;
		std::vector<uint32_t> Indices;
		auto AddFace = [&](int x, int y, int z, int Face) -> bool
		{
			if (Vertices.size() > std::numeric_limits<uint32_t>::max() - 4) return false;
			const uint32_t Base = (uint32_t)Vertices.size();
			for (int i = 0; i < 4; ++i)
			{
				const int* C = Corners[Face][i];
				Vertices.emplace_back(
					m_BV.Min.x + (x + C[0]) * m_VoxelSize,
					m_BV.Min.y + (y + C[1]) * m_VoxelHeight,
					m_BV.Min.z + (z + C[2]) * m_VoxelSize);
				VertexNormals.push_back(Normals[Face]);
			}
			Indices.insert(Indices.end(), {Base, Base + 1, Base + 2, Base, Base + 2, Base + 3});
			return true;
		};

		for (int z = z1; z <= z2; ++z)
		for (int x = x1; x <= x2; ++x)
		{
			const int Index = z * m_SizeX + x;
			// Neighbor cursors only advance in Y, avoiding a dense 3D occupancy allocation.
			const Voxel* Neighbors[4] = {
				x > x1 ? m_Fields[Index - 1] : nullptr,
				x < x2 ? m_Fields[Index + 1] : nullptr,
				z > z1 ? m_Fields[Index - m_SizeX] : nullptr,
				z < z2 ? m_Fields[Index + m_SizeX] : nullptr
			};
			const int SideFaces[4] = {0, 1, 4, 5};
			int PreviousMax = -2;
			for (const Voxel* V = m_Fields[Index]; V; V = V->next)
			{
				if (int(V->ymin) > PreviousMax + 1 && !AddFace(x, V->ymin, z, 2)) return false;
				if ((!V->next || int(V->next->ymin) > int(V->ymax) + 1) && !AddFace(x, V->ymax, z, 3)) return false;
				for (int y = V->ymin; y <= V->ymax; ++y)
				{
					for (int Side = 0; Side < 4; ++Side)
					{
						const Voxel*& N = Neighbors[Side];
						while (N && N->ymax < y) N = N->next;
						if ((!N || N->ymin > y) && !AddFace(x, y, z, SideFaces[Side])) return false;
					}
				}
				PreviousMax = V->ymax;
			}
		}

		if (!Vertices.empty())
		{
			Mesh.SetData(Vertices.data(), Indices.data(), (uint32_t)Vertices.size(), (uint32_t)(Indices.size() / 3), false, true);
			Mesh.mNormals = std::move(VertexNormals);
			Mesh.BoundingVolume = Box3(Vertices.data(), Vertices.size());
		}
		return true;
	}

	StaticMesh* VoxelField::CreateDebugMesh(int x1, int x2, int z1, int z2) const
	{
		StaticMesh* Mesh = new StaticMesh();
		if (!GenerateMesh(*Mesh, x1, x2, z1, z2))
		{
			delete Mesh;
			return nullptr;
		}
		return Mesh;
	}

	bool VoxelField::SerializeTo(const char* filename)
	{
		if (!filename || !Verify()) return false;
		VoxelFileHeader Header = {};
		Header.Version = VOXEL_FILE_VERSION;
		Header.SizeX = m_SizeX;
		Header.SizeY = m_SizeY;
		Header.SizeZ = m_SizeZ;
		Header.VoxelSize = m_VoxelSize;
		Header.VoxelHeight = m_VoxelHeight;
		Header.BV = m_BV;
		std::vector<VoxelFast> Voxels;
		std::vector<VoxelFileField> Fields;
		for (size_t i = 0; i < m_Fields.size(); ++i)
		{
			if (!m_Fields[i]) continue;
			VoxelFileField Field = {(uint32_t)i, (uint32_t)Voxels.size(), 0};
			for (const Voxel* V = m_Fields[i]; V; V = V->next)
			{
				Voxels.push_back({V->data, V->ymin, V->ymax});
				++Field.numVoxels;
			}
			Fields.push_back(Field);
		}
		Header.nVoxels = (uint32_t)Voxels.size();
		Header.nFields = (uint32_t)Fields.size();
		FileWriter File(filename);
		if (!File.IsLoaded()) return false;
		const uint32_t Magic = VOXEL_FILE_MAGIC;
		return File.Write(&Magic, sizeof(Magic)) == sizeof(Magic) &&
			File.Write(&Header, sizeof(Header)) == sizeof(Header) &&
			(Voxels.empty() || File.Write(Voxels.data(), Voxels.size() * sizeof(VoxelFast)) == Voxels.size() * sizeof(VoxelFast)) &&
			(Fields.empty() || File.Write(Fields.data(), Fields.size() * sizeof(VoxelFileField)) == Fields.size() * sizeof(VoxelFileField));
	}

	bool VoxelField::SerializeFrom(const char* filename)
	{
		if (!filename) return false;
		FileReader File(filename);
		uint32_t Magic = 0;
		VoxelFileHeader Header = {};
		if (!File.IsLoaded() || File.Read(&Magic, sizeof(Magic)) != sizeof(Magic) || Magic != VOXEL_FILE_MAGIC ||
			File.Read(&Header, sizeof(Header)) != sizeof(Header) || Header.Version != VOXEL_FILE_VERSION) return false;
		const uint64_t ExpectedSize = sizeof(Magic) + sizeof(Header) +
			uint64_t(Header.nVoxels) * sizeof(VoxelFast) + uint64_t(Header.nFields) * sizeof(VoxelFileField);
		if (File.GetFileSize() != ExpectedSize || Header.nVoxels > uint32_t(std::numeric_limits<int>::max())) return false;

		const bool Empty = Header.SizeX == 0 && Header.SizeY == 0 && Header.SizeZ == 0;
		if (Empty)
		{
			if (Header.nFields || Header.nVoxels || Header.VoxelSize != 0.0f || Header.VoxelHeight != 0.0f ||
				Header.BV.Min != Vector3::Zero() || Header.BV.Max != Vector3::Zero()) return false;
			VoxelField Field;
			Swap(Field);
			return true;
		}
		if (!VoxelDimensionsValid(Header.BV, Header.SizeX, Header.SizeY, Header.SizeZ, Header.VoxelSize, Header.VoxelHeight) ||
			Header.nFields > uint64_t(Header.SizeX) * Header.SizeZ || Header.nFields > Header.nVoxels) return false;

		std::vector<VoxelFast> Voxels(Header.nVoxels);
		std::vector<VoxelFileField> Fields(Header.nFields);
		if ((!Voxels.empty() && File.Read(Voxels.data(), Voxels.size() * sizeof(VoxelFast)) != Voxels.size() * sizeof(VoxelFast)) ||
			(!Fields.empty() && File.Read(Fields.data(), Fields.size() * sizeof(VoxelFileField)) != Fields.size() * sizeof(VoxelFileField))) return false;

		uint32_t NextVoxel = 0;
		uint32_t PreviousIndex = 0;
		for (size_t i = 0; i < Fields.size(); ++i)
		{
			const VoxelFileField& Field = Fields[i];
			if (Field.idx >= uint64_t(Header.SizeX) * Header.SizeZ || (i && Field.idx <= PreviousIndex) ||
				Field.firstVoxel != NextVoxel || Field.numVoxels == 0 || Field.numVoxels > Header.nVoxels - NextVoxel) return false;
			int PreviousMax = -1;
			for (uint32_t j = 0; j < Field.numVoxels; ++j)
			{
				const VoxelFast& V = Voxels[NextVoxel + j];
				if (V.ymin > V.ymax || V.ymax >= Header.SizeY || V.ymin <= PreviousMax) return false;
				PreviousMax = V.ymax;
			}
			PreviousIndex = Field.idx;
			NextVoxel += Field.numVoxels;
		}
		if (NextVoxel != Header.nVoxels) return false;

		// Allocate into a temporary field only after the entire file has been validated.
		VoxelField Result;
		Result.MakeEmpty(Header.BV, Header.SizeX, Header.SizeY, Header.SizeZ, Header.VoxelSize, Header.VoxelHeight);
		for (const VoxelFileField& Field : Fields)
		{
			Voxel** Link = &Result.m_Fields[Field.idx];
			for (uint32_t j = 0; j < Field.numVoxels; ++j)
			{
				const VoxelFast& Source = Voxels[Field.firstVoxel + j];
				Voxel* V = Result.AllocVoxel();
				V->data = Source.data;
				V->ymin = Source.ymin;
				V->ymax = Source.ymax;
				V->next = nullptr;
				*Link = V;
				Link = &V->next;
			}
		}
		Swap(Result);
		return true;
	}
}

#pragma once

#include <map>
#include <set>
#include <vector>
#include "../Maths/Box3d.h"

// Sweep and Prune (SAP) algorithm
// http://www.codercorner.com/SAP.pdf

typedef unsigned long long OverlapKey;

static_assert(sizeof(OverlapKey) == 8, "sizeof OverlapKey2 is not valid");

class SAP
{
public:
	class BoundingVolumeProxy
	{
	public:
		virtual int			GetBoundingVolumeCount() const = 0;
		virtual float*		GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const = 0;
		virtual bool		Overlaps(int bv_i, int bv_j) const = 0;
	};

	SAP(BoundingVolumeProxy* Proxy, const std::vector<int>& axis_list)
	{
		m_Proxy = Proxy;
		for (int axis : axis_list) {
			m_AxisList.push_back(axis);
		}
		m_Axis.resize(m_AxisList.size());
	}

	void Prune(std::set<OverlapKey>* overlaps)
	{
		std::map<OverlapKey, int> overlaps_count;

		int axis_filter = 0;
		for (auto k = 0; k < m_AxisList.size(); ++k)
		{
			InitAxis(k);
			QSort(m_Axis[k], 0, (int)m_Axis[k].size() - 1);
			PruneAxis(m_Axis[k], overlaps_count, 1 << m_AxisList[k]);
			axis_filter |= (1 << m_AxisList[k]);
		}

		overlaps->clear();
		for (auto it : overlaps_count)
		{
			if (it.second == axis_filter)
			{
				overlaps->insert(it.first);
			}
		}
	}

	static OverlapKey PackOverlapKey(int id1, int id2)
	{
		if (id1 > id2)
		{
			int t = id1;
			id1 = id2;
			id2 = t;
		}
		return ((OverlapKey)id1 << 32) | id2;
	}

	static void UnpackOverlapKey(OverlapKey key, int* id1, int* id2)
	{
		*id1 = key >> 32;
		*id2 = key & 0xFFFFFFFF;
	}

protected:

	struct SweepPoint
	{
		SweepPoint() {}
		SweepPoint(int id, bool left, float* val)
		{
			data = left ? ((1 << 31) | id) : id;
			value = val;
		}

		inline bool left() const
		{
			return data & (1 << 31);
		}

		inline int id() const
		{
			return data & 0x7FFFFFFF;
		}

		unsigned int	data;
		float* value;
	};

	void InitAxis(int k)
	{
		std::vector<SweepPoint>& axis = m_Axis[k];

		int nBV = m_Proxy->GetBoundingVolumeCount();
		axis.resize(2 * nBV);

		for (int i = 0; i < nBV; ++i)
		{
			float* val1 = m_Proxy->GetBoundingVolumeCoordinate(i, true, k);
			float* val2 = m_Proxy->GetBoundingVolumeCoordinate(i, false, k);
			axis[2 * i] = SweepPoint(i, true, val1);
			axis[2 * i + 1] = SweepPoint(i, false, val2);
		}
	}

	static void Swap(SweepPoint* a, SweepPoint* b)
	{
		SweepPoint t = *a;
		*a = *b;
		*b = t;
	}

	static void QSort(std::vector<SweepPoint>& axis, int low, int high)
	{
		if (low < high)
		{
			SweepPoint pivot = axis[high];
			int i = low - 1;

			for (int j = low; j <= high - 1; ++j)
			{
				if (*axis[j].value <= *pivot.value)
				{
					i++;
					Swap(&axis[i], &axis[j]);
				}
			}
			Swap(&axis[i + 1], &axis[high]);

			QSort(axis, low, i);
			QSort(axis, i + 2, high);
		}
	}

	static void PruneAxis(const std::vector<SweepPoint>& axis, std::map<OverlapKey, int>& overlaps_count, int filter)
	{
		std::set<int> active;

		for (size_t i = 0; i < axis.size(); ++i)
		{
			const SweepPoint* curr = &axis[i];
			if (curr->left())
			{
				for (auto active_id : active)
				{
					if (curr->id() != active_id)
					{
						OverlapKey key = PackOverlapKey(curr->id(), active_id);
						if (overlaps_count.find(key) == overlaps_count.end())
						{
							overlaps_count.insert(std::pair<OverlapKey, int>(key, filter));
						}
						else
						{
							overlaps_count[key] |= filter;
						}
					}
				}

				active.insert(curr->id());
			}
			else
			{
				active.erase(curr->id());
			}
		}
	}

protected:
	BoundingVolumeProxy*					m_Proxy;
	std::vector<std::vector<SweepPoint>>	m_Axis;
	std::vector<int>						m_AxisList;
};

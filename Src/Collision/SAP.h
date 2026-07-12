#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include "../Maths/Box3.h"

// Sweep and Prune (SAP) algorithm
// http://www.codercorner.com/SAP.pdf

namespace Riemann
{
	typedef uint64_t OverlapKey;

	static_assert(sizeof(OverlapKey) == 8, "sizeof OverlapKey is not valid");

	class SAP
	{
	public:
		class BoundingVolumeProxy
		{
		public:
			virtual ~BoundingVolumeProxy() = default;
			virtual int			GetBoundingVolumeCount() const = 0;
			virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const = 0;
			virtual bool		Overlaps(int bv_i, int bv_j) const = 0;
			virtual uint64_t	CalculateBoundingVolumeHash() const = 0;
		};

		SAP(BoundingVolumeProxy* Proxy, const std::vector<int>& axis_list)
		{
			m_Proxy = Proxy;
			for (int axis : axis_list) {
				m_AxisList.push_back(axis);
			}
			m_Axis.resize(m_AxisList.size());
			m_Hash = Proxy->CalculateBoundingVolumeHash();
		}

		void Prune(std::set<OverlapKey>* overlaps)
		{
			std::map<OverlapKey, int> overlaps_count;

			int axis_filter = 0;
			for (auto k = 0; k < m_AxisList.size(); ++k)
			{
				InitAxis(k, m_AxisList[k]);
				std::sort(m_Axis[k].begin(), m_Axis[k].end(), Less);
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
			SweepPoint(int id, bool left, float* pval)
			{
				data = left ? ((1 << 31) | id) : id;
				pvalue = pval;
			}

			inline bool left() const
			{
				return data & (1 << 31);
			}

			inline int id() const
			{
				return data & 0x7FFFFFFF;
			}

			inline float value() const
			{
				return *pvalue;
			}

			uint32_t	data;
			float* pvalue;
		};

		static bool Less(const SweepPoint& A, const SweepPoint& B)
		{
			if (A.value() != B.value())
			{
				return A.value() < B.value();
			}
			if (A.left() != B.left())
			{
				return A.left();
			}
			return A.id() < B.id();
		}

		void InitAxis(int i, int axis)
		{
			std::vector<SweepPoint>& Axis = m_Axis[i];

			int nBV = m_Proxy->GetBoundingVolumeCount();
			Axis.resize(2 * nBV);

			for (int i = 0; i < nBV; ++i)
			{
				float* val1 = m_Proxy->GetBoundingVolumeCoordinate(i, true, axis);
				float* val2 = m_Proxy->GetBoundingVolumeCoordinate(i, false, axis);
				Axis[2 * i] = SweepPoint(i, true, val1);
				Axis[2 * i + 1] = SweepPoint(i, false, val2);
			}
		}

		static void PruneAxis(const std::vector<SweepPoint>& axis, std::map<OverlapKey, int>& overlaps_count, int filter)
		{
			std::set<int> active;

			for (const SweepPoint& curr : axis)
			{
				const int curr_id = curr.id();
				if (curr.left())
				{
					for (int active_id : active)
					{
						OverlapKey key = PackOverlapKey(curr_id, active_id);
						if (overlaps_count.find(key) == overlaps_count.end())
						{
							overlaps_count.emplace(key, filter);
						}
						else
						{
							overlaps_count[key] |= filter;
						}
					}

					active.insert(curr_id);
				}
				else
				{
					active.erase(curr_id);
				}
			}
		}

	protected:
		BoundingVolumeProxy* m_Proxy;
		std::vector<std::vector<SweepPoint>>	m_Axis;
		std::vector<int>						m_AxisList;
		uint64_t								m_Hash;
	};
}

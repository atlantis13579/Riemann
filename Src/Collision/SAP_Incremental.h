#pragma once

#include <map>
#include <set>
#include <vector>
#include "SAP.h"

// Incremental Sweep and Prune (SAP) algorithm
// http://www.codercorner.com/SAP.pdf

class IncrementalSAP
{
public:
	IncrementalSAP(SAP::BoundingVolumeProxy* Proxy, const std::vector<int>& axis_list)
	{
		m_Proxy = Proxy;
		for (int axis : axis_list) {
			m_AxisList.push_back(axis);
		}
		m_Axis.resize(m_AxisList.size());
		m_IsDirty = true;
	}

	void SetDirty()
	{
		m_IsDirty = true;
	}

	void IncrementalPrune(std::set<OverlapKey>* overlaps)
	{
		if (m_IsDirty)
		{
			SAP sap(m_Proxy, m_AxisList);
			sap.Prune(overlaps);

			m_Axis = sap.GetSweepAxis();
			m_IsDirty = false;
			return;
		}

		for (size_t k = 0; k < m_AxisList.size(); ++k)
		{
			InsertionSort(m_Axis[k], overlaps);
		}

		return;
	}

private:
	void InsertionSort(std::vector<SAP::SweepPoint>& axis, std::set<OverlapKey>* overlaps)
	{
		int nsize = (int)axis.size();
		for (int j = 1; j < nsize; j++)
		{
			SAP::SweepPoint pivot = axis[j];
			float pivot_value = *pivot.value;

			int i = j - 1;

			while (i >= 0 && *axis[i].value > pivot_value)
			{
				SAP::SweepPoint curr = axis[i];

				if (pivot.left() && !curr.left())
				{
					int id_current = curr.id();
					int id_pivot = pivot.id();
					if (id_current != id_pivot)
					{
						if (m_Proxy->Overlaps(id_current, id_pivot))
						{
							overlaps->insert(SAP::PackOverlapKey(id_current, id_pivot));
						}
					}
				}

				if (!pivot.left() && curr.left())
				{
					overlaps->erase(SAP::PackOverlapKey(curr.id(), pivot.id()));
				}

				axis[i + 1] = curr;
				i = i - 1;
			}
			axis[i + 1] = pivot;
		}
	}

private:
	SAP::BoundingVolumeProxy*				   m_Proxy;
	bool										m_IsDirty;
	std::vector<std::vector<SAP::SweepPoint>>   m_Axis;
	std::vector<int>							m_AxisList;
};

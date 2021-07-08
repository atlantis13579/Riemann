
#include "SAP.h"

sap_key sap_pack_key(int id1, int id2)
{
	if (id1 > id2)
	{
		int t = id1;
		id1 = id2;
		id2 = t;
	}
	return ((sap_key)id1 << 32) | id2;
}

void sap_unpack_key(sap_key key, int* id1, int* id2)
{
	*id1 = key >> 32;
	*id2 = key & 0xFFFFFFFF;
}

static void sap_swap(sweep_point* a, sweep_point* b)
{
	sweep_point t = *a;
	*a = *b;
	*b = t;
}

static void sap_axis_qsort(sweep_point* axis, int low, int high)
{
	if (low < high)
	{
		sweep_point* pivot = &axis[high];
		int i = low - 1;

		for (int j = low; j <= high - 1; ++j)
		{
			if (axis[j].value <= pivot->value)
			{
				i++;
				sap_swap(&axis[i], &axis[j]);
			}
		}
		sap_swap(&axis[i + 1], &axis[high]);

		sap_axis_qsort(axis, low, i);
		sap_axis_qsort(axis, i + 2, high);
	}
}

static void sweep_and_prune(const std::vector<sweep_point>& axis, std::map<sap_key, int>& overlaps_count, int filter)
{
	std::set<int> active;

	for (size_t i = 0; i < axis.size(); ++i)
	{
		const sweep_point* curr = &axis[i];
		if (curr->left())
		{
			for (auto active_id : active)
			{
				if (curr->id() != active_id)
				{
					sap_key key = sap_pack_key(curr->id(), active_id);
					if (overlaps_count.find(key) == overlaps_count.end())
					{
						overlaps_count.insert(std::pair<sap_key, int>(key, filter));
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

void sap_direct(const std::vector<BoundingBox3d>& boxes, std::set<sap_key>* overlaps, int axis_filter /*= 7*/)
{
	std::map<sap_key, int> overlaps_count;
	std::vector<sweep_point> axis;
	axis.resize(2 * boxes.size());

	int nsize = (int)boxes.size();
	for (int k = 0; k < 3; ++k)
	{
		int filter = 1 << k;
		if ((filter & axis_filter) == 0)
			continue;

		for (int i = 0; i < boxes.size(); ++i)
		{
			axis[2 * i] = sweep_point(i, true, boxes[i].Min[k]);
			axis[2 * i + 1] = sweep_point(i, false, boxes[i].Max[k]);
		}
		sap_axis_qsort(&axis[0], 0, (int)axis.size() - 1);
		sweep_and_prune(axis, overlaps_count, filter);

		continue;
	}

	overlaps->clear();
	for (auto it : overlaps_count)
	{
		if (it.second == axis_filter)
		{
			overlaps->insert(it.first);
		}
	}

	return;
}


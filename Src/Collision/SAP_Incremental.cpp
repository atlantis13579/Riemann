
#include "SAP_Incremental.h"

static void sap_axis_insertion_sort(std::vector<sweep_point>& axis, std::vector<GeometryObject*>& objs, std::set<sap_key>* overlaps)
{
	int nsize = (int)axis.size();
	for (int j = 1; j < nsize; j++)
	{
		sweep_point pivot = axis[j];
		float pivot_value = *pivot.value;

		int i = j - 1;

		while (i >= 0 && *axis[i].value > pivot_value)
		{
			sweep_point curr = axis[i];

			if (pivot.left() && !curr.left())
			{
				int id_current = curr.id();
				int id_pivot = pivot.id();
				if (id_current != id_pivot)
				{
					const BoundingBox3d& box1 = objs[id_current]->GetBoundingBoxWorld();
					const BoundingBox3d& box2 = objs[id_pivot]->GetBoundingBoxWorld();
					if (box1.Intersect(box2))
					{
						overlaps->insert(sap_pack_key(id_current, id_pivot));
					}
				}
			}

			if (!pivot.left() && curr.left())
			{
				overlaps->erase(sap_pack_key(curr.id(), pivot.id()));
			}

			axis[i + 1] = curr;
			i = i - 1;
		}
		axis[i + 1] = pivot;
	}
}

void sap_incremental_init(std::vector<GeometryObject*>& objs, int k, std::vector<sweep_point>& axis)
{
	axis.resize(2 * objs.size());
	for (int i = 0; i < objs.size(); ++i)
	{
		const BoundingBox3d& box = objs[i]->GetBoundingBoxWorld();
		float* p = (float*)&box;
		axis[2 * i] = sweep_point(i, true, p + k);
		axis[2 * i + 1] = sweep_point(i, false, p + 3 + k);
	}
}

void sap_incremental(std::vector<std::vector<sweep_point>>& axis, std::vector<GeometryObject*>& objs, std::set<sap_key>* overlaps)
{
	for (size_t k = 0; k < axis.size(); ++k)
	{
		if (axis[k].empty())
			continue;

		sap_axis_insertion_sort(axis[k], objs, overlaps);

		continue;
	}

	return;
}


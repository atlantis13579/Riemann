#pragma once

#include "sap.h"

void sap_axis_insertion_sort(std::vector<sweep_point>& axis, const std::vector<BoundingBox3d>& boxes, std::set<sap_key>* overlaps)
{
    int nsize = (int)axis.size();
    for (int j = 1; j < nsize; j++)
    {
        sweep_point pivot = axis[j];
        float pivot_value = pivot.value;

        int i = j - 1;

        while (i >= 0 && axis[i].value > pivot_value)
        {
            sweep_point curr = axis[i];

            if (pivot.left() && !curr.left())
            {
                int id_current = curr.id();
                int id_pivot = pivot.id();
                if (id_current != id_pivot)
                {
                    if (boxes[id_current].Intersect(boxes[id_pivot]))
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


// http://www.codercorner.com/SAP.pdf
void sap_incremental(const std::vector<BoundingBox3d>& boxes, std::set<sap_key>* overlaps)
{
    overlaps->clear();

    std::vector<sweep_point> axis;
    axis.resize(2 * boxes.size());

    int nsize = (int)boxes.size();
    for (int k = 0; k < 3; ++k)
    {
        for (int i = 0; i < boxes.size(); ++i)
        {
            axis[2 * i] = sweep_point(i, false, boxes[i].Max[k]);
            axis[2 * i + 1] = sweep_point(i, true, boxes[i].Min[k]);
        }

        sap_axis_insertion_sort(axis, boxes, overlaps);

        continue;
    }

    return;
}

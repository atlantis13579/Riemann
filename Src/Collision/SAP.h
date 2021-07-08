
#pragma once

#include <map>
#include <set>
#include <vector>
#include "../Maths/BoundingBox3d.h"

struct sweep_point
{
    sweep_point() {}
    sweep_point(int id, bool left, float val)
    {
        data = left ? ((1 << 31) | id ) : id;
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

    unsigned int    data;
    float           value;
};

typedef unsigned long long sap_key;

static_assert(sizeof(sap_key) == 8, "sizeof sap_key is not valid");

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

void sap_unpack_key(sap_key key, int *id1, int *id2)
{
    *id1 = key >> 32;
    *id2 = key & 0xFFFFFFFF;
}

void sap_swap(sweep_point* a, sweep_point* b)
{
    sweep_point t = *a;
    *a = *b;
    *b = t;
}

void sap_axis_qsort(sweep_point* axis, int low, int high)
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

void sap_axis_insertion_sort(std::vector<sweep_point>& axis, const std::vector<BoundingBox3d>& boxes, std::set<sap_key> *overlaps)
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

void sweep_and_prune(const std::vector<sweep_point>& axis, std::map<sap_key, int>& overlaps_count, int filter)
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

// http://www.codercorner.com/SAP.pdf
void sap_full(const std::vector<BoundingBox3d>& boxes, std::set<sap_key> *overlaps, int axis_filter = 7)
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
            axis[2*i] = sweep_point(i, false, boxes[i].Max[k]);
            axis[2*i+1] = sweep_point(i, true, boxes[i].Min[k]);
        }

        sap_axis_insertion_sort(axis, boxes, overlaps);

        continue;
    }

    return;
}


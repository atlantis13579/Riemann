
#pragma once

#include <map>
#include <set>
#include <vector>
#include "../Maths/BoundingBox3d.h"

struct sweep_point
{
    sweep_point(int idx, bool min, float _val)
    {
        data = min ? ((1 << 31) | idx ) : idx;
        value = _val;
    }

    inline bool is_left() const
    {
        return data & (1 << 31);
    }

    inline int index() const
    {
        return data & 0x7FFFFFFF;
    }

    unsigned int    data;
    float           value;
};

typedef unsigned long long sap_key;

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

void sap_point_sort(sweep_point* point_list, int low, int high)
{
    if (low < high)
    {
        sweep_point* pivot = &point_list[high];
        int i = low - 1;

        for (int j = low; j <= high - 1; ++j)
        {
            if (point_list[j].value > pivot->value)
            {
                i++;
                sap_swap(&point_list[i], &point_list[j]);
            }
        }
        sap_swap(&point_list[i + 1], &point_list[high]);

        sap_point_sort(point_list, low, i);
        sap_point_sort(point_list, i + 2, high);
    }
}

void sweep_axis(std::vector<sweep_point>& point_list, const std::vector<BoundingBox3d>& boxes, std::set<sap_key> *overlaps)
{
    int nsize = (int)point_list.size();
    for (int j = 1; j < nsize; j++)
    {
        sweep_point pivot = point_list[j];
        float pivot_value = pivot.value;

        int i = j - 1;

        while (i >= 0 && point_list[i].value > pivot_value)
        {
            sweep_point curr_box = point_list[i];

            if (pivot.is_left() && !curr_box.is_left())
            {
                int id_current = curr_box.index();
                int id_key = pivot.index();
                if (id_current != id_key)
                {
                    if (boxes[id_current].Intersect(boxes[id_key]))
                    {
                        overlaps->insert(sap_pack_key(id_current, id_key));
                    }
                }
            }

            if (!pivot.is_left() && curr_box.is_left())
            {
                overlaps->erase(sap_pack_key(curr_box.index(), pivot.index()));
            }

            point_list[i + 1] = curr_box;
            i = i - 1;
        }
        point_list[i + 1] = pivot;
    }
}

// http://www.codercorner.com/SAP.pdf
void sap_full(const std::vector<BoundingBox3d>& boxes, std::set<sap_key> *overlaps)
{
    std::vector<sweep_point> x;
    std::vector<sweep_point> y;
    std::vector<sweep_point> z;

    int nsize = (int)boxes.size();
    for (int i = 0; i < boxes.size(); ++i)
    {
        x.emplace_back(i, false, boxes[i].Max.x);
        x.emplace_back(i, true, boxes[i].Min.x);

        y.emplace_back(i, false, boxes[i].Max.y);
        y.emplace_back(i, true, boxes[i].Min.y);

        z.emplace_back(i, false, boxes[i].Max.z);
        z.emplace_back(i, true, boxes[i].Min.z);
    }

    sap_point_sort(&x[0], 0, (int)x.size() - 1);
    sap_point_sort(&y[0], 0, (int)y.size() - 1);
    sap_point_sort(&z[0], 0, (int)z.size() - 1);

    overlaps->clear();
    sweep_axis(x, boxes, overlaps);
    sweep_axis(y, boxes, overlaps);
    sweep_axis(y, boxes, overlaps);

    return;
}


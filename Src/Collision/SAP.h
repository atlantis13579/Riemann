
#pragma once

#include <map>
#include <set>
#include <vector>
#include "../Maths/BoundingBox3d.h"

struct sweep_point
{
    sweep_point() {}
    sweep_point(int id, bool left, float *val)
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
    float*          value;
};

typedef unsigned long long sap_key;

static_assert(sizeof(sap_key) == 8, "sizeof sap_key is not valid");

sap_key sap_pack_key(int id1, int id2);

void sap_unpack_key(sap_key key, int *id1, int *id2);

// http://www.codercorner.com/SAP.pdf
void sap_direct(const std::vector<BoundingBox3d>& boxes, std::set<sap_key> *overlaps, int axis_filter = 7);


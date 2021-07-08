#pragma once

#include <initializer_list>
#include <map>
#include <set>
#include <vector>

// Incremental Sweep and Prune (SAP) algorithm
// http://www.codercorner.com/SAP.pdf

class IBoundingVolumeProxy
{
public:
    virtual int     GetBoundingVolumeCount() const  = 0;
    virtual float*  GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const = 0;
    virtual bool    Overlaps(int bv_i, int bv_j) const  = 0;
};

typedef unsigned long long OverlapKey;

static_assert(sizeof(OverlapKey) == 8, "sizeof OverlapKey is not valid");

class IncrementalSAP
{
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

        unsigned int    data;
        float* value;
    };

public:
    IncrementalSAP(IBoundingVolumeProxy* Proxy, std::initializer_list<int> list)
    {
        m_Proxy = Proxy;
        for (const auto& axis : list) {
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
            std::map<OverlapKey, int> overlaps_count;

            int axis_filter = 0;
            for (auto k = 0; k < m_AxisList.size(); ++k)
            {
                InitAxis(m_AxisList[k]);
                QSort(m_Axis[k], 0, (int)m_Axis[k].size() - 1);
                FullPrune(m_Axis[k], overlaps_count, 1 << m_AxisList[k]);
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
            return;
        }

        for (size_t k = 0; k < m_Axis.size(); ++k)
        {
            InsertionSort(m_Axis[k], overlaps);
        }

        return;
    }

private:
    void InitAxis(int k)
    {
        std::vector<SweepPoint>& axis = m_Axis[k];

        int nBV = m_Proxy->GetBoundingVolumeCount();
        axis.resize(2 * nBV);

        for (int i = 0; i < nBV; ++i)
        {
            float *val1 = m_Proxy->GetBoundingVolumeCoordinate(i, true, k);
            float *val2 = m_Proxy->GetBoundingVolumeCoordinate(i, false, k);
            axis[2 * i] = SweepPoint(i, true, val1);
            axis[2 * i + 1] = SweepPoint(i, false, val2);
        }
    }

    void Swap(SweepPoint* a, SweepPoint* b)
    {
        SweepPoint t = *a;
        *a = *b;
        *b = t;
    }

    void QSort(std::vector<SweepPoint>& axis, int low, int high)
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

    void InsertionSort(std::vector<SweepPoint>& axis, std::set<OverlapKey>* overlaps)
    {
        int nsize = (int)axis.size();
        for (int j = 1; j < nsize; j++)
        {
            SweepPoint pivot = axis[j];
            float pivot_value = *pivot.value;

            int i = j - 1;

            while (i >= 0 && *axis[i].value > pivot_value)
            {
                SweepPoint curr = axis[i];

                if (pivot.left() && !curr.left())
                {
                    int id_current = curr.id();
                    int id_pivot = pivot.id();
                    if (id_current != id_pivot)
                    {
                        if (m_Proxy->Overlaps(id_current, id_pivot))
                        {
                            overlaps->insert(PackOverlapKey(id_current, id_pivot));
                        }
                    }
                }

                if (!pivot.left() && curr.left())
                {
                    overlaps->erase(PackOverlapKey(curr.id(), pivot.id()));
                }

                axis[i + 1] = curr;
                i = i - 1;
            }
            axis[i + 1] = pivot;
        }
    }

    void FullPrune(const std::vector<SweepPoint>& axis, std::map<OverlapKey, int>& overlaps_count, int filter)
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
   
public:
    static void UnpackOverlapKey(OverlapKey key, int* id1, int* id2)
    {
        *id1 = key >> 32;
        *id2 = key & 0xFFFFFFFF;
    }

private:
    IBoundingVolumeProxy*                   m_Proxy;
    bool                                    m_IsDirty;
    std::vector<std::vector<SweepPoint>>    m_Axis;
    std::vector<int>                        m_AxisList;
};

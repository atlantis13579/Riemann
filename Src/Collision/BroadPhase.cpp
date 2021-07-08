
#include "BroadPhase.h"
#include "GeometryObject.h"
#include "SAP.h"
#include "SAP_Incremental.h"

void BroadPhaseBruteforceImplementation::ProduceOverlaps(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair>* overlaps)
{
	overlaps->clear();

	for (size_t i = 0; i < AllObjects.size(); ++i)
	for (size_t j = 0; j < AllObjects.size(); ++j)
	{
		if (i == j) continue;
		overlaps->emplace_back(AllObjects[i], AllObjects[j]);
	}
}


class BroadPhaseSAPImplementation : public BroadPhase
{
public:
	BroadPhaseSAPImplementation()
	{
	}

	virtual ~BroadPhaseSAPImplementation()
	{

	}

	void SetBBoxDirty()
	{
		bBBoxDirty = true;
	}

public:

	virtual void ProduceOverlaps(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair>* overlaps)
	{
		if (AllObjects.empty())
		{
			return;
		}

		SAP_Incremental(AllObjects, overlaps);
	}

	void SAP_Direct(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair>* overlaps)
	{
		std::vector<BoundingBox3d> boxes;
		boxes.resize(AllObjects.size());
		for (size_t i = 0; i < AllObjects.size(); ++i)
		{
			boxes[i] = AllObjects[i]->GetBoundingBoxWorld();
		}

		std::set<sap_key> m_overlaps;
		sap_direct(boxes, &m_overlaps);

		GetOverlapPairs(AllObjects, m_overlaps, overlaps);
	}

	void SAP_Incremental(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair>* overlaps)
	{
		if (bBBoxDirty)
		{
			sap_incremental(axis_cache, AllObjects, &overlaps_cache, true, 7);
			bBBoxDirty = false;
			return;
		}

		sap_incremental(axis_cache, AllObjects, &overlaps_cache, false, 7);

		GetOverlapPairs(AllObjects, overlaps_cache, overlaps);
	}

	void GetOverlapPairs(std::vector<GeometryObject*>& AllObjects, const std::set<sap_key>&m_overlaps, std::vector<OverlapPair>* overlaps)
	{
		overlaps->clear();
		for (auto it : m_overlaps)
		{
			int i, j;
			sap_unpack_key(it, &i, &j);
			overlaps->emplace_back(AllObjects[i], AllObjects[j]);
		}
	}

private:
	std::vector<sweep_point>			axis_cache[3];
	std::set<sap_key>					overlaps_cache;
	bool bBBoxDirty;
};


BroadPhase* BroadPhase::CreatSAP()
{
	return new BroadPhaseSAPImplementation();
}
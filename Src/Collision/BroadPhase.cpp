
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
		axis.resize(3);
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

		std::set<sap_key> m_overlaps;

		SAP_Incremental(AllObjects, &m_overlaps);

		GetOverlapPairs(AllObjects, m_overlaps, overlaps);
	}

	void SAP_Direct(std::vector<GeometryObject*>& AllObjects, std::set<sap_key>* overlaps)
	{
		std::vector<BoundingBox3d> boxes;
		boxes.resize(AllObjects.size());
		for (size_t i = 0; i < AllObjects.size(); ++i)
		{
			boxes[i] = AllObjects[i]->GetBoundingBoxWorld();
		}

		sap_direct(boxes, overlaps);
	}

	void SAP_Incremental(std::vector<GeometryObject*>& AllObjects, std::set<sap_key>* overlaps)
	{
		if (bBBoxDirty)
		{
			sap_incremental_init(AllObjects, 0, axis[0]);
			sap_incremental_init(AllObjects, 1, axis[1]);
			sap_incremental_init(AllObjects, 2, axis[2]);
			bBBoxDirty = false;
		}

		sap_incremental(axis, AllObjects, overlaps);
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
	std::vector<std::vector<sweep_point>> axis;
	bool bBBoxDirty;
};


BroadPhase* BroadPhase::CreatSAP()
{
	return new BroadPhaseSAPImplementation();
}
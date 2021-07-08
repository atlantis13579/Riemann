
#include "BroadPhase.h"
#include "GeometryObject.h"
#include "SAP.h"

void BroadPhaseBruteforceImplementation::ProduceOverlaps(const std::vector<GeometryObject*> AllObjects, std::vector<OverlapsPair>* overlaps)
{
	overlaps->clear();

	for (size_t i = 0; i < AllObjects.size(); ++i)
	for (size_t j = 0; j < AllObjects.size(); ++j)
	{
		if (i == j) continue;
		overlaps->emplace_back(AllObjects[i], AllObjects[j]);
	}
}


void BroadPhaseSAPImplementation::ProduceOverlaps(const std::vector<GeometryObject*> AllObjects, std::vector<OverlapsPair>* overlaps)
{
	if (AllObjects.empty())
	{
		return;
	}

	std::vector<BoundingBox3d> boxes;
	boxes.resize(AllObjects.size());
	for (size_t i = 0; i < AllObjects.size(); ++i)
	{
		boxes[i] = AllObjects[i]->GetBoundingBoxWorld();
	}

	std::set<sap_key> m_overlaps;
	sap_direct(boxes, &m_overlaps);

	overlaps->clear();
	for (auto it : m_overlaps)
	{
		int i, j;
		sap_unpack_key(it, &i, &j);
		overlaps->emplace_back(AllObjects[i], AllObjects[j]);
	}
}

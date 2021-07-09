
#include "BroadPhase.h"
#include "GeometryObject.h"
#include "SAP_Incremental.h"

void BroadPhaseBruteforceImplementation::ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair>* overlaps)
{
	overlaps->clear();

	for (size_t i = 0; i < AllObjects.size(); ++i)
	for (size_t j = 0; j < AllObjects.size(); ++j)
	{
		if (i == j) continue;
		overlaps->emplace_back(AllObjects[i], AllObjects[j]);
	}
}


class BroadPhaseSAPImplementation : public BroadPhase, public SAP::BoundingVolumeProxy
{
public:
	BroadPhaseSAPImplementation()
	{
		m_SAP = new IncrementalSAP(this, { 0, 1, 2 });
	}

	virtual ~BroadPhaseSAPImplementation()
	{
		if (m_SAP) delete m_SAP;
	}

	void SetDirty()
	{
		m_Dirty = true;
	}

public:

	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair>* overlaps)
	{
		if (AllObjects.empty())
		{
			return;
		}
		
		m_pObjects = &AllObjects;

		if (m_Dirty)
		{
			m_SAP->SetDirty();
		}

		m_SAP->IncrementalPrune(&m_Overlaps);

		overlaps->clear();
		for (auto it : m_Overlaps)
		{
			int i, j;
			SAP::UnpackOverlapKey(it, &i, &j);
			overlaps->emplace_back(AllObjects[i], AllObjects[j]);
		}
	}

private:
	virtual int     GetBoundingVolumeCount() const
	{
		return (int)m_pObjects->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const
	{
		const BoundingBox3d& box = m_pObjects->at(bv_i)->GetBoundingBoxWorld();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool    Overlaps(int bv_i, int bv_j) const
	{
		const BoundingBox3d& box1 = m_pObjects->at(bv_i)->GetBoundingBoxWorld();
		const BoundingBox3d& box2 = m_pObjects->at(bv_j)->GetBoundingBoxWorld();
		return box1.Intersect(box2);
	}

private:
	bool						m_Dirty;
	IncrementalSAP				*m_SAP;
	std::set<OverlapKey>		m_Overlaps;

	std::vector<Geometry*>* m_pObjects;
};


BroadPhase* BroadPhase::CreatSAP()
{
	return new BroadPhaseSAPImplementation();
}
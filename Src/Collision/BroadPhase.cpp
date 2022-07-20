
#include "BroadPhase.h"
#include "GeometryObject.h"
#include "SAP_Incremental.h"

class BroadPhaseBruteforceImplementation : public BroadPhase
{
public:
	virtual ~BroadPhaseBruteforceImplementation() {}

	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair>* overlaps) override final
	{
		overlaps->clear();

		for (size_t i = 0; i < AllObjects.size(); ++i)
		for (size_t j = 0; j < AllObjects.size(); ++j)
		{
			if (i == j) continue;
			overlaps->emplace_back(AllObjects[i], AllObjects[j]);
		}
	}
};

BroadPhase* BroadPhase::Create_Bruteforce()
{
	return new BroadPhaseBruteforceImplementation();
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
		if (m_SAP)
		{
			delete m_SAP;
		}
	}

	void SetDirty()
	{
		if (m_SAP)
		{
			m_SAP->SetDirty();
		}
	}

public:

	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair>* overlaps) override final
	{
		if (AllObjects.empty())
		{
			return;
		}
		
		m_pObjects = &AllObjects;
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
	virtual int     GetBoundingVolumeCount() const override final
	{
		return (int)m_pObjects->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override final
	{
		const Box3d& box = m_pObjects->at(bv_i)->GetBoundingVolume_WorldSpace();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool    Overlaps(int bv_i, int bv_j) const override final
	{
		const Box3d& box1 = m_pObjects->at(bv_i)->GetBoundingVolume_WorldSpace();
		const Box3d& box2 = m_pObjects->at(bv_j)->GetBoundingVolume_WorldSpace();
		return box1.Intersect(box2);
	}

	virtual uint64_t	CalculateBoundingVolumeHash() const override final
	{
		if (m_pObjects == nullptr || m_pObjects->empty())
		{
			return 0;
		}

		uint64_t hash = 0;
		for (size_t i = 0; i < m_pObjects->size(); ++i)
		{
			hash ^= reinterpret_cast<uint64_t>(m_pObjects->at(i));
			hash *= static_cast<uint64_t>(1099511628211ULL);
		}
		return hash;
	}

private:
	IncrementalSAP				*m_SAP;
	std::set<OverlapKey>		m_Overlaps;
	std::vector<Geometry*>*		m_pObjects;
};


BroadPhase* BroadPhase::Create_SAP()
{
	return new BroadPhaseSAPImplementation();
}



#include "BroadPhase.h"
#include "RigidBody.h"
#include "../Collision/DynamicAABBTree.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/SAP_Incremental.h"

static bool IsMovingRigid(Geometry* geom)
{
	RigidBody *body = geom->GetParent<RigidBody>();
	if (body == nullptr)
	{
		return false;
	}
	
	if (body->mRigidType == RigidType::Static)
	{
		return false;
	}

	return true;
}

static bool HasMovingRigid(Geometry* geom1, Geometry* geom2)
{
	return IsMovingRigid(geom1) || IsMovingRigid(geom2);
}

class BroadPhaseAllPairsImplementation : public BroadPhase
{
public:
	virtual ~BroadPhaseAllPairsImplementation() {}

	virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
	{
		overlaps->clear();

		int n = (int)geoms.size();
		for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			if (i == j) continue;
			overlaps->emplace_back(i, j);
		}
	}
};

class BroadPhaseBruteforceImplementation : public BroadPhase
{
public:
	virtual ~BroadPhaseBruteforceImplementation() {}

	virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
	{
		overlaps->clear();

		int n = (int)geoms.size();
		for (int i = 0; i < n; ++i)
		for (int j = i + 1; j < n; ++j)
		{
			const Box3d& box1 = geoms[i]->GetBoundingVolume_WorldSpace();
			const Box3d& box2 = geoms[j]->GetBoundingVolume_WorldSpace();
			if (box1.Intersect(box2))
			{
				Geometry *gi = geoms[i];
				Geometry *gj = geoms[j];
				if (!HasMovingRigid(gi, gj))
					continue;
				overlaps->emplace_back(i, j);
			}
		}
	}
};

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

	virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
	{
		if (geoms.empty())
		{
			return;
		}
		
		m_pObjects = &geoms;
		m_SAP->IncrementalPrune(&m_Overlaps);

		overlaps->clear();
		for (auto it : m_Overlaps)
		{
			int i, j;
			SAP::UnpackOverlapKey(it, &i, &j);
			Geometry *gi = geoms[i];
			Geometry *gj = geoms[j];
			if (!HasMovingRigid(gi, gj))
				continue;
			overlaps->emplace_back(i, j);
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
	IncrementalSAP					*m_SAP;
	std::set<OverlapKey>			m_Overlaps;
	const std::vector<Geometry*>*	m_pObjects;
};

class BroadPhaseDynamicAABBImplementation : public BroadPhase
{
public:
	BroadPhaseDynamicAABBImplementation(DynamicAABBTree *tree)
	{
		m_tree = tree;
	}
	virtual ~BroadPhaseDynamicAABBImplementation() {}

	virtual void ProduceOverlaps(const std::vector<Geometry*>& geoms, std::vector<OverlapPair>* overlaps) override final
	{
		std::vector<void*> result;
		int n = (int)geoms.size();
		for (int i = 0; i < n; ++i)
		{
			Geometry *gi = geoms[i];
			if (!IsMovingRigid(gi))
				continue;
			
			m_tree->Query(gi->GetBoundingVolume_WorldSpace(), &result);
			for (int j = 0; j < (int)result.size(); ++j)
			{
				Geometry *gj = static_cast<Geometry*>(result[j]);
				if (gj <= gi)
					continue;
				overlaps->emplace_back(i, j);
			}
		}
	}
	
private:
	DynamicAABBTree *m_tree;
};

BroadPhase* BroadPhase::Create_SAP()
{
	return new BroadPhaseSAPImplementation();
}

BroadPhase* BroadPhase::Create_DynamicAABB(DynamicAABBTree *tree)
{
	return new BroadPhaseDynamicAABBImplementation(tree);
}

BroadPhase* BroadPhase::Create_Bruteforce()
{
	return new BroadPhaseBruteforceImplementation();
}

BroadPhase* BroadPhase::Create_AllPairs()
{
	return new BroadPhaseAllPairsImplementation();
}

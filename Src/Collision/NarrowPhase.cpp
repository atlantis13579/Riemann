
#include "NarrowPhase.h"

#include "EPA.h"
#include "GJK.h"

class NarrowPhase_GJK_EPA : public NarrowPhase
{
public:
	NarrowPhase_GJK_EPA()
	{

	}
	virtual ~NarrowPhase_GJK_EPA()
	{

	}

	virtual void ProduceCollision(std::vector<OverlapPair>& overlaps, std::vector<CollisionPair>* collides) override
	{
		collides->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			if (GJK(overlaps[i].Geom1, overlaps[i].Geom2))
			{
				collides->emplace_back(overlaps[i].Geom1, overlaps[i].Geom2);
				EPA(collides->back());
			}
		}
	}

	bool GJK(const Geometry *Geom1, const Geometry* Geom2)
	{
		return true;
	}

	void EPA(CollisionPair& collide_pair)
	{
	}

private:


};

NarrowPhase* NarrowPhase::Create_GJK_EPA()
{
	return new NarrowPhase_GJK_EPA();
}
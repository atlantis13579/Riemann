
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

	virtual void ProduceCollision(std::vector<OverlapPair>& overlaps, std::vector<ContactPair>* collides) override
	{
		collides->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			if (Collision_Detection_GJK(overlaps[i].Geom1, overlaps[i].Geom2))
			{
				collides->emplace_back(overlaps[i].Geom1, overlaps[i].Geom2);
				ContactGeneration_EPA(collides->back());
			}
		}
	}

	bool Collision_Detection_GJK(const Geometry *Geom1, const Geometry* Geom2)
	{
		return true;
	}

	void ContactGeneration_EPA(ContactPair& collide_pair)
	{
	}

private:


};

NarrowPhase* NarrowPhase::Create_GJK_EPA()
{
	return new NarrowPhase_GJK_EPA();
}
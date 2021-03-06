
#pragma once

#include <vector>

class Geometry;
class DynamicAABBTree;

struct OverlapPair
{
	OverlapPair(Geometry* _Geom1, Geometry* _Geom2)
	{
		Geom1 = _Geom1;
		Geom2 = _Geom2;
	}
	Geometry* Geom1;
	Geometry* Geom2;
};

class BroadPhase
{
public:
	virtual ~BroadPhase() {}
	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair> *overlaps) = 0;

	static BroadPhase* Create_AllPairs();		// Slow, debug purpose
	static BroadPhase* Create_Bruteforce();		// Slow, debug purpose
	static BroadPhase* Create_SAP();
	static BroadPhase* Create_DynamicAABB(DynamicAABBTree *tree);
};

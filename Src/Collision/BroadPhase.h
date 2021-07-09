
#pragma once

#include <vector>

class Geometry;

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
	~BroadPhase() {}
	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair> *overlaps) = 0;

	static BroadPhase* Create_SAP();
};


// Slow, Testing purpose
class BroadPhaseBruteforceImplementation : public BroadPhase
{
public:
	~BroadPhaseBruteforceImplementation() {}

	virtual void ProduceOverlaps(std::vector<Geometry*>& AllObjects, std::vector<OverlapPair>* overlaps);
};


#pragma once

#include <vector>
#include "../Maths/BoundingBox3d.h"

class GeometryObject;

struct OverlapPair
{
	OverlapPair(GeometryObject* _Geom1, GeometryObject* _Geom2)
	{
		Geom1 = _Geom1;
		Geom2 = _Geom2;
	}
	GeometryObject* Geom1;
	GeometryObject* Geom2;
};

class BroadPhase
{
public:
	~BroadPhase() {}
	virtual void ProduceOverlaps(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair> *overlaps) = 0;

	static BroadPhase* CreatSAP();
};


// Slow, Testing purpose
class BroadPhaseBruteforceImplementation : public BroadPhase
{
public:
	~BroadPhaseBruteforceImplementation() {}

	virtual void ProduceOverlaps(std::vector<GeometryObject*>& AllObjects, std::vector<OverlapPair>* overlaps);
};


#pragma once

#include <vector>
#include "../Maths/BoundingBox3d.h"

class GeometryObject;

struct OverlapsPair
{
	OverlapsPair(GeometryObject* _Geom1, GeometryObject* _Geom2)
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
	virtual void ProduceOverlaps(const std::vector<GeometryObject*> AllObjects, std::vector<OverlapsPair> *overlaps) = 0;
};


// Slow, Testing purpose
class BroadPhaseBruteforceImplementation : public BroadPhase
{
public:
	virtual void ProduceOverlaps(const std::vector<GeometryObject*> AllObjects, std::vector<OverlapsPair>* overlaps);
};


class BroadPhaseSAPImplementation : public BroadPhase
{
public:
	BroadPhaseSAPImplementation() {}
	~BroadPhaseSAPImplementation() {}

public:
	virtual void ProduceOverlaps(const std::vector<GeometryObject*> AllObjects, std::vector<OverlapsPair>* overlaps);
};
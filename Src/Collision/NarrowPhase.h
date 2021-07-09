
#pragma once

#include <vector>

#include "BroadPhase.h"

class Geometry;

struct ContactPair
{
	ContactPair(Geometry* _Geom1, Geometry* _Geom2)
	{
		Geom1 = _Geom1;
		Geom2 = _Geom2;
	}
	Geometry* Geom1;
	Geometry* Geom2;

	// Contact Informations
};

class NarrowPhase
{
public:
	~NarrowPhase() {}
	virtual void ProduceCollision(std::vector<OverlapPair>& overlaps, std::vector<ContactPair>* collides) = 0;

	static NarrowPhase* Create_GJK_EPA();
};
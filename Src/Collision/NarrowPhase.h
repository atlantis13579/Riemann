
#pragma once

#include <vector>

#include "BroadPhase.h"

class Geometry;

struct CollisionPair
{
	CollisionPair(Geometry* _Geom1, Geometry* _Geom2)
	{
		Geom1 = _Geom1;
		Geom2 = _Geom2;
	}
	Geometry* Geom1;
	Geometry* Geom2;

	// Collisiom
};

class NarrowPhase
{
public:
	~NarrowPhase() {}
	virtual void ProduceCollision(std::vector<OverlapPair>& overlaps, std::vector<CollisionPair>* collides) = 0;

	static NarrowPhase* Create_GJK_EPA();
};
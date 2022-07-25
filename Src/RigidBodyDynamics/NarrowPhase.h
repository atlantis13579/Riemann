
#pragma once

#include <vector>

#include "BroadPhase.h"

class ContactManifold;

class NarrowPhase
{
public:
	virtual ~NarrowPhase() {}
	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold*>* contact) = 0;

	static NarrowPhase* Create_GJKEPA();
};

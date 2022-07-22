#pragma once

#include <vector>
#include "Contact.h"

class WarmStart
{
public:
	static void ApplyVelocityConstraint(std::vector<ContactManifold> &manifolds, float dt);
	static void Apply(Geometry *Geom1, Geometry *Geom2, Contact& contact, float dt);
};
#pragma once

#include <vector>
#include "../Collision/Contact.h"

class WarmStart
{
public:
	static void Manifolds(std::vector<ContactManifold> &manifolds, float dt);
	static void Contact(Geometry *Geom1, Geometry *Geom2, ContactResult& contact, float dt);
};
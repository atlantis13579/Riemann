#pragma once

#include <vector>

class Geometry;
struct Contact;
class ContactManifold;

class WarmStart
{
public:
	static void ApplyVelocityConstraint(std::vector<ContactManifold*> &manifolds, float dt);
	static void Apply(Geometry *Geom1, Geometry *Geom2, Contact& contact, float dt);
};

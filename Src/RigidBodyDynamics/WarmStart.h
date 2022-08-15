#pragma once

#include <vector>

class Geometry;
struct Contact;
class ContactManifold;
class RigidBody;

class WarmStart
{
public:
	static void ApplyVelocityConstraint(const std::vector<Geometry*>& AllObjects, std::vector<ContactManifold*> &manifolds, float dt);
	static void Apply(RigidBody *BodyA, RigidBody *BodyB, Contact& contact, float dt);
};

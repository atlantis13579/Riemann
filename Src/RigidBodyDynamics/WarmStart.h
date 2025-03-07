#pragma once

#include <vector>

namespace Riemann
{
	class Geometry;
	struct Contact;
	class ContactManifold;
	class RigidBody;

	class WarmStart
	{
	public:
		static void ApplyVelocityConstraint(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds, float dt);
		static void Apply(RigidBody* BodyA, RigidBody* BodyB, Contact& contact, float dt);
	};
}
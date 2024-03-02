#pragma once

#include <vector>

namespace Riemann
{
	class GeometryBase;
	struct Contact;
	class ContactManifold;
	class RigidBody;

	class WarmStart
	{
	public:
		static void ApplyVelocityConstraint(const std::vector<GeometryBase*>& geoms, std::vector<ContactManifold*>& manifolds, float dt);
		static void Apply(RigidBody* BodyA, RigidBody* BodyB, Contact& contact, float dt);
	};
}
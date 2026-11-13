#pragma once

#include <vector>

namespace Riemann
{
	class Geometry;
	struct Contact;
	class ContactManifold;
	class RigidBody;
	struct ContactJacobianSolver;
	struct ContactPersistenceSettings;

	class WarmStart
	{
	public:
		static void ApplyVelocityConstraint(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds, float dt);
		static void Apply(RigidBody* BodyA, RigidBody* BodyB, Contact& contact, float dt);
		static void Apply(ContactJacobianSolver& Constraint, float Dt, const ContactPersistenceSettings& Settings);
	};
}

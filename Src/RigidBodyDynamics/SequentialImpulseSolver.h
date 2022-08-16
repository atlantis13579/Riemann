#pragma once

#include <vector>

class ContactManifold;

class ConstraintSolver
{
public:
	virtual ~ConstraintSolver() {}
	virtual void	ResolveContact(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds, float dt) = 0;

	static ConstraintSolver* CreateSequentialImpulseSolver();
};

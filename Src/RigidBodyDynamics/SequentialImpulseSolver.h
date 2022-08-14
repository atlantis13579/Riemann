#pragma once

#include <vector>

class ContactManifold;

class ResolutionPhase
{
public:
	virtual ~ResolutionPhase() {}
	virtual void	ResolveContact(const std::vector<Geometry*>& AllObjects, std::vector<ContactManifold*>& manifolds, float dt) = 0;

	static ResolutionPhase* CreateSequentialImpulseSolver();
};

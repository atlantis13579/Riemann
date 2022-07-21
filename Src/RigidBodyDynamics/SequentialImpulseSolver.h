#pragma once

#include <vector>
#include "Contact.h"

class ResolutionPhase
{
public:
	virtual ~ResolutionPhase() {}
	virtual void	ResolveContact(std::vector<ContactManifold>& manifolds, float dt) = 0;

	static ResolutionPhase* CreateSequentialImpulseSolver();
};

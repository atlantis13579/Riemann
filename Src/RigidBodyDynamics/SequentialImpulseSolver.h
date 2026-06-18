#pragma once

#include <vector>

namespace Riemann
{
	class ContactManifold;

	class ConstraintSolver
	{
	public:
		virtual ~ConstraintSolver() {}
		virtual void	PreResolve(const std::vector<Geometry*>& geoms) = 0;
		virtual void	ResolveContact(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds, float dt) = 0;
		virtual void	PostResolve(const std::vector<Geometry*>& geoms) = 0;

		static ConstraintSolver* CreateSequentialImpulseSolver();
	};
}
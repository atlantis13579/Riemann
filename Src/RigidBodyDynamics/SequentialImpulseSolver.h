#pragma once

#include <vector>

namespace Riemann
{
	class ContactManifold;

	class ConstraintSolver
	{
	public:
		virtual ~ConstraintSolver() {}
		virtual void	PreResolve(const std::vector<GeometryBase*>& geoms) = 0;
		virtual void	ResolveContact(const std::vector<GeometryBase*>& geoms, std::vector<ContactManifold*>& manifolds, float dt) = 0;
		virtual void	PostResolve(const std::vector<GeometryBase*>& geoms) = 0;

		static ConstraintSolver* CreateSequentialImpulseSolver();
	};
}
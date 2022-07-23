
#include "SequentialImpulseSolver.h"
#include "Jacobian.h"
#include "WarmStart.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"


class SequentialImpulseSolver : public ResolutionPhase
{
public:
	SequentialImpulseSolver()
	{
		m_nMaxVelocityIterations = 8;
	}

	virtual ~SequentialImpulseSolver()
	{
	}

	virtual void	ResolveContact(std::vector<ContactManifold>& manifolds, float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::ApplyVelocityConstraint(manifolds, dt);

		std::vector<ContactVelocityConstraintSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				ContactManifold* manifold = &manifolds[i];
				RigidBody* bodyA = static_cast<RigidBody*>(manifold->GeomA->GetEntity());
				RigidBody* bodyB = static_cast<RigidBody*>(manifold->GeomB->GetEntity());

				velocityConstraints.push_back(ContactVelocityConstraintSolver());
				velocityConstraints.back().Setup(&manifold->ContactPoints[j], bodyA, bodyB, dt);
			}
		}

		int it = 0;
		while (it++ <= m_nMaxVelocityIterations)
		{
			for (size_t k = 0; k < velocityConstraints.size(); ++k)
			{
				velocityConstraints[k].Solve();
			}
		}

		for (size_t k = 0; k < velocityConstraints.size(); ++k)
		{
			velocityConstraints[k].Finalize();
		}
	}

private:
	int m_nMaxVelocityIterations;
};

ResolutionPhase* ResolutionPhase::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}

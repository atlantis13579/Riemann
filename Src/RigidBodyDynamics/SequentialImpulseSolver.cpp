
#include "Contact.h"
#include "PositionConstraint.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "SequentialImpulseSolver.h"
#include "WarmStart.h"
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

	virtual void	ResolveContact(std::vector<ContactManifold*>& manifolds, float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::ApplyVelocityConstraint(manifolds, dt);

		std::vector<ContactVelocityConstraintSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i]->NumContactPointCount; ++j)
			{
				ContactManifold* manifold = manifolds[i];
				RigidBody* bodyA = manifold->GeomA->GetParent<RigidBody>();
				RigidBody* bodyB = manifold->GeomB->GetParent<RigidBody>();

				velocityConstraints.push_back(ContactVelocityConstraintSolver());
				velocityConstraints.back().Setup(&manifold->ContactPoints[j], bodyA, bodyB, dt);
			}
		}

		if (!velocityConstraints.empty())
		{
			int it = 0;
			const float kStopVelocityIterationThreshold = 1e-6f;
			while (it++ <= m_nMaxVelocityIterations)
			{
				float sumSqrError = 0.0;
				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					velocityConstraints[k].Solve();

					sumSqrError += velocityConstraints[k].GetSquaredError();
				}

				sumSqrError /= velocityConstraints.size();
				if (sumSqrError < kStopVelocityIterationThreshold)
				{
					break;
				}
			}

			for (size_t k = 0; k < velocityConstraints.size(); ++k)
			{
				velocityConstraints[k].Finalize();
			}

			return;
		}


	}

private:
	int 	m_nMaxVelocityIterations;
	int 	m_nMaxPositionIterations;
	
	std::vector<PositionConstraint>	m_PositionConstraints;
};

ResolutionPhase* ResolutionPhase::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}

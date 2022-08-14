
#include "CollidingContact.h"
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

	virtual void	ResolveContact(const std::vector<Geometry*>& AllObjects,
								   std::vector<ContactManifold*>& manifolds,
								   float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::ApplyVelocityConstraint(AllObjects, manifolds, dt);
		
		if (m_PhaseSpace.size() < AllObjects.size())
		{
			m_PhaseSpace.resize(AllObjects.size());
		}
		for (size_t i = 0; i < AllObjects.size(); ++i)
		{
			RigidBody* body = AllObjects[i]->GetParent<RigidBody>();
			m_PhaseSpace[i].v = body->GetLinearVelocity();
			m_PhaseSpace[i].w = body->GetAngularVelocity();
		}

		std::vector<ContactVelocityConstraintSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i]->NumContactPointCount; ++j)
			{
				ContactManifold* manifold = manifolds[i];
				RigidBody* bodyA = AllObjects[manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = AllObjects[manifold->indexB]->GetParent<RigidBody>();

				velocityConstraints.push_back(ContactVelocityConstraintSolver(
					m_PhaseSpace.data(), manifold->indexA, manifold->indexB, bodyA, bodyB));
				velocityConstraints.back().Setup(&manifold->ContactPoints[j], dt);
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
			
			for (size_t i = 0; i < AllObjects.size(); ++i)
			{
				RigidBody* body = AllObjects[i]->GetParent<RigidBody>();
				body->SetLinearVelocity(m_PhaseSpace[i].v);
				body->SetAngularVelocity(m_PhaseSpace[i].w);
			}

			return;
		}


	}

private:
	int 	m_nMaxVelocityIterations;
	int 	m_nMaxPositionIterations;
	
	std::vector<PositionConstraint*>	m_PositionConstraints;
	std::vector<GeneralizedVelocity>	m_PhaseSpace;
};

ResolutionPhase* ResolutionPhase::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}

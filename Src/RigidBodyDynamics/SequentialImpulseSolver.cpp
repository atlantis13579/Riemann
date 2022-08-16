
#include "CollidingContact.h"
#include "PositionConstraint.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "SequentialImpulseSolver.h"
#include "WarmStart.h"
#include "../Collision/GeometryObject.h"

class SequentialImpulseSolver : public ConstraintSolver
{
public:
	SequentialImpulseSolver()
	{
		m_nMaxVelocityIterations = 8;
	}

	virtual ~SequentialImpulseSolver()
	{
	}

	virtual void	ResolveContact(const std::vector<Geometry*>& geoms,
								   std::vector<ContactManifold*>& manifolds,
								   float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::ApplyVelocityConstraint(geoms, manifolds, dt);
		
		if (m_Buffer.size() < geoms.size())
		{
			m_Buffer.resize(geoms.size());
		}
		for (size_t i = 0; i < geoms.size(); ++i)
		{
			RigidBody* body = geoms[i]->GetParent<RigidBody>();
			m_Buffer[i].v = body->GetLinearVelocity();
			m_Buffer[i].w = body->GetAngularVelocity();
		}

		std::vector<ContactVelocityConstraintSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i]->NumContactPointCount; ++j)
			{
				ContactManifold* manifold = manifolds[i];
				RigidBody* bodyA = geoms[manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = geoms[manifold->indexB]->GetParent<RigidBody>();

				velocityConstraints.push_back(ContactVelocityConstraintSolver(
					m_Buffer.data(), manifold->indexA, manifold->indexB, bodyA, bodyB));
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
			
			for (size_t i = 0; i < geoms.size(); ++i)
			{
				RigidBody* body = geoms[i]->GetParent<RigidBody>();
				body->SetLinearVelocity(m_Buffer[i].v);
				body->SetAngularVelocity(m_Buffer[i].w);
			}

			return;
		}


	}

private:
	int 	m_nMaxVelocityIterations;
	int 	m_nMaxPositionIterations;
	
	std::vector<PositionConstraint*>	m_PositionConstraints;
	std::vector<GeneralizedVelocity>	m_Buffer;
};

ConstraintSolver* ConstraintSolver::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}


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
		m_nMaxPositionIterations = 8;
		m_nMaxVelocityIterations = 1;
	}

	virtual ~SequentialImpulseSolver()
	{
	}
	
	virtual void	PreResolve(const std::vector<Geometry*>& geoms) override final
	{
		int n = (int)geoms.size();
		if (m_Buffer.size() < 2 * n)
		{
			m_Buffer.resize(2 * n);
		}
		for (int i = 0; i < n; ++i)
		{
			RigidBody* body = geoms[i]->GetParent<RigidBody>();
			m_Buffer[i].v = body->GetLinearVelocity();
			m_Buffer[i].w = body->GetAngularVelocity();
		}
		for (int i = n; i < 2 * n; ++i)
		{
			RigidBody* body = geoms[i - n]->GetParent<RigidBody>();
			m_Buffer[i].v = body->GetLinearVelocity();
			m_Buffer[i].w = body->GetAngularVelocity();
		}
	}

	virtual void	ResolveContact(const std::vector<Geometry*>& geoms,
								   std::vector<ContactManifold*>& manifolds,
								   float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::ApplyVelocityConstraint(geoms, manifolds, dt);

		std::vector<ContactJacobianSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i]->NumContactPointCount; ++j)
			{
				ContactManifold* manifold = manifolds[i];
				RigidBody* bodyA = geoms[manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = geoms[manifold->indexB]->GetParent<RigidBody>();

				velocityConstraints.push_back(ContactJacobianSolver(
					m_Buffer.data(), manifold->indexA, manifold->indexB, bodyA, bodyB));
				velocityConstraints.back().SetupPositionPass(&manifold->ContactPoints[j], dt);
			}
		}

		if (!velocityConstraints.empty())
		{
			int it = 0;
			const float kStopPositionIterationThreshold = 1e-6f;
			while (it++ <= m_nMaxPositionIterations)
			{
				float sumSqrError = 0.0;
				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					velocityConstraints[k].Solve();

					sumSqrError += velocityConstraints[k].GetSquaredError();
				}

				sumSqrError /= velocityConstraints.size();
				if (sumSqrError < kStopPositionIterationThreshold)
				{
					break;
				}
			}
			
			for (size_t k = 0; k < velocityConstraints.size(); ++k)
			{
				velocityConstraints[k].Finalize();
			}
			
			for (size_t k = 0; k < velocityConstraints.size(); ++k)
			{
				velocityConstraints[k].SetupVelocityPass((int)m_Buffer.size() / 2);
			}
			
			it = 0;
			while (it++ <= m_nMaxVelocityIterations)
			{
				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					velocityConstraints[k].Solve();
				}
			}
			
			return;
		}
	}
	
	virtual void	PostResolve(const std::vector<Geometry*>& geoms) override final
	{
		int n = (int)geoms.size();
		for (int i = 0; i < n; ++i)
		{
			RigidBodyDynamic* body = geoms[i]->GetParent<RigidBody>()->CastDynamic();
			if (body)
			{
				body->SetLinearVelocity(m_Buffer[i].v);
				body->SetAngularVelocity(m_Buffer[i].w);
				body->SolverV = m_Buffer[i + n].v;
				body->SolverW = m_Buffer[i + n].w;
			}
		}
	}

private:
	int 	m_nMaxPositionIterations;
	int 	m_nMaxVelocityIterations;
	
	std::vector<PositionConstraint*>	m_PositionConstraints;
	std::vector<GeneralizedVelocity>	m_Buffer;
};

ConstraintSolver* ConstraintSolver::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}

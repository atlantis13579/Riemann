
#include "CollidingContact.h"
#include "PositionConstraint.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "SequentialImpulseSolver.h"
#include "WarmStart.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
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
				m_Buffer[i].v = m_Buffer[i].w = Vector3(INFINITY, INFINITY, INFINITY);
			}
		}

		virtual void	ResolveContact(const std::vector<Geometry*>& geoms,
			std::vector<ContactManifold*>& manifolds,
			float dt) override final
		{
			if (manifolds.empty())
				return;

			WarmStart::ApplyVelocityConstraint(geoms, manifolds, dt);

			int n = (int)geoms.size();
			std::vector<ContactJacobianSolver> velocityConstraints;
			for (size_t i = 0; i < manifolds.size(); ++i)
			{
				ContactManifold* manifold = manifolds[i];
				RigidBody* bodyA = geoms[manifold->indexA]->GetParent<RigidBody>();
				RigidBody* bodyB = geoms[manifold->indexB]->GetParent<RigidBody>();

				for (int j = 0; j < manifold->NumContactPointCount; ++j)
				{
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

				int k = 0;
				for (size_t i = 0; i < manifolds.size(); ++i)
				{
					ContactManifold* manifold = manifolds[i];
					RigidBody* bodyA = geoms[manifold->indexA]->GetParent<RigidBody>();
					RigidBody* bodyB = geoms[manifold->indexB]->GetParent<RigidBody>();

					m_Buffer[n + manifold->indexA].v = bodyA->GetLinearVelocity();
					m_Buffer[n + manifold->indexA].w = bodyA->GetAngularVelocity();
					m_Buffer[n + manifold->indexB].v = bodyB->GetLinearVelocity();
					m_Buffer[n + manifold->indexB].w = bodyB->GetAngularVelocity();

					for (int j = 0; j < manifold->NumContactPointCount; ++j)
					{
						velocityConstraints[k].SetupVelocityPass(&manifold->ContactPoints[j], dt, n);
						k++;
					}
				}

				it = 0;
				while (it++ <= m_nMaxVelocityIterations)
				{
					for (size_t i = 0; i < velocityConstraints.size(); ++i)
					{
						velocityConstraints[i].Solve();
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
}
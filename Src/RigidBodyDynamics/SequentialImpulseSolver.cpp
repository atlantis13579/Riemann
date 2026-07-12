
#include "CollidingContact.h"
#include "Constraints.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "SequentialImpulseSolver.h"
#include "WarmStart.h"
#include "../Collision/GeometryObject.h"

#include <algorithm>
#include <unordered_map>

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
			m_GeometryToBodySlot.assign(geoms.size(), -1);
			m_Bodies.clear();

			std::unordered_map<RigidBody*, int> BodySlots;
			BodySlots.reserve(geoms.size());
			for (size_t GeometryIndex = 0; GeometryIndex < geoms.size(); ++GeometryIndex)
			{
				Geometry* Geom = geoms[GeometryIndex];
				RigidBody* Body = Geom ? Geom->GetParent<RigidBody>() : nullptr;
				if (Body == nullptr)
				{
					continue;
				}

				auto It = BodySlots.find(Body);
				if (It == BodySlots.end())
				{
					const int BodySlot = (int)m_Bodies.size();
					BodySlots.emplace(Body, BodySlot);
					m_Bodies.push_back(Body);
					m_GeometryToBodySlot[GeometryIndex] = BodySlot;
				}
				else
				{
					m_GeometryToBodySlot[GeometryIndex] = It->second;
				}
			}

			const int BodyCount = (int)m_Bodies.size();
			m_Buffer.resize(2 * BodyCount);
			for (int BodySlot = 0; BodySlot < BodyCount; ++BodySlot)
			{
				RigidBody* Body = m_Bodies[(size_t)BodySlot];
				m_Buffer[(size_t)BodySlot].v = Body->GetLinearVelocity();
				m_Buffer[(size_t)BodySlot].w = Body->GetAngularVelocity();
				m_Buffer[(size_t)BodyCount + BodySlot] = m_Buffer[(size_t)BodySlot];
			}
		}

		virtual void	ResolveContact(const std::vector<Geometry*>& geoms,
			std::vector<ContactManifold*>& manifolds,
			float dt) override final
		{
			if (manifolds.empty() || m_Bodies.empty())
				return;

			const int BodyCount = (int)m_Bodies.size();
			auto GetBodySlot = [this, &geoms](int GeometryIndex, RigidBody*& Body) -> int
			{
				Body = nullptr;
				if (GeometryIndex < 0 || (size_t)GeometryIndex >= geoms.size() ||
					(size_t)GeometryIndex >= m_GeometryToBodySlot.size() || geoms[(size_t)GeometryIndex] == nullptr)
				{
					return -1;
				}

				const int BodySlot = m_GeometryToBodySlot[(size_t)GeometryIndex];
				if (BodySlot < 0 || (size_t)BodySlot >= m_Bodies.size())
				{
					return -1;
				}

				Body = m_Bodies[(size_t)BodySlot];
				if (Body == nullptr || geoms[(size_t)GeometryIndex]->GetParent<RigidBody>() != Body)
				{
					Body = nullptr;
					return -1;
				}
				return BodySlot;
			};

			struct ManifoldContext
			{
				ContactManifold* Manifold;
				RigidBody* BodyA;
				RigidBody* BodyB;
				int BodySlotA;
				int BodySlotB;
			};

			std::vector<ManifoldContext> ValidManifolds;
			std::vector<ContactManifold*> WarmStartManifolds;
			for (size_t i = 0; i < manifolds.size(); ++i)
			{
				ContactManifold* manifold = manifolds[i];
				if (manifold == nullptr || manifold->NumContactPointCount <= 0)
				{
					continue;
				}

				RigidBody* bodyA = nullptr;
				RigidBody* bodyB = nullptr;
				const int BodySlotA = GetBodySlot(manifold->indexA, bodyA);
				const int BodySlotB = GetBodySlot(manifold->indexB, bodyB);
				if (BodySlotA < 0 || BodySlotB < 0 || BodySlotA == BodySlotB)
				{
					continue;
				}
				ValidManifolds.push_back({ manifold, bodyA, bodyB, BodySlotA, BodySlotB });
				WarmStartManifolds.push_back(manifold);
			}

			WarmStart::ApplyVelocityConstraint(geoms, WarmStartManifolds, dt);

			std::vector<ContactJacobianSolver> velocityConstraints;
			for (const ManifoldContext& Context : ValidManifolds)
			{
				const int ContactCount = std::min(Context.Manifold->NumContactPointCount, MAX_CONTACT_POINTS);
				for (int j = 0; j < ContactCount; ++j)
				{
					velocityConstraints.push_back(ContactJacobianSolver(
						m_Buffer.data(), Context.BodySlotA, Context.BodySlotB, Context.BodyA, Context.BodyB));
					velocityConstraints.back().SetupPositionPass(&Context.Manifold->ContactPoints[j], dt);
				}
			}

			if (!velocityConstraints.empty())
			{
				int it = 0;
				const float kStopPositionIterationThreshold = 1e-6f;
				while (it++ < m_nMaxPositionIterations)
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

				for (ContactJacobianSolver& Constraint : velocityConstraints)
				{
					Constraint.SetupVelocityPass(Constraint.m_contact, dt, 0);
				}

				it = 0;
				while (it++ < m_nMaxVelocityIterations)
				{
					for (size_t i = 0; i < velocityConstraints.size(); ++i)
					{
						velocityConstraints[i].Solve();
					}
				}

				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					velocityConstraints[k].Finalize(true);
				}

				return;
			}
		}

		virtual void	PostResolve(const std::vector<Geometry*>& geoms) override final
		{
			(void)geoms;
			const int BodyCount = (int)m_Bodies.size();
			if (m_Buffer.size() < (size_t)(2 * BodyCount))
			{
				return;
			}

			for (int BodySlot = 0; BodySlot < BodyCount; ++BodySlot)
			{
				RigidBody* Body = m_Bodies[(size_t)BodySlot];
				RigidBodyDynamic* body = Body ? Body->CastDynamic() : nullptr;
				if (body)
				{
					body->SetLinearVelocity(m_Buffer[(size_t)BodySlot].v);
					body->SetAngularVelocity(m_Buffer[(size_t)BodySlot].w);
					body->SolverV = m_Buffer[(size_t)BodySlot].v;
					body->SolverW = m_Buffer[(size_t)BodySlot].w;
				}
			}
		}

	private:
		int 	m_nMaxPositionIterations;
		int 	m_nMaxVelocityIterations;

		std::vector<PositionConstraint*>	m_PositionConstraints;
		std::vector<int>					m_GeometryToBodySlot;
		std::vector<RigidBody*>			m_Bodies;
		std::vector<GeneralizedVelocity>	m_Buffer;
	};

	ConstraintSolver* ConstraintSolver::CreateSequentialImpulseSolver()
	{
		return new SequentialImpulseSolver();
	}
}

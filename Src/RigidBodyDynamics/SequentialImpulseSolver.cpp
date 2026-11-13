
#include "CollidingContact.h"
#include "Ragdoll.h"
#include "Constraints.h"
#include "Island.h"
#include "FrictionPatch.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "SequentialImpulseSolver.h"
#include "WarmStart.h"
#include "../Collision/GeometryObject.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>

namespace Riemann
{
	class SequentialImpulseSolver : public ConstraintSolver
	{
	public:
		explicit SequentialImpulseSolver(const ContactPersistenceSettings& Settings, const SequentialImpulseSettings& Iterations)
			: m_PersistenceSettings(Settings), m_ImpulseCache(Settings)
		{
			if (!Iterations.IsValid()) throw std::invalid_argument("Invalid sequential impulse iteration settings");
			m_nMaxPositionIterations = Iterations.PositionIterations;
			m_nMaxVelocityIterations = Iterations.VelocityIterations;
		}

		virtual ~SequentialImpulseSolver()
		{
		}

		void BeginFrame(uint64_t Frame) override
		{
			m_ImpulseCache.BeginFrame(Frame);
			m_Friction.BeginFrame(Frame);
			m_WarmStartContactCount = 0;
		}
		void ResetState() override { m_ImpulseCache.Reset(); m_Friction.Reset(); m_WarmStartContactCount = 0; }
		void SetFrictionPatchSettings(const FrictionPatchSettings& Settings) override { m_Friction.SetSettings(Settings); }
		void SetWarmStartingEnabled(bool Enabled) override
		{
			if (m_WarmStartingEnabled != Enabled) ResetState();
			m_WarmStartingEnabled = Enabled;
			m_Friction.SetWarmStartingEnabled(Enabled);
		}
		int GetWarmStartContactCount() const override { return m_WarmStartContactCount; }

		virtual void	PreResolve(const std::vector<Geometry*>& geoms, const std::vector<PositionConstraint*>& Joints) override final
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

			for (auto* Joint : Joints) if (Joint && Joint->IsEnabled())
				for (auto* Body : { Joint->GetBodyA(), Joint->GetBodyB() }) if (Body && BodySlots.emplace(Body, (int)m_Bodies.size()).second) m_Bodies.push_back(Body);

			std::unordered_set<Ragdoll*> Owners;
			for (size_t I = 0; I < m_Bodies.size(); ++I) if (m_Bodies[I]->ReducedRagdoll && Owners.insert(m_Bodies[I]->ReducedRagdoll).second)
				for (auto* Link : m_Bodies[I]->ReducedRagdoll->GetBodies())
					if (BodySlots.emplace(Link, (int)m_Bodies.size()).second) m_Bodies.push_back(Link);

			const int BodyCount = (int)m_Bodies.size();
			m_Buffer.resize(2 * BodyCount);
			for (int BodySlot = 0; BodySlot < BodyCount; ++BodySlot)
			{
				RigidBody* Body = m_Bodies[(size_t)BodySlot];
				m_Buffer[(size_t)BodySlot].v = Body->GetLinearVelocity();
				m_Buffer[(size_t)BodySlot].w = Body->GetAngularVelocity();
				m_Buffer[(size_t)BodyCount + BodySlot] = m_Buffer[(size_t)BodySlot];
			}
			if (Owners.empty()) m_ReducedRagdolls.Begin({}, nullptr);
			else m_ReducedRagdolls.Begin(m_Bodies, m_Buffer.data());
			m_Friction.BeginSolve(m_Bodies, m_Buffer.data(), true, &m_ReducedRagdolls);
		}

		void ResolveContact(const std::vector<Geometry*>& Geometries, std::vector<ContactManifold*>& Manifolds, float Dt) override
		{
			ResolveIsland(Geometries, Manifolds, {}, Dt, 0);
		}
		void ResolveConstraints(const std::vector<Geometry*>& Geometries, std::vector<ContactManifold*>& Manifolds,
			const std::vector<PositionConstraint*>& Joints, float Dt, int Iterations) override
		{
			ResolveIsland(Geometries, Manifolds, Joints, Dt, Iterations);
		}
		void ResolveIsland(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds,
			const std::vector<PositionConstraint*>& Joints, float dt, int JointIterations)
		{
			if ((manifolds.empty() && Joints.empty()) || m_Bodies.empty() || dt <= 0.0f)
				return;

			JointVelocityPass JointPass(m_Bodies, m_Buffer.data(), Joints);
			const bool PatchFriction = m_Friction.GetSettings().Enabled;
			if (PatchFriction) m_Friction.Prepare(geoms, manifolds, dt);
			const int CoupledIterations = Joints.empty() ? 0 : std::max(1, JointIterations);
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
				// A second solve in the same frame must not replay a historical load.
				for (int Point = 0; Point < manifold->NumContactPointCount; ++Point)
				{
					Contact& C = manifold->ContactPoints[Point];
					C.totalImpulseNormal = C.totalImpulseTangent = C.totalImpulseBinormal = C.CachedDt = 0.0f;
				}
				if (m_WarmStartingEnabled)
					m_WarmStartContactCount += m_ImpulseCache.Match(MakeState(geoms[(size_t)manifold->indexA]),
						MakeState(geoms[(size_t)manifold->indexB]), manifold, true);
			}

			std::vector<RigidBody*> IslandBodies;
			for (const auto& Context : ValidManifolds) { IslandBodies.push_back(Context.BodyA); IslandBodies.push_back(Context.BodyB); }
			std::vector<ContactJacobianSolver> velocityConstraints;
			for (const ManifoldContext& Context : ValidManifolds)
			{
				const int ContactCount = std::min(Context.Manifold->NumContactPointCount, MAX_CONTACT_POINTS);
				for (int j = 0; j < ContactCount; ++j)
				{
					velocityConstraints.push_back(ContactJacobianSolver(
						m_Buffer.data(), Context.BodySlotA, Context.BodySlotB, Context.BodyA, Context.BodyB, &m_ReducedRagdolls));
					velocityConstraints.back().SetupPositionPass(&Context.Manifold->ContactPoints[j], dt);
					velocityConstraints.back().UsePatchFriction = PatchFriction;
				}
			}

			if (!velocityConstraints.empty())
			{
				if (m_WarmStartingEnabled)
					for (ContactJacobianSolver& Constraint : velocityConstraints) WarmStart::Apply(Constraint, dt, m_PersistenceSettings);
				int it = 0;
				const float kStopPositionIterationThreshold = 1e-6f;
				while (it++ < std::max(m_nMaxPositionIterations, CoupledIterations))
				{
					m_ReducedRagdolls.SolveControls(dt, IslandBodies);
					float sumSqrError = JointPass.Solve(dt);
					for (size_t k = 0; k < velocityConstraints.size(); ++k)
					{
						velocityConstraints[k].Solve();
						if (PatchFriction) velocityConstraints[k].m_contact->totalImpulseNormal = velocityConstraints[k].m_jN.m_totalLambda;

						sumSqrError += velocityConstraints[k].GetSquaredError();
					}

					if (PatchFriction) sumSqrError += m_Friction.Solve(dt);
					sumSqrError /= velocityConstraints.size();
					if (sumSqrError < kStopPositionIterationThreshold)
					{
						break;
					}
				}

				std::vector<float> PositionImpulses;
				if (PatchFriction) PositionImpulses.resize(velocityConstraints.size());
				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					if (PatchFriction) PositionImpulses[k] = velocityConstraints[k].m_jN.m_totalLambda;
					if (!PatchFriction) velocityConstraints[k].Finalize();
				}

				for (ContactJacobianSolver& Constraint : velocityConstraints)
				{
					Constraint.SetupVelocityPass(Constraint.m_contact, dt, 0);
				}

				it = 0;
				while (it++ < std::max(m_nMaxVelocityIterations, CoupledIterations))
				{
					m_ReducedRagdolls.SolveControls(dt, IslandBodies);
					JointPass.Solve(dt);
					for (size_t i = 0; i < velocityConstraints.size(); ++i)
					{
						velocityConstraints[i].Solve();
						if (PatchFriction) velocityConstraints[i].m_contact->totalImpulseNormal = PositionImpulses[i] + velocityConstraints[i].m_jN.m_totalLambda;
					}
					if (PatchFriction) m_Friction.Solve(dt);
				}

				for (size_t k = 0; k < velocityConstraints.size(); ++k)
				{
					if (!PatchFriction) velocityConstraints[k].Finalize(true);
				}
				if (m_WarmStartingEnabled)
				{
					for (const ManifoldContext& Context : ValidManifolds)
						m_ImpulseCache.Store(MakeState(geoms[(size_t)Context.Manifold->indexA]),
							MakeState(geoms[(size_t)Context.Manifold->indexB]), *Context.Manifold, dt, true, true);
				}

				return;
			}
			for (int Iteration = 0; Iteration < CoupledIterations; ++Iteration) JointPass.Solve(dt);
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
			m_ReducedRagdolls.Commit();
		}

	private:
		RagdollVelocityContext m_ReducedRagdolls;
		static GeometryWorldState MakeState(Geometry* Shape)
		{
			GeometryWorldState State;
			State.Geom = Shape;
			State.Body = Shape->GetParent<RigidBody>();
			State.ShapeToWorld = State.Body ? State.Body->GetGeometryTransform(Shape) : *Shape->GetTransform();
			if (State.Body) State.Handle = State.Body->FindGeometryHandle(Shape);
			return State;
		}
		ContactPersistenceSettings m_PersistenceSettings;
		ContactManifoldManager m_ImpulseCache;
		FrictionPatchSolver m_Friction;
		bool m_WarmStartingEnabled = true;
		int m_WarmStartContactCount = 0;
		int 	m_nMaxPositionIterations;
		int 	m_nMaxVelocityIterations;

		std::vector<PositionConstraint*>	m_PositionConstraints;
		std::vector<int>					m_GeometryToBodySlot;
		std::vector<RigidBody*>			m_Bodies;
		std::vector<GeneralizedVelocity>	m_Buffer;
	};

	ConstraintSolver* ConstraintSolver::CreateSequentialImpulseSolver(const ContactPersistenceSettings* Settings, const SequentialImpulseSettings* Iterations)
	{
		return new SequentialImpulseSolver(Settings ? *Settings : ContactPersistenceSettings(),
			Iterations ? *Iterations : SequentialImpulseSettings());
	}
}

#include "PhysicsWorld.h"
#include "../Fluid/AerodynamicsSimulation.h"
#include "../Fluid/AerodynamicsRigidCoupling.h"
#include "../Deformation/Cloth.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	AerodynamicsSimulation* PhysicsWorld::CreateAerodynamics(const AerodynamicsSimulationSettings& Settings)
	{
		return CreateAerodynamics(Settings, AerodynamicsRigidCouplingSettings());
	}
	AerodynamicsSimulation* PhysicsWorld::CreateAerodynamics(const AerodynamicsSimulationSettings& Settings,
		const AerodynamicsRigidCouplingSettings& Coupling, bool CoupleRigids, bool CoupleCloths)
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit aerodynamics between world steps");
		std::unique_ptr<AerodynamicsSimulation> Next(new AerodynamicsSimulation(Settings));
		if (!Next->IsInitialized()) return nullptr;
		std::unique_ptr<AerodynamicsRigidCoupling> Field;
		if (CoupleRigids)
		{
			Field.reset(new AerodynamicsRigidCoupling(Next.get(), Coupling));
			if (!Field->IsValid()) return nullptr;
		}
		RemoveAerodynamics();
		m_Aerodynamics = std::move(Next); m_AerodynamicsClothCoupling = CoupleCloths;
		m_AerodynamicsDomain.reset(GeometryFactory::CreateOBB((Settings.BoundsMin + Settings.BoundsMax) * 0.5f, (Settings.BoundsMax - Settings.BoundsMin) * 0.5f));
		m_AerodynamicsDomain->SetSimulationEnabled(false); m_AerodynamicsDomain->SetQueryEnabled(false);
		if (Field) { AddForceField(Field.get()); m_AerodynamicsCoupling = Field.release(); }
		return m_Aerodynamics.get();
	}
	bool PhysicsWorld::RemoveAerodynamics()
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit aerodynamics between world steps");
		if (!m_Aerodynamics) return false;
		if (m_AerodynamicsCoupling) RemoveForceField(m_AerodynamicsCoupling);
		m_AerodynamicsCoupling = nullptr;
		RemoveGeometryWorldState(m_AerodynamicsDomain.get()); m_AerodynamicsDomain.reset();
		m_Aerodynamics.reset(); return true;
	}
	int PhysicsWorld::AerodynamicsSubsteps(float Dt) const
	{
		if (!m_Aerodynamics || !m_Aerodynamics->SupportsMovingObstacles()) return 1;
		if (!m_Aerodynamics->IsInitialized()) throw std::runtime_error("Aerodynamics backend is unavailable");
		float Step = m_Aerodynamics->GetStableTimeStep();
		const auto H = m_Aerodynamics->GetCellSize(); float Rate = 0;
		for (auto* B : m_DynamicBodies) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, std::fabs(B->V[A]) / H[A]);
		for (auto* C : m_Cloths) if (m_AerodynamicsClothCoupling && C->GetSettings().AerodynamicsEnabled)
			for (int I = 0; I < C->GetParticleCount(); ++I) for (int A = 0; A < 3; ++A) Rate = std::max(Rate, std::fabs(C->GetParticleVelocity(I)[A]) / H[A]);
		Step = std::min(Step, 0.9f / std::max(Rate, 1e-5f));
		const double Count = std::ceil((double)Dt / Step);
		if (!std::isfinite(Count) || Count > 4096) throw std::runtime_error("Aerodynamics world substep budget exceeded");
		return std::max(1, (int)Count);
	}
	void PhysicsWorld::SimulateAerodynamics(float Dt)
	{
		if (!m_Aerodynamics) return;
		const bool MovingBoundaries = m_Aerodynamics->SupportsMovingObstacles();
		const BroadPhaseResult& Pairs = GetCollisionCandidates();
		std::vector<Geometry*> Shapes;
		std::vector<ClothSystem*> Cloths;
		for (const auto& P : Pairs.Pairs)
		{
			auto* A = Pairs.GetGeometry(P.index1); auto* B = Pairs.GetGeometry(P.index2);
			if (!A || !B) continue;
			auto* G = A == m_AerodynamicsDomain.get() ? B : B == m_AerodynamicsDomain.get() ? A : nullptr;
			if (!G) continue;
			if (m_AerodynamicsCoupling && G->GetParent<RigidBody>() && G->IsSimulationEnabled() && !G->IsTriggerEnabled()) Shapes.push_back(G);
			if (MovingBoundaries && m_AerodynamicsClothCoupling)
				for (auto* C : m_Cloths) if (C->GetRenderGeometry() == G && C->GetSettings().AerodynamicsEnabled) { Cloths.push_back(C); break; }
		}
		if (!MovingBoundaries)
		{
			if (m_AerodynamicsCoupling && !m_AerodynamicsCoupling->Synchronize(Shapes))
				throw std::runtime_error("Aerodynamics boundary synchronization failed");
			m_Aerodynamics->Step(Dt); return;
		}
		std::vector<AerodynamicsSurfaceMesh> Surfaces;
		for (auto* C : Cloths)
		{
			AerodynamicsSurfaceMesh S; S.Thickness = C->GetSettings().Thickness;
			for (int I = 0; I < C->GetParticleCount(); ++I) { S.Positions.push_back(C->GetParticle(I).Position); S.Velocities.push_back(C->GetParticleVelocity(I)); S.InverseMasses.push_back(C->GetParticle(I).Pinned ? 0 : C->GetParticle(I).InverseMass); }
			for (const auto& T : C->GetTopology().GetTriangles()) S.Triangles.push_back(T.Particles);
			Surfaces.push_back(std::move(S));
		}
		const bool Ready = m_AerodynamicsCoupling ? m_AerodynamicsCoupling->Synchronize(Shapes, Surfaces) : m_Aerodynamics->SetMovingBoundaries({}, Surfaces);
		if (!Ready || !m_Aerodynamics->StepSynchronized(Dt)) throw std::runtime_error("Aerodynamics failed to consume the world substep");
		std::vector<std::vector<Vector3>> Forces;
		if (!m_Aerodynamics->ComputeSurfaceLoads(Forces) || Forces.size() != Cloths.size()) throw std::runtime_error("Aerodynamics surface loads are unavailable");
		for (size_t I = 0; I < Cloths.size(); ++I) for (size_t J = 0; J < Forces[I].size(); ++J)
			Cloths[I]->ApplyParticleImpulse((int)J, Forces[I][J] * Dt);
	}
}

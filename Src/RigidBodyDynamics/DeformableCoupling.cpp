#include "PhysicsWorld.h"
#include "../Deformation/DeformableStep.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* Cross-domain attachments are world-owned. Endpoint removal erases their
	   records before either object can be deleted, and identities survive Reset.
	   This first adapter joins the existing PBD/XPBD position iterations. SI/LCP
	   need an impulse-stage adapter; continuous attachment trajectories need a
	   sweep of constraint-generated motion. Reject these unsupported combinations
	   explicitly instead of silently adding a second pose integration. */
	int PhysicsWorld::DeformableCouplingSubsteps() const
	{
		if (m_DeformableRigidAttachments.empty() && !HasDeformableRigidContacts()) return 1;
		if (!m_PositionSolver || m_SupportStabilization.Enabled) throw std::invalid_argument("Rigid deformable coupling requires PBD/XPBD without support projection");
		DeformableStep::Validate(m_DeformableBodies);
		for (const auto& Link : m_DeformableRigidAttachments)
			if (Link.Rigid->ReducedRagdoll || (Link.Rigid->CastDynamic() && Link.Rigid->CastDynamic()->mMotionType == MorionType::Continuous))
				throw std::invalid_argument("Rigid deformable attachments currently require independent discrete bodies");
		int Count = 1;
		for (auto* Body : m_DeformableBodies) Count = std::max(Count, Body->GetSettings().Substeps);
		return Count;
	}
	bool PhysicsWorld::HasDeformableRigidContacts() const
	{
		for (auto* Body : m_DeformableBodies) if (Body->GetSettings().Collision.RigidCollision) return true;
		return false;
	}
	uint64_t PhysicsWorld::AttachDeformableToRigid(DeformableBody* Soft, const BarycentricPoint& Point, RigidBody* Rigid,
		const Vector3& WorldAnchor, double Compliance)
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit deformable attachments between world steps");
		if (!Soft || !Rigid || std::find(m_DeformableBodies.begin(), m_DeformableBodies.end(), Soft) == m_DeformableBodies.end()
			|| (std::find(m_DynamicBodies.begin(), m_DynamicBodies.end(), Rigid) == m_DynamicBodies.end()
				&& std::find(m_StaticBodies.begin(), m_StaticBodies.end(), Rigid) == m_StaticBodies.end())
			|| !Point.IsValid(Soft->GetParticles().size()) || !std::isfinite(Compliance) || Compliance < 0)
			throw std::invalid_argument("Invalid rigid deformable attachment endpoints");
		if (!m_PositionSolver || m_SupportStabilization.Enabled || Rigid->ReducedRagdoll
			|| (Rigid->CastDynamic() && Rigid->CastDynamic()->mMotionType == MorionType::Continuous))
			throw std::invalid_argument("Rigid deformable attachments require PBD/XPBD, discrete bodies and no support projection");
		if (!m_NextDeformableRigidAttachmentId) throw std::overflow_error("Rigid deformable attachment identity exhausted");
		const Vector3 LocalAnchor = Rigid->WorldToBodyLocal(WorldAnchor);
		auto* Dynamic = Rigid->CastDynamic(); const bool Movable = Dynamic && !Dynamic->Freezing;
		RigidPointResponse Validate({ &Rigid->X, &Rigid->Q, Movable ? Rigid->InvMass : 0,
			Movable ? Rigid->InvInertia : Matrix3::Zero() }, LocalAnchor - Rigid->CM);
		RigidDeformableAttachment Link; Link.Id = m_NextDeformableRigidAttachmentId; Link.Soft = Soft; Link.Rigid = Rigid;
		Link.Point = Point; Link.LocalAnchor = LocalAnchor; Link.Compliance = Compliance;
		m_DeformableRigidAttachments.push_back(Link);
		++Soft->m_RigidAttachmentCount;
		if (Dynamic) Dynamic->Wakeup();
		return m_NextDeformableRigidAttachmentId++;
	}
	bool PhysicsWorld::RemoveDeformableRigidAttachment(uint64_t Id)
	{
		if (m_SimulationInProgress) throw std::logic_error("Edit deformable attachments between world steps");
		const auto Found = std::find_if(m_DeformableRigidAttachments.begin(), m_DeformableRigidAttachments.end(),
			[Id](const RigidDeformableAttachment& Link) { return Link.Id == Id; });
		if (Found == m_DeformableRigidAttachments.end()) return false;
		if (auto* Dynamic = Found->Rigid->CastDynamic()) Dynamic->Wakeup();
		--Found->Soft->m_RigidAttachmentCount;
		m_DeformableRigidAttachments.erase(Found); return true;
	}
	size_t PhysicsWorld::GetDeformableRigidAttachmentCount() const { return m_DeformableRigidAttachments.size(); }
	Maths::TVector3<double> PhysicsWorld::GetDeformableRigidAttachmentForce(uint64_t Id) const
	{
		for (const auto& Link : m_DeformableRigidAttachments) if (Link.Id == Id) return Link.Force;
		throw std::out_of_range("Unknown rigid deformable attachment");
	}
	Maths::TVector3<double> PhysicsWorld::GetDeformableRigidAttachmentTorque(uint64_t Id) const
	{
		for (const auto& Link : m_DeformableRigidAttachments) if (Link.Id == Id) return Link.Torque;
		throw std::out_of_range("Unknown rigid deformable attachment");
	}
}

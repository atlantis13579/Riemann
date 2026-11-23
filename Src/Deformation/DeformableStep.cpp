#include "DeformableStep.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace Riemann
{
	/* A coupled position step has one clock: rigid and soft endpoints predict
	   the same interval, alternate material/attachment/contact projection, and
	   reconstruct velocity only after the shared solve. The world promotes soft
	   substeps to physical world substeps; it must not integrate a rigid endpoint
	   again for each soft body's private substep budget.

	   This adapter uses the world's PBD/XPBD pose reconstruction, including the
	   angular response at an off-center anchor. Attachment multipliers describe
	   the force on the soft body; torque is the opposite endpoint's moment about
	   the rigid center of mass. Their frame reports average physical impulses.
	   It does not apply the reported force again as an external load.

	   All soft bodies share this stage while rigid attachments are present. This
	   also joins chains through soft-soft links or a common rigid body. Forces
	   persist across physical substeps and are consumed at the outer boundary.
	   Unattached groups use the same frame boundaries: clearing forces or resetting
	   contact statistics at an inner world substep would lose part of the frame.
	   Topology, pins and attachment edits belong between steps because rows borrow
	   particle storage. A failed step releases that borrowing guard; it does not
	   claim to roll the entire physics world back. */
	void DeformableStep::Validate(const std::vector<DeformableBody*>& Bodies)
	{
		std::unordered_set<DeformableBody*> Members;
		for (auto* Body : Bodies)
			if (!Body || !Members.insert(Body).second || Body->m_CoupledStepActive) throw std::invalid_argument("Invalid coupled deformable group");
		for (auto* Body : Bodies)
		{
			for (auto* User : Body->m_AttachmentUsers) if (!Members.count(User)) throw std::invalid_argument("Coupled step requires every attached deformable");
			for (const auto& Link : Body->m_Attachments) if (Link.Other && !Members.count(Link.Other)) throw std::invalid_argument("Coupled step requires every attached deformable");
		}
	}
	DeformableStep::DeformableStep(const std::vector<DeformableBody*>& Bodies, std::vector<RigidDeformableAttachment>& Attachments,
		float Dt, float FrameDt, bool First, bool Last, const std::vector<DeformableRigidCollider>& Colliders, CollisionFilter* Filter)
		: m_Bodies(Bodies), m_Dt(Dt), m_FrameDt(FrameDt), m_Last(Last)
	{
		if (!std::isfinite(Dt) || !std::isfinite(FrameDt) || !(Dt > 0) || FrameDt < Dt)
			throw std::invalid_argument("Invalid coupled deformable timestep");
		Validate(Bodies);
		m_Collision.reset(new DeformableCollision(Bodies));
		m_RigidCollision.reset(new DeformableRigidCollision(Bodies, Colliders, Filter));
		auto Bind = [](DeformableBody* Body, const BarycentricPoint& Point, SurfaceContactParticle* Out)
		{
			for (int I = 0; I < Point.Count; ++I) Out[I] = { &Body->m_Particles[Point.Indices[I]].Position, Body->GetInverseMass(Point.Indices[I]) };
		};
		size_t SoftCount = 0; for (auto* Body : Bodies) SoftCount += Body->m_Attachments.size();
		m_SoftRows.reserve(SoftCount); m_RigidRows.reserve(Attachments.size());
		for (auto* Body : Bodies) for (auto& Link : Body->m_Attachments)
		{
			SurfaceContactParticle A[BarycentricPoint::MaximumVertices], B[BarycentricPoint::MaximumVertices]; Bind(Body, Link.Point, A);
			const double WorldWeight = 1; const double* Weights = &WorldWeight; int Count = 1;
			if (Link.Other) { Bind(Link.Other, Link.OtherPoint, B); Weights = Link.OtherPoint.Weights.data(); Count = Link.OtherPoint.Count; }
			else B[0] = { &Link.Target, 0 };
			m_SoftRows.push_back({ &Link, BarycentricAttachmentConstraint(A, Link.Point.Weights.data(), Link.Point.Count, B, Weights, Count, Link.Compliance) });
		}
		for (auto& Link : Attachments)
		{
			SurfaceContactParticle A[BarycentricPoint::MaximumVertices]; Bind(Link.Soft, Link.Point, A);
			auto* Dynamic = Link.Rigid->CastDynamic(); const bool Movable = Dynamic && !Dynamic->Freezing;
			RigidConstraintBody Rigid{ &Link.Rigid->X, &Link.Rigid->Q, Movable ? Link.Rigid->InvMass : 0,
				Movable ? Link.Rigid->InvInertia : Matrix3::Zero() };
			m_RigidRows.push_back({ &Link, BarycentricAttachmentConstraint(A, Link.Point.Weights.data(), Link.Point.Count,
				Rigid, Link.LocalAnchor - Link.Rigid->CM, Link.Compliance) });
		}
		if (First)
		{
			for (auto* Body : Bodies) Body->m_CollisionStats = DeformableCollisionStats();
			for (auto& Row : m_SoftRows) Row.Link->Force = Maths::TVector3<double>(0);
			for (auto& Row : m_RigidRows) Row.Link->Force = Row.Link->Torque = Maths::TVector3<double>(0);
		}
		try
		{
			for (auto* Body : Bodies) { Body->PrepareStep(Dt); Body->m_CoupledStepActive = true; }
		}
		catch (...)
		{
			// Construction failure does not invoke this stage's destructor. Release
			// every earlier participant if a later material preparation fails.
			for (auto* Body : Bodies)
			{
				Body->m_CoupledStepActive = false;
				if (Body->m_ProjectiveMaterials) Body->m_ProjectiveMaterials->Solver.End();
			}
			throw;
		}
	}
	DeformableStep::~DeformableStep()
	{
		for (auto* Body : m_Bodies)
		{
			Body->m_CoupledStepActive = false;
			if (Body->m_ProjectiveMaterials) Body->m_ProjectiveMaterials->Solver.End();
		}
	}
	void DeformableStep::Solve()
	{
		SolveConstraints();
		if (!m_Bodies.empty() && m_Bodies[0]->m_CollisionQuery)
		{
			SolveContacts(m_Bodies[0]->m_CollisionQuery());
			return;
		}
		std::vector<Geometry*> Geometries; CollectCollisionGeometries(Geometries);
		BroadPhase::BuildStandalone(Geometries, m_LocalPairs);
		SolveContacts(m_LocalPairs);
	}
	void DeformableStep::SolveConstraints()
	{
		for (auto* Body : m_Bodies) { Body->SolveMaterials(m_Dt); Body->SolveEdges(m_Dt); Body->SolveVolumes(m_Dt); Body->SolveGround(); }
		for (auto& Row : m_SoftRows) Row.Constraint.Solve(m_Dt);
		for (auto& Row : m_RigidRows) { Row.Constraint.Solve(m_Dt); Row.Link->Rigid->UpdateGeometries(); }
	}
	void DeformableStep::CollectCollisionGeometries(std::vector<Geometry*>& Geometries)
	{
		for (auto* B : m_Bodies)
		{
			const auto& C = B->GetSettings().Collision;
			if (!C.RigidCollision && !C.InterCollision) continue;
			B->UpdateRenderGeometry(); auto* G = B->GetRenderGeometry(); if (!G) continue;
			G->SetFilterData(C.FilterData); G->SetContactOffsets(B->GetSettings().ParticleRadius + C.ContactTolerance, 0);
			Geometries.push_back(G);
		}
	}
	void DeformableStep::SolveContacts(const BroadPhaseResult& Pairs)
	{
		m_Collision->Solve(m_Dt, Pairs); m_RigidCollision->Solve(m_Dt, Pairs);
	}
	void DeformableStep::Finish()
	{
		const double Weight = (double)m_Dt / m_FrameDt;
		for (auto& Row : m_SoftRows) Row.Link->Force += Row.Constraint.GetForce(m_Dt) * Weight;
		for (auto& Row : m_RigidRows)
		{
			Row.Link->Force += Row.Constraint.GetForce(m_Dt) * Weight;
			Row.Link->Torque += Row.Constraint.GetRigidTorque(m_Dt) * Weight;
		}
		for (auto* Body : m_Bodies)
		{
			Body->FinishStep(m_Dt);
			if (m_Last) for (auto& P : Body->m_Particles) P.Force = Vector3::Zero();
			Body->UpdateRenderGeometry();
		}
	}
}

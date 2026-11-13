#include "DeformableBody.h"
#include "DeformableStep.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace Riemann
{
	/* Attachment ownership is directional, but simulation connectivity is not.
	   The owner stores material-point indices; the target records its users so
	   either endpoint can be destroyed without leaving a dangling attachment.
	   Particle pointers are rebound each substep, after topology storage settles.

	   All bodies in one attachment component predict positions first, alternate
	   material and attachment projection, then reconstruct velocities together.
	   Advancing the endpoints independently would give the second body an extra
	   integration of the first body's correction and destroy momentum symmetry.
	   A component uses the largest requested substep/iteration count. Unconnected
	   components retain their own budgets and remain independently schedulable.
	   Reaction reporting averages substep impulses over the caller's frame time.
	   Rigid/contact adapters must join these phase boundaries when introduced. */
	namespace
	{
		using DVector = Maths::TVector3<double>;
		constexpr float MinimumDeformableStep = 1e-8f;
		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool ValidCompliance(double Value) { return std::isfinite(Value) && Value >= 0; }
	}
	void DeformableBody::RequireIdle() const
	{
		if (m_CoupledStepActive) throw std::logic_error("Edit deformable bodies between coupled world steps");
	}
	Vector3 DeformableBody::GetPointPosition(const BarycentricPoint& Point) const
	{
		if (!Point.IsValid(m_Particles.size())) throw std::invalid_argument("Invalid deformable attachment point");
		DVector Position(0); double Sum = 0;
		for (int I = 0; I < Point.Count; ++I)
		{
			const auto& P = m_Particles[Point.Indices[I]].Position;
			Position += DVector(P.x, P.y, P.z) * Point.Weights[I]; Sum += Point.Weights[I];
		}
		Position = Position / Sum;
		const Vector3 Result((float)Position.x, (float)Position.y, (float)Position.z);
		if (!Finite(Result)) throw std::overflow_error("Non-finite deformable attachment position");
		return Result;
	}
	uint64_t DeformableBody::AttachToWorld(const BarycentricPoint& Point, const Vector3& Target, double Compliance)
	{
		RequireIdle();
		if (!Point.IsValid(m_Particles.size()) || !Finite(Target) || !ValidCompliance(Compliance))
			throw std::invalid_argument("Invalid deformable world attachment");
		if (!m_NextAttachmentId) throw std::overflow_error("Deformable attachment identity exhausted");
		Attachment Link; Link.Id = m_NextAttachmentId; Link.Point = Point; Link.Target = Target; Link.Compliance = Compliance;
		m_Attachments.push_back(Link); return m_NextAttachmentId++;
	}
	uint64_t DeformableBody::AttachToBody(const BarycentricPoint& Point, DeformableBody* Other, const BarycentricPoint& OtherPoint, double Compliance)
	{
		RequireIdle();
		if (Other) Other->RequireIdle();
		if (!Other || !Point.IsValid(m_Particles.size()) || !OtherPoint.IsValid(Other->m_Particles.size()) || !ValidCompliance(Compliance))
			throw std::invalid_argument("Invalid deformable body attachment");
		if (!m_NextAttachmentId) throw std::overflow_error("Deformable attachment identity exhausted");
		Attachment Link; Link.Id = m_NextAttachmentId; Link.Point = Point; Link.Other = Other; Link.OtherPoint = OtherPoint; Link.Compliance = Compliance;
		m_Attachments.push_back(Link);
		try
		{
			if (Other != this && std::find(Other->m_AttachmentUsers.begin(), Other->m_AttachmentUsers.end(), this) == Other->m_AttachmentUsers.end())
				Other->m_AttachmentUsers.push_back(this);
		}
		catch (...) { m_Attachments.pop_back(); throw; }
		return m_NextAttachmentId++;
	}
	bool DeformableBody::SetAttachmentTarget(uint64_t Id, const Vector3& Target)
	{
		RequireIdle();
		if (!Finite(Target)) return false;
		for (auto& Link : m_Attachments) if (Link.Id == Id && !Link.Other) { Link.Target = Target; Link.Force = DVector(0); return true; }
		return false;
	}
	bool DeformableBody::RemoveAttachment(uint64_t Id)
	{
		RequireIdle();
		const auto Found = std::find_if(m_Attachments.begin(), m_Attachments.end(), [Id](const Attachment& Link) { return Link.Id == Id; });
		if (Found == m_Attachments.end()) return false;
		auto* Other = Found->Other; m_Attachments.erase(Found);
		if (Other && Other != this && std::none_of(m_Attachments.begin(), m_Attachments.end(), [Other](const Attachment& Link) { return Link.Other == Other; }))
		{
			auto& Users = Other->m_AttachmentUsers; Users.erase(std::remove(Users.begin(), Users.end(), this), Users.end());
		}
		return true;
	}
	size_t DeformableBody::RemoveAttachments(const DeformableBody* Other)
	{
		RequireIdle();
		size_t Count = 0;
		for (size_t I = 0; I < m_Attachments.size();)
			if (m_Attachments[I].Other == Other) { RemoveAttachment(m_Attachments[I].Id); ++Count; } else ++I;
		return Count;
	}
	void DeformableBody::ClearAttachments()
	{
		RequireIdle();
		while (!m_Attachments.empty()) RemoveAttachment(m_Attachments.back().Id);
	}
	void DeformableBody::DisconnectAttachments()
	{
		ClearAttachments();
		while (!m_AttachmentUsers.empty()) m_AttachmentUsers.back()->RemoveAttachments(this);
	}
	DVector DeformableBody::GetAttachmentForce(uint64_t Id) const
	{
		for (const auto& Link : m_Attachments) if (Link.Id == Id) return Link.Force;
		throw std::out_of_range("Unknown deformable attachment");
	}
	void DeformableBody::Simulate(float Dt) { SimulateGroup({ this }, Dt); }
	void DeformableBody::SimulateGroup(const std::vector<DeformableBody*>& Bodies, float Dt)
	{
		SimulateGroup(Bodies, Dt, Dt, true, true);
	}
	void DeformableBody::SimulateGroup(const std::vector<DeformableBody*>& Bodies, float Dt, float FrameDt, bool First, bool Last)
	{
		if (!std::isfinite(Dt) || Dt <= 0 || Bodies.empty()) return;
		std::unordered_map<DeformableBody*, size_t> Indices;
		std::vector<size_t> Roots(Bodies.size());
		for (size_t I = 0; I < Bodies.size(); ++I)
		{
			if (!Bodies[I] || !Indices.emplace(Bodies[I], I).second) throw std::invalid_argument("Invalid deformable simulation group");
			Bodies[I]->RequireIdle();
			if (Bodies[I]->m_RigidAttachmentCount) throw std::invalid_argument("Step rigid-attached deformables through their physics world");
			if (Bodies[I]->m_Settings.Collision.RigidCollision) throw std::invalid_argument("Step rigid-contact deformables through their physics world");
			Roots[I] = I;
		}
		auto Root = [&Roots](size_t I)
		{
			while (Roots[I] != I) { Roots[I] = Roots[Roots[I]]; I = Roots[I]; } return I;
		};
		// Check the complete membership before predicting any body, including
		// incoming links whose owner is not the body being stepped directly.
		for (size_t I = 0; I < Bodies.size(); ++I)
		{
			for (auto* User : Bodies[I]->m_AttachmentUsers) if (Indices.find(User) == Indices.end())
				throw std::invalid_argument("Step attached deformables together with SimulateGroup");
			for (const auto& Link : Bodies[I]->m_Attachments) if (Link.Other)
			{
				const auto Found = Indices.find(Link.Other);
				if (Found == Indices.end()) throw std::invalid_argument("Step attached deformables together with SimulateGroup");
				Roots[Root(Found->second)] = Root(I);
			}
		}
		std::vector<std::vector<DeformableBody*>> Groups(Bodies.size());
		// Potential inter-body contacts share one clock even before their bounds
		// meet. Broad-phase rejection belongs inside that synchronized solve.
		size_t ContactRoot = Bodies.size();
		for (size_t I = 0; I < Bodies.size(); ++I) if (Bodies[I]->m_Settings.Collision.InterCollision)
		{
			if (ContactRoot == Bodies.size()) ContactRoot = I;
			else Roots[Root(I)] = Root(ContactRoot);
		}
		for (size_t I = 0; I < Bodies.size(); ++I) Groups[Root(I)].push_back(Bodies[I]);
		for (auto& Group : Groups)
		{
			if (Group.empty()) continue;
			int Substeps = 1, Iterations = 1;
			for (auto* Body : Group)
			{
				Substeps = std::max(Substeps, Body->m_Settings.Substeps); Iterations = std::max(Iterations, Body->m_Settings.SolverIterations);
			}
			const float Step = Dt / Substeps;
			if (Step < MinimumDeformableStep) continue;
			std::vector<RigidDeformableAttachment> RigidAttachments;
			for (int Substep = 0; Substep < Substeps; ++Substep)
			{
				DeformableStep Stage(Group, RigidAttachments, Step, FrameDt, First && Substep == 0, Last && Substep + 1 == Substeps);
				for (int Iteration = 0; Iteration < Iterations; ++Iteration) Stage.Solve();
				Stage.Finish();
			}
		}
	}
}

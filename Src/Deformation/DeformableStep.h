#pragma once

#include "DeformableBody.h"
#include "DeformableRigidCollision.h"

namespace Riemann
{
	class RigidBody;
	struct RigidDeformableAttachment
	{
		uint64_t Id = 0;
		DeformableBody* Soft = nullptr;
		RigidBody* Rigid = nullptr;
		BarycentricPoint Point;
		Vector3 LocalAnchor = Vector3::Zero();
		double Compliance = 0;
		Maths::TVector3<double> Force = Maths::TVector3<double>(0), Torque = Maths::TVector3<double>(0);
	};
	class DeformableStep
	{
	public:
		static void Validate(const std::vector<DeformableBody*>& Bodies);
		DeformableStep(const std::vector<DeformableBody*>& Bodies, std::vector<RigidDeformableAttachment>& Attachments,
			float Dt, float FrameDt, bool First, bool Last, const std::vector<DeformableRigidCollider>& Colliders = {}, CollisionFilter* Filter = nullptr);
		~DeformableStep();
		DeformableStep(const DeformableStep&) = delete;
		DeformableStep& operator=(const DeformableStep&) = delete;
		void Solve();
		void Finish();
		const std::vector<RigidBody*>& GetContactBodies() const { return m_RigidCollision->GetTouchedBodies(); }
	private:
		struct SoftRow { DeformableBody::Attachment* Link; BarycentricAttachmentConstraint Constraint; };
		struct RigidRow { RigidDeformableAttachment* Link; BarycentricAttachmentConstraint Constraint; };
		std::vector<DeformableBody*> m_Bodies;
		std::vector<SoftRow> m_SoftRows;
		std::vector<RigidRow> m_RigidRows;
		std::unique_ptr<DeformableCollision> m_Collision;
		std::unique_ptr<DeformableRigidCollision> m_RigidCollision;
		float m_Dt, m_FrameDt;
		bool m_Last;
	};
}

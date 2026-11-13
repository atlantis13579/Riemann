#pragma once

#include <memory>
#include <vector>
#include "Constraints.h"

namespace Riemann
{
	class PhysicsWorld;
	class RigidBodyDynamic;
	class RigidBodyStatic;
	struct PhysicalAnimationBone
	{
		RigidBodyDynamic* Body = nullptr;
		int Parent = -1;
	};
	enum class PhysicalAnimationSpace { Parent, World };
	enum class PhysicalAnimationState { Active, Passive, Recovering };
	struct PhysicalAnimationControl
	{
		PhysicalAnimationControl();
		PhysicalAnimationSpace Space = PhysicalAnimationSpace::Parent;
		bool Enabled = true;
		JointDriveSettings Linear, Angular;
		float Strength = 1, PhysicsBlend = 1;
		bool IsValid() const;
	};
	class PhysicalAnimation
	{
	public:
		~PhysicalAnimation();
		PhysicalAnimation(const PhysicalAnimation&) = delete;
		PhysicalAnimation& operator=(const PhysicalAnimation&) = delete;
		const std::vector<PhysicalAnimationBone>& GetBones() const { return m_Bones; }
		const PhysicalAnimationControl& GetControl(size_t Bone) const { return m_Controls.at(Bone); }
		void SetControl(size_t Bone, const PhysicalAnimationControl& Control);
		void SetControlBelow(size_t Bone, const PhysicalAnimationControl& Control, bool IncludeSelf = true);
		void SetAnimationPose(const std::vector<Transform>& WorldPose, float BlendTime = 0);
		const std::vector<Transform>& GetAnimationPose() const { return m_Animation; }
		const std::vector<Transform>& GetTargetPose() const { return m_Targets; }
		std::vector<Transform> GetPhysicsPose() const;
		std::vector<Transform> GetBlendedPose() const;
		void SetStrength(float Strength);
		float GetStrength() const { return m_Strength; }
		void SetActive(bool Active);
		void BeginRecovery(float Duration);
		PhysicalAnimationState GetState() const { return m_State; }
		float GetRecoveryProgress() const;
		const JointReaction& GetReaction(size_t Bone) const;
	private:
		friend class PhysicsWorld;
		PhysicalAnimation(PhysicsWorld& World, const std::vector<PhysicalAnimationBone>& Bones);
		void BeginStep(float Dt);
		void Wakeup();
		bool UsesBody(const RigidBody* Body) const;
		bool OwnsConstraint(const PositionConstraint* Constraint) const;
		std::vector<Transform> BlendPoses(const std::vector<Transform>& A, const std::vector<Transform>& B, const std::vector<float>& Weights) const;
		PhysicsWorld& m_World;
		std::vector<PhysicalAnimationBone> m_Bones;
		std::vector<PhysicalAnimationControl> m_Controls;
		std::unique_ptr<RigidBodyStatic> m_Reference;
		std::vector<PoseDriveConstraint*> m_Drives;
		std::vector<Transform> m_Animation, m_AnimationStart, m_AnimationEnd, m_Targets, m_RecoveryStart;
		float m_AnimationTime = 0, m_AnimationDuration = 0, m_RecoveryTime = 0, m_RecoveryDuration = 0, m_Strength = 1;
		bool m_ResetTargetVelocity = false;
		PhysicalAnimationState m_State = PhysicalAnimationState::Active;
	};
}

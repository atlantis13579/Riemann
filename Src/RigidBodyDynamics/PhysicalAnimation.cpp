#include "PhysicalAnimation.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* Physical animation sits between a sampled animation pose and the constraint
	   solver. Bone indices are parent-before-child; poses are WORLD COM transforms
	   in the bound body's axes. A skeleton adapter must apply its bone-to-body bind
	   offsets before submitting poses. This API consumes samples, not asset files.
	   The existing procedural ragdoll and caller-generated poses need no new asset.

	   Parent-space controls drive relative COM pose against the live physical
	   parent, with equal/opposite response. A root in Parent mode is undriven.
	   World mode uses a private, stationary reference and can control a pelvis or
	   a selected limb. Linear channels default off so passive joint anchors retain
	   their authored lengths. Angular channels use implicit SLERP muscle torques.
	   Acceleration gains normalize a row's body/pair response, not the mass of an
	   entire character. A root supporting a chain needs gains sized for that load.
	   These are force-limited servos, not balance, foot placement or motion planning;
	   applications still choose reachable poses and an appropriate recovery root.

	   BlendTime submits the next animation sample over a known interval. Sampling
	   occurs once per physics substep; finite differences of target poses supply
	   linear/angular feed-forward velocity. Child interpolation is parent-relative,
	   avoiding world-space chords that shrink rotating limbs. A zero-time update is
	   a target discontinuity with no inferred velocity spike, not a body teleport.

	   Passive disables muscle rows while retaining collisions and passive joints.
	   Recovery captures the current physical hierarchy, smoothly blends its target
	   toward animation, and ramps muscle strength from zero. Interruption captures
	   a fresh pose. Output physics weights ramp from one to each bone's configured
	   value. GetBlendedPose is a separate, hierarchy-aware presentation result; it
	   never overwrites solver poses, velocities, mass or collision transforms.

	   Each bone owns one drive-only row block, inserted into the ordinary joint
	   graph before contacts are solved. Mode changes switch its reference before
	   graph construction. The world removes controllers before their borrowed
	   bodies and forgets the private reference from cached solver/sleep state.
	   Per-body strength profiles and local/world targets use implicit spring drives
	   defined in Constraints.cpp. */
	namespace
	{
		bool UnitInterval(float X) { return std::isfinite(X) && X >= 0 && X <= 1; }
		float SmoothStep(float T) { return T * T * (3 - 2 * T); }
		Transform Interpolate(const Transform& A, const Transform& B, float T)
		{
			return Transform(A.pos + (B.pos - A.pos) * T, Quaternion::Slerp(A.quat, B.quat, T).Unit());
		}
		Vector3 AngularRate(const Quaternion& Previous, const Quaternion& Current, float Dt)
		{
			Quaternion Delta = (Current * Previous.Conjugate()).Unit(); if (Delta.w < 0) Delta = -Delta;
			const Vector3 Imaginary(Delta.x, Delta.y, Delta.z); const float Sine = Imaginary.Length();
			return Sine > 0 ? Imaginary * (2 * std::atan2(Sine, Delta.w) / (Sine * Dt)) : Vector3::Zero();
		}
		JointDriveSettings Scaled(const JointDriveSettings& Input, float Strength)
		{
			auto Output = Input; Output.Stiffness *= Strength; Output.Damping *= Strength; Output.MaxForce *= Strength; return Output;
		}
	}
	PhysicalAnimationControl::PhysicalAnimationControl()
	{
		Angular.PositionEnabled = Angular.VelocityEnabled = true; Angular.Acceleration = true;
		Angular.Stiffness = 40; Angular.Damping = 12; Angular.MaxForce = 100;
		Linear.Stiffness = 200; Linear.Damping = 40; Linear.MaxForce = 100;
	}
	bool PhysicalAnimationControl::IsValid() const
	{
		return (Space == PhysicalAnimationSpace::Parent || Space == PhysicalAnimationSpace::World)
			&& Linear.IsValid() && Angular.IsValid() && UnitInterval(Strength) && UnitInterval(PhysicsBlend);
	}
	PhysicalAnimation::PhysicalAnimation(PhysicsWorld& World, const std::vector<PhysicalAnimationBone>& Bones) : m_World(World), m_Bones(Bones)
	{
		m_Controls.resize(Bones.size()); m_Drives.reserve(Bones.size());
		m_Reference.reset(RigidBodyStatic::CreateRigidBody(RigidBodyParam(), Transform::Identity()));
		m_Reference->SetGuid(0);
		m_Animation = m_Targets = GetPhysicsPose(); m_AnimationStart = m_AnimationEnd = m_Animation;
		try
		{
			for (const auto& Bone : Bones)
			{
				auto* Reference = Bone.Parent < 0 ? (RigidBody*)m_Reference.get() : Bones[Bone.Parent].Body;
				auto* Drive = m_World.CreatePoseDriveConstraint(Reference, Bone.Body);
				if (!Drive) throw std::invalid_argument("Failed to bind physical animation drive");
				m_Drives.push_back(Drive); Drive->SetEnabled(false);
			}
		}
		catch (...) { for (auto* Drive : m_Drives) m_World.RemoveConstraint(Drive); throw; }
	}
	PhysicalAnimation::~PhysicalAnimation()
	{
		for (auto* Drive : m_Drives) m_World.RemoveConstraint(Drive);
	}
	bool PhysicalAnimation::UsesBody(const RigidBody* Body) const
	{
		return std::any_of(m_Bones.begin(), m_Bones.end(), [&](const PhysicalAnimationBone& Bone) { return Bone.Body == Body; });
	}
	bool PhysicalAnimation::OwnsConstraint(const PositionConstraint* Constraint) const
	{
		return std::find(m_Drives.begin(), m_Drives.end(), Constraint) != m_Drives.end();
	}
	void PhysicalAnimation::Wakeup() { for (const auto& Bone : m_Bones) Bone.Body->Wakeup(); }
	void PhysicalAnimation::SetControl(size_t Bone, const PhysicalAnimationControl& Control)
	{
		if (!Control.IsValid()) throw std::invalid_argument("Invalid physical animation control");
		m_Controls.at(Bone) = Control; Wakeup();
	}
	void PhysicalAnimation::SetControlBelow(size_t Bone, const PhysicalAnimationControl& Control, bool IncludeSelf)
	{
		if (Bone >= m_Bones.size() || !Control.IsValid()) throw std::invalid_argument("Invalid physical animation subtree control");
		for (size_t I = 0; I < m_Bones.size(); ++I)
			for (int Ancestor = IncludeSelf ? (int)I : m_Bones[I].Parent; Ancestor >= 0; Ancestor = m_Bones[Ancestor].Parent)
				if ((size_t)Ancestor == Bone) { m_Controls[I] = Control; break; }
		Wakeup();
	}
	std::vector<Transform> PhysicalAnimation::GetPhysicsPose() const
	{
		std::vector<Transform> Pose; Pose.reserve(m_Bones.size()); for (const auto& Bone : m_Bones) Pose.emplace_back(Bone.Body->X, Bone.Body->Q); return Pose;
	}
	std::vector<Transform> PhysicalAnimation::BlendPoses(const std::vector<Transform>& A, const std::vector<Transform>& B, const std::vector<float>& Weights) const
	{
		std::vector<Transform> Output(m_Bones.size());
		for (size_t I = 0; I < m_Bones.size(); ++I)
		{
			const int Parent = m_Bones[I].Parent;
			const auto Local = Interpolate(Parent < 0 ? A[I] : A[Parent].TransformInv(A[I]), Parent < 0 ? B[I] : B[Parent].TransformInv(B[I]), Weights[I]);
			Output[I] = Parent < 0 ? Local : Output[Parent] * Local;
		}
		return Output;
	}
	void PhysicalAnimation::SetAnimationPose(const std::vector<Transform>& WorldPose, float BlendTime)
	{
		if (WorldPose.size() != m_Bones.size() || !std::isfinite(BlendTime) || BlendTime < 0)
			throw std::invalid_argument("Invalid physical animation sample");
		auto Pose = WorldPose;
		for (auto& T : Pose)
		{
			PoseDriveSettings Validation; Validation.Target = T;
			if (!Validation.IsValid()) throw std::invalid_argument("Invalid physical animation transform");
			T.quat = T.quat.Unit();
		}
		m_AnimationStart = m_Animation; m_AnimationEnd = Pose; m_AnimationTime = 0; m_AnimationDuration = BlendTime;
		m_ResetTargetVelocity = BlendTime == 0;
		if (BlendTime == 0)
		{
			m_Animation = Pose;
			if (m_State != PhysicalAnimationState::Recovering) m_Targets = Pose;
		}
		if (m_State != PhysicalAnimationState::Passive && m_Strength > 0 && m_World.GetJointDrivesEnabled())
			for (size_t I = 0; I < m_Controls.size(); ++I)
			{
				const auto& C = m_Controls[I];
				if (C.Enabled && C.Strength > 0 && (C.Space == PhysicalAnimationSpace::World || m_Bones[I].Parent >= 0)
					&& (C.Linear.IsActive() || C.Angular.IsActive())) { Wakeup(); break; }
			}
	}
	float PhysicalAnimation::GetRecoveryProgress() const
	{
		return m_State == PhysicalAnimationState::Active ? 1 : m_State == PhysicalAnimationState::Passive ? 0
			: std::min(1.0f, m_RecoveryTime / m_RecoveryDuration);
	}
	void PhysicalAnimation::SetStrength(float Strength)
	{
		if (!UnitInterval(Strength)) throw std::invalid_argument("Physical animation strength must be in [0,1]");
		m_Strength = Strength; Wakeup();
	}
	void PhysicalAnimation::SetActive(bool Active)
	{
		m_State = Active ? PhysicalAnimationState::Active : PhysicalAnimationState::Passive; m_Targets = m_Animation; Wakeup();
		if (!Active) for (auto* Drive : m_Drives) Drive->SetEnabled(false);
	}
	void PhysicalAnimation::BeginRecovery(float Duration)
	{
		if (!std::isfinite(Duration) || Duration < 0) throw std::invalid_argument("Invalid physical animation recovery time");
		if (Duration == 0) { SetActive(true); return; }
		m_RecoveryStart = m_Targets = GetPhysicsPose(); m_RecoveryTime = 0; m_RecoveryDuration = Duration;
		m_State = PhysicalAnimationState::Recovering; Wakeup();
	}
	std::vector<Transform> PhysicalAnimation::GetBlendedPose() const
	{
		const float Progress = SmoothStep(GetRecoveryProgress()); std::vector<float> Weights; Weights.reserve(m_Bones.size());
		for (const auto& Control : m_Controls) Weights.push_back(1 + (Control.PhysicsBlend - 1) * Progress);
		return BlendPoses(m_Targets, GetPhysicsPose(), Weights);
	}
	const JointReaction& PhysicalAnimation::GetReaction(size_t Bone) const { return m_Drives.at(Bone)->GetReaction(); }
	void PhysicalAnimation::BeginStep(float Dt)
	{
		const auto Previous = m_Targets;
		if (m_AnimationDuration > 0)
		{
			m_AnimationTime = std::min(m_AnimationTime + Dt, m_AnimationDuration);
			m_Animation = BlendPoses(m_AnimationStart, m_AnimationEnd, std::vector<float>(m_Bones.size(), m_AnimationTime / m_AnimationDuration));
		}
		if (m_State == PhysicalAnimationState::Recovering)
		{
			m_RecoveryTime = std::min(m_RecoveryTime + Dt, m_RecoveryDuration);
			if (m_RecoveryTime >= m_RecoveryDuration) m_State = PhysicalAnimationState::Active;
			m_Targets = BlendPoses(m_RecoveryStart, m_Animation, std::vector<float>(m_Bones.size(), SmoothStep(GetRecoveryProgress())));
		}
		else m_Targets = m_Animation;
		const float Strength = m_Strength * SmoothStep(GetRecoveryProgress());
		for (size_t I = 0; I < m_Bones.size(); ++I)
		{
			const auto& Control = m_Controls[I]; const int Parent = m_Bones[I].Parent;
			const bool Local = Control.Space == PhysicalAnimationSpace::Parent;
			const bool Enabled = Control.Enabled && Strength * Control.Strength > 0 && (!Local || Parent >= 0)
				&& (Control.Linear.IsActive() || Control.Angular.IsActive()) && m_World.GetJointDrivesEnabled();
			auto* Drive = m_Drives[I]; Drive->SetEnabled(Enabled); if (!Enabled) continue;
			Drive->SetReference(Local ? (RigidBody*)m_Bones[Parent].Body : m_Reference.get());
			const Transform Current = Local ? m_Targets[Parent].TransformInv(m_Targets[I]) : m_Targets[I];
			const Transform Before = m_ResetTargetVelocity ? Current : Local ? Previous[Parent].TransformInv(Previous[I]) : Previous[I];
			PoseDriveSettings Settings; Settings.Target = Current;
			Settings.Linear = Scaled(Control.Linear, Strength * Control.Strength); Settings.Angular = Scaled(Control.Angular, Strength * Control.Strength);
			Settings.LinearVelocityTarget = (Current.pos - Before.pos) / Dt; Settings.AngularVelocityTarget = AngularRate(Before.quat, Current.quat, Dt);
			Drive->SetDrive(Settings);
		}
		m_ResetTargetVelocity = false;
	}
}

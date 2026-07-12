#include "Ragdoll.h"

#include <algorithm>
#include <unordered_map>

#include "PhysicsWorld.h"
#include "RagdollDefinition.h"
#include "Constraints.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
	RagdollSettings::RagdollSettings()
		: Scale(1.0f)
		, LinearDamping(0.995f)
		, AngularDamping(0.995f)
		, LinearVelocity(Vector3::Zero())
		, AngularVelocity(Vector3::Zero())
		, JointVelocity(0.0f)
		, EnableCCD(false)
		, DisableSelfCollision(false)
		, DefinitionFile("Contents/Config/default_ragdoll.json")
	{
	}

	Ragdoll::Ragdoll(PhysicsWorld* world, const RagdollSettings& settings, const Transform& rootTransform)
		: m_World(world)
		, m_Settings(settings)
		, m_RootTransform(rootTransform)
		, m_Valid(false)
	{
		m_Settings.Scale = std::max(0.05f, m_Settings.Scale);
		Build();
	}

	RigidBodyDynamic* Ragdoll::GetBone(RagdollBone bone) const
	{
		const size_t index = (size_t)bone;
		return index < m_Bodies.size() ? m_Bodies[index] : nullptr;
	}

	void Ragdoll::Destroy()
	{
		if (m_World == nullptr)
		{
			return;
		}
		for (PositionConstraint* constraint : m_Constraints)
		{
			m_World->RemoveConstraint(constraint);
		}
		m_Constraints.clear();
		for (RigidBodyDynamic* body : m_Bodies)
		{
			if (m_World->RemoveRigidBody(body))
			{
				body->ReleaseGeometries();
				delete body;
			}
		}
		m_Bodies.clear();
		m_Valid = false;
	}

	Vector3 Ragdoll::ToWorld(const Vector3& localPosition) const
	{
		return m_RootTransform.LocalToWorld(localPosition * m_Settings.Scale);
	}

	Vector3 Ragdoll::ToWorldDirection(const Vector3& localDirection) const
	{
		return m_RootTransform.LocalToWorldDirection(localDirection).SafeUnit();
	}

	RigidBodyDynamic* Ragdoll::CreateBoxBone(const Vector3& center, const Vector3& halfExtent, const Quaternion& rotation)
	{
		Geometry* geometry = GeometryFactory::CreateOBB(ToWorld(center), halfExtent * m_Settings.Scale, m_RootTransform.quat * rotation);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	RigidBodyDynamic* Ragdoll::CreateCapsuleBone(const Vector3& start, const Vector3& end, float radius)
	{
		Geometry* geometry = GeometryFactory::CreateCapsule(ToWorld(start), ToWorld(end), radius * m_Settings.Scale);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	RigidBodyDynamic* Ragdoll::CreateSphereBone(const Vector3& center, float radius)
	{
		Geometry* geometry = GeometryFactory::CreateSphere(ToWorld(center), radius * m_Settings.Scale);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	void Ragdoll::Build()
	{
		if (m_World == nullptr)
		{
			return;
		}

		RagdollDefinition definition;
		std::vector<std::string> candidates;
		candidates.push_back(m_Settings.DefinitionFile);
		candidates.push_back("../" + m_Settings.DefinitionFile);
		candidates.push_back("../../" + m_Settings.DefinitionFile);
		candidates.push_back("../../../" + m_Settings.DefinitionFile);
		bool loaded = false;
		for (const std::string& candidate : candidates)
		{
			if (LoadRagdollDefinition(candidate.c_str(), &definition, &m_Error))
			{
				loaded = true;
				break;
			}
		}
		if (!loaded) return;

		const char* expectedBoneNames[] = { "pelvis", "torso", "head",
			"left_upper_arm", "left_lower_arm", "right_upper_arm", "right_lower_arm",
			"left_upper_leg", "left_lower_leg", "right_upper_leg", "right_lower_leg" };
		if (definition.Bones.size() != (size_t)RagdollBone::Count)
		{
			m_Error = "ragdoll definition must contain every bone";
			return;
		}

		std::unordered_map<std::string, RigidBodyDynamic*> bodiesByName;
		for (size_t index = 0; index < definition.Bones.size(); ++index)
		{
			const RagdollBoneDefinition& bone = definition.Bones[index];
			if (bone.Name != expectedBoneNames[index])
			{
				m_Error = "ragdoll bones are missing or out of order";
				return;
			}
			RigidBodyDynamic* body = nullptr;
			if (bone.Shape == RagdollBoneShape::Box)
				body = CreateBoxBone(bone.Center, bone.HalfExtent, bone.Rotation);
			else if (bone.Shape == RagdollBoneShape::Capsule)
				body = CreateCapsuleBone(bone.Start, bone.End, bone.Radius);
			else if (bone.Shape == RagdollBoneShape::Sphere)
				body = CreateSphereBone(bone.Center, bone.Radius);
			if (body == nullptr)
			{
				m_Error = "failed to create ragdoll bone " + bone.Name;
				return;
			}
			bodiesByName[bone.Name] = body;
		}

		for (const RagdollJointDefinition& joint : definition.Joints)
		{
			const auto bodyA = bodiesByName.find(joint.BodyA);
			const auto bodyB = bodiesByName.find(joint.BodyB);
			if (bodyA == bodiesByName.end() || bodyB == bodiesByName.end())
			{
				m_Error = "ragdoll joint references an unknown bone";
				return;
			}
			PositionConstraint* constraint = nullptr;
			if (joint.Type == RagdollJointType::Cone)
			{
				constraint = m_World->CreateConeConstraint(bodyA->second, bodyB->second,
					ToWorld(joint.Anchor), ToWorldDirection(joint.AxisA),
					ToWorldDirection(joint.AxisB), joint.ConeLimit);
			}
			else
			{
				constraint = m_World->CreateHingeConstraint(bodyA->second, bodyB->second,
					ToWorld(joint.Anchor), ToWorldDirection(joint.AxisA),
					joint.MinAngle, joint.MaxAngle);
			}
			if (constraint == nullptr)
			{
				m_Error = "failed to create ragdoll joint";
				return;
			}
			m_Constraints.push_back(constraint);
		}

		if (!m_Settings.DisableSelfCollision)
		{
			// Jointed neighbors overlap at their anchors and should not collide. Other
			// bone pairs retain self collision, matching common ragdoll group filters.
			for (PositionConstraint* constraint : m_Constraints)
			{
				constraint->GetBodyA()->IgnoreCollisionWith(constraint->GetBodyB());
			}
		}

		const Vector3 rootPosition = ToWorld(Vector3::Zero());
		for (RigidBodyDynamic* body : m_Bodies)
		{
			body->V = m_Settings.LinearVelocity + m_Settings.AngularVelocity.Cross(body->X - rootPosition);
			body->W = m_Settings.AngularVelocity;
		}
		for (size_t index = 0; index < definition.Bones.size(); ++index)
		{
			const Vector3 axis = definition.Bones[index].JointVelocityAxis;
			if (!axis.IsZero()) m_Bodies[index]->W += ToWorldDirection(axis) * m_Settings.JointVelocity;
		}

		m_Valid = m_Bodies.size() == (size_t)RagdollBone::Count
			&& m_Constraints.size() == definition.Joints.size();
		if (m_Valid) m_Error.clear();
	}
}

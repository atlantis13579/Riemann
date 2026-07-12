#pragma once

#include <string>
#include <vector>

#include "../Maths/Transform.h"

namespace Riemann
{
	class Geometry;
	class PhysicsWorld;
	class PositionConstraint;
	class RigidBodyDynamic;

	enum class RagdollBone
	{
		Pelvis,
		Torso,
		Head,
		LeftUpperArm,
		LeftLowerArm,
		RightUpperArm,
		RightLowerArm,
		LeftUpperLeg,
		LeftLowerLeg,
		RightUpperLeg,
		RightLowerLeg,
		Count,
	};

	struct RagdollSettings
	{
		RagdollSettings();

		float Scale;
		float LinearDamping;
		float AngularDamping;
		Vector3 LinearVelocity;
		Vector3 AngularVelocity;
		float JointVelocity;
		bool EnableCCD;
		bool DisableSelfCollision;
		std::string DefinitionFile;
	};

	class Ragdoll
	{
	public:
		Ragdoll(PhysicsWorld* world, const RagdollSettings& settings, const Transform& rootTransform);
		~Ragdoll() {}

		bool IsValid() const { return m_Valid; }
		const std::string& GetError() const { return m_Error; }
		void Destroy();
		RigidBodyDynamic* GetBone(RagdollBone bone) const;
		const std::vector<RigidBodyDynamic*>& GetBodies() const { return m_Bodies; }
		const std::vector<PositionConstraint*>& GetConstraints() const { return m_Constraints; }
		size_t GetBodyCount() const { return m_Bodies.size(); }
		size_t GetConstraintCount() const { return m_Constraints.size(); }

	private:
		RigidBodyDynamic* CreateBoxBone(const Vector3& center, const Vector3& halfExtent, const Quaternion& rotation);
		RigidBodyDynamic* CreateCapsuleBone(const Vector3& start, const Vector3& end, float radius);
		RigidBodyDynamic* CreateSphereBone(const Vector3& center, float radius);
		Vector3 ToWorld(const Vector3& localPosition) const;
		Vector3 ToWorldDirection(const Vector3& localDirection) const;
		void Build();

	private:
		PhysicsWorld* m_World;
		RagdollSettings m_Settings;
		Transform m_RootTransform;
		std::vector<RigidBodyDynamic*> m_Bodies;
		std::vector<PositionConstraint*> m_Constraints;
		std::string m_Error;
		bool m_Valid;
	};
}

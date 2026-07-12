#pragma once

#include <string>
#include <vector>

#include "../Maths/Quaternion.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	enum class RagdollBoneShape
	{
		Box,
		Capsule,
		Sphere,
	};

	struct RagdollBoneDefinition
	{
		std::string Name;
		RagdollBoneShape Shape = RagdollBoneShape::Sphere;
		Vector3 Center = Vector3::Zero();
		Vector3 HalfExtent = Vector3::Zero();
		Quaternion Rotation = Quaternion::One();
		Vector3 Start = Vector3::Zero();
		Vector3 End = Vector3::Zero();
		float Radius = 0.0f;
		Vector3 JointVelocityAxis = Vector3::Zero();
	};

	enum class RagdollJointType
	{
		Cone,
		Hinge,
	};

	struct RagdollJointDefinition
	{
		RagdollJointType Type = RagdollJointType::Cone;
		std::string BodyA;
		std::string BodyB;
		Vector3 Anchor = Vector3::Zero();
		Vector3 AxisA = Vector3::UnitY();
		Vector3 AxisB = Vector3::UnitY();
		float MinAngle = 0.0f;
		float MaxAngle = 0.0f;
		float ConeLimit = 0.0f;
	};

	struct RagdollDefinition
	{
		std::vector<RagdollBoneDefinition> Bones;
		std::vector<RagdollJointDefinition> Joints;
	};

	bool LoadRagdollDefinition(const char* FileName, RagdollDefinition* Definition,
		std::string* ErrorMessage = nullptr);
}

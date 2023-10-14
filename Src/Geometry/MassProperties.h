#pragma once

#include <vector>
#include "../Maths/Pose.h"

void ComputeCompositeMassProperties(
	const std::vector<Pose>& vPose,
	const std::vector<float>& vMass,
	const std::vector<float>& vVolume,
	const std::vector<Matrix3>& vInertia,
	const std::vector<Vector3>& vCenterOfMass,
	float& Mass, float& Volume, Matrix3& InertiaMat, Vector3& InertiaVec, Vector3& CenterOfMass);
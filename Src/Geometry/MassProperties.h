#pragma once

#include <vector>
#include "../Maths/Transform.h"

struct MassProperties
{
	float Mass;
	float Volume;
	Matrix3 InertiaMat;
	Vector3 InertiaVec;
	Vector3 CenterOfMass;
};

void ComputeCompositeMassProperties(const std::vector<Pose> &vPose, const std::vector<MassProperties>& vProperties, MassProperties& P);
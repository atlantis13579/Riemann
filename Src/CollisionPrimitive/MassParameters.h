#pragma once

#include <vector>
#include "../Maths/Box3.h"
#include "../Maths/Transform.h"
#include "ShapeType.h"

namespace Geometry
{
	void ComputeCompositeMassParameters(const std::vector<Transform>& vPose, const std::vector<MassParameters>& vProperties, MassParameters& P);
	void ComputeCompositeMassParameters(const std::vector<const Transform*>& vPose, const std::vector<const MassParameters*>& vProperties, MassParameters& P);
	Vector3 DiagonalizeInertiaMat(const Matrix3& InertiaMat, float Mass, Matrix3& Rotation);
}
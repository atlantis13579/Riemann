#pragma once

#include <vector>
#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"
#include "ShapeType.h"

void ComputeCompositeMassParameters(const std::vector<Pose>& vPose, const std::vector<MassParameters>& vProperties, MassParameters& P);
void ComputeCompositeMassParameters(const std::vector<const Pose*> &vPose, const std::vector<const MassParameters*>& vProperties, MassParameters& P);
Vector3 DiagonalizeInertiaMat(const Matrix3& InertiaMat, float Mass, Matrix3& Rotation);
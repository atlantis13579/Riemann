#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

struct HullFace3d
{
	Vector3				norm;
	float				w;
	std::vector<short>	verts;
};

struct ConvexHull3d
{
	std::vector<Vector3>	verts;
	std::vector<HullFace3d>	faces;
};

Matrix3 ComputePointCloudInertiaTensor_PCA(const Vector3 *Vertices, int NumVertices);

Matrix3 ComputePolyhedralInertiaTensor_VolumeIntegration(const ConvexHull3d &hull);
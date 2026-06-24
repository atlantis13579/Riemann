#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
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

	struct ConvexHull3Options
	{
		float DistanceTolerance = 1e-5f;
	};

	bool BuildConvexHull3(const Vector3* Points, int NumPoints, ConvexHull3d& OutHull, const ConvexHull3Options& Options = ConvexHull3Options());
	bool BuildConvexHull3(const std::vector<Vector3>& Points, ConvexHull3d& OutHull, const ConvexHull3Options& Options = ConvexHull3Options());

	Matrix3 ComputePointCloudInertiaTensor_PCA(const Vector3* Vertices, int NumVertices);

	Matrix3 ComputePolyhedralInertiaTensor_VolumeIntegration(const ConvexHull3d& hull, float density, float& Volume, float& Mass, Vector3& CenterOfMass);
}

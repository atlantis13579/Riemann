#include <assert.h>
#include "MassProperties.h"

// Calculating inertia for a multi shape rigid body
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
void ComputeCompositeMassProperties(const std::vector<Pose>& vPose, const std::vector<MassProperties>& vProperties, MassProperties& P)
{
	assert(vPose.size() == vProperties.size());

	P.Mass = 0.0f;
	P.Volume = 0.0f;
	P.CenterOfMass = Vector3::Zero();
	P.InertiaMat = Matrix3::Identity();
	P.InertiaVec = Vector3::One();

	if (vProperties.empty())
	{
		return;
	}

	for (size_t i = 0; i < vProperties.size(); ++i)
	{
		P.Mass += vProperties[i].Mass;
		P.Volume += vProperties[i].Volume;
		P.CenterOfMass += (vPose[i].pos + vProperties[i].CenterOfMass) * vProperties[i].Mass;
	}
	P.CenterOfMass = fabsf(P.Mass) > 1e-3f ? (P.CenterOfMass / P.Mass) : Vector3(0.0f, 0.0f, 0.0f);
	P.Mass = std::max(1e-3f, P.Mass);
	P.Volume = std::max(1e-3f, P.Volume);

	P.InertiaMat = Matrix3::Zero();
	for (size_t i = 0; i < vProperties.size(); ++i)
	{
		const float d = vProperties[i].Mass * (vPose[i].pos + vProperties[i].CenterOfMass - P.CenterOfMass).SquareLength();
		const Matrix3 D(Vector3(d, d, d), Vector3(d, d, d), Vector3(d, d, d), true);
		P.InertiaMat += vPose[i].quat.ToRotationMatrix3() * vProperties[i].InertiaMat + D;
	}

	float eigen_values[3];
	Vector3 eigen_vectors[3];
	if (P.InertiaMat.SolveEigenSymmetric(eigen_values, eigen_vectors))
	{
		P.InertiaVec = Vector3(fabsf(eigen_values[0]), fabsf(eigen_values[1]), fabsf(eigen_values[2]));
	}
	else
	{
		// default value
		P.InertiaVec = Vector3(P.Mass, P.Mass, P.Mass);
	}
}
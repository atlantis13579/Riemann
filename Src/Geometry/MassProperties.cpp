#include <assert.h>
#include "MassProperties.h"

// Calculating inertia for a multi shape rigid body
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
void ComputeCompositeMassProperties(
	const std::vector<Pose>& vPose,
	const std::vector<float>& vMass,
	const std::vector<float>& vVolume,
	const std::vector<Matrix3>& vInertia,
	const std::vector<Vector3>& vCenterOfMass,
	float& Mass, float& Volume, Matrix3& InertiaMat, Vector3& InertiaVec, Vector3& CenterOfMass)
{
	assert(vPose.size() == vMass.size());
	assert(vPose.size() == vInertia.size());
	assert(vPose.size() == vVolume.size());
	assert(vPose.size() == vCenterOfMass.size());

	Mass = 0.0f;
	Volume = 0.0f;
	CenterOfMass = Vector3::Zero();
	InertiaMat = Matrix3::Identity();
	InertiaVec = Vector3::One();

	if (vPose.empty())
	{
		return;
	}

	for (size_t i = 0; i < vMass.size(); ++i)
	{
		Mass += vMass[i];
		Volume += vVolume[i];
		CenterOfMass += (vPose[i].pos + vCenterOfMass[i]) * vMass[i];
	}
	CenterOfMass = fabsf(Mass) > 1e-3f ? (CenterOfMass / Mass) : Vector3(0.0f, 0.0f, 0.0f);
	Mass = std::max(1e-3f, Mass);
	Volume = std::max(1e-3f, Volume);

	InertiaMat = Matrix3::Zero();
	for (size_t i = 0; i < vMass.size(); ++i)
	{
		const float d = vMass[i] * (vPose[i].pos + vCenterOfMass[i] - CenterOfMass).SquareLength();
		const Matrix3 D(Vector3(d, d, d), Vector3(d, d, d), Vector3(d, d, d), true);
		InertiaMat += vPose[i].quat.ToRotationMatrix3() * vInertia[i] + D;
	}

	float eigen_values[3];
	Vector3 eigen_vectors[3];
	if (InertiaMat.SolveEigenSymmetric(eigen_values, eigen_vectors))
	{
		InertiaVec = Vector3(fabsf(eigen_values[0]), fabsf(eigen_values[1]), fabsf(eigen_values[2]));
	}
	else
	{
		// default value
		InertiaVec = Vector3(Mass, Mass, Mass);
	}
}
#include <assert.h>
#include "MassParameters.h"

namespace Geometry
{
	// Calculating inertia for a multi shape rigid body
	// https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
	void ComputeCompositeMassParameters(const std::vector<Transform>& vPose, const std::vector<MassParameters>& vProperties, MassParameters& P)
	{
		assert(vPose.size() == vProperties.size());

		if (vPose.size() == 1)
		{
			P.Volume = vProperties[0].Volume;
			P.Mass = vProperties[0].Mass;
			P.CenterOfMass = vPose[0].pos + vPose[0].quat * vProperties[0].CenterOfMass;
			P.InertiaMat = vProperties[0].InertiaMat;
			P.BoundingVolume = vProperties[0].BoundingVolume;
			return;
		}

		P.Mass = 0.0f;
		P.Volume = 0.0f;
		P.CenterOfMass = Vector3::Zero();
		P.InertiaMat = Matrix3::Zero();
		P.BoundingVolume.SetEmpty();

		if (vPose.empty())
		{
			return;
		}

		for (size_t i = 0; i < vProperties.size(); ++i)
		{
			P.Mass += vProperties[i].Mass;
			P.Volume += vProperties[i].Volume;
			P.CenterOfMass += (vPose[i].pos + vPose[i].quat * vProperties[i].CenterOfMass) * vProperties[i].Mass;
			P.BoundingVolume.Encapsulate(vPose[i].pos);
		}
		P.Mass = std::max(1e-3f, P.Mass);
		P.Volume = std::max(1e-3f, P.Volume);
		P.CenterOfMass = P.CenterOfMass / P.Mass;

		for (size_t i = 0; i < vProperties.size(); ++i)
		{
			const Vector3 d = vProperties[i].Mass * (vPose[i].pos + vPose[i].quat * vProperties[i].CenterOfMass - P.CenterOfMass);
			const Matrix3 D(Vector3(0.0f, -d.z, d.y), Vector3(d.z, 0.0f, -d.x), Vector3(-d.y, d.x, 0.0f), false);
			const Matrix3 m = vPose[i].quat.ToRotationMatrix3();
			P.InertiaMat += m * vProperties[i].InertiaMat * m.Transpose() + D * (vProperties[i].Mass) * D.Transpose();
		}
	}

	// Calculating inertia for a multi shape rigid body
	// https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
	void ComputeCompositeMassParameters(const std::vector<const Transform*>& vPose, const std::vector<const MassParameters*>& vProperties, MassParameters& P)
	{
		assert(vPose.size() == vProperties.size());

		if (vPose.size() == 1)
		{
			P.Volume = vProperties[0]->Volume;
			P.Mass = vProperties[0]->Mass;
			P.CenterOfMass = vPose[0]->pos + vPose[0]->quat * vProperties[0]->CenterOfMass;
			P.InertiaMat = vProperties[0]->InertiaMat;
			P.BoundingVolume = vProperties[0]->BoundingVolume;
			return;
		}

		P.Mass = 0.0f;
		P.Volume = 0.0f;
		P.CenterOfMass = Vector3::Zero();
		P.InertiaMat = Matrix3::Zero();
		P.BoundingVolume.SetEmpty();

		if (vPose.empty())
		{
			return;
		}

		for (size_t i = 0; i < vProperties.size(); ++i)
		{
			P.Mass += vProperties[i]->Mass;
			P.Volume += vProperties[i]->Volume;
			P.CenterOfMass += (vPose[i]->pos + vPose[i]->quat * vProperties[i]->CenterOfMass) * vProperties[i]->Mass;
			P.BoundingVolume.Encapsulate(vPose[i]->pos);
		}
		P.Mass = std::max(1e-3f, P.Mass);
		P.Volume = std::max(1e-3f, P.Volume);
		P.CenterOfMass = P.CenterOfMass / P.Mass;

		for (size_t i = 0; i < vProperties.size(); ++i)
		{
			const Vector3 d = vProperties[i]->Mass * (vPose[i]->pos + vPose[i]->quat * vProperties[i]->CenterOfMass - P.CenterOfMass);
			const Matrix3 D(Vector3(0.0f, -d.z, d.y), Vector3(d.z, 0.0f, -d.x), Vector3(-d.y, d.x, 0.0f), false);
			const Matrix3 m = vPose[i]->quat.ToRotationMatrix3();
			P.InertiaMat += m * vProperties[i]->InertiaMat * m.Transpose() + D * (vProperties[i]->Mass) * D.Transpose();
		}
	}

	Vector3 DiagonalizeInertiaMat(const Matrix3& InertiaMat, float Mass, Matrix3& Rotation)
	{
		Vector3 InertiaVec = Vector3::One();

		float eigen_values[3];
		Vector3 eigen_vectors[3];
		if (InertiaMat.SolveEigenSymmetric(eigen_values, eigen_vectors))
		{
			eigen_vectors[0].Normalize();
			eigen_vectors[1].Normalize();
			eigen_vectors[2].Normalize();
			if (eigen_vectors[0].Dot(eigen_vectors[1].Cross(eigen_vectors[2])) < 0.0f)
			{
				eigen_vectors[2] = -eigen_vectors[2];
			}

			Rotation = Matrix3(eigen_vectors[0], eigen_vectors[1], eigen_vectors[2], false);
			InertiaVec = Vector3(fabsf(eigen_values[0]), fabsf(eigen_values[1]), fabsf(eigen_values[2]));
		}
		else
		{
			// default value
			Rotation = Matrix3::Identity();
			InertiaVec = Vector3(Mass, Mass, Mass);
		}
		return InertiaVec;
	}
}
#include <assert.h>
#include "MassParameters.h"

namespace Riemann
{
	namespace
	{
		void ComputeCompositeMassParametersInternal(const std::vector<const Transform*>& poses, const std::vector<const MassParameters*>& properties, MassParameters& result)
		{
			assert(poses.size() == properties.size());

			result.Mass = 0.0f;
			result.Volume = 0.0f;
			result.CenterOfMass = Vector3::Zero();
			result.InertiaMat = Matrix3::Zero();
			result.BoundingVolume.SetEmpty();
			if (poses.empty())
			{
				return;
			}

			for (size_t i = 0; i < properties.size(); ++i)
			{
				const Transform& pose = *poses[i];
				const MassParameters& shape = *properties[i];
				const Vector3 shapeCenter = pose.pos + pose.quat * shape.CenterOfMass;
				result.Mass += shape.Mass;
				result.Volume += shape.Volume;
				result.CenterOfMass += shapeCenter * shape.Mass;
				result.BoundingVolume.Encapsulate(Box3::Transform(shape.BoundingVolume, pose.pos, pose.quat));
			}

			if (result.Mass <= 1e-6f)
			{
				result.Mass = 1e-3f;
				return;
			}
			result.CenterOfMass /= result.Mass;
			result.Volume = std::max(1e-3f, result.Volume);

			for (size_t i = 0; i < properties.size(); ++i)
			{
				const Transform& pose = *poses[i];
				const MassParameters& shape = *properties[i];
				const Matrix3 rotation = pose.quat.ToRotationMatrix3();
				const Vector3 offset = pose.pos + pose.quat * shape.CenterOfMass - result.CenterOfMass;
				const float offsetLengthSq = offset.SquareLength();
				const Matrix3 parallelAxis = (Matrix3::Identity() * offsetLengthSq - Matrix3::OuterProduct(offset, offset)) * shape.Mass;
				result.InertiaMat += rotation * shape.InertiaMat * rotation.Transpose() + parallelAxis;
			}
		}
	}

	// Calculating inertia for a multi shape rigid body
	// https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
	void ComputeCompositeMassParameters(const std::vector<Transform>& vPose, const std::vector<MassParameters>& vProperties, MassParameters& P)
	{
		std::vector<const Transform*> poses;
		std::vector<const MassParameters*> properties;
		poses.reserve(vPose.size());
		properties.reserve(vProperties.size());
		for (size_t i = 0; i < vPose.size(); ++i)
		{
			poses.push_back(&vPose[i]);
			properties.push_back(&vProperties[i]);
		}
		ComputeCompositeMassParametersInternal(poses, properties, P);
	}

	// Calculating inertia for a multi shape rigid body
	// https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// https://stackoverflow.com/questions/13602661/calculating-inertia-for-a-multi-shape-rigid-body
	void ComputeCompositeMassParameters(const std::vector<const Transform*>& vPose, const std::vector<const MassParameters*>& vProperties, MassParameters& P)
	{
		ComputeCompositeMassParametersInternal(vPose, vProperties, P);
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

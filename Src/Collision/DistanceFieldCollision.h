#pragma once

#include "../Maths/Transform.h"
#include "../Geometry/RuntimeDistanceField.h"
#include <vector>
#include <cstdint>

namespace Riemann
{
	class Geometry;
	struct DistanceFieldContact
	{
		Vector3 PointA, PointB, Normal;
		float Separation = 0.0f;
		uint32_t FeatureA = UINT32_MAX, FeatureB = UINT32_MAX;
	};

	struct DistanceFieldPairDistance
	{
		Vector3 PointA = Vector3::Zero(), PointB = Vector3::Zero(), Normal = Vector3::Zero();
		float LowerBound = 0.0f, UpperBound = std::numeric_limits<float>::infinity();
		bool Converged = false, WitnessValid = false;
		DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::InvalidInput;
		DistanceFieldQueryStats Stats;
	};

	struct DistanceFieldSweepResult
	{
		Vector3 Position = Vector3::Zero(), Normal = Vector3::Zero();
		float Time = 0.0f, DistanceError = 0.0f;
		bool Hit = false, InitialOverlap = false, Conservative = false;
		DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::InvalidInput;
		uint32_t Iterations = 0;
		DistanceFieldQueryStats Stats;
	};

	class DistanceFieldCollision
	{
	public:
		static bool HasField(const Geometry* A, const Geometry* B);
		static bool Generate(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
			float ContactDistance, std::vector<DistanceFieldContact>& Contacts);
		static bool Intersect(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB);
		static bool Distance(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
			DistanceFieldPairDistance& Result, float DistanceTolerance = 0.0f);
		static bool SweepBounded(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
			const Vector3& LocalDirection, float MaxTime, DistanceFieldSweepResult& Result);
		static bool Sweep(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
			const Vector3& LocalDirection, Vector3* Position, Vector3* Normal, float* Time);
	};
}

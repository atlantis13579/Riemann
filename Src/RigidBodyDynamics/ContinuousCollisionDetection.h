#pragma once

#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	class Geometry;
	class CollisionFilter;
	class RigidBody;
	class RigidBodyDynamic;
	class PositionConstraint;

	struct CCDContinuationSettings
	{
		bool Enabled = false;
		int MaxPasses = 8;
		int ImpactIterations = 12;
		float SimultaneousTolerance = 1.0e-5f;
		float VelocityTolerance = 1.0e-4f;
		float ContactTolerance = 5.0e-4f;
		float RestitutionThreshold = 1.0f;
		bool IsValid() const;
	};

	struct CCDContinuationStats
	{
		int ImpactCount = 0;
		int ResweepCount = 0;
		int ClampedBodyCount = 0;
		float MaximumDroppedTime = 0.0f;
	};

	struct ContinuousCollisionHit
	{
		ContinuousCollisionHit();

		RigidBody* Body;
		RigidBody* OtherBody;
		Geometry* Shape;
		Geometry* OtherShape;
		Vector3 Position;
		Vector3 Normal;
		float Fraction;
		bool AngularHit;
		bool ConservativeHit;
	};

	class ContinuousCollisionDetection
	{
	public:
		static bool IsSupportedMovingShape(const Geometry* shape);
		static bool FindEarliestHit(RigidBodyDynamic* body, const std::vector<RigidBody*>& bodies, float dt, ContinuousCollisionHit* hit, CollisionFilter* Filter = nullptr);
		static void FindTriggerHits(const std::vector<RigidBody*>& bodies, float dt, std::vector<ContinuousCollisionHit>* hits, CollisionFilter* Filter = nullptr);
		static CCDContinuationStats ResolveRemainingTime(std::vector<RigidBodyDynamic*>& DynamicBodies,
			const std::vector<RigidBody*>& Bodies, float Dt, const CCDContinuationSettings& Settings,
			const std::vector<PositionConstraint*>& Joints, std::vector<ContinuousCollisionHit>* TriggerHits = nullptr,
			CollisionFilter* Filter = nullptr);
		static int Resolve(std::vector<RigidBodyDynamic*>& dynamicBodies, const std::vector<RigidBody*>& bodies, float dt, CollisionFilter* Filter = nullptr);
	};
}

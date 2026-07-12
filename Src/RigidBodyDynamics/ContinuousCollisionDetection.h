#pragma once

#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class RigidBodyDynamic;

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
	};

	class ContinuousCollisionDetection
	{
	public:
		static bool IsSupportedMovingShape(const Geometry* shape);
		static bool FindEarliestHit(RigidBodyDynamic* body, const std::vector<RigidBody*>& bodies, float dt, ContinuousCollisionHit* hit);
		static void FindTriggerHits(const std::vector<RigidBody*>& bodies, float dt, std::vector<ContinuousCollisionHit>* hits);
		static int Resolve(std::vector<RigidBodyDynamic*>& dynamicBodies, const std::vector<RigidBody*>& bodies, float dt);
	};
}

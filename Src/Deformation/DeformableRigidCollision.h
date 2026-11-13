#pragma once

#include <memory>
#include <vector>
#include "../Maths/Transform.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class DeformableBody;
	class CollisionFilter;
	struct DeformableRigidCollider
	{
		Geometry* Shape = nullptr;
		RigidBody* Body = nullptr;
		Transform StartPose;
	};
	class DeformableRigidCollision
	{
	public:
		DeformableRigidCollision(const std::vector<DeformableBody*>& Bodies, const std::vector<DeformableRigidCollider>& Colliders, CollisionFilter* Filter);
		~DeformableRigidCollision();
		void Solve(float Dt);
		const std::vector<RigidBody*>& GetTouchedBodies() const;
	private:
		struct Implementation;
		std::unique_ptr<Implementation> m_Data;
	};
}

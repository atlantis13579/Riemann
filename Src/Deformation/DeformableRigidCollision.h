#pragma once

#include <memory>
#include <vector>
#include "../Maths/Transform.h"
#include "../Collision/GeometryObject.h"
#include "SurfaceContactConstraint.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class DeformableBody;
	class CollisionFilter;
	struct BroadPhaseResult;
	// Borrowed surface data shared by tetrahedral bodies and cloth during one world substep.
	struct RigidContactSurface
	{
		Geometry* BroadPhaseGeometry = nullptr;
		std::vector<SurfaceContactParticle> Particles;
		std::vector<const Vector3*> PreviousPositions;
		std::vector<Index3> Triangles;
		float Thickness = 0, Tolerance = 1e-5f;
		SurfaceContactMaterial Material;
		CollisionData FilterData = CollisionData::ForLayer(0);
		size_t* Candidates = nullptr;
		size_t* Contacts = nullptr;
		size_t* RigidContacts = nullptr;
		double* MaximumPenetration = nullptr;
	};
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
		DeformableRigidCollision(const std::vector<RigidContactSurface>& Surfaces, const std::vector<DeformableRigidCollider>& Colliders, CollisionFilter* Filter);
		~DeformableRigidCollision();
		void Solve(float Dt, const BroadPhaseResult& Pairs);
		const std::vector<RigidBody*>& GetTouchedBodies() const;
	private:
		struct Implementation;
		std::unique_ptr<Implementation> m_Data;
	};
}

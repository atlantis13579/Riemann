#pragma once

#include <memory>
#include <vector>
#include "SurfaceContactConstraint.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
	class DeformableBody;
	struct BroadPhaseResult;
	struct DeformableCollisionSettings
	{
		bool SelfCollision = false, InterCollision = false;
		bool RigidCollision = false;
		CollisionData FilterData = CollisionData::ForLayer(0);
		bool ExcludeSelfNeighbors = true;
		float ContactTolerance = 1e-5f;
		SurfaceContactMaterial Material;
		bool IsValid() const;
	};
	struct DeformableCollisionStats
	{
		size_t Candidates = 0, Contacts = 0, SelfContacts = 0, InterContacts = 0, RigidContacts = 0;
		double MaximumPenetration = 0;
	};
	class DeformableCollision
	{
	public:
		explicit DeformableCollision(const std::vector<DeformableBody*>& Bodies);
		~DeformableCollision();
		DeformableCollision(const DeformableCollision&) = delete;
		DeformableCollision& operator=(const DeformableCollision&) = delete;
		static bool CanInteract(const DeformableBody& A, const DeformableBody& B);
		void Solve(float Dt, const BroadPhaseResult& Pairs);
	private:
		struct Implementation;
		std::unique_ptr<Implementation> m_Data;
	};
}

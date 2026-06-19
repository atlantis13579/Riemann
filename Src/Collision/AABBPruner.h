#pragma once

#include <limits.h>
#include <vector>

#include "../CollisionPrimitive/Ray3.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct RayCastOption;
	struct RayCastResult;
	struct IntersectOption;
	struct IntersectResult;
	struct SweepOption;
	struct SweepResult;
	class Geometry;

	class AABBPruner
	{
	public:
		virtual ~AABBPruner();

		static AABBPruner* CreateSimple(int maxObjects = 64);
		static AABBPruner* CreateBucketStyle(int maxObjects = INT_MAX, int bucketSize = 16);

		virtual void		Clear() = 0;
		virtual bool		Add(Geometry* object) = 0;
		virtual bool		Remove(Geometry* object) = 0;
		virtual bool		Update(Geometry* object) = 0;

		virtual bool		RayCast(const Ray3& ray, const RayCastOption* option, RayCastResult* result) const = 0;
		virtual bool		Intersect(const Geometry* geometry, const IntersectOption* option, IntersectResult* result) const = 0;
		virtual bool		Sweep(const Geometry* geometry, const Vector3& direction, const SweepOption* option, SweepResult* result) const = 0;
		virtual void		CollectAABBs(std::vector<Box3>* aabbs) const = 0;
		virtual int			GetObjectCount() const = 0;

	protected:
		AABBPruner();
	};
}

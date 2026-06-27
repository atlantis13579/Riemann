#pragma once

#include <vector>

#include "BroadPhase.h"

namespace Riemann
{
	class ContactManifold;

	class NarrowPhase
	{
	public:
		virtual ~NarrowPhase() {}
		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* contact) = 0;

		static NarrowPhase* Create_GJKEPA();
		static NarrowPhase* Create_SAT();
		static NarrowPhase* Create_PCM();
	};
}

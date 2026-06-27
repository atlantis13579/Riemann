#pragma once

#include <stdint.h>
#include <vector>

#include "RigidBody.h"

namespace Riemann
{
	class Geometry;

	struct BroadPhaseProxy
	{
		BroadPhaseProxy()
			: Handle()
			, Displacement(Vector3::Zero())
			, Version(0)
			, FrameId(0)
			, SyncId(0)
			, StateIndex(-1)
			, PrunerHandle(-1)
			, Moved(false)
			, MovingRigid(false)
			, Active(false)
		{
		}

		GeometryHandle	Handle;
		Box3			WorldBounds;
		Vector3			Displacement;
		uint64_t		Version;
		uint64_t		FrameId;
		uint64_t		SyncId;
		int				StateIndex;
		int				PrunerHandle;
		bool			Moved;
		bool			MovingRigid;
		bool			Active;
	};

	struct OverlapPair
	{
		OverlapPair(int _Geom1, int _Geom2)
		{
			index1 = _Geom1;
			index2 = _Geom2;
		}
		int index1;
		int index2;
	};

	class BroadPhase
	{
	public:
		virtual ~BroadPhase() {}
		void ProduceOverlaps(const std::vector<GeometryWorldState>& states, std::vector<OverlapPair>* overlaps);
		virtual void ProduceOverlaps(GeometryWorldStateSpan states, std::vector<OverlapPair>* overlaps) = 0;

		static BroadPhase* Create_AllPairs();		// Slow, debug purpose
		static BroadPhase* Create_Bruteforce();		// Slow, debug purpose
		static BroadPhase* Create_SAP();
		static BroadPhase* Create_ABP();
		static BroadPhase* Create_MBP();
		static BroadPhase* Create_DynamicAABB();
	};
}

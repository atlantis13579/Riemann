#pragma once

#include <vector>

namespace Riemann
{
	class GeometryBase;
	class DynamicAABBTree;

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
		virtual void ProduceOverlaps(const std::vector<GeometryBase*>& geoms, std::vector<OverlapPair>* overlaps) = 0;

		static BroadPhase* Create_AllPairs();		// Slow, debug purpose
		static BroadPhase* Create_Bruteforce();		// Slow, debug purpose
		static BroadPhase* Create_SAP();
		static BroadPhase* Create_DynamicAABB(DynamicAABBTree* tree);
	};
}
#pragma once

#include <vector>
#include "ConnectionGraph.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	class DestructionSet
	{
	public:
		ConnectionGraph				mGraph;
		Box3						mBounds;
	};
}	// namespace Riemann

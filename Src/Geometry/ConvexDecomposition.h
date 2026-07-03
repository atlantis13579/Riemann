#pragma once

#include <vector>

namespace Riemann
{
	class ConvexMesh;
	class StaticMesh;

	namespace ConvexDecomposition
	{
		struct VHACDParameters
		{
			int MaxPieceCount = 16;
			int VoxelResolution = 10000;
			double MinimumVolumePercentErrorAllowed = 4.0;
			int MaxRecursionDepth = 6;
			int MaxVerticesPerHull = 48;
			bool ShrinkWrap = false;
		};

		bool DecomposeVHACD(const StaticMesh& sourceMesh, const VHACDParameters& parameters, std::vector<ConvexMesh>& convexMeshes);
	}
}

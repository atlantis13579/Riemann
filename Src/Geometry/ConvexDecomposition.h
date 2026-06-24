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

		struct COACDParameters
		{
			int MaxPieceCount = 16;
			double Threshold = 0.08;
			int PreprocessResolution = 30;
			int SampleResolution = 800;
			int MctsNodes = 12;
			int MctsIterations = 80;
			int MctsMaxDepth = 2;
			bool Merge = true;
			bool Decimate = false;
			int MaxVerticesPerHull = 48;
			bool Extrude = false;
			double ExtrudeMargin = 0.01;
			unsigned int Seed = 0;
		};

		bool DecomposeVHACD(const StaticMesh& sourceMesh, const VHACDParameters& parameters, std::vector<ConvexMesh>& convexMeshes);
		bool DecomposeCOACD(const StaticMesh& sourceMesh, const COACDParameters& parameters, std::vector<ConvexMesh>& convexMeshes);
	}
}

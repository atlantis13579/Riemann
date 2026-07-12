#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	struct FluidSurfaceMesh
	{
		void Clear();

		std::vector<Vector3> Positions;
		std::vector<Vector3> Normals;
		std::vector<uint32_t> Indices;
	};

	struct MarchingCubesInput
	{
		MarchingCubesInput();

		Vector3 Origin;
		Vector3 Spacing;
		Vector3i Resolution;
		const float* Values;
		size_t ValueCount;
		float IsoValue;
		bool HighValuesInside;
	};

	class MarchingCubes
	{
	public:
		static bool Extract(const MarchingCubesInput& Input, FluidSurfaceMesh* Mesh);
	};
}

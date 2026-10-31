#pragma once

#include <algorithm>
#include <cmath>
#include "../Maths/Vector3.h"

namespace Riemann
{
	// Specialize the staggered component so the inner eight-corner MAC
	// interpolation has no component switches or repeated coordinate clamping.
	// The normal component uses zero ghost faces; tangential values extend to
	// the wall. Both fluid and gas advection use this path.
	template<int Component>
	inline float SampleMACComponent(const Vector3* Field, const Vector3i& Resolution,
		const Vector3& GridPosition)
	{
		const float GX = GridPosition.x - (Component == 0 ? 0.5f : 0.0f);
		const float GY = GridPosition.y - (Component == 1 ? 0.5f : 0.0f);
		const float GZ = GridPosition.z - (Component == 2 ? 0.5f : 0.0f);
		const int X = (int)std::floor(GX), Y = (int)std::floor(GY), Z = (int)std::floor(GZ);
		const float TX = GX - (float)X, TY = GY - (float)Y, TZ = GZ - (float)Z;
		const int X0 = std::max(0, std::min(Resolution.x - 1, X));
		const int X1 = std::max(0, std::min(Resolution.x - 1, X + 1));
		const int Y0 = std::max(0, std::min(Resolution.y - 1, Y)) * Resolution.x;
		const int Y1 = std::max(0, std::min(Resolution.y - 1, Y + 1)) * Resolution.x;
		const int Slice = Resolution.x * Resolution.y;
		const int Z0 = std::max(0, std::min(Resolution.z - 1, Z)) * Slice;
		const int Z1 = std::max(0, std::min(Resolution.z - 1, Z + 1)) * Slice;
		// Tangential velocity extends to the wall; normal ghost faces are zero.
		const float WX0 = (Component != 0 || (X >= 0 && X < Resolution.x)) ? 1.0f - TX : 0.0f;
		const float WX1 = (Component != 0 || (X + 1 >= 0 && X + 1 < Resolution.x)) ? TX : 0.0f;
		const float WY0 = (Component != 1 || (Y >= 0 && Y < Resolution.y)) ? 1.0f - TY : 0.0f;
		const float WY1 = (Component != 1 || (Y + 1 >= 0 && Y + 1 < Resolution.y)) ? TY : 0.0f;
		const float WZ0 = (Component != 2 || (Z >= 0 && Z < Resolution.z)) ? 1.0f - TZ : 0.0f;
		const float WZ1 = (Component != 2 || (Z + 1 >= 0 && Z + 1 < Resolution.z)) ? TZ : 0.0f;
		const float C00 = Field[X0 + Y0 + Z0][Component] * WX0 + Field[X1 + Y0 + Z0][Component] * WX1;
		const float C10 = Field[X0 + Y1 + Z0][Component] * WX0 + Field[X1 + Y1 + Z0][Component] * WX1;
		const float C01 = Field[X0 + Y0 + Z1][Component] * WX0 + Field[X1 + Y0 + Z1][Component] * WX1;
		const float C11 = Field[X0 + Y1 + Z1][Component] * WX0 + Field[X1 + Y1 + Z1][Component] * WX1;
		return (C00 * WY0 + C10 * WY1) * WZ0 + (C01 * WY0 + C11 * WY1) * WZ1;
	}

	inline Vector3 SampleMACVelocity(const Vector3* Field, const Vector3i& Resolution,
		const Vector3& GridPosition)
	{
		return Vector3(SampleMACComponent<0>(Field, Resolution, GridPosition),
			SampleMACComponent<1>(Field, Resolution, GridPosition),
			SampleMACComponent<2>(Field, Resolution, GridPosition));
	}
}

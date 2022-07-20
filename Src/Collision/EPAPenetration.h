#pragma once

#include "Simplex.h"

enum class EPA_result
{
	Valid,
	Touching,
	Degenerated,
	NonConvex,
	InvalidHull,
	OutOfFaces,
	OutOfVertices,
	AccuraryReached,
	FallBack,
	Failed
};

class EPAPenetration
{
public:
	EPA_result	result;
	Vector3d	penetration_normal;
	float		penetration_depth;

	EPAPenetration()
	{
		result = EPA_result::Failed;
		penetration_normal = Vector3d(0, 0, 0);
		penetration_depth = 0;
	}

	EPA_result Solve(Simplex& simplex);
};

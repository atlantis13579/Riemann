#pragma once

#include "Simplex.h"

enum class EPA_status
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
	EPA_status	status;
	Vector3		penetration_normal;
	float		penetration_depth;
	Simplex		result;

	EPAPenetration()
	{
		status = EPA_status::Failed;
		penetration_normal = Vector3(0, 0, 0);
		penetration_depth = 0;
	}

	EPA_status Solve(const Simplex& simplex);
};

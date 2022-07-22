
#pragma once

#include <functional>
#include "../Maths/Matrix3.h"

class PolarDecompose
{
public:
	static float OneNorm(const Matrix3d& F);
	static float InfNorm(const Matrix3d& F);
	static void Compute(const Matrix3d& F, Matrix3d& R, Matrix3d& S);
	static void ComputeFull(const Matrix3d& F, Matrix3d& R, Matrix3d& S);
};

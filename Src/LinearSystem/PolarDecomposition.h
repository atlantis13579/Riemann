
#pragma once

#include <functional>
#include "../Maths/Matrix3.h"

class PolarDecompose
{
public:
	static float OneNorm(const Matrix3& F);
	static float InfNorm(const Matrix3& F);
	static void Compute(const Matrix3& F, Matrix3& R, Matrix3& S);
	static void ComputeFull(const Matrix3& F, Matrix3& R, Matrix3& S);
};

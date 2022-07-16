
#pragma once

#include <vector>

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Jacobi_method

class JacobiIteration_CPU
{
public:
	bool Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps = 0.00001f);
};

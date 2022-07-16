
#pragma once

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method

class GaussSeidelIteration_CPU
{
public:
	static bool Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps = 0.00001f);
};
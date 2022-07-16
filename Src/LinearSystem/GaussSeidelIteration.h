
#pragma once

// Solve A * X = B , A is square Matrix of size N, B is Vector of size N
// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method

class GaussSeidelIteration_CPU
{
public:
	// Relaxation:
	// In numerical linear algebra, the method of successive over - relaxation(SOR)
	// is a variant of the Gauss¨CSeidel method for solving a linear system of equations,
	// resulting in faster convergence.
	// http://www.gamedev.ru/community/gd_physcomm/articles/?id=709
	static bool Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps = 0.00001f, const float Relaxation = 1.0f);
};

#pragma once

class ProjectedGaussSeidel_CPU
{
public:
	// Solve X, A * X + B >= 0, X >= 0 and dot(A * X + B, X) = 0, X, A is square Matrix of size N, B is Vector of size N
	static bool Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps = 0.00001f);

	// Solve X, A * X + B >= 0, X2 >= X >= X1 and dot(A * X + B, X) = 0, X, A is square Matrix of size N, B is Vector of size N
	static bool Solve(const float* A, const float* B, const float* X1, const float* X2, int N, float* X, const int MaxIteration, const float kEps = 0.00001f);
};

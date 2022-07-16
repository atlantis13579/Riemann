
#pragma once

// Projected Successive Over Relaxation (PSOR)
// Projected Gauss Seidel (PGS)

class ProjectedGaussSeidel_CPU
{
public:
	// A is square Matrix of size N, B is Vector of size N
	// Solve X, A * X + B >= 0, X >= 0 and dot(A * X + B, X) = 0, 
	static bool Solve(const float* A, const float* B, int N, float* X, const int MaxIteration, const float kEps = 0.00001f, const float Relaxation = 1.0f);

	// A is square Matrix of size N, B is Vector of size N
	// Solve X, Y, X2 >= X >= X1, Y == A * X + B and (X_i == X1_i and Y_i > 0) or (X_i == X2_i and Y_i < 0) or (X1_i < X_i < X2_i and Y_i == 0)
	static bool Solve(const float* A, const float* B, const float* X1, const float* X2, int N, float* X, const int MaxIteration, const float kEps = 0.00001f, const float Relaxation = 1.0f);
};

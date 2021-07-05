#pragma once

#include <vector>
#include "../Maths/Maths.h"
#include "../LinearSystem/SparseMatrix.h"

enum DOF
{
	X = 0, Y = 1, Z = 2
};

class ConjugateGradientSolver
{
public:
	ConjugateGradientSolver();
	~ConjugateGradientSolver();

	void InitSparseSolverCompressed3x3(const float *A, int n);
	void Solve(float *x, const float* b);
	void SolveSparseCompressed3x3(float *x, const float *b);

	float Dot(const float *a, const float *b)
	{
		float dp = 0;
		for(int i = 0; i < m_nSize; ++i)
			dp += a[i] * b[i];
		return dp;
	}

private:
	float *r, *x, *d, *q, *tempo;
	float *flatA;
	float *m_A;
	int m_nSize;
	SparseMatrix* m_SA;
};


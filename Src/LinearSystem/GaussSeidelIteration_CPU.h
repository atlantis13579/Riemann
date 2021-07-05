
#pragma once

// Solve A * X = B , A, B is square Matrix of size N
// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
class GaussSeidelIteration_CPU
{
public:
	static void Solve(const float* A, const float* B, int N, int MaxIteration, float* X)
	{
        int Iter = 0;
        while (Iter++ < MaxIteration)
        {
            for (int i = 0; i < N; ++i)
            {
                float s = 0.0f;
                for (int j = 0; j < i; ++j)
                    s += A[i * N + j] * X[j];

                for (int j = i + 1; j < N; ++j)
                    s += A[i * N + j] * X[j];

                X[i] = (B[i] - s) / A[i * N + i];
            }

            float Norm = 0.f;
            for (int i = 0; i < N; i++)
                Norm += X[i] * X[i];

            bool converge = Norm < 0.00001f;
            if (converge)
            {
                break;
            }
        }
    }
};
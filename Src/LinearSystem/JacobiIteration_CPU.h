
#pragma once

#include <vector>

// Solve A * X = B , A, B is square Matrix of size N
// https://en.wikipedia.org/wiki/Jacobi_method
class JacobiIteration_CPU
{
public:
	JacobiIteration_CPU() {}

	void Solve(const float* A, const float* B, int N, int MaxIteration, float* X)
	{
        if (m_buf.size() < N)
        {
            m_buf.resize(N);
        }
        float* DX = &m_buf[0];

        int Iter = 0;
        while (Iter++ < MaxIteration)
        {
            for (int i = 0; i < N; ++i)
            {
                float s = 0.f;
                for (int j = 0; j < i; ++j)
                    s += A[i * N + j] * X[j];

                for (int j = i + 1; j < N; ++j)
                    s += A[i * N + j] * X[j];

                DX[i] = (B[i] - s) / A[i * N + i];
            }

            memcpy(X, DX, sizeof(float) * N);

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

private:
	
	std::vector<float> m_buf;
};
#pragma once

// Linear¨Cquadratic regulator
// https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator

#include <cmath>
#include <vector>
#include "../../Maths/MatrixMxN.h"

template<int nState, int mInput>
class LinearQuadraticRegulator
{
public:
    using VectorMx1 = VectorN<mInput>;
    using MatrixNxN = MatrixN<nState>;
    using MatrixMxM = MatrixN<mInput>;
    using InputMatrix = MatrixMxN<nState, mInput>;
    using GainMatrix = MatrixMxN<mInput, nState>;
    using State = VectorN<nState>;

public:
    LinearQuadraticRegulator(MatrixNxN _A, InputMatrix _B, MatrixNxN _Q, MatrixMxM _R)
        : A(_A), B(_B), Q(_Q), R(_R)
    {
        mTolerance = 0.1f;
        K = ComputeGain();
    }

    bool Solve(float dt, float max_time, State initial, State target, std::vector<State> *path)
    {
        if (path == nullptr)
        {
            return false;
        }
        path->clear();
        path->push_back(initial);

        State e = initial - target;

		float time = 0.0f;
        while (time < max_time)
        {
            time += dt;

            VectorMx1 u = Input(e);
            e = A * e + B * u;

            path->push_back(e + target);

            if (e.SquaredLength() <= mTolerance * mTolerance)
            {
                return true;
            }
        }

        path->clear();
        return false;
    }

    void SetTolerance(double tol)
    {
        mTolerance = tol;
    }

private:
    VectorMx1 Input(State x) const
    {
        return -K * x;
    }

    GainMatrix ComputeGain()
    {
        MatrixNxN X = SolveDARE(100, 0.01f);
        return (B.Transpose() * X * B + R).Transpose() * (B.Transpose() * X * A);
    }

    // Solves discrete-time algebraic Riccati equation
    MatrixNxN SolveDARE(int max_iter, float tor) const
    {
        MatrixNxN X = Q, Xn = Q;
        MatrixNxN AT = A.Transpose();
        GainMatrix BT = B.Transpose();
        for (int i = 0; i < max_iter; ++i)
        {
			Xn = AT * X * A - AT * X * B * (R + BT * X * B).Inverse() * BT * X * A + Q;
            if ((Xn - X).LInfinityNorm() < tor)
                break;
            X = Xn;
        }
        return Xn;
    }

    MatrixNxN A;
    InputMatrix B;
    MatrixNxN Q;
    MatrixMxM R;
    GainMatrix K;

    float mTolerance;
};
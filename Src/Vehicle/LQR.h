#pragma once

// Linear¨Cquadratic regulator
// https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator

#include <cmath>
#include "../Maths/MatrixMxN.h"

template<int StateSize, int InputSize>
class LinearQuadraticRegulator
{
public:
    using VectorMx1 = VectorNd<InputSize>;
    using MatrixNxN = SquareMatrix<StateSize>;
    using MatrixMxM = SquareMatrix<InputSize>;
    using InputMatrix = MatrixMxN<StateSize, InputSize>;
    using GainMatrix = MatrixMxN<InputSize, StateSize>;

    using State = VectorNd<StateSize>;

public:
    LinearQuadraticRegulator(MatrixNxN _A, InputMatrix _B, MatrixNxN _Q, MatrixMxM _R)
        : A(_A), B(_B), Q(_Q), R(_R)
    {
        K = ComputeGain();
    }

    std::vector<State> Solve(State initial, State target, float dt)
    {
        std::vector<State> path{ initial };

        State x = initial - target;
        VectorMx1 u;

        bool path_found = false;
        float time = 0;
        float goal_dist_squared = std::powf(goal_dist, 2);

        while (time < max_time) {
            time += dt;

            u = Input(x);
            x = A * x + B * u;

            path.push_back(x + target);

            if (x.SquaredNorm() <= goal_dist_squared) {
                path_found = true;
                break;
            }
        }

        if (!path_found) {
            return {};
        }

        return path;
    }

    void SetTimeLimit(double limit) { max_time = limit; }

    void SetFinalPositionTolerance(double tol) { goal_dist = tol; }

private:
    VectorMx1 Input(State x) const { return -K * x; }

    GainMatrix ComputeGain()
    {
        MatrixNxN X = SolveDARE(100, 0.01f);
        return (B.Transpose() * X * B + R).Transpose() * (B.Transpose() * X * A);
    }

    // Solves discrete-time algebraic Riccati equation
    MatrixNxN SolveDARE(int max_iter, float tor) const
    {
        MatrixNxN X = Q, Xn = Q;
        auto AT = A.Transpose();
        auto BT = B.Transpose();
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

    float max_time{ 100.0f };
    float goal_dist{ 0.1f };
};
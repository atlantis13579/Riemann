#pragma once

// Linear¨Cquadratic regulator
// https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator

#include <cmath>
#include "../Maths/MatrixMxN.h"

template<int StateSize, int InputSize>
class LinearQuadraticRegulator
{
public:
    using VectorNx1 = VectorNd<StateSize>;
    using VectorMx1 = VectorNd<InputSize>;

    using MatrixNxN = SquareMatrix<StateSize>;
    using MatrixMxM = SquareMatrix<InputSize>;
    using InputMatrix = MatrixMxN<StateSize, InputSize>;
    using GainMatrix = MatrixMxN<InputSize, StateSize>;

    using State = VectorNx1;

public:
    LinearQuadraticRegulator(MatrixNxN _A, InputMatrix _B, MatrixNxN _Q, MatrixMxM _R)
        : A(_A), B(_B), Q(_Q), R(_R), K(ComputeGain())
    {
    }

    std::vector<State> Solve(State initial, State target, double dt)
    {
        std::vector<State> path{ initial };
        path.reserve((unsigned int)std::round(max_time / dt) + 1);

        State x = initial - target;
        VectorMx1 u;

        bool path_found = false;
        double time = 0;
        double goal_dist_squared = std::pow(goal_dist, 2);

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

    void SetIterationsLimit(int limit) { max_iter = limit; }

    void SetDARETolerance(double tol) { eps = tol; }

private:
    VectorMx1 Input(State x) const { return -K * x; }

    GainMatrix ComputeGain()
    {
        MatrixNxN X = SolveDARE();
        return (B.Transpose() * X * B + R).Transpose() * (B.Transpose() * X * A);
    }

    MatrixNxN SolveDARE() const
    {
        MatrixNxN X = Q, Xn = Q;
        for (auto _ = 0; _ < max_iter; _++) {
			Xn = A.Transpose() * X * A
				- A.Transpose() * X * B * (R + B.Transpose() * X * B).Inverse() * B.Transpose()
				* X * A
				+ Q;
            if ((Xn - X).InfinityNorm() < eps) break;
            X = Xn;
        }
        return Xn;
    }

    MatrixNxN A;
    InputMatrix B;
    MatrixNxN Q;
    MatrixMxM R;
    GainMatrix K;

    double max_time{ 100.0 };
    double goal_dist{ 0.1 };
    int max_iter{ 100 };
    double eps{ 0.01 };
};
#pragma once

// Linear¨Cquadratic regulator
// https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator

#include "../Maths/VectorNd.h"

/*
class LinearQuadraticRegulator
{
public:
    static_assert(StateSize > 0);
    static_assert(InputSize > 0);

    using VectorNx1 = VectorNd<StateSize>;
    using VectorMx1 = VectorNd<InputSize>;

    using MatrixNxN = SquareMatrix<StateSize>;
    using MatrixMxM = SquareMatrix<InputSize>;
    using InputMatrix = Matrix<StateSize, InputSize>;
    using GainMatrix = Matrix<InputSize, StateSize>;

    using State = VectorNx1;

public:
    LQR(MatrixNxN A, InputMatrix B, MatrixNxN Q, MatrixMxM R)
        : A(A), B(B), Q(Q), R(R), K(ComputeGain())
    {
    }

    std::vector<State> Solve(State initial, State target, double dt)
    {
        std::vector<State> path{ initial };
        path.reserve((unsigned int)std::round(max_time / dt)
            + 1);  // TODO: currently assuming the worst case

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

            if (x.squaredNorm() <= goal_dist_squared) {
                path_found = true;
                break;
            }
        }

        if (!path_found) {
            std::cerr << "Couldn't find a path\n";
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
        return (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
    }

    MatrixNxN SolveDARE() const
    {
        MatrixNxN X = Q, Xn = Q;
        for (auto _ = 0; _ < max_iter; _++) {
            Xn = A.transpose() * X * A
                - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose()
                * X * A
                + Q;
            if ((Xn - X).template lpNorm<Eigen::Infinity>() < eps) break;
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
    int max_iter{ 10 };
    double eps{ 0.01 };
};
*/

#include "Test.h"

#include "../Src/Modules/VehicleDynamics/PID.h"
#include "../Src/Modules/VehicleDynamics/LQR.h"

// using namespace Riemann;

void TestPID()
{
	printf("Running TestPID\n");
	PID_Controller pid(0.5f, 0.08f, 0.1f, -10.0f, 10.0f, -1.0f, 1.0f);
	float c = 0.0f, t = 1.0f;
	for (int i = 0; i < 100; ++i)
	{
		float d = pid.Compute(0.033f, c, t);
		c = c + d;
		// printf("%d : c = %.2f, d = %.2f\n", i, c, fabsf(d));
	}
	return;
}

void TestLqr()
{
	static constexpr int N = 2;
	static constexpr int M = 1;

	const float dt = 0.1f;

	// State matrix
	Maths::MatrixN<2> A = { dt, 1.0f, 0.0f, dt };
	Maths::MatrixMxN<2, 1> B = { 0.0f, 1.0f };

	Maths::MatrixN<N> Q = Maths::MatrixN<N>::Identity();
	Maths::MatrixN<M> R = Maths::MatrixN<M>::Identity();

	LinearQuadraticRegulator<N, M> planner(A, B, Q, R);
	LinearQuadraticRegulator<N, M>::State i = {0.0f, 0.0f};
	LinearQuadraticRegulator<N, M>::State t = {1.0f, 1.5f};

	std::vector<LinearQuadraticRegulator<N, M>::State> path;
	bool b = planner.Solve(0.033f, 1.0f, i, t, &path);
	EXPECT(b);

	return;
}

void TestVehicle()
{
	TestPID();
	TestLqr();
	return;
}

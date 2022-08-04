
#include "Test.h"
#include "../Src/Optimization/LBFGS.h"
#include "../Src/Optimization/PCA.h"
#include "../Src/Optimization/StochasticGradientDescent.h"
#include "../Src/Optimization/LinearRegression.h"
#include "../Src/Optimization/LevenbergMarquardt.h"
#include "../Src/Optimization/NonConvexFunctions.h"

class Rosenbrock2DEvalFunction : public LBFGSEvalFunction<double>
{
public:
	Rosenbrock2DEvalFunction() {}
	~Rosenbrock2DEvalFunction() {}
	virtual void Evaluate(const double* X, int Dim, double* Y, double* Gradient) const override final
	{
		Rosenbrock2D(1.0, 100.0, X, Dim, Y, Gradient);
	}
};

void TestLBFGS()
{
	Rosenbrock2DEvalFunction func;

	LBFGSMinimizer<double> minimizer;
	double miny = minimizer.Minimize(&func, 100);
	EXPECT(abs(miny) < 0.0001f);
	return;
}

void TestNonConvexFunctions()
{
	printf("Running TestNonConvexFunctions\n");
	auto func = Rosenbrock<double>;
	auto a = func(1.0, 2.0, 1.0, 1.0);
	EXPECT(a >= 0);
	return;
}

void TestOptimization()
{
	TestNonConvexFunctions();
	TestLBFGS();
	return;
}

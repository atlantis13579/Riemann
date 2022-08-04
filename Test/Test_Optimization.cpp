
#include "Test.h"
#include "../Src/LinearSystem/DenseVector.h"
#include "../Src/Optimization/LBFGS.h"
#include "../Src/Optimization/PCA.h"
#include "../Src/Optimization/StochasticGradientDescent.h"
#include "../Src/Optimization/LinearRegression.h"
#include "../Src/Optimization/LevenbergMarquardt.h"
#include "../Src/Optimization/NonConvexFunctions.h"

class Rosenbrock2DEvalFunction : public LBFGSEvalFunction<double>
{
public:
	virtual ~Rosenbrock2DEvalFunction() {}
	virtual void Evaluate(const double* X, int Dim, double* Y, double* Gradient) const override final
	{
		Rosenbrock2D(1.0, 100.0, X, Dim, Y, Gradient);
	}
};

class TestEvalFunction : public LBFGSEvalFunction<double>
{
public:
	TestEvalFunction(double c) : c_(c) { }
	virtual ~TestEvalFunction() {}
	virtual void Evaluate(const double* X, int Dim, double* Y, double* Gradient) const override final
	{
		double sum = 0.0f;
		for (int i = 0; i < Dim; ++i)
		{
			sum += (X[i] - c_) * (X[i] - c_);
			Gradient[i] = 2.0 * (X[i] - c_);
		}
		*Y = sum;
	}
private:
	double c_;
};

void TestLBFGS()
{
	Rosenbrock2DEvalFunction func1;

	LBFGSMinimizer<double> minimizer;
	double miny = minimizer.Minimize(&func1, 100);
	EXPECT(abs(miny) < 0.0001);

	TestEvalFunction func2(10.0);
	TDenseVector<double> x(2);
	x.LoadZero();
	miny = minimizer.Minimize(&func2, 2, x.GetData());
	EXPECT(abs(miny) < 0.0001);
	EXPECT(abs(x[0] - 10.0) < 0.0001);
	EXPECT(abs(x[1] - 10.0) < 0.0001);
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

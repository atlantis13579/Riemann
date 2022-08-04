
#include "Test.h"
#include "../Src/LinearSystem/DenseVector.h"
#include "../Src/LinearSystem/DenseMatrix.h"
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
	virtual void Evaluate(const double* X, int Dim, double* F, double* Gradient) const override final
	{
		Rosenbrock2D(1.0, 100.0, X, Dim, F, Gradient);
	}
};

template<typename T>
class TestEvalFunction : public LBFGSEvalFunction<T>
{
public:
	TestEvalFunction(T c) : c_(c) { }
	virtual ~TestEvalFunction() {}
	virtual void Evaluate(const T* X, int Dim, T* F, T* Gradient) const override final
	{
		*F = _Eval(X, Dim);
		T* pX = (T*)X;
		NUMERIAL_GRADIENT(_Eval, pX, Dim, 0.01, Gradient);
	}
	T _Eval(const T* X, int Dim) const
	{
		T sum = (T)0;
		for (int i = 0; i < Dim; ++i)
		{
			sum += (X[i] - c_) * (X[i] - c_);
		}
		return sum;
	}
private:
	T c_;
};

void TestLBFGS()
{
	printf("Running TestLBFGS\n");

	Rosenbrock2DEvalFunction func1;

	LBFGSMinimizer<double> minimizer;
	double miny = minimizer.Minimize(&func1, 100);
	EXPECT(fabs(miny) < 0.0001);

	TestEvalFunction<double> func2(10.0);
	double x[2] = { 0 };
	miny = minimizer.Minimize(&func2, x, 2);
	EXPECT(fabs(miny) < 0.0001);
	EXPECT(fabs(x[0] - 10.0) < 0.0001);
	EXPECT(fabs(x[1] - 10.0) < 0.0001);
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

void TestLR()
{
	printf("Running TestLR\n");

	const int N = 100;
	float a = 1.0f, b = 0.0f;
	TDenseMatrix<float> x(N, 1);
	TDenseVector<float> y(N);
	for (int i = 0; i < N; ++i)
	{
		float xx = 1.0f * rand() / RAND_MAX;
		x[i][0] = xx;
		y[i] = a * xx + b + 0.01f * rand() / RAND_MAX;
	}

	LRParam param;
	param.algorithm = LR_algorithm::GradientDescent;
	LRModel<float> lr(1);
	lr.Fit(x.GetData(), y.GetData(), N, param);
	printf("a = %.2f, b = %.2f\n", lr.coef[0], lr.coef[1]);
	return;
}

void TestOptimization()
{
	TestNonConvexFunctions();
	TestLBFGS();
	TestLR();
	return;
}

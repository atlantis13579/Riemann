
#include "Test.h"
#include "../Src/LinearSystem/DenseVector.h"
#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/Optimization/LBFGS.h"
#include "../Src/Optimization/PCA.h"
#include "../Src/Optimization/GradientDescent.h"
#include "../Src/Optimization/StochasticGradientDescent.h"
#include "../Src/Optimization/LinearRegression.h"
#include "../Src/Optimization/PolynomialFit.h"
#include "../Src/Optimization/LevenbergMarquardt.h"
#include "../Src/Optimization/SpecialFunctions.h"

class Rosenbrock2DEvalFunction : public LBFGSEvalFunction<double>
{
public:
	virtual ~Rosenbrock2DEvalFunction() {}
	virtual double Evaluate(const double* X, int Dim, double* Gradient) const override final
	{
		return Rosenbrock2D(1.0, 100.0, X, Dim, Gradient);
	}
};

template<typename T>
class TestEvalFunction : public LBFGSEvalFunction<T>
{
public:
	TestEvalFunction(T c) : c_(c) { }
	virtual ~TestEvalFunction() {}
	virtual T Evaluate(const T* X, int Dim, T* Gradient) const override final
	{
		T* pX = (T*)X;
		NUMERIAL_GRADIENT(_Eval, pX, Dim, 0.01, Gradient);
		return _Eval(X, Dim);
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
	float a = 1.0f, b = 2.0f, c = 0.0f;
	TDenseMatrix<float> x(N, 2);
	TDenseVector<float> y(N);
	for (int i = 0; i < N; ++i)
	{
		float x0 = 1.0f * rand() / RAND_MAX;
		float x1 = 1.0f * rand() / RAND_MAX;
		x[i][0] = x0;
		x[i][1] = x1;
		y[i] = a * x0 + b * x1 + c + 0.01f * rand() / RAND_MAX;
	}

	LRParam param;
	param.solver = LR_solver::NormalEquation;
	LRModel<float> lr(2);
	lr.Fit(x.GetData(), y.GetData(), N, param);
	printf("lr: a = %.2f, b = %.2f, c = %.2f\n", lr.coef[0], lr.coef[1], lr.intercept);
	return;
}

class TestSGDLR : public SGDModel<double>
{
public:
	virtual ~TestSGDLR() {}
	virtual double Evaluate(const double* X, double* G) const override final
	{
		G[0] = X[0];
		G[1] = 1.0;
		return a * X[0] + b;
	}
	virtual void ApplyGradient(double* G) override final
	{
		a += G[0];
		b += G[1];
	}

public:
	double a = 0;
	double b = 0;
};

void TestSGD()
{
	printf("Running TestSGD\n");
	
	const int N = 100000;
	float a = 1.0f, b = 0.0f;
	TDenseVector<double> x(N);
	TDenseVector<double> y(N);
	for (int i = 0; i < N; ++i)
	{
		double xx = 1.0 * rand() / RAND_MAX;
		x[i] = xx;
		y[i] = a * xx + b + 0.01 * rand() / RAND_MAX;
	}

	TestSGDLR lr;
	SGDParam param;
	SGDLeastSquaresOptimizer<double> solver;
	solver.Optimize(&lr, 2, x.GetData(), 1, y.GetData(), N, param);
	printf("sgd: a = %.2f, b = %.2f\n", lr.a, lr.b);
	return;
}

class GDTest : public GradientDescentEvalFunction<float>
{
public:
	virtual ~GDTest() {}
	virtual float Evaluate(const float* X, float* Gradient) const override final
	{
		float a = X[0];
		float b = X[1];
		Gradient[0] = 2 * (a - 1.0f);
		Gradient[1] = 2 * (b - 2.0f);
		return (a - 1.0f) * (a - 1.0f) + (b - 2.0f) * (b - 2.0f);
	}
};

void TestGD()
{
	printf("Running TestGD\n");

	GDTest sqr;
	GDParam param;
	GradientDescentMinimizer<float> minimizer;
	float x[2] = { 0, 0 };
	float miny = minimizer.Minimize(&sqr, x, 2, param);
	EXPECT(fabs(miny) < 0.001);
	EXPECT(fabs(x[0] - 1.0f) < 0.01);
	EXPECT(fabs(x[1] - 2.0f) < 0.01);
	return;
}

void TestPolyfit()
{
	printf("Running TestPolyfit\n");
	const int N = 100;
	DenseVector x(N);
	DenseVector y(N);
	float a = 2.0f, b = 1.0f, c = 1.5f, d = 0.5f;
	for (int i = 0; i < N; ++i)
	{
		float xx = 1.0f * rand() / RAND_MAX;
		x[i] = xx;
		y[i] = a * xx * xx * xx + b * xx * xx + c * xx + d + 0.01f * rand() / RAND_MAX;
	}
	
	CubicFit<float> p3;
	p3.Fit(x.GetData(), y.GetData(), N);
	
	printf("polyfit: a = %.2f, b = %.2f, c = %.2f, c = %.3f\n", p3.coef[3], p3.coef[2], p3.coef[1], p3.coef[0]);
	
	return;
}

void TestApproximation()
{
	printf("Running TestApproximation\n");
	
	PolynomialApproximation<float, 2> p2;
	p2.Approximate(expf, 0.0f, 1.0f);
	
	float f0 = p2.Eval(0.0f);
	float f1 = p2.Eval(0.5f);
	float f2 = p2.Eval(1.0f);
	
	float e0 = expf(0.0f);
	float e1 = expf(0.5f);
	float e2 = expf(1.0f);
	
	printf("app : diff(0) = %.2f, diff(0.5) = %.2f, diff(1) = %.2f\n", fabsf(f0 - e0), fabsf(f1 - e1), fabsf(f2 - e2));
	
	return;
}

void TestOptimization()
{
	TestNonConvexFunctions();
	TestLBFGS();
	TestLR();
	TestSGD();
	TestGD();
	TestPolyfit();
	TestApproximation();
	return;
}
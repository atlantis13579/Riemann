
#include "Test.h"
#include "../Src/Optimization/LBFGS.h"
#include "../Src/Optimization/PCA.h"
#include "../Src/Optimization/StochasticGradientDescent.h"
#include "../Src/Optimization/LinearRegression.h"
#include "../Src/Optimization/LevenbergMarquardt.h"
#include "../Src/Optimization/NonConvexFunctions.h"

template<typename FloatType>
class LBFGSSolver
{
public:
	~LBFGSSolver() {}
};

template<typename FloatType>
class LBFGSEvalFunction
{
public:
	virtual ~LBFGSEvalFunction() {}
	virtual FloatType Evaluate(const FloatType *X, FloatType *Gradient) const = 0;
};

class Rosenbrock2D : public LBFGSEvalFunction<double>
{
public:
	virtual ~Rosenbrock2D() {}
	virtual double Evaluate(const double *X, double *Gradient) const override final
	{
		
		return 0.0;
	}
};

static lbfgsfloatval_t evaluate(
	void *instance,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int n,
	const lbfgsfloatval_t step
	)
{
	int i;
	lbfgsfloatval_t fx = 0.0;
	for (i = 0; i < n; i += 2) {
		lbfgsfloatval_t t1 = 1.0 - x[i];
		lbfgsfloatval_t t2 = 10.0 * (x[i+1] - x[i] * x[i]);
		g[i+1] = 20.0 * t2;
		g[i] = -2.0 * (x[i] * g[i+1] + t1);
		fx += t1 * t1 + t2 * t2;
	}
	return fx;
}

static int progress(
	void *instance,
	const lbfgsfloatval_t *x,
	const lbfgsfloatval_t *g,
	const lbfgsfloatval_t fx,
	const lbfgsfloatval_t xnorm,
	const lbfgsfloatval_t gnorm,
	const lbfgsfloatval_t step,
	int n,
	int k,
	int ls
	)
{
	printf("Iteration %d:\n", k);
	printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
	printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
	printf("\n");
	return 0;
}
#define N   100

void TestLBFGS()
{
	int i, ret = 0;
	lbfgsfloatval_t fx;
	lbfgsfloatval_t *x = lbfgs_malloc(N);
	lbfgs_parameter_t param;
	if (x == NULL) {
		printf("ERROR: Failed to allocate a memory block for variables.\n");
		return;
	}
	/* Initialize the variables. */
	for (i = 0;i < N;i += 2) {
			x[i] = -1.2;
			x[i+1] = 1.0;
		}
	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	/*param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;*/
	/*
		Start the L-BFGS optimization; this will invoke the callback functions
		evaluate() and progress() when necessary.
	*/
	ret = lbfgs(N, x, &fx, evaluate, progress, NULL, &param);
	/* Report the result. */
	printf("L-BFGS optimization terminated with status code = %d\n", ret);
	printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
	lbfgs_free(x);
	return;
}

void TestNonConvexFunctions()
{
	printf("Running TestNonConvexFunctions\n");
	auto func = Rosenbrock<double>;
	auto a = func(1.0, 2.0, 1.0, 1.0);
	EXPECT(a > 0);
	return;
}

void TestOptimization()
{
	TestNonConvexFunctions();
	TestLBFGS();
	return;
}

#pragma once

// https://en.wikipedia.org/wiki/Rosenbrock_function

template<typename lbfgsfloatval_t>
lbfgsfloatval_t Rosenbrock(const lbfgsfloatval_t a, const lbfgsfloatval_t b, const lbfgsfloatval_t x1, const lbfgsfloatval_t x2)
{
	return (a - x1) * (a - x1) + b * (x2 - x1 * x1) * (x2 - x1 * x1);
}

template<typename lbfgsfloatval_t>
lbfgsfloatval_t RosenbrockN(const lbfgsfloatval_t a, const lbfgsfloatval_t b, const lbfgsfloatval_t *x, int dim)
{
	lbfgsfloatval_t sum = (lbfgsfloatval_t)0;
	for (int i = 1; i < dim; ++i)
	{
		lbfgsfloatval_t t1 = a - x[i-1];
		lbfgsfloatval_t t2 = x[i] - x[i-1] * x[i-1];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename lbfgsfloatval_t>
lbfgsfloatval_t Rosenbrock2D(const lbfgsfloatval_t a, const lbfgsfloatval_t b, const lbfgsfloatval_t *x, int dim)
{
	lbfgsfloatval_t sum = (lbfgsfloatval_t)0;
	for (int i = 0; i < dim; i += 2)
	{
		lbfgsfloatval_t t1 = a - x[i];
		lbfgsfloatval_t t2 = x[i+1] - x[i] * x[i];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename lbfgsfloatval_t>
void Rosenbrock2D(const lbfgsfloatval_t a, const lbfgsfloatval_t b, const lbfgsfloatval_t* x, int dim, lbfgsfloatval_t* y, lbfgsfloatval_t* gradient)
{
	lbfgsfloatval_t sum = (lbfgsfloatval_t)0;
	for (int i = 0; i < dim; i += 2)
	{
		lbfgsfloatval_t t1 = a - x[i];
		lbfgsfloatval_t t2 = x[i + 1] - x[i] * x[i];
		gradient[i] = -(lbfgsfloatval_t)2 * t1 - (lbfgsfloatval_t)4 * b * t2 * x[i];
		gradient[i+1] = (lbfgsfloatval_t)2 * b * t2;
		sum += t1 * t1 + b * t2 * t2;
	}
	*y = sum;
}
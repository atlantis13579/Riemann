#pragma once

// https://en.wikipedia.org/wiki/Rosenbrock_function

template<typename FloatType>
FloatType Rosenbrock(const FloatType a, const FloatType b, const FloatType x1, const FloatType x2)
{
	return (a - x1) * (a - x1) + b * (x2 - x1 * x1) * (x2 - x1 * x1);
}

template<typename FloatType>
FloatType RosenbrockN(const FloatType a, const FloatType b, const FloatType *x, int dim)
{
	FloatType sum = (FloatType)0;
	for (int i = 1; i < dim; ++i)
	{
		FloatType t1 = a - x[i-1];
		FloatType t2 = x[i] - x[i-1] * x[i-1];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename FloatType>
FloatType Rosenbrock2D(const FloatType a, const FloatType b, const FloatType *x, int dim)
{
	FloatType sum = (FloatType)0;
	for (int i = 0; i < dim; i += 2)
	{
		FloatType t1 = a - x[i];
		FloatType t2 = x[i+1] - x[i] * x[i];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename FloatType>
void Rosenbrock2D(const FloatType a, const FloatType b, const FloatType* x, int dim, FloatType* y, FloatType* gradient)
{
	FloatType sum = (FloatType)0;
	for (int i = 0; i < dim; i += 2)
	{
		FloatType t1 = a - x[i];
		FloatType t2 = x[i + 1] - x[i] * x[i];
		gradient[i] = -(FloatType)2 * t1 - (FloatType)4 * b * t2 * x[i];
		gradient[i+1] = (FloatType)2 * b * t2;
		sum += t1 * t1 + b * t2 * t2;
	}
	*y = sum;
}
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
FloatType Rosenbrock2D(const FloatType a, const FloatType b, const FloatType* x, int dim, FloatType* gradient)
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
	return sum;
}

#define NUMERIAL_GRADIENT(_eval, _x, _dim, _dx, _gradient)	\
		for (int i = 0; i < _dim; ++i)	\
		{ \
			auto tx = _x[i]; \
			_x[i] = tx - _dx; \
			auto f1 = _eval(_x, Dim); \
			_x[i] = tx + _dx; \
			auto f2 = _eval(_x, Dim); \
			_x[i] = tx; \
			_gradient[i] = (f2 - f1) / (_dx * 2); \
		}

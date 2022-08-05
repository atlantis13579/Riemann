#pragma once

// https://en.wikipedia.org/wiki/Rosenbrock_function

template<typename T>
T Rosenbrock(const T a, const T b, const T x1, const T x2)
{
	return (a - x1) * (a - x1) + b * (x2 - x1 * x1) * (x2 - x1 * x1);
}

template<typename T>
T RosenbrockN(const T a, const T b, const T *x, int dim)
{
	T sum = (T)0;
	for (int i = 1; i < dim; ++i)
	{
		T t1 = a - x[i-1];
		T t2 = x[i] - x[i-1] * x[i-1];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename T>
T Rosenbrock2D(const T a, const T b, const T *x, int dim)
{
	T sum = (T)0;
	for (int i = 0; i < dim; i += 2)
	{
		T t1 = a - x[i];
		T t2 = x[i+1] - x[i] * x[i];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename T>
T Rosenbrock2D(const T a, const T b, const T* x, int dim, T* gradient)
{
	T sum = (T)0;
	for (int i = 0; i < dim; i += 2)
	{
		T t1 = a - x[i];
		T t2 = x[i + 1] - x[i] * x[i];
		gradient[i] = -(T)2 * t1 - (T)4 * b * t2 * x[i];
		gradient[i+1] = (T)2 * b * t2;
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

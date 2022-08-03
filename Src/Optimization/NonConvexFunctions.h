#pragma once

template<typename FloatType>
FloatType Rosenbrock(const FloatType a, const FloatType b, FloatType x1, FloatType x2)
{
	return (a - x1) * (a - x1) + b * (x2 - x1 * x1) * (x2 - x1 * x1);
}

template<typename FloatType>
FloatType RosenbrockN(const FloatType a, const FloatType b, FloatType *x, int N)
{
	FloatType sum = (FloatType)0;
	for (int i = 1; i < N; ++i)
	{
		FloatType t1 = a - x[i-1];
		FloatType t2 = x[i] - x[i-1];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

template<typename FloatType>
FloatType Rosenbrock2D(const FloatType a, const FloatType b, FloatType *x, int N)
{
	FloatType sum = (FloatType)0;
	for (int i = 0; i < N; i += 2)
	{
		FloatType t1 = a - x[i];
		FloatType t2 = x[i+1] - x[i];
		sum += t1 * t1 + b * t2 * t2;
	}
	return sum;
}

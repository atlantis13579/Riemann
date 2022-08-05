#pragma once

template<typename FloatType>
FloatType Square(FloatType* X, int Dim)
{
	FloatType sum = (FloatType)0;
	for (int i = 0; i < Dim; ++i)
	{
		sum += X[i] * X[i];
	}
	return sum;
}

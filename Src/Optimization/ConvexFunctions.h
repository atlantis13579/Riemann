#pragma once

template<typename T>
T Square(T* X, int Dim)
{
	T sum = (T)0;
	for (int i = 0; i < Dim; ++i)
	{
		sum += X[i] * X[i];
	}
	return sum;
}

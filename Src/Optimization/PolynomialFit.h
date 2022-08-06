#pragma once

template<typename T, int Dim>
class PolynomialFit
{
public:
	bool Fit(const T* X, const T* Y, int N)
	{
		T A[2*(Dim + 1)*(Dim + 1)] = {0};
		T B[Dim+1] = {0};
		T Pow[2*Dim+3] = {0};
		const T eps = (T)0.0000001;

		if (N <= Dim)
			return false;

		for (int i = 0; i < N; ++i)
		{
			T x = X[i];
			T y = Y[i];
			T powx = (T)1;

			for (int j = 0; j <= Dim; ++j)
			{
				B[j] = B[j] + (y * powx);
				powx  = powx * x;
			}
		}

		Pow[0] = (T)N;

		for (int i = 0; i < N; ++i)
		{
			T x = X[i];
			T powx = X[i];

			for (int j = 1; j <= 2 * (Dim + 1); ++j)
			{
				Pow[j] += powx;
				powx  = powx * x;
			}
		}

		for (int i = 0; i <= Dim; ++i)
		{
			for (int j = 0; j <= Dim; ++j)
			{
				A[(i * (2 * (Dim + 1))) + j] = Pow[i+j];
			}

			A[(i*(2 * (Dim + 1))) + (i + (Dim + 1))] = (T)1;
		}

		for (int i = 0; i <= Dim; ++i)
		{
			T x = A[(i * (2 * (Dim + 1))) + i];
			T absx = x >= 0 ? x : -x;
			if (absx <= eps)
			{
				return false;
			}

			for (int k = 0; k < 2 * (Dim + 1); ++k)
			{
				A[(i * (2 * (Dim + 1))) + k] = A[(i * (2 * (Dim + 1))) + k] / x;
			}

			for (int j = 0; j <= Dim; ++j)
			{
				if (j == i)
					continue;
				
				T y = A[(j * (2 * (Dim + 1))) + i];
				for (int k = 0; k < 2 * (Dim + 1); ++k)
				{
					A[(j * (2 * (Dim + 1))) + k] = A[(j * (2 * (Dim + 1))) + k] - y * A[(i * (2 * (Dim + 1))) + k];
				}
			}
		}

		for (int i = 0; i <= Dim; ++i)
		{
			for (int j = 0; j <= Dim; ++j)
			{
				T sum = (T)0;
				for (int k = 0; k <= Dim; ++k)
				{
					sum += (A[(i * (2 * (Dim + 1))) + (k + (Dim + 1))] * B[k]);
				}
				coef[i] = sum;
			}
		}

		return true;
	}
	
	T Eval(const T x) const
	{
		T sum = (T)0;
		T powx = (T)1;
		for (int i = 0; i <= Dim; ++i)
		{
			sum += coef[i] * powx;
			powx *= x;
		}
		return sum;
	}
	
public:
	T coef[Dim+1] = {0};	// f(x) = c0 + c1 * x + c2 * x^2 + ... + cN * c^N
};

template<typename T>
using LinearFit = PolynomialFit<T, 1>;

template<typename T>
using QuadraticFit = PolynomialFit<T, 2>;

template<typename T>
using CubicFit = PolynomialFit<T, 3>;

template<typename T>
using Quartic = PolynomialFit<T, 4>;

template<typename T>
using QuinticFit = PolynomialFit<T, 5>;

template<typename T>
using SrcFunction = T (*)(T);

template<typename T, int Dim>
class PolynomialApproximation {
public:
	bool Approximate(const SrcFunction<T> &F, const T interval_min, const T interval_max, int samples = 128)
	{
		if (samples <= Dim)
			return false;
		
		T *buffer = new T[2*samples];
		T *X = buffer;
		T *Y = buffer + samples;
		T di = (interval_max - interval_min) / (samples - 1);
		for (int i = 0; i < samples; ++i)
		{
			X[i] = i < samples - 1 ? di * i : interval_max;
			Y[i] = F(X[i]);
		}
		
		PolynomialFit<T, Dim> poly;
		bool succ = poly.Fit(X, Y, samples);
		
		delete []buffer;
		
		if (succ)
		{
			for (int i = 0; i <= Dim; ++i)
			{
				coef[i] = poly.coef[i];
			}
		}
		return succ;
	}
	
	T Eval(const T x) const
	{
		T sum = (T)0;
		T powx = (T)1;
		for (int i = 0; i <= Dim; ++i)
		{
			sum += coef[i] * powx;
			powx *= x;
		}
		return sum;
	}
	
public:
	T coef[Dim+1] = {0};	// f(x) = c0 + c1 * x + c2 * x^2 + ... + cN * c^N
};

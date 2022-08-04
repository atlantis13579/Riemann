#pragma once

template<typename T>
class LRModel
{
public:
	LRModel(int Dim)
	{
		dim = Dim;
		coef = new T[dim + 1];
		for (int i = 0; i <= dim; ++i)
		{
			coef[i] = (T)0;
		}
	}

	~LRModel()
	{
		if (coef) delete []coef;
	}

	void Fit(const T *X, const T * Y, int N)
	{
		FitGradientDescent(X, Y, N, 0.2f, 100);
	}

	T Eval(const T* X) const
	{
		return EvalLR(coef, dim, X);
	}

private:
	static T EvalLR(T* coef, int dim, const T* X)
	{
		T sum = coef[dim];
		for (int i = 0; i < dim; ++i)
		{
			sum += X[i] * coef[i];
		}
		return sum;
	}

	void FitGradientDescent(const T* X, const T* Y, const int N, const T LearningRate, const int MaxIteration)
	{
		for (int i = 0; i <= dim; ++i)
		{
			coef[i] = (T)0;
		}

		T *gradient = new T[dim + 1];

		int it = 0;
		while (it++ < MaxIteration)
		{
			for (int i = 0; i <= dim; ++i)
			{
				gradient[i] = 0.0f;
			}

			T sum_sqr_delta = (T)0;
			for (int k = 0; k < N; ++k)
			{
				const T* pX = X + dim * k;
				const T delta = EvalLR(coef, dim, pX) - Y[k];
				for (int i = 0; i < dim; ++i)
				{
					gradient[i] += 2 * delta * pX[i];
				}
				gradient[dim] += 2 * delta;
				sum_sqr_delta += delta * delta;
			}

			sum_sqr_delta /= N;

			if (sum_sqr_delta < 0.00001)
			{
				break;
			}
			
			for (int i = 0; i <= dim; ++i)
			{
				coef[i] -= LearningRate * gradient[i] / N;
			}
		}

		delete gradient;
	}

public:
	T* coef;
	int dim;
};
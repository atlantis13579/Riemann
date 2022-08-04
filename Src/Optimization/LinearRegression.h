#pragma once

#include <stdlib.h>

enum class LR_algorithm : uint8_t
{
	Auto,
	GradientDescent,
	BatchGradientDescent,
	StochasticGradientDescent,
	NormalEquation,
};

struct LRParam
{
	LR_algorithm	algorithm = LR_algorithm::Auto;
	double 			learningRate = 0.1;
	int 			maxIterations = 100;
	int				batchSize = 128;
	int				epochs = 10;
	double			convergeMSE = 0.01;
	bool			randomInitCoef = false;
};

template <typename T>
class LRModel
{
public:
	LRModel(int Dim)
	{
		dim = Dim;
		coef = new T[2*(dim + 1)];
		for (int i = 0; i <= dim; ++i)
		{
			coef[i] = (T)0;
		}
		gradient = coef + dim + 1;
	}

	~LRModel()
	{
		if (coef) delete []coef;
	}

	void Fit(const T *X, const T * Y, int N, const LRParam &param)
	{
		LR_algorithm algorithm = param.algorithm;
		if (algorithm == LR_algorithm::Auto)
		{
			if (dim <= 8 && N <= 1024)
				algorithm = LR_algorithm::NormalEquation;
			else if (N <= 128)
				algorithm = LR_algorithm::BatchGradientDescent;
			else if (N <= 10000)
				algorithm = LR_algorithm::GradientDescent;
			else
				algorithm = LR_algorithm::StochasticGradientDescent;
		}
		
		switch (algorithm) {
		case LR_algorithm::GradientDescent:
			GradientDescent(X, Y, N, param);
			break;
		case LR_algorithm::BatchGradientDescent:
			BatchGradientDescent(X, Y, N, param);
			break;
		case LR_algorithm::StochasticGradientDescent:
			StochasticGradientDescent(X, Y, N, param);
			break;
		case LR_algorithm::NormalEquation:
			break;
		default:
			break;
		}
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
	
	void InitCoef(bool random)
	{
		for (int i = 0; i <= dim; ++i)
		{
			coef[i] = random ? (T)1 * rand() / RAND_MAX : (T)0;
		}
	}
	
	void GradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef(param.randomInitCoef);

		for (int j = 0; j < N; ++j)
		{
			const T* pX = X + dim * j;
			const T delta = EvalLR(coef, dim, pX) - Y[j];
			for (int i = 0; i < dim; ++i)
			{
				coef[i] -= param.learningRate * delta * pX[i];
			}
			coef[dim] -= param.learningRate * delta;
		}
	}

	void BatchGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef(param.randomInitCoef);

		int it = 0;
		while (it++ < param.maxIterations)
		{
			for (int i = 0; i <= dim; ++i)
			{
				gradient[i] = 0.0f;
			}

			T mse = (T)0;
			for (int j = 0; j < N; ++j)
			{
				const T* pX = X + dim * j;
				const T delta = EvalLR(coef, dim, pX) - Y[j];
				for (int i = 0; i < dim; ++i)
				{
					gradient[i] += delta * pX[i];
				}
				gradient[dim] += delta;
				mse += delta * delta;
			}

			mse /= N;
			if (mse < param.convergeMSE)
			{
				break;
			}
			
			for (int i = 0; i <= dim; ++i)
			{
				coef[i] -= param.learningRate * gradient[i] / N;
			}
		}
	}
	
	void StochasticGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef(param.randomInitCoef);
		
		int it = 0, k = 0;
		const int epochs = param.epochs * N / param.batchSize;
		while (it++ < epochs)
		{
			for (int i = 0; i <= dim; ++i)
			{
				gradient[i] = 0.0f;
			}

			for (int j = 0; j < param.batchSize; ++j)
			{
				const T* pX = X + dim * k;
				const T delta = EvalLR(coef, dim, pX) - Y[k];
				for (int i = 0; i < dim; ++i)
				{
					gradient[i] += delta * pX[i];
				}
				gradient[dim] += delta;
				k = (k + 1) % N;
			}
			
			for (int i = 0; i <= dim; ++i)
			{
				coef[i] -= param.learningRate * gradient[i] / param.batchSize;
			}
		}
	}

public:
	T* coef;
	T* gradient;
	int dim;
};

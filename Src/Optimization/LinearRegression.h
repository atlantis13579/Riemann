#pragma once

#include "../LinearSystem/DenseMatrix.h"

enum class LR_solver : uint8_t
{
	Auto,
	GradientDescent,
	BatchGradientDescent,
	StochasticGradientDescent,
	NormalEquation,
};

struct LRParam
{
	LR_solver	solver = LR_solver::Auto;
	double 		learningRate = 0.1;
	int 		maxIterations = 100;
	int			batchSize = 128;
	int			epochs = 10;
	double		convergeMSE = 0.01;
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
		LR_solver solver = param.solver;
		if (solver == LR_solver::Auto)
		{
			if (dim <= 8 && N <= 1024)
				solver = LR_solver::NormalEquation;
			else if (N <= 128)
				solver = LR_solver::BatchGradientDescent;
			else if (N <= 10000)
				solver = LR_solver::GradientDescent;
			else
				solver = LR_solver::StochasticGradientDescent;
		}
		
		switch (solver) {
		case LR_solver::GradientDescent:
			GradientDescent(X, Y, N, param);
			break;
		case LR_solver::BatchGradientDescent:
			BatchGradientDescent(X, Y, N, param);
			break;
		case LR_solver::StochasticGradientDescent:
			StochasticGradientDescent(X, Y, N, param);
			break;
		case LR_solver::NormalEquation:
			NormalEquation(X, Y, N);
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
	static inline T EvalLR(T* coef, int dim, const T* X)
	{
		T sum = coef[dim];
		for (int i = 0; i < dim; ++i)
		{
			sum += X[i] * coef[i];
		}
		return sum;
	}
	
	void InitCoef()
	{
		for (int i = 0; i <= dim; ++i)
		{
			coef[i] = (T)0;
		}
	}
	
	void GradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef();

		T learningRate = (T)param.learningRate;
		for (int j = 0; j < N; ++j)
		{
			const T* pX = X + dim * j;
			const T error = EvalLR(coef, dim, pX) - Y[j];
			for (int i = 0; i < dim; ++i)
			{
				coef[i] -= learningRate * error * pX[i];
			}
			coef[dim] -= learningRate * error;
		}
	}

	void BatchGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef();

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
				const T error = EvalLR(coef, dim, pX) - Y[j];
				for (int i = 0; i < dim; ++i)
				{
					gradient[i] += error * pX[i];
				}
				gradient[dim] += error;
				mse += error * error;
			}

			mse /= N;
			if (mse < (T)param.convergeMSE)
			{
				break;
			}
			
			T learningRate = (T)param.learningRate;
			for (int i = 0; i <= dim; ++i)
			{
				coef[i] -= learningRate * gradient[i] / N;
			}
		}
	}
	
	void StochasticGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
	{
		InitCoef();
		
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
				const T error = EvalLR(coef, dim, pX) - Y[k];
				for (int i = 0; i < dim; ++i)
				{
					gradient[i] += error * pX[i];
				}
				gradient[dim] += error;
				k = (k + 1) % N;
			}
			
			T learningRate = (T)param.learningRate;
			for (int i = 0; i <= dim; ++i)
			{
				coef[i] -= learningRate * gradient[i] / param.batchSize;
			}
		}
	}

	void NormalEquation(const T* pX, const T* pY, const int N)
	{
		TDenseMatrix<T> XTX(dim + 1, dim + 1);
		for (int i = 0; i <= dim; ++i)
		for (int j = i; j <= dim; ++j)
		{
			T sum = (T)0;
			for (int k = 0; k < N; ++k)
			{
				T xi = i < dim ? pX[k * dim + i] : (T)1;
				T xj = j < dim ? pX[k * dim + j] : (T)1;
				sum += xi * xj;
			}
			XTX(i, j) = sum;
		}
		for (int i = 0; i <= dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			XTX(i, j) = XTX(j, i);
		}

		TDenseMatrix<T> invXTX = XTX.Inverse();
		TDenseMatrix<T> invXTXXT(dim + 1, N);
		for (int i = 0; i <= dim; ++i)
		for (int j = 0; j < N; ++j)
		{
			T sum = (T)0;
			for (int k = 0; k <= dim; ++k)
			{
				T xik = invXTX(i, k);
				T xkj = k < dim ? pX[j * dim + k] : (T)1;
				sum += xik * xkj;
			}
			invXTXXT(i, j) = sum;
		}

		for (int i = 0; i <= dim; ++i)
		{
			T sum = (T)0;
			for (int j = 0; j < N; ++j)
			{
				sum += invXTXXT(i, j) * pY[j];
			}
			coef[i] = sum;
		}

		return;
	}

public:
	T* coef;
	T* gradient;
	int dim;
};

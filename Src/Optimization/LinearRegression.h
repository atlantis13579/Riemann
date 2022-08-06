#pragma once

// #define USE_PSEUDO_INVERSE
#ifdef USE_PSEUDO_INVERSE
#include "../LinearSystem/MoorePenrosePseudoInverse.h"
#else
#include "../LinearSystem/GaussianElimination.h"
#endif // USE_PSEUDO_INVERSE

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

	bool Fit(const T *X, const T * Y, int N, const LRParam &param)
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
		
		bool succ = false;
		switch (solver) {
		case LR_solver::GradientDescent:
			succ = GradientDescent(X, Y, N, param);
			break;
		case LR_solver::BatchGradientDescent:
			succ = BatchGradientDescent(X, Y, N, param);
			break;
		case LR_solver::StochasticGradientDescent:
			succ = StochasticGradientDescent(X, Y, N, param);
			break;
		case LR_solver::NormalEquation:
			succ = NormalEquation(X, Y, N);
			break;
		default:
			break;
		}
		
		intercept = coef[dim];
		return succ;
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
	
	bool GradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
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
		
		return true;
	}

	bool BatchGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
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
		
		return true;
	}
	
	bool StochasticGradientDescent(const T* X, const T* Y, const int N, const LRParam &param)
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
		
		return true;
	}

	// coef = (X^T * X)^(-1) * X^T * Y
	bool NormalEquation(const T* pX, const T* pY, const int N)
	{
		const T one = (T)1;
		T *XTX = new T[2*(dim + 1)*(dim + 1) + (dim + 1)*N];
		for (int i = 0; i <= dim; ++i)
		for (int j = i; j <= dim; ++j)
		{
			T sum = (T)0;
			for (int k = 0; k < N; ++k)
			{
				T xi = i < dim ? pX[k * dim + i] : one;
				T xj = j < dim ? pX[k * dim + j] : one;
				sum += xi * xj;
			}
			XTX[i * (dim+1) + j] = sum;
		}
		for (int i = 0; i <= dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			XTX[i * (dim+1) + j] = XTX[j * (dim+1) + i];
		}

		T *invXTX = XTX + (dim + 1)*(dim + 1);
		
		#ifdef USE_PSEUDO_INVERSE
		bool succ = MoorePenrosePseudoInverse<T>()(XTX, dim + 1, invXTX);
		#else
		bool succ = GaussianElimination<T>()(XTX, dim + 1, invXTX, nullptr);
		#endif
		
		if (!succ)
		{
			return false;
		}

		T* invXTXXT = XTX + 2*(dim + 1)*(dim + 1);
		for (int i = 0; i <= dim; ++i)
		for (int j = 0; j < N; ++j)
		{
			T sum = (T)0;
			for (int k = 0; k <= dim; ++k)
			{
				T xik = invXTX[i * (dim + 1) +  k];
				T xkj = k < dim ? pX[j * dim + k] : one;
				sum += xik * xkj;
			}
			invXTXXT[i * N + j] = sum;
		}

		for (int i = 0; i <= dim; ++i)
		{
			T sum = (T)0;
			for (int j = 0; j < N; ++j)
			{
				sum += invXTXXT[i * N + j] * pY[j];
			}
			coef[i] = sum;
		}

		delete []XTX;
		
		return true;
	}

public:
	T* coef;
	T intercept;
	T* gradient;
	int dim;
};

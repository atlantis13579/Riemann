#pragma once

template<typename sgdfloatval_t>
class SGDModel
{
public:
	virtual ~SGDModel() {}
	virtual sgdfloatval_t Evaluate(const sgdfloatval_t* X, const sgdfloatval_t* F, sgdfloatval_t* G) const = 0;
	virtual void ApplyGradient(sgdfloatval_t* G) = 0;
};

struct SGDParam
{
	double	learningRate = 0.01;
	int		batchSize = 128;
	int		epochs = 10;
	double	convergeMSE = 0.0001;
};

template<typename sgdfloatval_t>
class SGDLeastSquaresOptimizer
{
public:
	SGDLeastSquaresOptimizer()
	{
		iterations = 0;
		mse = (sgdfloatval_t)-1;
	}

	void Optimize(SGDModel<sgdfloatval_t>* model, const int Dim, const sgdfloatval_t* X, const int DimX, const sgdfloatval_t *Y, const int N, const SGDParam &param)
	{
		sgdfloatval_t *gradient_accum = new sgdfloatval_t[2*Dim];
		sgdfloatval_t *gradient = gradient_accum + Dim;
		
		iterations = 0;
		int k = 0;
		const int max_iterations = param.epochs * N / param.batchSize;
		while (iterations++ < max_iterations)
		{
			bool end_of_epoch = false;

			for (int i = 0; i < Dim; ++i)
			{
				gradient_accum[i] = (sgdfloatval_t)0;
			}

			for (int j = 0; j < param.batchSize; ++j)
			{
				const sgdfloatval_t* pX = X + DimX * k;
				const sgdfloatval_t* pY = Y + k;
				const sgdfloatval_t loss = model->Evaluate(pX, pY, gradient) - Y[k];
				for (int i = 0; i < Dim; ++i)
				{
					gradient_accum[i] += loss * gradient[i];
				}
				k = (k + 1) % N;
				end_of_epoch = k == 0;
			}

			sgdfloatval_t learningRate = (sgdfloatval_t)param.learningRate;
			for (int i = 0; i < Dim; ++i)
			{
				gradient_accum[i] = -learningRate * gradient_accum[i] / param.batchSize;
			}

			model->ApplyGradient(gradient_accum);

			if (end_of_epoch)
			{
				mse = (sgdfloatval_t)0;
				for (int j = 0; j < N; ++j)
				{
					const sgdfloatval_t* pX = X + DimX * j;
					const sgdfloatval_t* pY = Y + j;
					const sgdfloatval_t error = model->Evaluate(pX, pY, gradient) - Y[j];
					mse += error * error;
				}
				mse /= N;

				if (mse < param.convergeMSE)
				{
					break;
				}
			}
		}
		
		delete []gradient_accum;
	}

public:
	int				iterations;
	sgdfloatval_t	mse;
};


template<typename gdfloatval_t>
class GradientDescentEvalFunction
{
public:
	virtual ~GradientDescentEvalFunction() {}
	virtual gdfloatval_t Evaluate(const gdfloatval_t* X, int Dim, gdfloatval_t* Gradient) const = 0;
};

struct GDParam
{
	int		maxIterations = 128;
	double	learningRate = 0.1;
	double	convergeEps = 0.00001;
};

template<typename gdfloatval_t>
class GradientDescentMinimizer
{
public:
	GradientDescentMinimizer()
	{
		status = 0;
		iterations = 0;
	}

	gdfloatval_t Minimize(GradientDescentEvalFunction<gdfloatval_t>* func, int Dim, const GDParam& param)
	{
		gdfloatval_t* X = new gdfloatval_t[Dim];
		gdfloatval_t min_Y = Minimize(func, X, Dim, param);
		delete []X;
		return min_Y;
	}

	gdfloatval_t Minimize(GradientDescentEvalFunction<gdfloatval_t>* func, gdfloatval_t* X, int Dim, const GDParam& param)
	{
		gdfloatval_t* gradient = new gdfloatval_t[Dim];
		gdfloatval_t min_Y = (gdfloatval_t)0;

		for (int i = 0; i < Dim; ++i)
		{
			X[i] = 0;
		}

		gdfloatval_t learningRate = (gdfloatval_t)param.learningRate;
		gdfloatval_t eps = (gdfloatval_t)param.convergeEps;

		iterations = 0;
		while (iterations++ < param.maxIterations)
		{
			gdfloatval_t Y = func->Evaluate(X, Dim, gradient);
			for (int i = 0; i < Dim; ++i)
			{
				X[i] -= (gdfloatval_t)param.learningRate * gradient[i];
			}

			gdfloatval_t diff = Y < min_Y ? min_Y - Y : Y - min_Y;
			min_Y = Y;
			if (iterations > 1 && diff < eps)
			{
				break;
			}
		}
		delete[]gradient;
		return min_Y;
	}

public:
	int status;
	int iterations;
};
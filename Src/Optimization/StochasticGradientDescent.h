#pragma once

template<typename sgdfloatval_t>
class SGDModel
{
public:
	virtual ~SGDModel() {}
	virtual sgdfloatval_t Evaluate(const sgdfloatval_t* X, sgdfloatval_t* G) const = 0;
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
				const sgdfloatval_t loss = model->Evaluate(pX, gradient) - Y[k];
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
					const sgdfloatval_t error = model->Evaluate(pX, gradient) - Y[j];
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

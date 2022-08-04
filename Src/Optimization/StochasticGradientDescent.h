#pragma once

template<typename sgdfloatval_t>
class SGDModel
{
public:
	virtual ~SGDModel() {}
	virtual void Evaluate(const sgdfloatval_t* X, int Dim, sgdfloatval_t* F, sgdfloatval_t* Gradient) const = 0;
	virtual sgdfloatval_t* GetCoef();
};

struct sgdparam_t
{
	double	learningRate = 0.01;
	int		batchSize = 128;
	int		epochs = 10;
	double	convergeMSE = 0.01;
	bool	randominit = true;
};

template<typename sgdfloatval_t>
class SGDSolver
{
public:
	SGDSolver()
	{
		mse = (sgdfloatval_t)0;
	}

	void Solve(SGDModel<sgdfloatval_t>* model, const sgdfloatval_t* X, const int Dim, const sgdfloatval_t *Y, const int N)
	{
		sgdparam_t param;
		
		sgdfloatval_t *coef = model->GetCoef();
		for (int i = 0; i < Dim; ++i)
		{
			coef[i] = param.randominit ? (sgdfloatval_t)1 * rand() / RAND_MAX : (sgdfloatval_t)0;
		}
		
		sgdfloatval_t *gradient = new sgdfloatval_t[2*Dim];
		sgdfloatval_t *gradient_j = gradient + Dim;
		
		int it = 0, k = 0;
		const int epochs = param.epochs * N / param.batchSize;
		while (it++ < epochs)
		{
			for (int i = 0; i < Dim; ++i)
			{
				gradient[i] = (sgdfloatval_t)0;
			}

			for (int j = 0; j < param.batchSize; ++j)
			{
				const sgdfloatval_t* pX = X + Dim * k;
				const sgdfloatval_t* pY = Y + k;
				const sgdfloatval_t delta = model->Evaluate(pX, Dim, pY, gradient_j) - Y[k];
				for (int i = 0; i < Dim; ++i)
				{
					gradient[i] += gradient_j[i];
				}
				k = (k + 1) % N;
			}
			
			for (int i = 0; i < Dim; ++i)
			{
				coef[i] -= param.learningRate * gradient[i] / param.batchSize;
			}
		}
		
		delete []gradient;
	}
	
private:
	

public:
	sgdfloatval_t mse;
};


template<typename sgdfloatval_t>
class SGDMinimizer
{
public:
	SGDMinimizer()
	{
		status = 0;
		iterations = 0;
	}
	
	sgdfloatval_t Minimize(LBFGSEvalFunction<sgdfloatval_t>* func, int Dim)
	{
		return 0;
	}

public:
	int status;
	int iterations;
};

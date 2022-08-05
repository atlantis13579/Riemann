#pragma once

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
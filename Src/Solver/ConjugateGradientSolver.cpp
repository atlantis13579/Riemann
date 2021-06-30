
#include "ConjugateGradientSolver.h"

#define MAX_ITER 100
#define EPSILON 0.0001f

ConjugateGradientSolver::ConjugateGradientSolver()
{
	m_SA = nullptr;
	r = nullptr;
	d = nullptr;
	q = nullptr;
	tempo = nullptr;
}

ConjugateGradientSolver::~ConjugateGradientSolver()
{
	if (m_SA) delete m_SA;

	if (r) delete []r;
	if (d) delete []d;
	if (q) delete []q;
	if (tempo) delete []tempo;
}

void ConjugateGradientSolver::InitSparseSolverCompressed3x3(const float *A, int _n)
{
	m_nSize = _n * 3;
	m_SA = new SparseMatrix(A, _n);

	r = new float[m_nSize];
	d = new float[m_nSize];
	q = new float[m_nSize];
	tempo = new float[m_nSize];
}

void ConjugateGradientSolver::SolveSparseCompressed3x3(float *xx, const float* b)
{
	float deltaOld, deltaNew, delta0, alpha, beta;
	int it;

	for (int i = 0; i < m_nSize; ++i)
	{
		r[i] = 0;
		d[i] = 0;
		//x[i]=0;
		q[i] = 0;
	}

	m_SA->MulXCompressed3x3(xx, tempo);

	for (int i = 0; i < m_nSize; ++i)
	{
		d[i] = r[i] = b[i] - tempo[i];
	}

	it = 0;
	deltaNew = Dot(r, r);
	delta0 = deltaNew;
	alpha = beta = 0;

	while (it < MAX_ITER && deltaNew > EPSILON*EPSILON*delta0)
	{
		it++;

		m_SA->MulXCompressed3x3(d, q);

		alpha = deltaNew / Dot(d, q);

		for (int i = 0; i < m_nSize; ++i)
		{
			xx[i] = xx[i] + alpha * d[i];
		}

		if (it % 50 == 0)
		{
			m_SA->MulXCompressed3x3(xx, tempo);
			for (int i = 0; i < m_nSize; ++i)
				r[i] = b[i] - tempo[i];
		}
		else
		{
			for (int i = 0; i < m_nSize; ++i)
				r[i] = r[i] - alpha * q[i];
		}

		deltaOld = deltaNew;
		deltaNew = Dot(r, r);
		beta = deltaNew / deltaOld;

		for (int i = 0; i < m_nSize; ++i)
			d[i] = r[i] + beta * d[i];
	}
}

void ConjugateGradientSolver::Solve(float *xx, const float *b)
{
	float deltaOld, deltaNew, delta0, alpha, beta;
	int it;

	for (int i = 0; i < m_nSize; ++i)
	{
		r[i] = 0;
		d[i] = 0;
	//	x[i]=0;
		q[i] = 0;
	}

	for (int i = 0; i < m_nSize; ++i)
	{
		d[i] = r[i] = b[i] - Dot(m_A + i * m_nSize, xx);
	}

	it = 0;
	deltaNew = Dot(r, r);
	delta0 = deltaNew;
	alpha = beta = 0;

	while (it < MAX_ITER && deltaNew > EPSILON*EPSILON*delta0)
	{
		it++;
		for (int i = 0; i < m_nSize; i++)
			q[i] = Dot(m_A + i * m_nSize, d);

		alpha = deltaNew / Dot(d, q);

		for (int i = 0; i < m_nSize; i++)
			xx[i] = xx[i] + alpha * d[i];

		if (it % 50 == 0)
		{
			//refresh r of its horrible floating point errors
			for (int i = 0; i < m_nSize; ++i)
				r[i] = b[i] - Dot(m_A + i * m_nSize, xx);
		}
		else
		{
			for (int i = 0; i < m_nSize; ++i)
				r[i] = r[i] - alpha * q[i];
		}

		deltaOld = deltaNew;
		deltaNew = Dot(r, r);
		beta = deltaNew / deltaOld;

		for (int i = 0; i < m_nSize; ++i)
			d[i] = r[i] + beta * d[i];

	}
}

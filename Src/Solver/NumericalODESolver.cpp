
#include "NumericalODESolver.h"

#include <math.h>
#include <string.h>
#include <vector>

namespace NumericalODESolver
{

// https://en.wikipedia.org/wiki/Explicit_and_implicit_methods
ExplicitEuler::ExplicitEuler(const ODEVectorFunction& dvdt, float t0)
{
	m_dvdt = dvdt;
	m_t = t0;
}

void ExplicitEuler::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() < nDof)
		m_buf.resize(nDof);
	float *dv = &m_buf[0];
	m_dvdt(v, nDof, m_t, dv);
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dv[i] * dt;
	}
	m_t += dt;
}


// https://en.wikipedia.org/wiki/Midpoint_method
MidpointRule::MidpointRule(const ODEVectorFunction& dvdt, float t0)
{
	m_dvdt = dvdt;
	m_t = t0;
}

void MidpointRule::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() < 2 * nDof)
		m_buf.resize(2 * nDof);
	float* dv = &m_buf[0];
	float* dh = dv + nDof;

	m_dvdt(v, nDof, m_t, dv);
	for (int i = 0; i < nDof; ++i)
	{
		dh[i] = v[i] + dv[i] * dt * 0.5f;
	}
	m_dvdt(dh, nDof, m_t, dv);
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dv[i] * dt;
	}
	m_t += dt;
}


// https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
SymplecticEuler::SymplecticEuler(const ODEVectorFunction2& dvdt, float t0)
{
	m_dvdt = dvdt;
	m_t = t0;
}

void SymplecticEuler::Integrate(float dt, int nDof, float* v)
{
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + m_dvdt(v, i, m_t) * dt;
	}
	m_t += dt;
}


// https://en.wikipedia.org/wiki/Backward_Euler_method
ImplicitEuler_FixedPointIteration::ImplicitEuler_FixedPointIteration(const ODEVectorFunction& dvdt, float t0, int MaxIter)
{
	m_dvdt = dvdt;
	m_MaxIter = MaxIter;
	m_t = t0;
}

void ImplicitEuler_FixedPointIteration::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() < nDof)
		m_buf.resize(nDof);

	float* dv = &m_buf[0];

	int nIter = 0;
	while (nIter++ < m_MaxIter)
	{
		m_dvdt(v, nDof, m_t, dv);
		bool converge = true;
		for (int i = 0; i < nDof; ++i)
		{
			float dev = fabsf(dv[i] * dt);
			if (dev < 0.0001f)
				continue;
			v[i] = v[i] + dv[i] * dt;
			converge = false;
		}
		if (converge)
			break;
	}
	m_t += dt;
}

// https://en.wikipedia.org/wiki/Backward_Euler_method
// https://en.wikipedia.org/wiki/Stiff_equation
ImplicitEuler_NewtonIteration::ImplicitEuler_NewtonIteration(const ODEVectorFunction& dvdt, const ODEVectorFunction& d2vdt2, float t0, int MaxIter)
{
	m_dvdt = dvdt;
	m_d2vt2 = d2vdt2;
	m_MaxIter = MaxIter;
	m_t = t0;
}

void ImplicitEuler_NewtonIteration::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() < 3 * nDof)
		m_buf.resize(3 * nDof);

	float* dv = &m_buf[0];
	float* d2v = dv + nDof;
	float* v0 = dv + 2 * nDof;
	memcpy(v0, v, sizeof(v[0]) * nDof);

	int nIter = 0;
	while (nIter++ < m_MaxIter)
	{
		m_dvdt(v, nDof, m_t, dv);
		m_d2vt2(v, nDof, m_t, d2v);
		bool converge = true;
		for (int i = 0; i < nDof; ++i)
		{
			// https://en.wikipedia.org/wiki/Newton%27s_method
			float F = v[i] - v0[i] - dt * dv[i];
			float DF = 1 - dt * d2v[i];
			float dev = fabsf(F / DF);
			if (dev < 0.00001f)
				continue;
			v[i] = v[i] - F / DF;
			converge = false;
		}
		if (converge)
			break;

	}
	m_t += dt;
}


// https://en.wikipedia.org/wiki/Verlet_integration
VerletIntegration::VerletIntegration(const ODEVectorFunction& d2vdt2, float t0)
{
	m_d2vdt2 = d2vdt2;
	m_t = t0;
}

void VerletIntegration::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() != 2 * nDof)
	{
		m_buf.resize(2 * nDof);
		memcpy(&m_buf[0], v, sizeof(v[0]) * nDof);
	}
	else
	{
		float* v_prev = &m_buf[0];
		float* d2v = v_prev + nDof;

		m_d2vdt2(v, nDof, m_t, d2v);
		for (int i = 0; i < nDof; ++i)
		{
			float prev = v_prev[i];
			v_prev[i] = v[i];
			v[i] += v[i] - prev + d2v[i] * dt * dt;
		}
	}

	m_t += dt;
}


// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
RungeKutta4::RungeKutta4(const ODEVectorFunction& dvdt, float t0)
{
	m_dvdt = dvdt;
	m_t = t0;
}

void RungeKutta4::Integrate(float dt, int nDof, float* v)
{
	if (m_buf.size() < 3 * nDof)
		m_buf.resize(3 * nDof);

	float* dv = &m_buf[0];
	float* v0 = dv + nDof;
	float* t = dv + 2 * nDof;

	memcpy(v0, v, sizeof(v[0]) * nDof);

	m_dvdt(v, nDof, m_t, &dv[0]);		// k1
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dt * dv[i] / 6.0f;
		t[i] = v0[i] + dt * dv[i] * 0.5f;
	}
	m_dvdt(t, nDof, m_t, &dv[0]);		// k2
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dt * dv[i] / 3.0f;
		t[i] = v0[i] + dt * dv[i] * 0.5f;
	}
	m_dvdt(t, nDof, m_t, &dv[0]);		// k3
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dt * dv[i] / 3.0f;
		t[i] = v0[i] + dt * dv[i];
	}
	m_dvdt(t, nDof, m_t, &dv[0]);		// k4
	for (int i = 0; i < nDof; ++i)
	{
		v[i] = v[i] + dt * dv[i] / 6.0f;
	}

	m_t += dt;
}


void circle_dvdt(const float* v, int nDof, float t, float* dv)
{
	float theta = atanf(v[1] / v[0]) - 3.1416f / 2;
	dv[0] = cosf(theta);
	dv[1] = sinf(theta);
}

float circle_dvdt_symp(const float* v, int Idx, float t)
{
	float theta = atanf(v[1] / v[0]) - 3.1416f / 2;
	return Idx == 0 ? cosf(theta) : sinf(theta);
}

void circle_d2vdt2(const float* v, int nDof, float t, float* dv)
{
	float x = v[0];
	float y = v[1];
	float m = sqrtf(x * x + y * y);
	dv[0] = -x / m;
	dv[1] = -y / m;
}

}
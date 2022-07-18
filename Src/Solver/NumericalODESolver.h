#pragma once

#include <functional>
#include <vector>

// ODE initial value Problem, given dvdt(v, t) = dv/dt and v(t0), solve y(t1)
// dydt is a vector function of form "void dvdt(float *v, int nDof, float t, float *dv)"

namespace NumericalODESolver
{

// (input vector, nDof, t, output vector)
typedef std::function<void(const float*, int, float, float*)> ODEVectorFunction;

// (input vector, output index, t)
typedef std::function<float(const float*, int, float)> ODEVectorFunction2;

class ExplicitEuler
{
public:
	ExplicitEuler(const ODEVectorFunction& dvdt, float t0);
	~ExplicitEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};


class MidpointEuler
{
public:
	MidpointEuler(const ODEVectorFunction& dvdt, float t0);
	~MidpointEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};


class SymplecticEuler
{
public:
	SymplecticEuler(const ODEVectorFunction2& dvdt, float t0);
	~SymplecticEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction2 m_dvdt;
	float m_t;
};


// For non-stiff problems, don't need the second derivative, sometimes not converge
class ImplicitEuler_FixedPointIteration
{
public:
	ImplicitEuler_FixedPointIteration(const ODEVectorFunction& dvdt, float t0, int MaxIter);
	~ImplicitEuler_FixedPointIteration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_dvdt;
	std::vector<float> m_buf;
	float m_t;
	int m_MaxIter;
};


// Need the second derivative, this is probably the best method 
class ImplicitEuler_NewtonIteration
{
public:
	ImplicitEuler_NewtonIteration(const ODEVectorFunction& dvdt, const ODEVectorFunction& d2vdt2, float t0, int MaxIter);
	~ImplicitEuler_NewtonIteration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_dvdt;		// first derivative
	ODEVectorFunction m_d2vt2;		// second derivative
	std::vector<float> m_buf;
	float m_t;
	int m_MaxIter;
};


// Know only the second derivative 
class VerletIntegration
{
public:
	VerletIntegration(const ODEVectorFunction& d2vdt2, float t0);
	~VerletIntegration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_d2vdt2;
	std::vector<float> m_buf;
	float m_t;
};


class RungeKutta4
{
public:
	RungeKutta4(const ODEVectorFunction& dvdt, float t0);
	~RungeKutta4() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ODEVectorFunction m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};

void circle_dvdt(const float* v, int nDof, float t, float* dv);
float circle_dvdt_symp(const float* v, int Idx, float t);
void circle_d2vdt2(const float* v, int nDof, float t, float* dv);
}


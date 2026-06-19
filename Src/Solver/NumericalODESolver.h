#pragma once

#include <functional>
#include <vector>

// ODE initial value Problem, given dvdt(v, t) = dv/dt and v(t0), solve y(t1)
// dydt is a vector function of form "void dvdt(float *v, int nDof, float t, float *dv)"

namespace NumericalODESolver
{

// (input vector, nDof, t, output vector)
typedef std::function<void(const float*, int, float, float*)> ode_vector_function_t;

// (input vector, output index, t)
typedef std::function<float(const float*, int, float)> ode_vector_function2_t;

class ExplicitEuler
{
public:
	ExplicitEuler(const ode_vector_function_t& dvdt, float t0);
	~ExplicitEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};


class MidpointEuler
{
public:
	MidpointEuler(const ode_vector_function_t& dvdt, float t0);
	~MidpointEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};


class SymplecticEuler
{
public:
	SymplecticEuler(const ode_vector_function2_t& dvdt, float t0);
	~SymplecticEuler() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function2_t m_dvdt;
	float m_t;
};


// For non-stiff problems, don't need the second derivative, sometimes not converge
class ImplicitEuler_FixedPointIteration
{
public:
	ImplicitEuler_FixedPointIteration(const ode_vector_function_t& dvdt, float t0, int MaxIter, float Eps);
	~ImplicitEuler_FixedPointIteration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_dvdt;
	std::vector<float> m_buf;
	float m_t;
	int m_MaxIter;
	float m_Eps;
};


// Need the second derivative, this is probably the best method 
class ImplicitEuler_NewtonIteration
{
public:
	ImplicitEuler_NewtonIteration(const ode_vector_function_t& dvdt, const ode_vector_function_t& d2vdt2, float t0, int MaxIter, float Eps);
	~ImplicitEuler_NewtonIteration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_dvdt;		// first derivative
	ode_vector_function_t m_d2vt2;		// second derivative
	std::vector<float> m_buf;
	float m_t;
	int m_MaxIter;
	float m_Eps;
};


// Know only the second derivative 
class VerletIntegration
{
public:
	VerletIntegration(const ode_vector_function_t& d2vdt2, float t0);
	~VerletIntegration() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_d2vdt2;
	std::vector<float> m_buf;
	float m_t;
};


class RungeKutta4
{
public:
	RungeKutta4(const ode_vector_function_t& dvdt, float t0);
	~RungeKutta4() {}

	void Integrate(float dt, int nDof, float* v);

private:
	ode_vector_function_t m_dvdt;
	std::vector<float> m_buf;
	float m_t;
};

void circle_dvdt(const float* v, int nDof, float t, float* dv);
float circle_dvdt_symp(const float* v, int Idx, float t);
void circle_d2vdt2(const float* v, int nDof, float t, float* dv);
}


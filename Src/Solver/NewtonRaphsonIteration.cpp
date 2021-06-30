
#include <functional>
#include "NewtonRaphsonIteration.h"

const float kNewtonEplison = 0.001f;

float Derivative(const std::function<float(float)> &f, float t)
{
	const float dt = 0.01f;
	float d1 = (f(t + dt) - f(t)) / dt;
	float d2 = (f(t) - f(t - dt)) / dt;
	return (d1 + d2) * 0.5f;
}

float NewtonRaphson::FindRoot(const std::function<float(float)> &f, const std::function<float(float)> &df, float x0, const int maxIteration)
{
	float x = x0;
	int it = 0;
	while (it++ < maxIteration && fabsf(f(x)) > kNewtonEplison)
	{
		float fx = f(x);
		float dfx = df(x);
		x = x - fx / dfx;
	}
	return x;
}

float NewtonRaphson::FindRoot(const std::function<float(float)> &f, float x0, const int maxIteration)
{
	float x = x0;
	int it = 0;
	while (it++ < maxIteration && fabsf(f(x)) > kNewtonEplison)
	{
		float fx = f(x);
		float dfx = Derivative(f, x);
		x = x - fx / dfx;
	}
	return x;
}
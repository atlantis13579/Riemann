
#pragma once

#include <functional>

#define MAX_ITERATION_NEWTON	10

float Derivative(const std::function<float(float)> &f, float t);

class NewtonRaphson
{
public:
	static float FindRoot(const std::function<float(float)> &f, const std::function<float(float)> &df, float x0, const int maxIteration = MAX_ITERATION_NEWTON);
	static float FindRoot(const std::function<float(float)> &f, float x0, const int maxIteration = MAX_ITERATION_NEWTON);
};

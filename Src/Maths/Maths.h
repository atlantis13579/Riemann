
#pragma once

#include <math.h>
#include <stdlib.h>

inline float RandomFloat01()
{
	return (float)rand() / RAND_MAX;
}

inline int RandomInt(int a, int b)
{
	int p = a + (int)((b - a) * RandomFloat01());
	return p;
}

inline float RandomFloat(float a, float b)
{
	float p = a + (b - a) * RandomFloat01();
	return p;
}

inline bool FloatEqual(float v1, float v2)
{
	return fabsf(v1 - v2) < 1e-6;
}

template<typename T>
inline T	Clamp(const T X, const T Min, const T Max)
{
	return X < Min ? Min : X < Max ? X : Max;
}

inline bool	IsFloatInf(float x)
{
	return false;		// TODO
}
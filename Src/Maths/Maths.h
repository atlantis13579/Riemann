
#pragma once

int		RandomInt(int a, int b);
float	RandomFloat(float a, float b);
float	RandomFloat01();

template<typename T>
T		Clamp(const T X, const T Min, const T Max)
{
	return X < Min ? Min : X < Max ? X : Max;
}

#include <math.h>
#include <stdlib.h>
#include "Maths.h"

int RandomInt(int a, int b)
{
	int p = a + (int)((b - a) * ((double)rand() / RAND_MAX));
	return p;
}

float RandomFloat(float a, float b)
{
	float p = a + (b - a) * ((float)rand() / RAND_MAX);
	return p;
}

float RandomFloat01()
{
	return (float)rand() / RAND_MAX;
}

float SpringBack(float t, float faction)
{
	return powf(2.0f, -10.0f * t) * sinf(((t - 0.1f) * 1.570797f) / faction) + 1.0f;
}
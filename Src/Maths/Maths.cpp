
#include <math.h>
#include <stdlib.h>
#include "Maths.h"

int RandomInt(int a, int b)
{
	int p = a + (int)((b - a) * RandomFloat01());
	return p;
}

float RandomFloat01()
{
	return (float)rand() / RAND_MAX;
}

bool FloatEqual(float v1, float v2)
{
	return fabsf(v1 - v2) < 1e-6;
}

float RandomFloat(float a, float b)
{
	float p = a + (b - a) * RandomFloat01();
	return p;
}

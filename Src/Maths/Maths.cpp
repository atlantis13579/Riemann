
#include <math.h>
#include "Maths.h"

float SpringBack(float t, float faction)
{
	return powf(2.0f, -10.0f * t) * sinf(((t - 0.1f) * 1.570797f) / faction) + 1.0f;
}
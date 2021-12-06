
#pragma once

#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <limits>

#ifndef PI
#define PI				(3.1415926536f)
#endif // !PI

#define PI_2			(6.2831853072f)
#define PI_OVER_2		(1.5707963268f)
#define PI_OVER_3		(1.0471975512f)
#define PI_OVER_4		(0.7853981634f)
#define PI_OVER_6		(0.5235987756f)
#define PI_OVER_8		(0.3926990817f)
#define PI_OVER_180		(0.0174532925f)
#define RAD_TO_DEG		(57.295779513f)
#define SMALL_NUMBER	(1e-3f)
#define TINY_NUMBER		(1e-6f)

inline float ToRadian(float degree)
{
	return degree * PI_OVER_180;
}

inline float ToDegree(float radian)
{
	return radian * RAD_TO_DEG;
}

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

inline float FloatDiff(float v1, float v2)
{
	return fabsf(v1 - v2);
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

template<typename T>
inline T	Interp(const T v1, const T v2, float mix)
{
	return v1 * mix + v2 * (1.0f - mix);
}

inline bool	IsFloatInf(float x)
{
	return false;		// TODO
}

// http://www.matrix67.com/data/InvSqrt.pdf
inline float InvSqrt(float x) {
	float xhalf = 0.5f * x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	x = x * (1.5f - xhalf * x * x);
	return x;
}

// from http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
inline int BitCount(unsigned int v)
{
	unsigned int const w = v - ((v >> 1) & 0x55555555);
	unsigned int const x = (w & 0x33333333) + ((w >> 2) & 0x33333333);
	return (((x + (x >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

inline bool IsPowerOfTwo(int x)
{
	return x != 0 && (x & (x - 1)) == 0;
}

static const float kEpsilon = 0.000001f;
static const float kInfinite = 9999999.0f;

template <typename T>
inline T Epsilon(T a) {
	const T aa = std::abs(a) + (T)1;
	if (aa == std::numeric_limits<T>::infinity()) {
		return (T)kEpsilon;
	}
	else {
		return (T)kEpsilon * aa;
	}
}

template<typename T>
T Max(const T& p)
{
	return p;
}

template<typename T, typename ... Ts>
T Max(const T& p0, Ts... args)
{
	T p1 = Max(args...);
	return p0 > p1 ? p0 : p1;
}

template<typename T>
T Min(const T& p)
{
	return p;
}

template<typename T, typename ... Ts>
T Min(const T& p0, Ts... args)
{
	T p1 = Min(args...);
	return p0 < p1 ? p0 : p1;
}

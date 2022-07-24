
#pragma once

// #define SIMD_SIMULATOR

#if defined(SIMD_SIMULATOR)
	#include "Vector2.h"
	#include "Vector3.h"
	#include "Vector4.h"
	typedef Vector2d Scaler;
	typedef Vector3d Vec3;
	typedef Vector4d Vec4;
	typedef TVector4<bool> BVec4;
	typedef TVector4<unsigned int> UVec4;
	typedef TVector4<int> IVec4;
#elif defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    typedef float32x2_t Scaler;
    typedef float32x4_t Vec3;
    typedef float32x4_t Vec4;
    typedef uint32x4_t  BVec4;
    typedef uint32x4_t  UVec4;
    typedef int32x4_t   IVec4;
#else
    #ifdef _WIN32
    #include <intrin.h>
    #else
    #include <x86intrin.h>
    #endif
    typedef __m128 Scaler;
    typedef __m128 Vec3;
    typedef __m128 Vec4;
    typedef __m128 BVec4;
    typedef __m128 UVec4;
    typedef __m128 IVec4;
#endif

static_assert(sizeof(Vec3) == 16, "simd size vector not correct!");
static_assert(sizeof(Vec4) == 16, "simd size vector not correct!");
static_assert(sizeof(BVec4) == 16, "simd size vector not correct!");
static_assert(sizeof(UVec4) == 16, "simd size vector not correct!");
static_assert(sizeof(IVec4) == 16, "simd size vector not correct!");

#include <float.h>
#include "Maths.h"
#include "Vector3.h"
#include "Vector4.h"

inline float Vec4_ReadX(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).x;
}

inline float Vec4_ReadY(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).y;
}

inline float Vec4_ReadZ(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).z;
}

inline float Vec4_ReadW(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).w;
}

inline const Vector3d& Vec4_ReadXYZ(const Vec4& v)
{
	return reinterpret_cast<const Vector3d&>(v);
}

inline const Vector4d& Vec4_ReadXYZW(const Vec4& v)
{
    return reinterpret_cast<const Vector4d&>(v);
}

#if _DEBUG && 0
#define ASSERT_ISALIGNED16(a) assert(isAligned16((void*)a))
#else
#define ASSERT_ISALIGNED16(a)
#endif

#if defined(__arm__) || defined(__aarch64__)
#include "SIMD_Neon.h"
#elif defined(SIMD_SIMULATOR)
#include "SIMD_Simulator.h"
#else
#include "SIMD_SSE.h"
#endif



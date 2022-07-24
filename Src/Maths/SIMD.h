
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

inline Vec4 V4Sin(const Vec4 a)
{
	const Vec4 recipTwoPi = Vec4_Load(0.5f / 3.141592655f);
	const Vec4 twoPi = Vec4_Load(3.141592655f * 2.0f);
	const Vec4 tmp = Vec4_Mul(a, recipTwoPi);
	const Vec4 b = Vec4_Round(tmp);
	const Vec4 V1 = Vec4_NegMSub(twoPi, b, a);

	const Vec4 V2 = Vec4_Mul(V1, V1);
	const Vec4 V3 = Vec4_Mul(V2, V1);
	const Vec4 V5 = Vec4_Mul(V3, V2);
	const Vec4 V7 = Vec4_Mul(V5, V2);
	const Vec4 V9 = Vec4_Mul(V7, V2);
	const Vec4 V11 = Vec4_Mul(V9, V2);
	const Vec4 V13 = Vec4_Mul(V11, V2);
	const Vec4 V15 = Vec4_Mul(V13, V2);
	const Vec4 V17 = Vec4_Mul(V15, V2);
	const Vec4 V19 = Vec4_Mul(V17, V2);
	const Vec4 V21 = Vec4_Mul(V19, V2);
	const Vec4 V23 = Vec4_Mul(V21, V2);

	const Scaler S1 = Scaler_Load(-0.166666667f);
	const Scaler S2 = Scaler_Load(8.333333333e-3f);
	const Scaler S3 = Scaler_Load(-1.984126984e-4f);
	const Scaler S4 = Scaler_Load(2.755731922e-6f);
	const Scaler S5 = Scaler_Load(-2.505210839e-8f);
	const Scaler S6 = Scaler_Load(1.605904384e-10f);
	const Scaler S7 = Scaler_Load(-7.647163732e-13f);
	const Scaler S8 = Scaler_Load(2.811457254e-15f);
	const Scaler S9 = Scaler_Load(-8.220635247e-18f);
	const Scaler S10 = Scaler_Load(1.957294106e-20f);
	const Scaler S11 = Scaler_Load(-3.868170171e-23f);

	Vec4 Result;
	Result = Vec4_ScaleAdd(V3, S1, V1);
	Result = Vec4_ScaleAdd(V5, S2, Result);
	Result = Vec4_ScaleAdd(V7, S3, Result);
	Result = Vec4_ScaleAdd(V9, S4, Result);
	Result = Vec4_ScaleAdd(V11, S5, Result);
	Result = Vec4_ScaleAdd(V13, S6, Result);
	Result = Vec4_ScaleAdd(V15, S7, Result);
	Result = Vec4_ScaleAdd(V17, S8, Result);
	Result = Vec4_ScaleAdd(V19, S9, Result);
	Result = Vec4_ScaleAdd(V21, S10, Result);
	Result = Vec4_ScaleAdd(V23, S11, Result);

	return Result;
}

inline Vec4 V4Cos(const Vec4 a)
{
	const Vec4 recipTwoPi = Vec4_Load(0.5f / 3.141592655f);
	const Vec4 twoPi = Vec4_Load(3.141592655f * 2.0f);
	const Vec4 tmp = Vec4_Mul(a, recipTwoPi);
	const Vec4 b = Vec4_Round(tmp);
	const Vec4 V1 = Vec4_NegMSub(twoPi, b, a);

	const Vec4 V2 = Vec4_Mul(V1, V1);
	const Vec4 V4 = Vec4_Mul(V2, V2);
	const Vec4 V6 = Vec4_Mul(V4, V2);
	const Vec4 V8 = Vec4_Mul(V4, V4);
	const Vec4 V10 = Vec4_Mul(V6, V4);
	const Vec4 V12 = Vec4_Mul(V6, V6);
	const Vec4 V14 = Vec4_Mul(V8, V6);
	const Vec4 V16 = Vec4_Mul(V8, V8);
	const Vec4 V18 = Vec4_Mul(V10, V8);
	const Vec4 V20 = Vec4_Mul(V10, V10);
	const Vec4 V22 = Vec4_Mul(V12, V10);

	const Scaler C1 = Scaler_Load(-0.5f);
	const Scaler C2 = Scaler_Load(4.166666667e-2f);
	const Scaler C3 = Scaler_Load(-1.388888889e-3f);
	const Scaler C4 = Scaler_Load(2.480158730e-5f);
	const Scaler C5 = Scaler_Load(-2.755731922e-7f);
	const Scaler C6 = Scaler_Load(2.087675699e-9f);
	const Scaler C7 = Scaler_Load(-1.147074560e-11f);
	const Scaler C8 = Scaler_Load(4.779477332e-14f);
	const Scaler C9 = Scaler_Load(-1.561920697e-16f);
	const Scaler C10 = Scaler_Load(4.110317623e-19f);
	const Scaler C11 = Scaler_Load(-8.896791392e-22f);

	Vec4 Result;
	Result = Vec4_ScaleAdd(V2, C1, Vec4_One());
	Result = Vec4_ScaleAdd(V4, C2, Result);
	Result = Vec4_ScaleAdd(V6, C3, Result);
	Result = Vec4_ScaleAdd(V8, C4, Result);
	Result = Vec4_ScaleAdd(V10, C5, Result);
	Result = Vec4_ScaleAdd(V12, C6, Result);
	Result = Vec4_ScaleAdd(V14, C7, Result);
	Result = Vec4_ScaleAdd(V16, C8, Result);
	Result = Vec4_ScaleAdd(V18, C9, Result);
	Result = Vec4_ScaleAdd(V20, C10, Result);
	Result = Vec4_ScaleAdd(V22, C11, Result);

	return Result;
}



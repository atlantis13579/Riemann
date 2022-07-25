
#pragma once

// #define SIMD_SIMULATOR

#if defined(SIMD_SIMULATOR)
	#include "Vector2.h"
	#include "Vector3.h"
	#include "Vector4.h"
	typedef float				Scaler;
	typedef Vector4			F128;
	typedef TVector4<bool>		B128;
	typedef TVector4<uint32_t>	U128;
	typedef TVector4<int>		I128;
#elif defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    typedef float32x2_t Scaler;
    typedef float32x4_t F128;
    typedef uint32x4_t  B128;
    typedef uint32x4_t  U128;
    typedef int32x4_t   I128;
#else
    #ifdef _WIN32
    #include <intrin.h>
    #else
    #include <x86intrin.h>
    #endif
    typedef __m128 Scaler;
    typedef __m128 F128;
    typedef __m128 B128;
    typedef __m128 U128;
    typedef __m128 I128;
#endif

static_assert(sizeof(F128) == 16, "simd size vector not correct!");
static_assert(sizeof(B128) == 16, "simd size vector not correct!");
static_assert(sizeof(U128) == 16, "simd size vector not correct!");
static_assert(sizeof(I128) == 16, "simd size vector not correct!");

#include <float.h>
#include "Maths.h"
#include "Vector3.h"
#include "Vector4.h"

inline float F128_ReadX(const F128& v)
{
	return reinterpret_cast<const Vector4&>(v).x;
}

inline float F128_ReadY(const F128& v)
{
	return reinterpret_cast<const Vector4&>(v).y;
}

inline float F128_ReadZ(const F128& v)
{
	return reinterpret_cast<const Vector4&>(v).z;
}

inline float F128_ReadW(const F128& v)
{
	return reinterpret_cast<const Vector4&>(v).w;
}

inline const Vector3& F128_ReadXYZ(const F128& v)
{
	return reinterpret_cast<const Vector3&>(v);
}

inline const Vector4& F128_ReadXYZW(const F128& v)
{
    return reinterpret_cast<const Vector4&>(v);
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

inline F128 operator-(const F128 f)
{
	return F128_Neg(f);
}

inline F128 operator+(const F128 a, const F128 b)
{
	return F128_Add(a, b);
}

inline F128 operator-(const F128 a, const F128 b)
{
	return F128_Sub(a, b);
}

inline F128 operator*(const F128 a, const F128 b)
{
	return F128_Mul(a, b);
}

inline F128 operator/(const F128 a, const F128 b)
{
	return F128_Div(a, b);
}

inline B128 operator>(const F128 a, const F128 b)
{
	return F128_Greater(a, b);
}

inline B128 operator>=(const F128 a, const F128 b)
{
	return F128_GreaterEqual(a, b);
}

inline B128 operator==(const F128 a, const F128 b)
{
	return F128_Equal(a, b);
}

inline B128 operator&&(const B128 a, const B128 b)
{
	return B128_And(a, b);
}

inline B128 operator!(const B128 a)
{
	return B128_Not(a);
}

inline B128 operator||(const B128 a, const B128 b)
{
	return B128_Or(a, b);
}

inline U128 operator|(U128 a, U128 b)
{
	return U128_OR(a, b);
}

inline U128 operator^(U128 a, U128 b)
{
	return U128_XOR(a, b);
}

inline U128 operator&(U128 a, U128 b)
{
	return U128_AND(a, b);
}

inline F128 V4Sin(const F128 a)
{
	const F128 recipTwoPi = F128_Load(0.5f / 3.141592655f);
	const F128 twoPi = F128_Load(3.141592655f * 2.0f);
	const F128 tmp = a * recipTwoPi;
	const F128 b = F128_Round(tmp);
	const F128 V1 = F128_NegMSub(twoPi, b, a);

	const F128 V2 = V1 * V1;
	const F128 V3 = V2 * V1;
	const F128 V5 = V3 * V2;
	const F128 V7 = V5 * V2;
	const F128 V9 = V7 * V2;
	const F128 V11 = V9 * V2;
	const F128 V13 = V11 * V2;
	const F128 V15 = V13 * V2;
	const F128 V17 = V15 * V2;
	const F128 V19 = V17 * V2;
	const F128 V21 = V19 * V2;
	const F128 V23 = V21 * V2;

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

	F128 Result;
	Result = F128_ScaleAdd(V3, S1, V1);
	Result = F128_ScaleAdd(V5, S2, Result);
	Result = F128_ScaleAdd(V7, S3, Result);
	Result = F128_ScaleAdd(V9, S4, Result);
	Result = F128_ScaleAdd(V11, S5, Result);
	Result = F128_ScaleAdd(V13, S6, Result);
	Result = F128_ScaleAdd(V15, S7, Result);
	Result = F128_ScaleAdd(V17, S8, Result);
	Result = F128_ScaleAdd(V19, S9, Result);
	Result = F128_ScaleAdd(V21, S10, Result);
	Result = F128_ScaleAdd(V23, S11, Result);

	return Result;
}

inline F128 V4Cos(const F128 a)
{
	const F128 recipTwoPi = F128_Load(0.5f / 3.141592655f);
	const F128 twoPi = F128_Load(3.141592655f * 2.0f);
	const F128 tmp = a * recipTwoPi;
	const F128 b = F128_Round(tmp);
	const F128 V1 = F128_NegMSub(twoPi, b, a);

	const F128 V2 = V1 * V1;
	const F128 V4 = V2 * V2;
	const F128 V6 = V4 * V2;
	const F128 V8 = V4 * V4;
	const F128 V10 = V6 * V4;
	const F128 V12 = V6 * V6;
	const F128 V14 = V8 * V6;
	const F128 V16 = V8 * V8;
	const F128 V18 = V10 * V8;
	const F128 V20 = V10 * V10;
	const F128 V22 = V12 * V10;

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

	F128 Result;
	Result = F128_ScaleAdd(V2, C1, F128_One());
	Result = F128_ScaleAdd(V4, C2, Result);
	Result = F128_ScaleAdd(V6, C3, Result);
	Result = F128_ScaleAdd(V8, C4, Result);
	Result = F128_ScaleAdd(V10, C5, Result);
	Result = F128_ScaleAdd(V12, C6, Result);
	Result = F128_ScaleAdd(V14, C7, Result);
	Result = F128_ScaleAdd(V16, C8, Result);
	Result = F128_ScaleAdd(V18, C9, Result);
	Result = F128_ScaleAdd(V20, C10, Result);
	Result = F128_ScaleAdd(V22, C11, Result);

	return Result;
}



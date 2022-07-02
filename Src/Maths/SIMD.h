
#pragma once

// #define SIMD_REF

#if defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    typedef float32x4_t float_v;
    //typedef __m128 vec3_v;
    //typedef __m128 vec4_v;
    //typedef __m128 bool_v;
    //typedef __m128 uint32_v;
    //typedef __m128 int32_v;
    //typedef __m128 uint16_v;
    //typedef __m128 int16_v;
#elif defined(SIMD_REF)

#else
    #ifdef _WIN32
    #include <intrin.h>
    #else
    #include <x86intrin.h>
    #endif
    typedef __m128 FloatV;
    typedef __m128 Vec3V;
    typedef __m128 Vec4V;
    typedef __m128 BoolV;
    typedef __m128 VecU32V;
    typedef __m128 VecI32V;
    typedef __m128 VecU16V;
    typedef __m128 VecI16V;
#endif
static_assert(sizeof(FloatV) == 16, "simd size not correct!");
static_assert(sizeof(Vec3V) == 16, "simd size not correct!");
static_assert(sizeof(Vec4V) == 16, "simd size not correct!");
static_assert(sizeof(BoolV) == 16, "simd size not correct!");
static_assert(sizeof(VecU32V) == 16, "simd size not correct!");
static_assert(sizeof(VecI32V) == 16, "simd size not correct!");
static_assert(sizeof(VecU16V) == 16, "simd size not correct!");
static_assert(sizeof(VecI16V) == 16, "simd size not correct!");

#include <float.h>
#include "Maths.h"
#include "Vector3d.h"
#include "Vector4d.h"

inline void V3WriteX(Vec3V& v, float f)
{
	reinterpret_cast<Vector3d&>(v).x = f;
}

inline void V3WriteY(Vec3V& v, float f)
{
	reinterpret_cast<Vector3d&>(v).y = f;
}

inline void V3WriteZ(Vec3V& v, float f)
{
	reinterpret_cast<Vector3d&>(v).z = f;
}

inline void V3WriteXYZ(Vec3V& v, const Vector3d& f)
{
	reinterpret_cast<Vector3d&>(v) = f;
}

inline float V3ReadX(const Vec3V& v)
{
	return reinterpret_cast<const Vector3d&>(v).x;
}

inline float V3ReadY(const Vec3V& v)
{
	return reinterpret_cast<const Vector3d&>(v).y;
}

inline float V3ReadZ(const Vec3V& v)
{
	return reinterpret_cast<const Vector3d&>(v).z;
}

inline const Vector3d& V3ReadXYZ(const Vec3V& v)
{
	return reinterpret_cast<const Vector3d&>(v);
}

inline void V4WriteX(Vec4V& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).x = f;
}

inline void V4WriteY(Vec4V& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).y = f;
}

inline void V4WriteZ(Vec4V& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).z = f;
}

inline void V4WriteW(Vec4V& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).w = f;
}

inline void V4WriteXYZ(Vec4V& v, const Vector3d& f)
{
	reinterpret_cast<Vector3d&>(v) = f;
}

inline float V4ReadX(const Vec4V& v)
{
	return reinterpret_cast<const Vector4d&>(v).x;
}

inline float V4ReadY(const Vec4V& v)
{
	return reinterpret_cast<const Vector4d&>(v).y;
}

inline float V4ReadZ(const Vec4V& v)
{
	return reinterpret_cast<const Vector4d&>(v).z;
}

inline float V4ReadW(const Vec4V& v)
{
	return reinterpret_cast<const Vector4d&>(v).w;
}

inline const Vector3d& V4ReadXYZ(const Vec4V& v)
{
	return reinterpret_cast<const Vector3d&>(v);
}


#if _DEBUG && 0
#define ASSERT_ISVALIDVEC3V(a) assert(isValidVec3V(a))
#define ASSERT_ISVALIDFLOATV(a) assert(isValidFloatV(a))
#define ASSERT_ISALIGNED16(a) assert(isAligned16((void*)a))
#define ASSERT_ISFINITELENGTH(a) //PX_ASSERT(isFiniteLength(a))
#else
#define ASSERT_ISVALIDVEC3V(a)
#define ASSERT_ISVALIDFLOATV(a)
#define ASSERT_ISALIGNED16(a)
#define ASSERT_ISFINITELENGTH(a)
#endif

#if defined(__arm__) || defined(__aarch64__)
#include "SIMD_Neon.h"
#elif defined(SIMD_REF)
#inlcude "SIMD_Ref.h"
#else
#include "SIMD_Intel.h"
#endif

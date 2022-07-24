
#pragma once

// #define SIMD_SIMULATOR

#if defined(SIMD_SIMULATOR)
	#include "Vector2.h"
	#include "Vector3.h"
	#include "Vector4.h"
	typedef Vector2d FloatV;
	typedef Vector3d Vec3;
	typedef Vector4d Vec4;
	typedef TVector4<bool> BVec4;
	typedef TVector4<unsigned int> UVec4;
	typedef TVector4<int> IVec4;
#elif defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    typedef float32x2_t FloatV;
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
    typedef __m128 FloatV;
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

inline void V3WriteX(Vec3& v, float f)
{
	reinterpret_cast<Vector3d&>(v).x = f;
}

inline void V3WriteY(Vec3& v, float f)
{
	reinterpret_cast<Vector3d&>(v).y = f;
}

inline void V3WriteZ(Vec3& v, float f)
{
	reinterpret_cast<Vector3d&>(v).z = f;
}

inline void V3WriteXYZ(Vec3& v, const Vector3d& f)
{
	reinterpret_cast<Vector3d&>(v) = f;
}

inline float V3ReadX(const Vec3& v)
{
	return reinterpret_cast<const Vector3d&>(v).x;
}

inline float V3ReadY(const Vec3& v)
{
	return reinterpret_cast<const Vector3d&>(v).y;
}

inline float V3ReadZ(const Vec3& v)
{
	return reinterpret_cast<const Vector3d&>(v).z;
}

inline const Vector3d& V3ReadXYZ(const Vec3& v)
{
	return reinterpret_cast<const Vector3d&>(v);
}

inline void V4WriteX(Vec4& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).x = f;
}

inline void V4WriteY(Vec4& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).y = f;
}

inline void V4WriteZ(Vec4& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).z = f;
}

inline void V4WriteW(Vec4& v, const float f)
{
	reinterpret_cast<Vector4d&>(v).w = f;
}

inline void V4WriteXYZ(Vec4& v, const Vector3d& f)
{
	reinterpret_cast<Vector3d&>(v) = f;
}

inline float V4ReadX(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).x;
}

inline float V4ReadY(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).y;
}

inline float V4ReadZ(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).z;
}

inline float V4ReadW(const Vec4& v)
{
	return reinterpret_cast<const Vector4d&>(v).w;
}

inline const Vector3d& V4ReadXYZ(const Vec4& v)
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
#elif defined(SIMD_SIMULATOR)
#include "SIMD_Simulator.h"
#else
#include "SIMD_SSE.h"
#endif



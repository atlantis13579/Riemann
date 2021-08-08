
#pragma once

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
typedef __m128 QuatV;

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

namespace internalWindowsSimd
{
inline __m128 m128_I2F(__m128i n)
{
	return _mm_castsi128_ps(n);
}

inline __m128i m128_F2I(__m128 n)
{
	return _mm_castps_si128(n);
}

inline uint32_t BAllTrue4_R(const BoolV a)
{
	const int moveMask = _mm_movemask_ps(a);
	return uint32_t(moveMask == 0xf);
}

inline uint32_t BAllTrue3_R(const BoolV a)
{
	const int moveMask = _mm_movemask_ps(a);
	return uint32_t((moveMask & 0x7) == 0x7);
}

inline uint32_t BAnyTrue4_R(const BoolV a)
{
	const int moveMask = _mm_movemask_ps(a);
	return uint32_t(moveMask != 0x0);
}

inline uint32_t BAnyTrue3_R(const BoolV a)
{
	const int moveMask = _mm_movemask_ps(a);
	return uint32_t(((moveMask & 0x7) != 0x0));
}

const alignas(16) uint32_t gMaskXYZ[4] = { 0xffffffff, 0xffffffff, 0xffffffff, 0 };
} //internalWindowsSimd

/////////////////////////////////////////////////////////////////////
////VECTORISED FUNCTION IMPLEMENTATIONS
/////////////////////////////////////////////////////////////////////


inline FloatV V4GetW(const Vec4V f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(3, 3, 3, 3));
}

inline FloatV V4GetX(const Vec4V f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0, 0, 0, 0));
}

inline FloatV V4GetY(const Vec4V f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(1, 1, 1, 1));
}

inline FloatV V4GetZ(const Vec4V f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 2, 2, 2));
}

inline Vec4V V4ClearW(const Vec4V v)
{
	return _mm_and_ps(v, (VecI32V&)internalWindowsSimd::gMaskXYZ);
}

inline FloatV FLoad(const float f)
{
	return _mm_load1_ps(&f);
}

inline Vec3V V3Load(const float f)
{
	return _mm_set_ps(0.0f, f, f, f);
}

inline Vec4V V4Load(const float f)
{
	return _mm_load1_ps(&f);
}

inline BoolV BLoad(const bool f)
{
	const uint32_t i = uint32_t(-(int)f);
	return _mm_load1_ps((float*)&i);
}

inline Vec3V V3LoadA(const Vector3d& f)
{
	ASSERT_ISALIGNED16(&f);
	return _mm_and_ps(_mm_load_ps(&f.x), reinterpret_cast<const Vec4V&>(internalWindowsSimd::gMaskXYZ));
}

inline Vec3V V3LoadU(const Vector3d& f)
{
	return _mm_set_ps(0.0f, f.z, f.y, f.x);
}

// w component of result is undefined
inline Vec3V V3LoadUnsafeA(const Vector3d& f)
{
	ASSERT_ISALIGNED16(&f);
	return _mm_load_ps(&f.x);
}

inline Vec3V V3LoadA(const float* const f)
{
	ASSERT_ISALIGNED16(f);
	return V4ClearW(_mm_load_ps(f));
}

inline Vec3V V3LoadU(const float* const i)
{
	return _mm_set_ps(0.0f, i[2], i[1], i[0]);
}

inline Vec3V Vec3V_From_Vec4V(Vec4V v)
{
	return V4ClearW(v);
}

inline Vec3V Vec3V_From_Vec4V_WUndefined(const Vec4V v)
{
	return v;
}

inline Vec4V Vec4V_From_Vec3V(Vec3V f)
{
	return f; // ok if it is implemented as the same type.
}

inline Vec4V Vec4V_From_FloatV(FloatV f)
{
	return f;
}

inline Vec3V Vec3V_From_FloatV(FloatV f)
{
	return Vec3V_From_Vec4V(Vec4V_From_FloatV(f));
}

inline Vec3V Vec3V_From_FloatV_WUndefined(FloatV f)
{
	return Vec3V_From_Vec4V_WUndefined(Vec4V_From_FloatV(f));
}

inline Vec4V Vec4V_From_Vector3d_WUndefined(const Vector3d& f)
{
	return _mm_set_ps(0.0f, f.z, f.y, f.x);
}

inline Vec4V V4LoadA(const float* const f)
{
	ASSERT_ISALIGNED16(f);
	return _mm_load_ps(f);
}

inline void V4StoreA(const Vec4V a, float* f)
{
	ASSERT_ISALIGNED16(f);
	_mm_store_ps(f, a);
}

inline void V4StoreU(const Vec4V a, float* f)
{
	_mm_storeu_ps(f, a);
}

inline void BStoreA(const BoolV a, uint32_t* f)
{
	ASSERT_ISALIGNED16(f);
	_mm_store_ps((float*)f, a);
}

inline void U4StoreA(const VecU32V uv, uint32_t* u)
{
	ASSERT_ISALIGNED16(u);
	_mm_store_ps((float*)u, uv);
}

inline void I4StoreA(const VecI32V iv, int* i)
{
	ASSERT_ISALIGNED16(i);
	_mm_store_ps((float*)i, iv);
}

inline Vec4V V4LoadU(const float* const f)
{
	return _mm_loadu_ps(f);
}

inline void FStore(const FloatV a, float* f)
{
	ASSERT_ISVALIDFLOATV(a);
	_mm_store_ss(f, a);
}

inline void V3StoreA(const Vec3V a, Vector3d& f)
{
	ASSERT_ISALIGNED16(&f);
	alignas(16) float f2[4];
	_mm_store_ps(f2, a);
	f = Vector3d(f2[0], f2[1], f2[2]);
}

inline void Store_From_BoolV(const BoolV b, uint32_t* b2)
{
	_mm_store_ss((float*)b2, b);
}

inline void V3StoreU(const Vec3V a, Vector3d& f)
{
	alignas(16) float f2[4];
	_mm_store_ps(f2, a);
	f = Vector3d(f2[0], f2[1], f2[2]);
}

//////////////////////////////////
// FLOATV
//////////////////////////////////

inline FloatV FZero()
{
	return _mm_setzero_ps();
}

inline FloatV FOne()
{
	return FLoad(1.0f);
}

inline FloatV FHalf()
{
	return FLoad(0.5f);
}

inline FloatV FEps()
{
	return FLoad(1e-7f);
}

inline FloatV FEps6()
{
	return FLoad(1e-6f);
}

inline FloatV FMax()
{
	return FLoad(FLT_MAX);
}

inline FloatV FNegMax()
{
	return FLoad(-FLT_MAX);
}

inline FloatV IZero()
{
	const uint32_t zero = 0;
	return _mm_load1_ps((float*)&zero);
}

inline FloatV IOne()
{
	const uint32_t one = 1;
	return _mm_load1_ps((float*)&one);
}

inline FloatV ITwo()
{
	const uint32_t two = 2;
	return _mm_load1_ps((float*)&two);
}

inline FloatV IThree()
{
	const uint32_t three = 3;
	return _mm_load1_ps((float*)&three);
}

inline FloatV IFour()
{
	const uint32_t four = 4;
	return _mm_load1_ps((float*)&four);
}

inline FloatV FNeg(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline FloatV FAdd(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_add_ps(a, b);
}

inline FloatV FSub(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_sub_ps(a, b);
}

inline FloatV FMul(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_mul_ps(a, b);
}

inline FloatV FDiv(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_div_ps(a, b);
}

inline FloatV FDivFast(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline FloatV FRecip(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return _mm_div_ps(FOne(), a);
}

inline FloatV FRecipFast(const FloatV a)
{
	return _mm_rcp_ps(a);
}

inline FloatV FRsqrt(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return _mm_div_ps(FOne(), _mm_sqrt_ps(a));
}

inline FloatV FSqrt(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return _mm_sqrt_ps(a);
}

inline FloatV FRsqrtFast(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return _mm_rsqrt_ps(a);
}

inline FloatV FScaleAdd(const FloatV a, const FloatV b, const FloatV c)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDFLOATV(c);
	return FAdd(FMul(a, b), c);
}

inline FloatV FNegScaleSub(const FloatV a, const FloatV b, const FloatV c)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDFLOATV(c);
	return FSub(c, FMul(a, b));
}

inline FloatV FAbs(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	alignas(16) const static uint32_t absMask[4] = { 0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF };
	return _mm_and_ps(a, _mm_load_ps((float*)absMask));
}

inline FloatV FSel(const BoolV c, const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(_mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a)));
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline BoolV FIsGrtr(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_cmpgt_ps(a, b);
}

inline BoolV FIsGrtrOrEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_cmpge_ps(a, b);
}

inline BoolV FIsEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_cmpeq_ps(a, b);
}

inline FloatV FMax(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_max_ps(a, b);
}

inline FloatV FMin(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_min_ps(a, b);
}

inline FloatV FClamp(const FloatV a, const FloatV minV, const FloatV maxV)
{
	ASSERT_ISVALIDFLOATV(minV);
	ASSERT_ISVALIDFLOATV(maxV);
	return _mm_max_ps(_mm_min_ps(a, maxV), minV);
}

inline uint32_t FAllGrtr(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return uint32_t(_mm_comigt_ss(a, b));
}

inline uint32_t FAllGrtrOrEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return uint32_t(_mm_comige_ss(a, b));
}

inline uint32_t FAllEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return uint32_t(_mm_comieq_ss(a, b));
}

inline FloatV FRound(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	// return _mm_round_ps(a, 0x0);
	const FloatV half = FLoad(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const FloatV aRound = FSub(FAdd(a, half), signBit);
	__m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}


//////////////////////////////////
// BoolV
//////////////////////////////////

inline BoolV BFFFF()
{
	return _mm_setzero_ps();
}

inline BoolV BFFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0,0xFFFFFFFF};
	const __m128 ffft=_mm_load_ps((float*)&f);
	return ffft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, 0));
}

inline BoolV BFFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0};
	const __m128 fftf=_mm_load_ps((float*)&f);
	return fftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, 0));
}

inline BoolV BFFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fftt=_mm_load_ps((float*)&f);
	return fftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, 0));
}

inline BoolV BFTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0};
	const __m128 ftff=_mm_load_ps((float*)&f);
	return ftff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, 0));
}

inline BoolV BFTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ftft=_mm_load_ps((float*)&f);
	return ftft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, 0));
}

inline BoolV BFTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 fttf=_mm_load_ps((float*)&f);
	return fttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, 0));
}

inline BoolV BFTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fttt=_mm_load_ps((float*)&f);
	return fttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, 0));
}

inline BoolV BTFFF()
{
	// const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0};
	// const __m128 tfff=_mm_load_ps((float*)&f);
	// return tfff;
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, 0, -1));
}

inline BoolV BTFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0xFFFFFFFF};
	const __m128 tfft=_mm_load_ps((float*)&f);
	return tfft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, -1));
}

inline BoolV BTFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0};
	const __m128 tftf=_mm_load_ps((float*)&f);
	return tftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, -1));
}

inline BoolV BTFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tftt=_mm_load_ps((float*)&f);
	return tftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, -1));
}

inline BoolV BTTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0};
	const __m128 ttff=_mm_load_ps((float*)&f);
	return ttff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, -1));
}

inline BoolV BTTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ttft=_mm_load_ps((float*)&f);
	return ttft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, -1));
}

inline BoolV BTTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 tttf=_mm_load_ps((float*)&f);
	return tttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, -1));
}

inline BoolV BTTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tttt=_mm_load_ps((float*)&f);
	return tttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, -1));
}

inline BoolV BXMask()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0};
	const __m128 tfff=_mm_load_ps((float*)&f);
	return tfff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, 0, -1));
}

inline BoolV BYMask()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0};
	const __m128 ftff=_mm_load_ps((float*)&f);
	return ftff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, 0));
}

inline BoolV BZMask()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0};
	const __m128 fftf=_mm_load_ps((float*)&f);
	return fftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, 0));
}

inline BoolV BWMask()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0,0xFFFFFFFF};
	const __m128 ffft=_mm_load_ps((float*)&f);
	return ffft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, 0));
}


inline Vec4V V4Zero()
{
	return _mm_setzero_ps();
}

inline Vec4V V4One()
{
	return V4Load(1.0f);
}

inline Vec4V V4Eps()
{
	return V4Load(1e-7f);
}

inline Vec4V V4Neg(const Vec4V f)
{
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline Vec4V V4Add(const Vec4V a, const Vec4V b)
{
	return _mm_add_ps(a, b);
}

inline Vec4V V4Sub(const Vec4V a, const Vec4V b)
{
	return _mm_sub_ps(a, b);
}

inline Vec4V V4Scale(const Vec4V a, const FloatV b)
{
	return _mm_mul_ps(a, b);
}

inline Vec4V V4Mul(const Vec4V a, const Vec4V b)
{
	return _mm_mul_ps(a, b);
}

inline Vec4V V4ScaleInv(const Vec4V a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	return _mm_div_ps(a, b);
}

inline Vec4V V4Div(const Vec4V a, const Vec4V b)
{
	return _mm_div_ps(a, b);
}

inline Vec4V V4ScaleInvFast(const Vec4V a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Vec4V V4DivFast(const Vec4V a, const Vec4V b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Vec4V V4Recip(const Vec4V a)
{
	return _mm_div_ps(V4One(), a);
}

inline Vec4V V4RecipFast(const Vec4V a)
{
	return _mm_rcp_ps(a);
}

inline Vec4V V4Rsqrt(const Vec4V a)
{
	return _mm_div_ps(V4One(), _mm_sqrt_ps(a));
}

inline Vec4V V4RsqrtFast(const Vec4V a)
{
	return _mm_rsqrt_ps(a);
}

inline Vec4V V4Sqrt(const Vec4V a)
{
	return _mm_sqrt_ps(a);
}

inline Vec4V V4ScaleAdd(const Vec4V a, const FloatV b, const Vec4V c)
{
	ASSERT_ISVALIDFLOATV(b);
	return V4Add(V4Scale(a, b), c);
}

inline Vec4V V4NegScaleSub(const Vec4V a, const FloatV b, const Vec4V c)
{
	ASSERT_ISVALIDFLOATV(b);
	return V4Sub(c, V4Scale(a, b));
}

inline Vec4V V4MulAdd(const Vec4V a, const Vec4V b, const Vec4V c)
{
	return V4Add(V4Mul(a, b), c);
}

inline Vec4V V4NegMulSub(const Vec4V a, const Vec4V b, const Vec4V c)
{
	return V4Sub(c, V4Mul(a, b));
}

inline Vec4V V4Max(const Vec4V a, const Vec4V b)
{
	return _mm_max_ps(a, b);
}

inline Vec4V V4Min(const Vec4V a, const Vec4V b)
{
	return _mm_min_ps(a, b);
}


inline Vec4V V4Abs(const Vec4V a)
{
	return V4Max(a, V4Neg(a));
}

inline FloatV V4Dot(const Vec4V a, const Vec4V b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // x,y,z,w
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 1, 0, 3)); // w,x,y,z
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 0, 3, 2)); // z,w,x,y
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 3, 2, 1)); // y,z,w,x
	return _mm_add_ps(_mm_add_ps(shuf2, shuf3), _mm_add_ps(dot1, shuf1));
}

inline FloatV V4Dot3(const Vec4V a, const Vec4V b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // aw*bw | az*bz | ay*by | ax*bx
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 0, 0, 0)); // ax*bx | ax*bx | ax*bx | ax*bx
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 1, 1, 1)); // ay*by | ay*by | ay*by | ay*by
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 2, 2, 2)); // az*bz | az*bz | az*bz | az*bz
	return _mm_add_ps(_mm_add_ps(shuf1, shuf2), shuf3);                       // ax*bx + ay*by + az*bz in each component
}

inline Vec4V V4Cross(const Vec4V a, const Vec4V b)
{
	const __m128 r1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	const __m128 r2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	return _mm_sub_ps(_mm_mul_ps(l1, l2), _mm_mul_ps(r1, r2));
}

inline FloatV V4Length(const Vec4V a)
{
	return _mm_sqrt_ps(V4Dot(a, a));
}

inline FloatV V4LengthSq(const Vec4V a)
{
	return V4Dot(a, a);
}

inline Vec4V V4Normalize(const Vec4V a)
{
	ASSERT_ISFINITELENGTH(a);
	return V4ScaleInv(a, _mm_sqrt_ps(V4Dot(a, a)));
}

inline Vec4V V4NormalizeFast(const Vec4V a)
{
	ASSERT_ISFINITELENGTH(a);
	return V4ScaleInvFast(a, _mm_sqrt_ps(V4Dot(a, a)));
}

inline Vec4V V4Sel(const BoolV c, const Vec4V a, const Vec4V b)
{
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline BoolV V4IsGrtr(const Vec4V a, const Vec4V b)
{
	return _mm_cmpgt_ps(a, b);
}

inline BoolV V4IsGrtrOrEq(const Vec4V a, const Vec4V b)
{
	return _mm_cmpge_ps(a, b);
}

inline BoolV V4IsEq(const Vec4V a, const Vec4V b)
{
	return _mm_cmpeq_ps(a, b);
}

inline FloatV V4ExtractMax(const Vec4V a)
{
	const __m128 shuf1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	const __m128 shuf2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	const __m128 shuf3 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));

	return _mm_max_ps(_mm_max_ps(a, shuf1), _mm_max_ps(shuf2, shuf3));
}

inline FloatV V4ExtractMin(const Vec4V a)
{
	const __m128 shuf1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	const __m128 shuf2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	const __m128 shuf3 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));

	return _mm_min_ps(_mm_min_ps(a, shuf1), _mm_min_ps(shuf2, shuf3));
}

inline Vec4V V4Clamp(const Vec4V a, const Vec4V minV, const Vec4V maxV)
{
	return V4Max(V4Min(a, maxV), minV);
}


//////////////////////////////////
// VEC3V
//////////////////////////////////

inline Vec3V V3UnitX()
{
	const alignas(16) float x[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	const __m128 x128 = _mm_load_ps(x);
	return x128;
}

inline Vec3V V3UnitY()
{
	const alignas(16) float y[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
	const __m128 y128 = _mm_load_ps(y);
	return y128;
}

inline Vec3V V3UnitZ()
{
	const alignas(16) float z[4] = { 0.0f, 0.0f, 1.0f, 0.0f };
	const __m128 z128 = _mm_load_ps(z);
	return z128;
}

inline FloatV V3GetX(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0, 0, 0, 0));
}

inline FloatV V3GetY(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(1, 1, 1, 1));
}

inline FloatV V3GetZ(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 2, 2, 2));
}

inline Vec3V V3SetX(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BFTTT(), v, f);
}

inline Vec3V V3SetY(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTFTT(), v, f);
}

inline Vec3V V3SetZ(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTFT(), v, f);
}

inline Vec3V V3ColX(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	Vec3V r = _mm_shuffle_ps(a, c, _MM_SHUFFLE(3, 0, 3, 0));
	return V3SetY(r, V3GetX(b));
}

inline Vec3V V3ColY(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	Vec3V r = _mm_shuffle_ps(a, c, _MM_SHUFFLE(3, 1, 3, 1));
	return V3SetY(r, V3GetY(b));
}

inline Vec3V V3ColZ(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	Vec3V r = _mm_shuffle_ps(a, c, _MM_SHUFFLE(3, 2, 3, 2));
	return V3SetY(r, V3GetZ(b));
}

inline Vec3V V3Zero()
{
	return _mm_setzero_ps();
}

inline Vec3V V3One()
{
	return V3Load(1.0f);
}

inline Vec3V V3Eps()
{
	return V3Load(1e-7f);
}

inline Vec3V V3Neg(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline Vec3V V3Add(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_add_ps(a, b);
}

inline Vec3V V3Sub(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_sub_ps(a, b);
}

inline Vec3V V3Scale(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_mul_ps(a, b);
}

inline Vec3V V3Mul(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_mul_ps(a, b);
}

inline Vec3V V3ScaleInv(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_div_ps(a, b);
}

inline VecU32V VecU32V_From_BoolV(const BoolV a)
{
	return a;
}

inline Vec3V V3Div(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return V4ClearW(_mm_div_ps(a, b));
}

inline Vec3V V3ScaleInvFast(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Vec3V V3DivFast(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return V4ClearW(_mm_mul_ps(a, _mm_rcp_ps(b)));
}

inline Vec3V V3Splat(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	const __m128 zero = V3Zero();
	const __m128 fff0 = _mm_move_ss(f, zero);
	return _mm_shuffle_ps(fff0, fff0, _MM_SHUFFLE(0, 1, 2, 3));
}

inline Vec3V V3Recip(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 zero = V3Zero();
	const __m128 tttf = BTTTF();
	const __m128 recipA = _mm_div_ps(V3One(), a);
	return V4Sel(tttf, recipA, zero);
}

inline Vec3V V3RecipFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 zero = V3Zero();
	const __m128 tttf = BTTTF();
	const __m128 recipA = _mm_rcp_ps(a);
	return V4Sel(tttf, recipA, zero);
}

inline Vec3V V3Rsqrt(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 zero = V3Zero();
	const __m128 tttf = BTTTF();
	const __m128 recipA = _mm_div_ps(V3One(), _mm_sqrt_ps(a));
	return V4Sel(tttf, recipA, zero);
}

inline Vec3V V3RsqrtFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 zero = V3Zero();
	const __m128 tttf = BTTTF();
	const __m128 recipA = _mm_rsqrt_ps(a);
	return V4Sel(tttf, recipA, zero);
}

inline Vec3V V3ScaleAdd(const Vec3V a, const FloatV b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDVEC3V(c);
	return V3Add(V3Scale(a, b), c);
}

inline Vec3V V3NegScaleSub(const Vec3V a, const FloatV b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDVEC3V(c);
	return V3Sub(c, V3Scale(a, b));
}

inline Vec3V V3MulAdd(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	return V3Add(V3Mul(a, b), c);
}

inline Vec3V V3NegMulSub(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	return V3Sub(c, V3Mul(a, b));
}

inline Vec3V V3Max(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_max_ps(a, b);
}

inline Vec3V V3Min(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_min_ps(a, b);
}

inline Vec3V V3Abs(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return V3Max(a, V3Neg(a));
}

inline FloatV V3Dot(const Vec3V a, const Vec3V b)	
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);

	const __m128 t0 = _mm_mul_ps(a, b);								//	aw*bw | az*bz | ay*by | ax*bx
	const __m128 t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(1,0,3,2));	//	ay*by | ax*bx | aw*bw | az*bz
	const __m128 t2 = _mm_add_ps(t0, t1);							//	ay*by + aw*bw | ax*bx + az*bz | aw*bw + ay*by | az*bz + ax*bx
	const __m128 t3 = _mm_shuffle_ps(t2, t2, _MM_SHUFFLE(2,3,0,1));	//	ax*bx + az*bz | ay*by + aw*bw | az*bz + ax*bx | aw*bw + ay*by
	return _mm_add_ps(t3, t2);										//	ax*bx + az*bz + ay*by + aw*bw 
																	//	ay*by + aw*bw + ax*bx + az*bz
																	//	az*bz + ax*bx + aw*bw + ay*by
																	//	aw*bw + ay*by + az*bz + ax*bx
}

inline Vec3V V3Cross(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	const __m128 r1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	const __m128 r2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	return _mm_sub_ps(_mm_mul_ps(l1, l2), _mm_mul_ps(r1, r2));
}

inline FloatV V3Length(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_sqrt_ps(V3Dot(a, a));
}

inline FloatV V3LengthSq(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return V3Dot(a, a);
}

inline Vec3V V3Normalize(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISFINITELENGTH(a);
	return V3ScaleInv(a, _mm_sqrt_ps(V3Dot(a, a)));
}

inline Vec3V V3NormalizeFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISFINITELENGTH(a);
	return V3Scale(a, _mm_rsqrt_ps(V3Dot(a, a)));
}

inline Vec3V V3Sel(const BoolV c, const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(_mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a)));
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline Vec3V V3NormalizeSafe(const Vec3V a, const Vec3V unsafeReturnValue)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 eps = FEps();
	const __m128 length = V3Length(a);
	const __m128 isGreaterThanZero = FIsGrtr(length, eps);
	return V3Sel(isGreaterThanZero, V3ScaleInv(a, length), unsafeReturnValue);
}

inline BoolV V3IsGrtr(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_cmpgt_ps(a, b);
}

inline BoolV V3IsGrtrOrEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_cmpge_ps(a, b);
}

inline BoolV V3IsEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return _mm_cmpeq_ps(a, b);
}

inline FloatV V3ExtractMax(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 shuf1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 shuf2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 shuf3 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_max_ps(_mm_max_ps(shuf1, shuf2), shuf3);
}

inline FloatV V3ExtractMin(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 shuf1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	const __m128 shuf2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	const __m128 shuf3 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_min_ps(_mm_min_ps(shuf1, shuf2), shuf3);
}

//// if(a > 0.0f) return 1.0f; else if a == 0.f return 0.f, else return -1.f;
// inline Vec3V V3MathSign(const Vec3V a)
//{
//	VECMATHAOS_ASSERT(isValidVec3V(a));
//
//	const __m128i ai = _mm_cvtps_epi32(a);
//	const __m128i bi = _mm_cvtps_epi32(V3Neg(a));
//	const __m128  aa = _mm_cvtepi32_ps(_mm_srai_epi32(ai, 31));
//	const __m128  bb = _mm_cvtepi32_ps(_mm_srai_epi32(bi, 31));
//	return _mm_or_ps(aa, bb);
//}

// return (a >= 0.0f) ? 1.0f : -1.0f;
inline Vec3V V3Sign(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 zero = V3Zero();
	const __m128 one = V3One();
	const __m128 none = V3Neg(one);
	return V3Sel(V3IsGrtrOrEq(a, zero), one, none);
}

inline Vec3V V3Clamp(const Vec3V a, const Vec3V minV, const Vec3V maxV)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(minV);
	ASSERT_ISVALIDVEC3V(maxV);
	return V3Max(V3Min(a, maxV), minV);
}

inline uint32_t V3AllGrtr(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalWindowsSimd::BAllTrue3_R(V4IsGrtr(a, b));
}

inline uint32_t V3AllGrtrOrEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalWindowsSimd::BAllTrue3_R(V4IsGrtrOrEq(a, b));
}

inline uint32_t V3AllEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalWindowsSimd::BAllTrue3_R(V4IsEq(a, b));
}

inline Vec3V V3Round(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	// return _mm_round_ps(a, 0x0);
	const Vec3V half = V3Load(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const Vec3V aRound = V3Sub(V3Add(a, half), signBit);
	__m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}

inline Vec3V V3PermYZZ(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 2, 2, 1));
}

inline Vec3V V3PermXYX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 1, 0));
}

inline Vec3V V3PermYZX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
}

inline Vec3V V3PermZXY(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2));
}

inline Vec3V V3PermZZY(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 2, 2));
}

inline Vec3V V3PermYXX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 0, 1));
}

inline Vec3V V3Perm_Zero_1Z_0Y(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);
	return _mm_shuffle_ps(v1, v0, _MM_SHUFFLE(3, 1, 2, 3));
}

inline Vec3V V3Perm_0Z_Zero_1X(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);
	return _mm_shuffle_ps(v0, v1, _MM_SHUFFLE(3, 0, 3, 2));
}

inline Vec3V V3Perm_1Y_0X_Zero(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);
	// There must be a better way to do this.
	Vec3V v2 = V3Zero();
	FloatV y1 = V3GetY(v1);
	FloatV x0 = V3GetX(v0);
	v2 = V3SetX(v2, y1);
	return V3SetY(v2, x0);
}

inline FloatV V3SumElems(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const __m128 shuf1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0)); // z,y,x,w
	const __m128 shuf2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1)); // y,x,w,z
	const __m128 shuf3 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2)); // x,w,z,y
	return _mm_add_ps(_mm_add_ps(shuf1, shuf2), shuf3);
}

inline void V3Transpose(Vec3V& col0, Vec3V& col1, Vec3V& col2)
{
	ASSERT_ISVALIDVEC3V(col0);
	ASSERT_ISVALIDVEC3V(col1);
	ASSERT_ISVALIDVEC3V(col2);
	const Vec3V col3 = _mm_setzero_ps();
	Vec3V tmp0 = _mm_unpacklo_ps(col0, col1);
	Vec3V tmp2 = _mm_unpacklo_ps(col2, col3);
	Vec3V tmp1 = _mm_unpackhi_ps(col0, col1);
	Vec3V tmp3 = _mm_unpackhi_ps(col2, col3);
	col0 = _mm_movelh_ps(tmp0, tmp2);
	col1 = _mm_movehl_ps(tmp2, tmp0);
	col2 = _mm_movelh_ps(tmp1, tmp3);
}

//////////////////////////////////
// VEC4V
//////////////////////////////////

inline Vec4V V4Splat(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	// return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0,0,0,0));
	return f;
}

inline Vec4V V4Merge(const FloatV* const floatVArray)
{
	ASSERT_ISVALIDFLOATV(floatVArray[0]);
	ASSERT_ISVALIDFLOATV(floatVArray[1]);
	ASSERT_ISVALIDFLOATV(floatVArray[2]);
	ASSERT_ISVALIDFLOATV(floatVArray[3]);
	const __m128 xw = _mm_move_ss(floatVArray[1], floatVArray[0]); // y, y, y, x
	const __m128 yz = _mm_move_ss(floatVArray[2], floatVArray[3]); // z, z, z, w
	return _mm_shuffle_ps(xw, yz, _MM_SHUFFLE(0, 2, 1, 0));
}

inline Vec4V V4PermYXWZ(const Vec4V a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 0, 1));
}

inline Vec4V V4PermXZXZ(const Vec4V a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 0, 2, 0));
}

inline Vec4V V4PermYWYW(const Vec4V a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 3, 1));
}

inline Vec4V V4PermYZXW(const Vec4V a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
}

inline Vec4V V4PermZWXY(const Vec4V a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
}

inline Vec4V V4SetX(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BFTTT(), v, f);
}

inline Vec4V V4SetY(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTFTT(), v, f);
}

inline Vec4V V4SetZ(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTFT(), v, f);
}


inline uint32_t V4AllGrtr(const Vec4V a, const Vec4V b)
{
	return internalWindowsSimd::BAllTrue4_R(V4IsGrtr(a, b));
}

inline uint32_t V4AllGrtrOrEq(const Vec4V a, const Vec4V b)
{
	return internalWindowsSimd::BAllTrue4_R(V4IsGrtrOrEq(a, b));
}

inline uint32_t V4AllGrtrOrEq3(const Vec4V a, const Vec4V b)
{
	return internalWindowsSimd::BAllTrue3_R(V4IsGrtrOrEq(a, b));
}

inline uint32_t V4AllEq(const Vec4V a, const Vec4V b)
{
	return internalWindowsSimd::BAllTrue4_R(V4IsEq(a, b));
}

inline uint32_t V4AnyGrtr3(const Vec4V a, const Vec4V b)
{
	return internalWindowsSimd::BAnyTrue3_R(V4IsGrtr(a, b));
}

inline Vec4V V4Round(const Vec4V a)
{
	// return _mm_round_ps(a, 0x0);
	const Vec4V half = V4Load(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const Vec4V aRound = V4Sub(V4Add(a, half), signBit);
	const __m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}

inline void V4Transpose(Vec4V& col0, Vec4V& col1, Vec4V& col2, Vec4V& col3)
{
	Vec4V tmp0 = _mm_unpacklo_ps(col0, col1);
	Vec4V tmp2 = _mm_unpacklo_ps(col2, col3);
	Vec4V tmp1 = _mm_unpackhi_ps(col0, col1);
	Vec4V tmp3 = _mm_unpackhi_ps(col2, col3);
	col0 = _mm_movelh_ps(tmp0, tmp2);
	col1 = _mm_movehl_ps(tmp2, tmp0);
	col2 = _mm_movelh_ps(tmp1, tmp3);
	col3 = _mm_movehl_ps(tmp3, tmp1);
}

inline BoolV BGetX(const BoolV f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0, 0, 0, 0));
}

inline BoolV BGetY(const BoolV f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(1, 1, 1, 1));
}

inline BoolV BGetZ(const BoolV f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 2, 2, 2));
}

inline BoolV BGetW(const BoolV f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(3, 3, 3, 3));
}

inline BoolV BSetX(const BoolV v, const BoolV f)
{
	return V4Sel(BFTTT(), v, f);
}

inline BoolV BSetY(const BoolV v, const BoolV f)
{
	return V4Sel(BTFTT(), v, f);
}

inline BoolV BSetZ(const BoolV v, const BoolV f)
{
	return V4Sel(BTTFT(), v, f);
}

inline BoolV BSetW(const BoolV v, const BoolV f)
{
	return V4Sel(BTTTF(), v, f);
}

template <int index>
BoolV BSplatElement(BoolV a)
{
	return internalWindowsSimd::m128_I2F(
	    _mm_shuffle_epi32(internalWindowsSimd::m128_F2I(a), _MM_SHUFFLE(index, index, index, index)));
}

inline BoolV BAnd(const BoolV a, const BoolV b)
{
	return _mm_and_ps(a, b);
}

inline BoolV BNot(const BoolV a)
{
	const BoolV bAllTrue(BTTTT());
	return _mm_xor_ps(a, bAllTrue);
}

inline BoolV BAndNot(const BoolV a, const BoolV b)
{
	return _mm_andnot_ps(b, a);
}

inline BoolV BOr(const BoolV a, const BoolV b)
{
	return _mm_or_ps(a, b);
}

inline BoolV BAllTrue4(const BoolV a)
{
	const BoolV bTmp =
	    _mm_and_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_and_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                  _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline BoolV BAnyTrue4(const BoolV a)
{
	const BoolV bTmp =
	    _mm_or_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_or_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                 _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline BoolV BAllTrue3(const BoolV a)
{
	const BoolV bTmp =
	    _mm_and_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2)));
	return _mm_and_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                  _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline BoolV BAnyTrue3(const BoolV a)
{
	const BoolV bTmp =
	    _mm_or_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2)));
	return _mm_or_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                 _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline uint32_t BAllEq(const BoolV a, const BoolV b)
{
	const BoolV bTest = internalWindowsSimd::m128_I2F(
	    _mm_cmpeq_epi32(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
	return internalWindowsSimd::BAllTrue4_R(bTest);
}

inline uint32_t BAllEqTTTT(const BoolV a)
{
	return uint32_t(_mm_movemask_ps(a)==15);
}

inline uint32_t BAllEqFFFF(const BoolV a)
{
	return uint32_t(_mm_movemask_ps(a)==0);
}

inline uint32_t BGetBitMask(const BoolV a)
{
	return uint32_t(_mm_movemask_ps(a));
}

inline Vec4V V4LoadXYZW(const float& x, const float& y, const float& z, const float& w)
{
	return _mm_set_ps(w, z, y, x);
}

inline VecU32V V4U32Sel(const BoolV c, const VecU32V a, const VecU32V b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_or_si128(_mm_andnot_si128(internalWindowsSimd::m128_F2I(c), internalWindowsSimd::m128_F2I(b)),
			_mm_and_si128(internalWindowsSimd::m128_F2I(c), internalWindowsSimd::m128_F2I(a))));
}

inline VecU32V V4U32or(VecU32V a, VecU32V b)
{
	return internalWindowsSimd::m128_I2F(_mm_or_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline VecU32V V4U32xor(VecU32V a, VecU32V b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_xor_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline VecU32V V4U32and(VecU32V a, VecU32V b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_and_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline VecU32V V4U32Andc(VecU32V a, VecU32V b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_andnot_si128(internalWindowsSimd::m128_F2I(b), internalWindowsSimd::m128_F2I(a)));
}

inline VecI32V U4Load(const uint32_t i)
{
	return _mm_load1_ps((float*)&i);
}

inline VecU32V U4LoadU(const uint32_t* i)
{
	return _mm_loadu_ps((float*)i);
}

inline VecU32V U4LoadA(const uint32_t* i)
{
	ASSERT_ISALIGNED16(i);
	return _mm_load_ps((float*)i);
}

inline VecI32V I4Load(const int i)
{
	return _mm_load1_ps((float*)&i);
}

inline VecI32V I4LoadU(const int* i)
{
	return _mm_loadu_ps((float*)i);
}

inline VecI32V I4LoadA(const int* i)
{
	ASSERT_ISALIGNED16(i);
	return _mm_load_ps((float*)i);
}

inline VecI32V V4I32Sel(const BoolV c, const VecI32V a, const VecI32V b)
{
	return V4U32Sel(c, a, b);
}

inline VecI32V VecI32V_Zero()
{
	return V4Zero();
}

inline VecI32V VecI32V_One()
{
	return I4Load(1);
}

inline VecI32V VecI32V_Two()
{
	return I4Load(2);
}

inline VecI32V VecI32V_MinusOne()
{
	return I4Load(-1);
}

inline VecU32V U4Zero()
{
	return U4Load(0);
}

inline VecU32V U4One()
{
	return U4Load(1);
}

inline VecU32V U4Two()
{
	return U4Load(2);
}

template <int index>
inline Vec4V V4SplatElement(Vec4V a)
{
	return internalWindowsSimd::m128_I2F(
		_mm_shuffle_epi32(internalWindowsSimd::m128_F2I(a), _MM_SHUFFLE(index, index, index, index)));
}

inline Vec4V Vec4V_From_PxVec3_WUndefined(const Vector3d& f)
{
	return _mm_set_ps(0.0f, f.z, f.y, f.x);
}

inline Vec4V Vec4V_ReinterpretFrom_VecU32V(VecU32V a)
{
	return Vec4V(a);
}

inline Vec4V Vec4V_ReinterpretFrom_VecI32V(VecI32V a)
{
	return Vec4V(a);
}

inline VecU32V VecU32V_ReinterpretFrom_Vec4V(Vec4V a)
{
	return VecU32V(a);
}

inline VecI32V VecI32V_ReinterpretFrom_Vec4V(Vec4V a)
{
	return VecI32V(a);
}

typedef union
{
	__m128 v;
	uint32_t m128_u32[4];
} _VecU32V;

inline VecU32V U4LoadXYZW(uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
	_VecU32V t;
	t.m128_u32[0] = x;
	t.m128_u32[1] = y;
	t.m128_u32[2] = z;
	t.m128_u32[3] = w;
	return t.v;
}

inline VecU32V V4IsGrtrV32u(const Vec4V a, const Vec4V b)
{
	return V4IsGrtr(a, b);
}

inline Vec4V V4Andc(const Vec4V a, const VecU32V b)
{
	VecU32V result32(a);
	result32 = V4U32Andc(result32, b);
	return Vec4V(result32);
}

inline void V4U32StoreAligned(VecU32V val, VecU32V* address)
{
	*address = val;
}
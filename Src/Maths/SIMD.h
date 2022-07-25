
#pragma once

// #define SIMD_INSTRUCTION_EMU

#include <float.h>
#include "Maths.h"
#include "Vector3.h"
#include "Vector4.h"

#if defined(SIMD_INSTRUCTION_EMU)
	typedef float				Scaler;
	typedef Vector4				F128;
	typedef TVector4<uint32_t>	B128;
	typedef TVector4<uint32_t>	U128;
	typedef TVector4<int>		I128;
#elif defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    typedef float32x2_t Scaler;
    typedef float32x4_t F128;
    typedef uint32x4_t  B128;
    typedef uint32x4_t  U128;
    typedef int32x4_t   I128;
	#define SIMD_INSTRUCTION_NEON
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
	#define SIMD_INSTRUCTION_SSE
#endif

static_assert(sizeof(F128) == 16, "simd size vector not correct!");
static_assert(sizeof(B128) == 16, "simd size vector not correct!");
static_assert(sizeof(U128) == 16, "simd size vector not correct!");
static_assert(sizeof(I128) == 16, "simd size vector not correct!");

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

#if _DEBUG
#include <assert.h>
#define ASSERT_ISALIGNED16(a) assert((((intptr_t)a) & 15) == 0)
#else
#define ASSERT_ISALIGNED16(a)
#endif

#ifdef SIMD_INSTRUCTION_NEON

#define VRECIPEQ recipq_newton<1>
#define VRECIPE recip_newton<1>
#define VRECIPSQRTEQ rsqrtq_newton<1>
#define VRECIPSQRTE rsqrt_newton<1>
#define VRECIPQ recipq_newton<4>
#define VRECIP recip_newton<4>
#define VRECIPSQRTQ rsqrtq_newton<4>
#define VRECIPSQRT rsqrt_newton<4>

inline Scaler Scaler_Load(const float f)
{
	return vdup_n_f32(reinterpret_cast<const float32_t&>(f));
}

inline Scaler Scaler_LoadA(const float* const f)
{
	ASSERT_ISALIGNED16(f);
	return vld1_f32(reinterpret_cast<const float32_t*>(f));
}

inline Scaler Scaler_Zero()
{
	return Scaler_Load(0.0f);
}

inline Scaler Scaler_One()
{
	return Scaler_Load(1.0f);
}

inline Scaler Scaler_Neg(const Scaler f)
{
	return vneg_f32(f);
}

inline Scaler Scaler_Add(const Scaler a, const Scaler b)
{
	return vadd_f32(a, b);
}

inline Scaler Scaler_Sub(const Scaler a, const Scaler b)
{
	return vsub_f32(a, b);
}

inline Scaler Scaler_Mul(const Scaler a, const Scaler b)
{
	return vmul_f32(a, b);
}

inline B128 Scaler_Equal(const Scaler a, const Scaler b)
{
	return vdupq_lane_u32(vceq_f32(a, b), 0);
}

inline Scaler Scaler_Sel(const B128 c, const Scaler a, const Scaler b)
{
	return vbsl_f32(vget_low_u32(c), a, b);
}

template <int n>
inline float32x2_t recip_newton(const float32x2_t& in)
{
	float32x2_t recip = vrecpe_f32(in);
	for (int i = 0; i < n; ++i)
		recip = vmul_f32(recip, vrecps_f32(in, recip));
	return recip;
}

template <int n>
inline float32x4_t recipq_newton(const float32x4_t& in)
{
	float32x4_t recip = vrecpeq_f32(in);
	for (int i = 0; i < n; ++i)
		recip = vmulq_f32(recip, vrecpsq_f32(recip, in));
	return recip;
}

template <int n>
inline float32x2_t rsqrt_newton(const float32x2_t& in)
{
	float32x2_t rsqrt = vrsqrte_f32(in);
	for (int i = 0; i < n; ++i)
		rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(rsqrt, rsqrt), in));
	return rsqrt;
}

template <int n>
inline float32x4_t rsqrtq_newton(const float32x4_t& in)
{
	float32x4_t rsqrt = vrsqrteq_f32(in);
	for (int i = 0; i < n; ++i)
		rsqrt = vmulq_f32(rsqrt, vrsqrtsq_f32(vmulq_f32(rsqrt, rsqrt), in));
	return rsqrt;
}

inline Scaler Scaler_Div(const Scaler a, const Scaler b)
{
	return vmul_f32(a, VRECIP(b));
}

inline Scaler Scaler_DivFast(const Scaler a, const Scaler b)
{
	return vmul_f32(a, VRECIPE(b));
}

inline Scaler Scaler_Recip(const Scaler a)
{
	return VRECIP(a);
}

inline Scaler Scaler_RecipFast(const Scaler a)
{
	return VRECIPE(a);
}

inline Scaler Scaler_RSqrt(const Scaler a)
{
	return VRECIPSQRT(a);
}

inline Scaler Scaler_Sqrt(const Scaler a)
{
	return Scaler_Sel(Scaler_Equal(a, Scaler_Zero()), a, vmul_f32(a, VRECIPSQRT(a)));
}

inline Scaler Scaler_RSqrtFast(const Scaler a)
{
	return VRECIPSQRTE(a);
}

inline Scaler Scaler_MAdd(const Scaler a, const Scaler b, const Scaler c)
{
	return vmla_f32(c, a, b);
}

inline Scaler Scaler_NegMSub(const Scaler a, const Scaler b, const Scaler c)
{
	return vmls_f32(c, a, b);
}

inline Scaler Scaler_Abs(const Scaler a)
{
	return vabs_f32(a);
}

inline B128 Scaler_Greater(const Scaler a, const Scaler b)
{
	return vdupq_lane_u32(vcgt_f32(a, b), 0);
}

inline B128 Scaler_GreaterEqual(const Scaler a, const Scaler b)
{
	return vdupq_lane_u32(vcge_f32(a, b), 0);
}

inline Scaler Scaler_Max(const Scaler a, const Scaler b)
{
	return vmax_f32(a, b);
}

inline Scaler Scaler_Min(const Scaler a, const Scaler b)
{
	return vmin_f32(a, b);
}

inline Scaler Scaler_Clamp(const Scaler a, const Scaler minV, const Scaler maxV)
{
	return vmax_f32(vmin_f32(a, maxV), minV);
}

inline Scaler Scaler_Round(const Scaler a)
{
	// truncate(a + (0.5f - sign(a)))
	const float32x2_t half = vdup_n_f32(0.5f);
	const float32x2_t sign = vcvt_f32_u32((vshr_n_u32(vreinterpret_u32_f32(a), 31)));
	const float32x2_t aPlusHalf = vadd_f32(a, half);
	const float32x2_t aRound = vsub_f32(aPlusHalf, sign);
	int32x2_t tmp = vcvt_s32_f32(aRound);
	return vcvt_f32_s32(tmp);
}

inline B128 BFFFF()
{
	return vmovq_n_u32(0);
}

inline B128 BFFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zeros, zo);
}

inline B128 BFFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zeros, oz);
}

inline B128 BFFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(zeros, ones);
}

inline B128 BFTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zeros);
}

inline B128 BFTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zo);
}

inline B128 BFTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zo, oz);
}

inline B128 BFTTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, ones);
}

inline B128 BTFFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	// const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zeros);
}

inline B128 BTFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zo);
}

inline B128 BTFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, oz);
}

inline B128 BTFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, ones);
}

inline B128 BTTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(ones, zeros);
}

inline B128 BTTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(ones, zo);
}

inline B128 BTTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(ones, oz);
}

inline B128 BTTTT()
{
	return vmovq_n_u32(0xffffFFFF);
}

inline B128 B128_And(const B128 a, const B128 b)
{
	return vandq_u32(a, b);
}

inline B128 B128_Not(const B128 a)
{
	return vmvnq_u32(a);
}

inline B128 B128_AndNot(const B128 a, const B128 b)
{
	// return vbicq_u32(a, b);
	return vandq_u32(a, vmvnq_u32(b));
}

inline B128 B128_Or(const B128 a, const B128 b)
{
	return vorrq_u32(a, b);
}

inline B128 B128_All(const B128 a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vceq_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

inline B128 B128_Any(const B128 a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vtst_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

inline F128 F128_Load(const float f)
{
	return vdupq_n_f32(reinterpret_cast<const float32_t&>(f));
}

inline F128 F128_LoadA(const float* f)
{
	ASSERT_ISALIGNED16(f);
	return vld1q_f32(reinterpret_cast<const float32_t*>(f));
}

inline F128 F128_Load(F128* addr)
{
	return vld1q_f32(reinterpret_cast<float32_t*>(addr));
}

inline void F128_StoreA(F128 a, float* f)
{
	ASSERT_ISALIGNED16(f);
	vst1q_f32(reinterpret_cast<float32_t*>(f), a);
}

inline F128 Vec4V_From_FloatV(Scaler f)
{
	return vcombine_f32(f, f);
}

inline F128 F128_Load_Vector3d(const Vector3& f)
{
	alignas(16) float data[4] = { f.x, f.y, f.z, 0.0f };
	return F128_LoadA(data);
}

inline F128 F128_PermYXWZ(const F128 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2_t yx = vext_f32(xy, xy, 1);
	const float32x2_t wz = vext_f32(zw, zw, 1);
	return vcombine_f32(yx, wz);
}

inline F128 F128_PermXZXZ(const F128 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[0], xzyw.val[0]);
}

inline F128 F128_PermYWYW(const F128 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[1], xzyw.val[1]);
}

inline F128 F128_PermYZXW(const F128 a)
{
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t yz = vext_u32(xy, zw, 1);
	const uint32x2_t xw = vrev64_u32(vext_u32(zw, xy, 1));
	return vreinterpretq_f32_u32(vcombine_u32(yz, xw));
}

inline F128 F128_PermZWXY(const F128 a)
{
	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);
	return vcombine_f32(high, low);
}

inline F128 F128_Sel(const B128 c, const F128 a, const F128 b)
{
	return vbslq_f32(c, a, b);
}

inline B128 F128_Equal(const F128 a, const F128 b)
{
	return vceqq_f32(a, b);
}

inline F128 F128_Zero()
{
	return vreinterpretq_f32_u32(vmovq_n_u32(0));
}

inline F128 F128_One()
{
	return vmovq_n_f32(1.0f);
}

inline F128 F128_Neg(const F128 f)
{
	return vnegq_f32(f);
}

inline F128 F128_Add(const F128 a, const F128 b)
{
	return vaddq_f32(a, b);
}

inline F128 F128_Sub(const F128 a, const F128 b)
{
	return vsubq_f32(a, b);
}

inline F128 F128_Scale(const F128 a, const Scaler b)
{
	return vmulq_lane_f32(a, b, 0);
}

inline F128 F128_Mul(const F128 a, const F128 b)
{
	return vmulq_f32(a, b);
}

inline F128 F128_ScaleInv(const F128 a, const Scaler b)
{
	const float32x2_t invB = VRECIP(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline F128 F128_Div(const F128 a, const F128 b)
{
	const float32x4_t invB = VRECIPQ(b);
	return vmulq_f32(a, invB);
}

inline F128 F128_ScaleInvFast(const F128 a, const Scaler b)
{
	const float32x2_t invB = VRECIPE(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline F128 F128_DivFast(const F128 a, const F128 b)
{
	const float32x4_t invB = VRECIPEQ(b);
	return vmulq_f32(a, invB);
}

inline F128 F128_Recip(const F128 a)
{
	return VRECIPQ(a);
}

inline F128 F128_RecipFast(const F128 a)
{
	return VRECIPEQ(a);
}

inline F128 F128_RSqrt(const F128 a)
{
	return VRECIPSQRTQ(a);
}

inline F128 F128_RSqrtFast(const F128 a)
{
	return VRECIPSQRTEQ(a);
}

inline F128 F128_Sqrt(const F128 a)
{
	return F128_Sel(F128_Equal(a, F128_Zero()), a, F128_Mul(a, VRECIPSQRTQ(a)));
}

inline F128 F128_ScaleAdd(const F128 a, const Scaler b, const F128 c)
{
	return vmlaq_lane_f32(c, a, b, 0);
}

inline F128 F128_MAdd(const F128 a, const F128 b, const F128 c)
{
	return vmlaq_f32(c, a, b);
}

inline F128 F128_NegMSub(const F128 a, const F128 b, const F128 c)
{
	return vmlsq_f32(c, a, b);
}

inline Scaler F128_GetW(const F128 f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 1);
}

inline Scaler F128_GetX(const F128 f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 0);
}

inline Scaler F128_GetY(const F128 f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 1);
}

inline Scaler F128_GetZ(const F128 f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 0);
}

inline F128 F128_SetW(const F128 v, const Scaler f)
{
	return F128_Sel(BTTTF(), v, vcombine_f32(f, f));
}

inline F128 F128_ClearW(const F128 v)
{
	return F128_Sel(BTTTF(), v, F128_Zero());
}

inline F128 F128_Abs(const F128 a)
{
	return vabsq_f32(a);
}

template <int index>
inline F128 F128_SplatElement(F128 a)
{
	if (index < 2)
	{
		return vdupq_lane_f32(vget_low_f32(a), index & 0x01);
	}
	else if (index == 2)
	{
		return vdupq_lane_f32(vget_high_f32(a), 0);
	}
	else if (index == 3)
	{
		return vdupq_lane_f32(vget_high_f32(a), 1);
	}
}

inline Scaler F128_Dot(const F128 a, const F128 b)
{
	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {z+w, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z+w, x+y+z+w}
	return sumWZYX;
}

inline Scaler F128_Dot3(const F128 aa, const F128 bb)
{
	// PT: the V3Dot code relies on the fact that W=0 so we can't reuse it as-is, we need to clear W first.
	// TODO: find a better implementation that does not need to clear W.
	const F128 a = F128_ClearW(aa);
	const F128 b = F128_ClearW(bb);

	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return sum0ZYX;
}

inline F128 F128_Cross(const F128 a, const F128 b)
{
	const uint32x2_t TF = { 0xffffFFFF, 0x0 };
	const float32x2_t ay_ax = vget_low_f32(a);  // d2
	const float32x2_t aw_az = vget_high_f32(a); // d3
	const float32x2_t by_bx = vget_low_f32(b);  // d4
	const float32x2_t bw_bz = vget_high_f32(b); // d5
	// Hi, Lo
	const float32x2_t bz_by = vext_f32(by_bx, bw_bz, 1); // bz, by
	const float32x2_t az_ay = vext_f32(ay_ax, aw_az, 1); // az, ay

	const float32x2_t azbx = vmul_f32(aw_az, by_bx);      // 0, az*bx
	const float32x2_t aybz_axby = vmul_f32(ay_ax, bz_by); // ay*bz, ax*by

	const float32x2_t azbxSUBaxbz = vmls_f32(azbx, bw_bz, ay_ax);                  // 0, az*bx-ax*bz
	const float32x2_t aybzSUBazby_axbySUBaybx = vmls_f32(aybz_axby, by_bx, az_ay); // ay*bz-az*by, ax*by-ay*bx

	const float32x2_t retLow = vext_f32(aybzSUBazby_axbySUBaybx, azbxSUBaxbz, 1);           // az*bx-ax*bz, ay*bz-az*by
	const uint32x2_t retHigh = vand_u32(TF, vreinterpret_u32_f32(aybzSUBazby_axbySUBaybx)); // 0, ax*by-ay*bx

	return vcombine_f32(retLow, vreinterpret_f32_u32(retHigh));
}

inline Scaler F128_Length(const F128 a)
{
	const float32x4_t tmp = vmulq_f32(a, a);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return Scaler_Sqrt(sumWZYX);
}

inline Scaler F128_SquareLength(const F128 a)
{
	return F128_Dot(a, a);
}

inline F128 F128_Normalize(const F128 a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return F128_ScaleInv(a, F128_Length(a));
}

inline F128 F128_NormalizeFast(const F128 a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return F128_Scale(a, Scaler_RSqrtFast(F128_Dot(a, a)));
}

inline B128 F128_Greater(const F128 a, const F128 b)
{
	return vcgtq_f32(a, b);
}

inline B128 F128_GreaterEqual(const F128 a, const F128 b)
{
	return vcgeq_f32(a, b);
}

inline F128 F128_Max(const F128 a, const F128 b)
{
	return vmaxq_f32(a, b);
}

inline F128 F128_Min(const F128 a, const F128 b)
{
	return vminq_f32(a, b);
}

inline F128 F128_Round(const F128 a)
{
	// truncate(a + (0.5f - sign(a)))
	const F128 half = F128_Load(0.5f);
	const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
	const F128 aPlusHalf = F128_Add(a, half);
	const F128 aRound = F128_Sub(aPlusHalf, sign);
	return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}

inline void F128_Transpose(F128& col0, F128& col1, F128& col2, F128& col3)
{
	const float32x4x2_t v0v1 = vzipq_f32(col0, col2);
	const float32x4x2_t v2v3 = vzipq_f32(col1, col3);
	const float32x4x2_t zip0 = vzipq_f32(v0v1.val[0], v2v3.val[0]);
	const float32x4x2_t zip1 = vzipq_f32(v0v1.val[1], v2v3.val[1]);
	col0 = zip0.val[0];
	col1 = zip0.val[1];
	col2 = zip1.val[0];
	col3 = zip1.val[1];
}

inline U128 U128_LoadXYZW(unsigned int x, unsigned int y, unsigned int z, unsigned int w)
{
	const uint32x4_t ret = { x, y, z, w };
	return ret;
}

inline U128 U128_Load(const unsigned int i)
{
	return vdupq_n_u32(i);
}

inline U128 U128_LoadU(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline U128 U128_LoadA(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline U128 U128_OR(U128 a, U128 b)
{
	return vorrq_u32(a, b);
}

inline U128 U128_XOR(U128 a, U128 b)
{
	return veorq_u32(a, b);
}

inline U128 U128_AND(U128 a, U128 b)
{
	return vandq_u32(a, b);
}

inline U128 U128_ANDNOT(U128 a, U128 b)
{
	// return vbicq_u32(a, b); // creates gcc compiler bug in RTreeQueries.cpp
	return vandq_u32(a, vmvnq_u32(b));
}

inline void U128_StoreA(const U128 uv, unsigned int* u)
{
	ASSERT_ISALIGNED16(u);
	vst1q_u32(reinterpret_cast<uint32_t*>(u), uv);
}

inline U128 U128_Load_BVec4(const B128& a)
{
	return a;
}

inline void U128_StoreAA(U128 val, U128* address)
{
	vst1q_u32(reinterpret_cast<uint32_t*>(address), val);
}

inline F128 Vec4V_From_VecU32V(U128 a)
{
	return vcvtq_f32_u32(a);
}

inline F128 Vec4V_ReinterpretFrom_VecU32V(U128 a)
{
	return vreinterpretq_f32_u32(a);
}

inline U128 U128_ReinterpretFrom_Vec4V(F128 a)
{
	return vreinterpretq_u32_f32(a);
}

#elif defined(SIMD_INSTRUCTION_SSE)

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

	alignas(16) const uint32_t gMaskXYZ[4] = { 0xffffffff, 0xffffffff, 0xffffffff, 0 };
}

inline Scaler Scaler_Load(const float f)
{
	return _mm_load1_ps(&f);
}

inline void Scaler_Store(const Scaler a, float* f)
{
	_mm_store_ss(f, a);
}

inline Scaler Scaler_Zero()
{
	return _mm_setzero_ps();
}

inline Scaler Scaler_One()
{
	return Scaler_Load(1.0f);
}

inline Scaler Scaler_Neg(const Scaler f)
{
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline Scaler Scaler_Add(const Scaler a, const Scaler b)
{
	return _mm_add_ps(a, b);
}

inline Scaler Scaler_Sub(const Scaler a, const Scaler b)
{
	return _mm_sub_ps(a, b);
}

inline Scaler Scaler_Mul(const Scaler a, const Scaler b)
{
	return _mm_mul_ps(a, b);
}

inline Scaler Scaler_Div(const Scaler a, const Scaler b)
{
	return _mm_div_ps(a, b);
}

inline Scaler Scaler_DivFast(const Scaler a, const Scaler b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Scaler Scaler_Recip(const Scaler a)
{
	return _mm_div_ps(Scaler_One(), a);
}

inline Scaler Scaler_RecipFast(const Scaler a)
{
	return _mm_rcp_ps(a);
}

inline Scaler Scaler_RSqrt(const Scaler a)
{
	return _mm_div_ps(Scaler_One(), _mm_sqrt_ps(a));
}

inline Scaler Scaler_Sqrt(const Scaler a)
{
	return _mm_sqrt_ps(a);
}

inline Scaler Scaler_RSqrtFast(const Scaler a)
{
	return _mm_rsqrt_ps(a);
}

inline Scaler Scaler_MAdd(const Scaler a, const Scaler b, const Scaler c)
{
	return Scaler_Add(Scaler_Mul(a, b), c);
}

inline Scaler Scaler_NegMSub(const Scaler a, const Scaler b, const Scaler c)
{
	return Scaler_Sub(c, Scaler_Mul(a, b));
}

inline Scaler Scaler_Abs(const Scaler a)
{
	alignas(16) const static uint32_t absMask[4] = { 0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF, 0x7fFFffFF };
	return _mm_and_ps(a, _mm_load_ps((float*)absMask));
}

inline Scaler Scaler_Sel(const B128 c, const Scaler a, const Scaler b)
{
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline B128 Scaler_Greater(const Scaler a, const Scaler b)
{
	return _mm_cmpgt_ps(a, b);
}

inline B128 Scaler_GreaterEqual(const Scaler a, const Scaler b)
{
	return _mm_cmpge_ps(a, b);
}

inline B128 Scaler_Equal(const Scaler a, const Scaler b)
{
	return _mm_cmpeq_ps(a, b);
}

inline Scaler Scaler_Max(const Scaler a, const Scaler b)
{
	return _mm_max_ps(a, b);
}

inline Scaler Scaler_Min(const Scaler a, const Scaler b)
{
	return _mm_min_ps(a, b);
}

inline Scaler Scaler_Clamp(const Scaler a, const Scaler minV, const Scaler maxV)
{
	return _mm_max_ps(_mm_min_ps(a, maxV), minV);
}

inline Scaler Scaler_Round(const Scaler a)
{
	// return _mm_round_ps(a, 0x0);
	const Scaler half = Scaler_Load(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const Scaler aRound = Scaler_Sub(Scaler_Add(a, half), signBit);
	__m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}

inline B128 BFFFF()
{
	return _mm_setzero_ps();
}

inline B128 BFFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0,0xFFFFFFFF};
	const __m128 ffft=_mm_load_ps((float*)&f);
	return ffft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, 0));
}

inline B128 BFFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0};
	const __m128 fftf=_mm_load_ps((float*)&f);
	return fftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, 0));
}

inline B128 BFFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fftt=_mm_load_ps((float*)&f);
	return fftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, 0));
}

inline B128 BFTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0};
	const __m128 ftff=_mm_load_ps((float*)&f);
	return ftff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, 0));
}

inline B128 BFTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ftft=_mm_load_ps((float*)&f);
	return ftft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, 0));
}

inline B128 BFTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 fttf=_mm_load_ps((float*)&f);
	return fttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, 0));
}

inline B128 BFTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fttt=_mm_load_ps((float*)&f);
	return fttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, 0));
}

inline B128 BTFFF()
{
	// const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0};
	// const __m128 tfff=_mm_load_ps((float*)&f);
	// return tfff;
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, 0, -1));
}

inline B128 BTFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0xFFFFFFFF};
	const __m128 tfft=_mm_load_ps((float*)&f);
	return tfft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, -1));
}

inline B128 BTFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0};
	const __m128 tftf=_mm_load_ps((float*)&f);
	return tftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, -1));
}

inline B128 BTFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tftt=_mm_load_ps((float*)&f);
	return tftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, -1));
}

inline B128 BTTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0};
	const __m128 ttff=_mm_load_ps((float*)&f);
	return ttff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, -1));
}

inline B128 BTTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ttft=_mm_load_ps((float*)&f);
	return ttft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, -1));
}

inline B128 BTTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 tttf=_mm_load_ps((float*)&f);
	return tttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, -1));
}

inline B128 BTTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tttt=_mm_load_ps((float*)&f);
	return tttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, -1));
}

inline B128 B128_And(const B128 a, const B128 b)
{
	return _mm_and_ps(a, b);
}

inline B128 B128_Not(const B128 a)
{
	const B128 bAllTrue(BTTTT());
	return _mm_xor_ps(a, bAllTrue);
}

inline B128 B128_AndNot(const B128 a, const B128 b)
{
	return _mm_andnot_ps(b, a);
}

inline B128 B128_Or(const B128 a, const B128 b)
{
	return _mm_or_ps(a, b);
}

inline B128 B128_All(const B128 a)
{
	const B128 bTmp =
		_mm_and_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_and_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
		_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline B128 B128_Any(const B128 a)
{
	const B128 bTmp =
		_mm_or_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_or_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
		_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline F128 F128_Load(const float f)
{
	return _mm_load1_ps(&f);
}

inline F128 F128_LoadA(const float* f)
{
	ASSERT_ISALIGNED16(f);
	return _mm_load_ps(f);
}

inline F128 F128_LoadU(const float* f)
{
	return _mm_loadu_ps(f);
}

inline void F128_StoreA(const F128 a, float* f)
{
	ASSERT_ISALIGNED16(f);
	_mm_store_ps(f, a);
}

inline void F128_StoreU(const F128 a, float* f)
{
	_mm_storeu_ps(f, a);
}

inline Scaler F128_GetX(const F128 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0, 0, 0, 0));
}

inline Scaler F128_GetY(const F128 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(1, 1, 1, 1));
}

inline Scaler F128_GetZ(const F128 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 2, 2, 2));
}

inline Scaler F128_GetW(const F128 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(3, 3, 3, 3));
}

inline F128 F128_ClearW(const F128 v)
{
	return _mm_and_ps(v, (I128&)internalWindowsSimd::gMaskXYZ);
}

inline F128 Vec4V_From_FloatV(Scaler f)
{
	return f;
}

inline F128 F128_PermYXWZ(const F128 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 0, 1));
}

inline F128 F128_PermXZXZ(const F128 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 0, 2, 0));
}

inline F128 F128_PermYWYW(const F128 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 3, 1));
}

inline F128 F128_PermYZXW(const F128 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
}

inline F128 F128_PermZWXY(const F128 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
}

inline F128 F128_LoadXYZW(const float& x, const float& y, const float& z, const float& w)
{
	return _mm_set_ps(w, z, y, x);
}

inline F128 F128_Zero()
{
	return _mm_setzero_ps();
}

inline F128 F128_One()
{
	return F128_Load(1.0f);
}

inline F128 F128_Neg(const F128 f)
{
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline F128 F128_Add(const F128 a, const F128 b)
{
	return _mm_add_ps(a, b);
}

inline F128 F128_Sub(const F128 a, const F128 b)
{
	return _mm_sub_ps(a, b);
}

inline F128 F128_Scale(const F128 a, const Scaler b)
{
	return _mm_mul_ps(a, b);
}

inline F128 F128_Mul(const F128 a, const F128 b)
{
	return _mm_mul_ps(a, b);
}

inline F128 F128_ScaleInv(const F128 a, const Scaler b)
{
	return _mm_div_ps(a, b);
}

inline F128 F128_Div(const F128 a, const F128 b)
{
	return _mm_div_ps(a, b);
}

inline F128 F128_ScaleInvFast(const F128 a, const Scaler b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline F128 F128_DivFast(const F128 a, const F128 b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline F128 F128_Recip(const F128 a)
{
	return _mm_div_ps(F128_One(), a);
}

inline F128 F128_RecipFast(const F128 a)
{
	return _mm_rcp_ps(a);
}

inline F128 F128_RSqrt(const F128 a)
{
	return _mm_div_ps(F128_One(), _mm_sqrt_ps(a));
}

inline F128 F128_RSqrtFast(const F128 a)
{
	return _mm_rsqrt_ps(a);
}

inline F128 F128_Sqrt(const F128 a)
{
	return _mm_sqrt_ps(a);
}

inline F128 F128_ScaleAdd(const F128 a, const Scaler b, const F128 c)
{
	return F128_Add(F128_Scale(a, b), c);
}

inline F128 F128_MAdd(const F128 a, const F128 b, const F128 c)
{
	return F128_Add(F128_Mul(a, b), c);
}

inline F128 F128_NegMSub(const F128 a, const F128 b, const F128 c)
{
	return F128_Sub(c, F128_Mul(a, b));
}

inline F128 F128_Max(const F128 a, const F128 b)
{
	return _mm_max_ps(a, b);
}

inline F128 F128_Min(const F128 a, const F128 b)
{
	return _mm_min_ps(a, b);
}

inline F128 F128_Abs(const F128 a)
{
	return F128_Max(a, F128_Neg(a));
}

inline Scaler F128_Dot(const F128 a, const F128 b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // x,y,z,w
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 1, 0, 3)); // w,x,y,z
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 0, 3, 2)); // z,w,x,y
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 3, 2, 1)); // y,z,w,x
	return _mm_add_ps(_mm_add_ps(shuf2, shuf3), _mm_add_ps(dot1, shuf1));
}

inline Scaler F128_Dot3(const F128 a, const F128 b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // aw*bw | az*bz | ay*by | ax*bx
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 0, 0, 0)); // ax*bx | ax*bx | ax*bx | ax*bx
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 1, 1, 1)); // ay*by | ay*by | ay*by | ay*by
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 2, 2, 2)); // az*bz | az*bz | az*bz | az*bz
	return _mm_add_ps(_mm_add_ps(shuf1, shuf2), shuf3);                       // ax*bx + ay*by + az*bz in each component
}

inline F128 F128_Cross(const F128 a, const F128 b)
{
	const __m128 r1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	const __m128 r2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	return _mm_sub_ps(_mm_mul_ps(l1, l2), _mm_mul_ps(r1, r2));
}

inline Scaler F128_Length(const F128 a)
{
	return _mm_sqrt_ps(F128_Dot(a, a));
}

inline Scaler F128_SquareLength(const F128 a)
{
	return F128_Dot(a, a);
}

inline F128 F128_Normalize(const F128 a)
{
	return F128_ScaleInv(a, _mm_sqrt_ps(F128_Dot(a, a)));
}

inline F128 F128_NormalizeFast(const F128 a)
{
	return F128_ScaleInvFast(a, _mm_sqrt_ps(F128_Dot(a, a)));
}

inline F128 F128_Sel(const B128 c, const F128 a, const F128 b)
{
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline B128 F128_Greater(const F128 a, const F128 b)
{
	return _mm_cmpgt_ps(a, b);
}

inline B128 F128_GreaterEqual(const F128 a, const F128 b)
{
	return _mm_cmpge_ps(a, b);
}

inline B128 F128_Equal(const F128 a, const F128 b)
{
	return _mm_cmpeq_ps(a, b);
}

inline F128 F128_Round(const F128 a)
{
	// return _mm_round_ps(a, 0x0);
	const F128 half = F128_Load(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const F128 aRound = F128_Sub(F128_Add(a, half), signBit);
	const __m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}

inline void F128_Transpose(F128& col0, F128& col1, F128& col2, F128& col3)
{
	F128 tmp0 = _mm_unpacklo_ps(col0, col1);
	F128 tmp2 = _mm_unpacklo_ps(col2, col3);
	F128 tmp1 = _mm_unpackhi_ps(col0, col1);
	F128 tmp3 = _mm_unpackhi_ps(col2, col3);
	col0 = _mm_movelh_ps(tmp0, tmp2);
	col1 = _mm_movehl_ps(tmp2, tmp0);
	col2 = _mm_movelh_ps(tmp1, tmp3);
	col3 = _mm_movehl_ps(tmp3, tmp1);
}

inline void U128_StoreAA(U128 val, U128* address)
{
	*address = val;
}

template <int index>
inline F128 F128_SplatElement(F128 a)
{
	return internalWindowsSimd::m128_I2F(
		_mm_shuffle_epi32(internalWindowsSimd::m128_F2I(a), _MM_SHUFFLE(index, index, index, index)));
}

inline F128 F128_Load_Vector3d(const Vector3& f)
{
	return _mm_set_ps(0.0f, f.z, f.y, f.x);
}

inline U128 U128_Load(const uint32_t i)
{
	return _mm_load1_ps((float*)&i);
}

inline U128 U128_Load_BVec4(const B128 a)
{
	return a;
}

inline U128 U128_LoadU(const uint32_t* i)
{
	return _mm_loadu_ps((float*)i);
}

inline U128 U128_LoadA(const uint32_t* i)
{
	ASSERT_ISALIGNED16(i);
	return _mm_load_ps((float*)i);
}

inline void U128_StoreA(const U128 uv, uint32_t* u)
{
	ASSERT_ISALIGNED16(u);
	_mm_store_ps((float*)u, uv);
}

inline F128 Vec4V_ReinterpretFrom_VecU32V(U128 a)
{
	return F128(a);
}

inline U128 U128_ReinterpretFrom_Vec4V(F128 a)
{
	return U128(a);
}

typedef union
{
	__m128 v;
	uint32_t m128_u32[4];
} _VecU32V;

inline U128 U128_LoadXYZW(uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
	_VecU32V t;
	t.m128_u32[0] = x;
	t.m128_u32[1] = y;
	t.m128_u32[2] = z;
	t.m128_u32[3] = w;
	return t.v;
}

inline U128 U128_OR(U128 a, U128 b)
{
	return internalWindowsSimd::m128_I2F(_mm_or_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline U128 U128_XOR(U128 a, U128 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_xor_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline U128 U128_AND(U128 a, U128 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_and_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline U128 U128_ANDNOT(U128 a, U128 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_andnot_si128(internalWindowsSimd::m128_F2I(b), internalWindowsSimd::m128_F2I(a)));
}

#elif defined(SIMD_INSTRUCTION_EMU)

namespace SIMD_EMU
{
	inline F128 F128_Load_Vector3d(const Vector3& f)
	{
		return F128(f.x, f.y, f.z, 0.0f);
	}

	template <int index>
	inline F128 F128_SplatElement(F128 a)
	{
		return F128(a[index], a[index], a[index], a[index]);
	}

	inline F128 F128_Load(const float f)
	{
		return F128(f, f, f, f);
	}

	inline F128 F128_LoadA(const float* f)
	{
		return F128(f[0], f[1], f[2], f[3]);
	}

	inline F128 F128_LoadU(const float* f)
	{
		return F128_LoadA(f);
	}

	inline void F128_StoreA(F128 a, float* f)
	{
		f[0] = a.x;
		f[1] = a.y;
		f[2] = a.z;
		f[3] = a.w;
	}

	inline void F128_StoreU(F128 a, float* f)
	{
		F128_StoreA(a, f);
	}

	inline B128 F128_Greater(const F128 a, const F128 b)
	{
		return TVector4<bool>(a.x > b.x, a.y > b.y, a.z > b.z, a.w > b.w);
	}

	inline B128 F128_GreaterOrEqual(const F128 a, const F128 b)
	{
		return TVector4<bool>(a.x >= b.x, a.y >= b.y, a.z >= b.z, a.w >= b.w);
	}

	inline B128 F128_Equal(const F128 a, const F128 b)
	{
		return TVector4<bool>(a.x == b.x, a.y == b.y, a.z == b.z, a.w == b.w);
	}

	inline B128 B128_And(const B128 a, const B128 b)
	{
		return TVector4<bool>(a.x && b.x, a.y && b.y, a.z && b.z, a.w && b.w);
	}

	inline B128 B128_Not(const B128 a)
	{
		return TVector4<bool>(!a.x, !a.y, !a.z, !a.w);
	}

	inline B128 B128_AndNot(const B128 a, const B128 b)
	{
		return TVector4<bool>(a.x && !b.x, a.y && !b.y, a.z && !b.z, a.w && !b.w);
	}

	inline B128 B128_Or(const B128 a, const B128 b)
	{
		return TVector4<bool>(a.x || b.x, a.y || b.y, a.z || b.z, a.w || b.w);
	}

	inline U128 U128_Load_BVec4(const B128 a)
	{
		return TVector4<unsigned int>(a.x, a.y, a.z, a.w);
	}

	inline void U128_StoreA(const U128 uv, uint32_t* u)
	{
		f[0] = a.x;
		f[1] = a.y;
		f[2] = a.z;
		f[3] = a.w;
	}
}

#endif

inline F128 F128_Sin(const F128 a)
{
	const F128 recipTwoPi = F128_Load(0.5f / 3.141592655f);
	const F128 twoPi = F128_Load(3.141592655f * 2.0f);
	const F128 tmp = F128_Mul(a, recipTwoPi);
	const F128 b = F128_Round(tmp);
	const F128 V1 = F128_NegMSub(twoPi, b, a);

	const F128 V2 = F128_Mul(V1, V1);
	const F128 V3 = F128_Mul(V2, V1);
	const F128 V5 = F128_Mul(V3, V2);
	const F128 V7 = F128_Mul(V5, V2);
	const F128 V9 = F128_Mul(V7, V2);
	const F128 V11 = F128_Mul(V9, V2);
	const F128 V13 = F128_Mul(V11, V2);
	const F128 V15 = F128_Mul(V13, V2);
	const F128 V17 = F128_Mul(V15, V2);
	const F128 V19 = F128_Mul(V17, V2);
	const F128 V21 = F128_Mul(V19, V2);
	const F128 V23 = F128_Mul(V21, V2);

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

inline F128 F128_Cos(const F128 a)
{
	const F128 recipTwoPi = F128_Load(0.5f / 3.141592655f);
	const F128 twoPi = F128_Load(3.141592655f * 2.0f);
	const F128 tmp = F128_Mul(a, recipTwoPi);
	const F128 b = F128_Round(tmp);
	const F128 V1 = F128_NegMSub(twoPi, b, a);

	const F128 V2 = F128_Mul(V1, V1);
	const F128 V4 = F128_Mul(V2, V2);
	const F128 V6 = F128_Mul(V4, V2);
	const F128 V8 = F128_Mul(V4, V4);
	const F128 V10 = F128_Mul(V6, V4);
	const F128 V12 = F128_Mul(V6, V6);
	const F128 V14 = F128_Mul(V8, V6);
	const F128 V16 = F128_Mul(V8, V8);
	const F128 V18 = F128_Mul(V10, V8);
	const F128 V20 = F128_Mul(V10, V10);
	const F128 V22 = F128_Mul(V12, V10);

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



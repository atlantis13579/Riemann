
#pragma once

BoolV BAnd(const BoolV a, const BoolV b);
BoolV BOr(const BoolV a, const BoolV b);
BoolV BTTTT();
BoolV BFFFF();
unsigned int BAllEqTTTT(const BoolV a);
unsigned int BAllEqFFFF(const BoolV a);
Vec3V V3Sel(const BoolV c, const Vec3V a, const Vec3V b);
FloatV V4GetX(const Vec4V f);
FloatV V4GetY(const Vec4V f);
FloatV V4GetZ(const Vec4V f);
FloatV V4GetW(const Vec4V f);
Vec4V V4LoadA(const float* const f);
Vec4V V4Sel(const BoolV c, const Vec4V a, const Vec4V b);
BoolV V4IsEq(const Vec4V a, const Vec4V b);

namespace internalUnitNeonSimd
{
unsigned int BAllTrue4_R(const BoolV a);
unsigned int BAllTrue3_R(const BoolV a);
unsigned int BAnyTrue4_R(const BoolV a);
unsigned int BAnyTrue3_R(const BoolV a);
}

struct alignas(16) VECTORF32 { float f[4]; };

static const VECTORF32 g_PXSinCoefficients0 = {{1.0f, -0.166666667f, 8.333333333e-3f, -1.984126984e-4f}};
static const VECTORF32 g_PXSinCoefficients1 = {{2.755731922e-6f, -2.505210839e-8f, 1.605904384e-10f, -7.647163732e-13f}};
static const VECTORF32 g_PXSinCoefficients2 = {{2.811457254e-15f, -8.220635247e-18f, 1.957294106e-20f, -3.868170171e-23f}};
static const VECTORF32 g_PXCosCoefficients0 = {{1.0f, -0.5f, 4.166666667e-2f, -1.388888889e-3f}};
static const VECTORF32 g_PXCosCoefficients1 = {{2.480158730e-5f, -2.755731922e-7f, 2.087675699e-9f, -1.147074560e-11f}};
static const VECTORF32 g_PXCosCoefficients2 = {{4.779477332e-14f, -1.561920697e-16f, 4.110317623e-19f, -8.896791392e-22f}};
static const VECTORF32 g_PXTanCoefficients0 = {{1.0f, 0.333333333f, 0.133333333f, 5.396825397e-2f}};
static const VECTORF32 g_PXTanCoefficients1 = {{2.186948854e-2f, 8.863235530e-3f, 3.592128167e-3f, 1.455834485e-3f}};
static const VECTORF32 g_PXTanCoefficients2 = {{5.900274264e-4f, 2.391290764e-4f, 9.691537707e-5f, 3.927832950e-5f}};
static const VECTORF32 g_PXASinCoefficients0 = {{-0.05806367563904f, -0.41861972469416f, 0.22480114791621f, 2.17337241360606f}};
static const VECTORF32 g_PXASinCoefficients1 = {{0.61657275907170f, 4.29696498283455f, -1.18942822255452f, -6.53784832094831f}};
static const VECTORF32 g_PXASinCoefficients2 = {{-1.36926553863413f, -4.48179294237210f, 1.41810672941833f, 5.48179257935713f}};
static const VECTORF32 g_PXATanCoefficients0 = {{1.0f, 0.333333334f, 0.2f, 0.142857143f}};
static const VECTORF32 g_PXATanCoefficients1 = {{1.111111111e-1f, 9.090909091e-2f, 7.692307692e-2f, 6.666666667e-2f}};
static const VECTORF32 g_PXATanCoefficients2 = {{5.882352941e-2f, 5.263157895e-2f, 4.761904762e-2f, 4.347826087e-2f}};
static const VECTORF32 g_PXSinEstCoefficients = {{1.0f, -1.66521856991541e-1f, 8.199913018755e-3f, -1.61475937228e-4f}};
static const VECTORF32 g_PXCosEstCoefficients = {{1.0f, -4.95348008918096e-1f, 3.878259962881e-2f, -9.24587976263e-4f}};
static const VECTORF32 g_PXTanEstCoefficients = {{2.484f, -1.954923183e-1f, 2.467401101f, 1.0f / 3.141592655f}};
static const VECTORF32 g_PXATanEstCoefficients = {{7.689891418951e-1f, 1.104742493348f, 8.661844266006e-1f, 3.141592655f * 0.5f}};
static const VECTORF32 g_PXASinEstCoefficients = {{-1.36178272886711f, 2.37949493464538f, -8.08228565650486e-1f, 2.78440142746736e-1f}};
static const VECTORF32 g_PXASinEstConstants = {{1.00000011921f, 3.141592655f * 0.5f, 0.0f, 0.0f}};
static const VECTORF32 g_PXPiConstants0 = {{3.141592655f, 3.141592655f * 2.0f, 1.0f / 3.141592655f, 0.5f / 3.141592655f}};
static const VECTORF32 g_PXReciprocalTwoPi = {{0.5f / 3.141592655f, 0.5f / 3.141592655f, 0.5f / 3.141592655f, 0.5f / 3.141592655f}};
static const VECTORF32 g_PXTwoPi = {{3.141592655f * 2.0f, 3.141592655f * 2.0f, 3.141592655f * 2.0f, 3.141592655f * 2.0f}};

#define VRECIPEQ recipq_newton<1>
#define VRECIPE recip_newton<1>
#define VRECIPSQRTEQ rsqrtq_newton<1>
#define VRECIPSQRTE rsqrt_newton<1>
#define VRECIPQ recipq_newton<4>
#define VRECIP recip_newton<4>
#define VRECIPSQRTQ rsqrtq_newton<4>
#define VRECIPSQRT rsqrt_newton<4>

//////////////////////////////////
// FLOATV
//////////////////////////////////
///
inline FloatV FLoad(const float f)
{
	return vdup_n_f32(reinterpret_cast<const float32_t&>(f));
}

inline FloatV FLoadA(const float* const f)
{
	ASSERT_ISALIGNED16(f);
	return vld1_f32(reinterpret_cast<const float32_t*>(f));
}

inline FloatV FZero()
{
	return FLoad(0.0f);
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
	return vreinterpret_f32_u32(vdup_n_u32(0));
}

inline FloatV IOne()
{
	return vreinterpret_f32_u32(vdup_n_u32(1));
}

inline FloatV ITwo()
{
	return vreinterpret_f32_u32(vdup_n_u32(2));
}

inline FloatV IThree()
{
	return vreinterpret_f32_u32(vdup_n_u32(3));
}

inline FloatV IFour()
{
	return vreinterpret_f32_u32(vdup_n_u32(4));
}

inline FloatV FNeg(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return vneg_f32(f);
}

inline FloatV FAdd(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vadd_f32(a, b);
}

inline FloatV FSub(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vsub_f32(a, b);
}

inline FloatV FMul(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vmul_f32(a, b);
}

inline BoolV FIsEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vdupq_lane_u32(vceq_f32(a, b), 0);
}

inline FloatV FSel(const BoolV c, const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(vbsl_f32(vget_low_u32(c), a, b));
	return vbsl_f32(vget_low_u32(c), a, b);
}

template <int n>
inline float32x2_t recip_newton(const float32x2_t& in)
{
	float32x2_t recip = vrecpe_f32(in);
	for(int i = 0; i < n; ++i)
		recip = vmul_f32(recip, vrecps_f32(in, recip));
	return recip;
}

template <int n>
inline float32x4_t recipq_newton(const float32x4_t& in)
{
	float32x4_t recip = vrecpeq_f32(in);
	for(int i = 0; i < n; ++i)
		recip = vmulq_f32(recip, vrecpsq_f32(recip, in));
	return recip;
}

template <int n>
inline float32x2_t rsqrt_newton(const float32x2_t& in)
{
	float32x2_t rsqrt = vrsqrte_f32(in);
	for(int i = 0; i < n; ++i)
		rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(rsqrt, rsqrt), in));
	return rsqrt;
}

template <int n>
inline float32x4_t rsqrtq_newton(const float32x4_t& in)
{
	float32x4_t rsqrt = vrsqrteq_f32(in);
	for(int i = 0; i < n; ++i)
		rsqrt = vmulq_f32(rsqrt, vrsqrtsq_f32(vmulq_f32(rsqrt, rsqrt), in));
	return rsqrt;
}

inline FloatV FDiv(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vmul_f32(a, VRECIP(b));
}

inline FloatV FDivFast(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vmul_f32(a, VRECIPE(b));
}

inline FloatV FRecip(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return VRECIP(a);
}

inline FloatV FRecipFast(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return VRECIPE(a);
}

inline FloatV FRsqrt(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return VRECIPSQRT(a);
}

inline FloatV FSqrt(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return FSel(FIsEq(a, FZero()), a, vmul_f32(a, VRECIPSQRT(a)));
}

inline FloatV FRsqrtFast(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return VRECIPSQRTE(a);
}

inline FloatV FScaleAdd(const FloatV a, const FloatV b, const FloatV c)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDFLOATV(c);
	return vmla_f32(c, a, b);
}

inline FloatV FNegScaleSub(const FloatV a, const FloatV b, const FloatV c)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDFLOATV(c);
	return vmls_f32(c, a, b);
}

inline FloatV FAbs(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);
	return vabs_f32(a);
}

inline BoolV FIsGrtr(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vdupq_lane_u32(vcgt_f32(a, b), 0);
}

inline BoolV FIsGrtrOrEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vdupq_lane_u32(vcge_f32(a, b), 0);
}

inline FloatV FMax(const FloatV a, const FloatV b)
{
	//ASSERT_ISVALIDFLOATV(a);
	//ASSERT_ISVALIDFLOATV(b);
	return vmax_f32(a, b);
}

inline FloatV FMin(const FloatV a, const FloatV b)
{
	//ASSERT_ISVALIDFLOATV(a);
	//ASSERT_ISVALIDFLOATV(b);
	return vmin_f32(a, b);
}

inline FloatV FClamp(const FloatV a, const FloatV minV, const FloatV maxV)
{
	ASSERT_ISVALIDFLOATV(minV);
	ASSERT_ISVALIDFLOATV(maxV);
	return vmax_f32(vmin_f32(a, maxV), minV);
}

inline unsigned int FAllGrtr(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vget_lane_u32(vcgt_f32(a, b), 0);
}

inline unsigned int FAllGrtrOrEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vget_lane_u32(vcge_f32(a, b), 0);
}

inline unsigned int FAllEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vget_lane_u32(vceq_f32(a, b), 0);
}

inline FloatV FRound(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);

	// truncate(a + (0.5f - sign(a)))
	const float32x2_t half = vdup_n_f32(0.5f);
	const float32x2_t sign = vcvt_f32_u32((vshr_n_u32(vreinterpret_u32_f32(a), 31)));
	const float32x2_t aPlusHalf = vadd_f32(a, half);
	const float32x2_t aRound = vsub_f32(aPlusHalf, sign);
	int32x2_t tmp = vcvt_s32_f32(aRound);
	return vcvt_f32_s32(tmp);
}

inline FloatV FSin(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);

	// Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
	const FloatV recipTwoPi = FLoadA(g_PXReciprocalTwoPi.f);
	const FloatV twoPi = FLoadA(g_PXTwoPi.f);
	const FloatV tmp = FMul(a, recipTwoPi);
	const FloatV b = FRound(tmp);
	const FloatV V1 = FNegScaleSub(twoPi, b, a);

	// sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
	//           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
	const FloatV V2 = FMul(V1, V1);
	const FloatV V3 = FMul(V2, V1);
	const FloatV V5 = FMul(V3, V2);
	const FloatV V7 = FMul(V5, V2);
	const FloatV V9 = FMul(V7, V2);
	const FloatV V11 = FMul(V9, V2);
	const FloatV V13 = FMul(V11, V2);
	const FloatV V15 = FMul(V13, V2);
	const FloatV V17 = FMul(V15, V2);
	const FloatV V19 = FMul(V17, V2);
	const FloatV V21 = FMul(V19, V2);
	const FloatV V23 = FMul(V21, V2);

	const Vec4V sinCoefficients0 = V4LoadA(g_PXSinCoefficients0.f);
	const Vec4V sinCoefficients1 = V4LoadA(g_PXSinCoefficients1.f);
	const Vec4V sinCoefficients2 = V4LoadA(g_PXSinCoefficients2.f);

	const FloatV S1 = V4GetY(sinCoefficients0);
	const FloatV S2 = V4GetZ(sinCoefficients0);
	const FloatV S3 = V4GetW(sinCoefficients0);
	const FloatV S4 = V4GetX(sinCoefficients1);
	const FloatV S5 = V4GetY(sinCoefficients1);
	const FloatV S6 = V4GetZ(sinCoefficients1);
	const FloatV S7 = V4GetW(sinCoefficients1);
	const FloatV S8 = V4GetX(sinCoefficients2);
	const FloatV S9 = V4GetY(sinCoefficients2);
	const FloatV S10 = V4GetZ(sinCoefficients2);
	const FloatV S11 = V4GetW(sinCoefficients2);

	FloatV Result;
	Result = FScaleAdd(S1, V3, V1);
	Result = FScaleAdd(S2, V5, Result);
	Result = FScaleAdd(S3, V7, Result);
	Result = FScaleAdd(S4, V9, Result);
	Result = FScaleAdd(S5, V11, Result);
	Result = FScaleAdd(S6, V13, Result);
	Result = FScaleAdd(S7, V15, Result);
	Result = FScaleAdd(S8, V17, Result);
	Result = FScaleAdd(S9, V19, Result);
	Result = FScaleAdd(S10, V21, Result);
	Result = FScaleAdd(S11, V23, Result);

	return Result;
}

inline FloatV FCos(const FloatV a)
{
	ASSERT_ISVALIDFLOATV(a);

	// Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
	const FloatV recipTwoPi = FLoadA(g_PXReciprocalTwoPi.f);
	const FloatV twoPi = FLoadA(g_PXTwoPi.f);
	const FloatV tmp = FMul(a, recipTwoPi);
	const FloatV b = FRound(tmp);
	const FloatV V1 = FNegScaleSub(twoPi, b, a);

	// cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
	//           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
	const FloatV V2 = FMul(V1, V1);
	const FloatV V4 = FMul(V2, V2);
	const FloatV V6 = FMul(V4, V2);
	const FloatV V8 = FMul(V4, V4);
	const FloatV V10 = FMul(V6, V4);
	const FloatV V12 = FMul(V6, V6);
	const FloatV V14 = FMul(V8, V6);
	const FloatV V16 = FMul(V8, V8);
	const FloatV V18 = FMul(V10, V8);
	const FloatV V20 = FMul(V10, V10);
	const FloatV V22 = FMul(V12, V10);

	const Vec4V cosCoefficients0 = V4LoadA(g_PXCosCoefficients0.f);
	const Vec4V cosCoefficients1 = V4LoadA(g_PXCosCoefficients1.f);
	const Vec4V cosCoefficients2 = V4LoadA(g_PXCosCoefficients2.f);

	const FloatV C1 = V4GetY(cosCoefficients0);
	const FloatV C2 = V4GetZ(cosCoefficients0);
	const FloatV C3 = V4GetW(cosCoefficients0);
	const FloatV C4 = V4GetX(cosCoefficients1);
	const FloatV C5 = V4GetY(cosCoefficients1);
	const FloatV C6 = V4GetZ(cosCoefficients1);
	const FloatV C7 = V4GetW(cosCoefficients1);
	const FloatV C8 = V4GetX(cosCoefficients2);
	const FloatV C9 = V4GetY(cosCoefficients2);
	const FloatV C10 = V4GetZ(cosCoefficients2);
	const FloatV C11 = V4GetW(cosCoefficients2);

	FloatV Result;
	Result = FScaleAdd(C1, V2, FOne());
	Result = FScaleAdd(C2, V4, Result);
	Result = FScaleAdd(C3, V6, Result);
	Result = FScaleAdd(C4, V8, Result);
	Result = FScaleAdd(C5, V10, Result);
	Result = FScaleAdd(C6, V12, Result);
	Result = FScaleAdd(C7, V14, Result);
	Result = FScaleAdd(C8, V16, Result);
	Result = FScaleAdd(C9, V18, Result);
	Result = FScaleAdd(C10, V20, Result);
	Result = FScaleAdd(C11, V22, Result);

	return Result;
}

inline unsigned int FOutOfBounds(const FloatV a, const FloatV min, const FloatV max)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(min);
	ASSERT_ISVALIDFLOATV(max);

	const BoolV c = BOr(FIsGrtr(a, max), FIsGrtr(min, a));
	return (unsigned int)(!BAllEqFFFF(c));
}

inline unsigned int FInBounds(const FloatV a, const FloatV min, const FloatV max)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(min);
	ASSERT_ISVALIDFLOATV(max);

	const BoolV c = BAnd(FIsGrtrOrEq(a, min), FIsGrtrOrEq(max, a));
	return (unsigned int)(BAllEqTTTT(c));
}

inline unsigned int FOutOfBounds(const FloatV a, const FloatV bounds)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(bounds);
	const uint32x2_t greater = vcagt_f32(a, bounds);
	return vget_lane_u32(greater, 0);
}

inline unsigned int FInBounds(const FloatV a, const FloatV bounds)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(bounds);
	const uint32x2_t geq = vcage_f32(bounds, a);
	return vget_lane_u32(geq, 0);
}


//////////////////////////////////
// BoolV
//////////////////////////////////

inline BoolV BFFFF()
{
	return vmovq_n_u32(0);
}

inline BoolV BFFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zeros, zo);
}

inline BoolV BFFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zeros, oz);
}

inline BoolV BFFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(zeros, ones);
}

inline BoolV BFTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zeros);
}

inline BoolV BFTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zo);
}

inline BoolV BFTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zo, oz);
}

inline BoolV BFTTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, ones);
}

inline BoolV BTFFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	// const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zeros);
}

inline BoolV BTFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zo);
}

inline BoolV BTFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, oz);
}

inline BoolV BTFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, ones);
}

inline BoolV BTTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(ones, zeros);
}

inline BoolV BTTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(ones, zo);
}

inline BoolV BTTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(ones, oz);
}

inline BoolV BTTTT()
{
	return vmovq_n_u32(0xffffFFFF);
}

inline BoolV BXMask()
{
	return BTFFF();
}

inline BoolV BYMask()
{
	return BFTFF();
}

inline BoolV BZMask()
{
	return BFFTF();
}

inline BoolV BWMask()
{
	return BFFFT();
}

inline BoolV BGetX(const BoolV f)
{
	const uint32x2_t fLow = vget_low_u32(f);
	return vdupq_lane_u32(fLow, 0);
}

inline BoolV BGetY(const BoolV f)
{
	const uint32x2_t fLow = vget_low_u32(f);
	return vdupq_lane_u32(fLow, 1);
}

inline BoolV BGetZ(const BoolV f)
{
	const uint32x2_t fHigh = vget_high_u32(f);
	return vdupq_lane_u32(fHigh, 0);
}

inline BoolV BGetW(const BoolV f)
{
	const uint32x2_t fHigh = vget_high_u32(f);
	return vdupq_lane_u32(fHigh, 1);
}

inline BoolV BSetX(const BoolV v, const BoolV f)
{
	return vbslq_u32(BFTTT(), v, f);
}

inline BoolV BSetY(const BoolV v, const BoolV f)
{
	return vbslq_u32(BTFTT(), v, f);
}

inline BoolV BSetZ(const BoolV v, const BoolV f)
{
	return vbslq_u32(BTTFT(), v, f);
}

inline BoolV BSetW(const BoolV v, const BoolV f)
{
	return vbslq_u32(BTTTF(), v, f);
}

inline BoolV BAnd(const BoolV a, const BoolV b)
{
	return vandq_u32(a, b);
}

inline BoolV BNot(const BoolV a)
{
	return vmvnq_u32(a);
}

inline BoolV BAndNot(const BoolV a, const BoolV b)
{
	// return vbicq_u32(a, b);
	return vandq_u32(a, vmvnq_u32(b));
}

inline BoolV BOr(const BoolV a, const BoolV b)
{
	return vorrq_u32(a, b);
}

inline BoolV BAllTrue4(const BoolV a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vceq_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

inline BoolV BAnyTrue4(const BoolV a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vtst_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

inline BoolV BAllTrue3(const BoolV a)
{
	const uint32x2_t allTrue3 = vmov_n_u32(0x00ffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vceq_u32(vand_u32(finalReduce, allTrue3), allTrue3);
	return vdupq_lane_u32(result, 0);
}

inline BoolV BAnyTrue3(const BoolV a)
{
	const uint32x2_t allTrue3 = vmov_n_u32(0x00ffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vtst_u32(vand_u32(finalReduce, allTrue3), allTrue3);
	return vdupq_lane_u32(result, 0);
}

inline unsigned int BAllEq(const BoolV a, const BoolV b)
{
	const BoolV bTest = vceqq_u32(a, b);
	return internalUnitNeonSimd::BAllTrue4_R(bTest);
}

inline unsigned int BAllEqTTTT(const BoolV a)
{
	return BAllEq(a, BTTTT());
}

inline unsigned int BAllEqFFFF(const BoolV a)
{
	return BAllEq(a, BFFFF());
}

inline unsigned int BGetBitMask(const BoolV a)
{
	alignas(16) static const unsigned int bitMaskData[4] = { 1, 2, 4, 8 };
	const uint32x4_t bitMask = *(reinterpret_cast<const uint32x4_t*>(bitMaskData));
	const uint32x4_t t0 = vandq_u32(a, bitMask);
	const uint32x2_t t1 = vpadd_u32(vget_low_u32(t0), vget_high_u32(t0)); // Pairwise add (0 + 1), (2 + 3)
	return (unsigned int)(vget_lane_u32(vpadd_u32(t1, t1), 0));
}

//////////////////////////////////
// Vec4V
//////////////////////////////////

inline Vec4V V4Load(const float f)
{
	return vdupq_n_f32(reinterpret_cast<const float32_t&>(f));
}

inline Vec4V V4LoadA(const float* const f)
{
	ASSERT_ISALIGNED16(f);
	return vld1q_f32(reinterpret_cast<const float32_t*>(f));
}

inline void V4StoreA(Vec4V a, float* f)
{
	ASSERT_ISALIGNED16(f);
	vst1q_f32(reinterpret_cast<float32_t*>(f), a);
}

inline Vec4V Vec4V_From_FloatV(FloatV f)
{
	return vcombine_f32(f, f);
}

inline Vec4V Vec4V_From_PxVec3_WUndefined(const Vector3d& f)
{
	alignas(16) float data[4] = { f.x, f.y, f.z, 0.0f };
	return V4LoadA(data);
}

inline Vec4V V4Splat(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return vcombine_f32(f, f);
}

inline Vec4V V4Merge(const FloatV* const floatVArray)
{
	ASSERT_ISVALIDFLOATV(floatVArray[0]);
	ASSERT_ISVALIDFLOATV(floatVArray[1]);
	ASSERT_ISVALIDFLOATV(floatVArray[2]);
	ASSERT_ISVALIDFLOATV(floatVArray[3]);

	const uint32x2_t xLow = vreinterpret_u32_f32(floatVArray[0]);
	const uint32x2_t yLow = vreinterpret_u32_f32(floatVArray[1]);
	const uint32x2_t zLow = vreinterpret_u32_f32(floatVArray[2]);
	const uint32x2_t wLow = vreinterpret_u32_f32(floatVArray[3]);

	const uint32x2_t dLow = vext_u32(xLow, yLow, 1);
	const uint32x2_t dHigh = vext_u32(zLow, wLow, 1);

	return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}

inline Vec4V V4Merge(const FloatV& x, const FloatV& y, const FloatV& z, const FloatV& w)
{
	ASSERT_ISVALIDFLOATV(x);
	ASSERT_ISVALIDFLOATV(y);
	ASSERT_ISVALIDFLOATV(z);
	ASSERT_ISVALIDFLOATV(w);

	const uint32x2_t xLow = vreinterpret_u32_f32(x);
	const uint32x2_t yLow = vreinterpret_u32_f32(y);
	const uint32x2_t zLow = vreinterpret_u32_f32(z);
	const uint32x2_t wLow = vreinterpret_u32_f32(w);

	const uint32x2_t dLow = vext_u32(xLow, yLow, 1);
	const uint32x2_t dHigh = vext_u32(zLow, wLow, 1);

	return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}

inline Vec4V V4MergeW(const Vec4V& x, const Vec4V& y, const Vec4V& z, const Vec4V& w)
{
	const float32x2_t xx = vget_high_f32(x);
	const float32x2_t yy = vget_high_f32(y);
	const float32x2_t zz = vget_high_f32(z);
	const float32x2_t ww = vget_high_f32(w);

	const float32x2x2_t zipL = vzip_f32(xx, yy);
	const float32x2x2_t zipH = vzip_f32(zz, ww);

	return vcombine_f32(zipL.val[1], zipH.val[1]);
}

inline Vec4V V4MergeZ(const Vec4V& x, const Vec4V& y, const Vec4V& z, const Vec4V& w)
{
	const float32x2_t xx = vget_high_f32(x);
	const float32x2_t yy = vget_high_f32(y);
	const float32x2_t zz = vget_high_f32(z);
	const float32x2_t ww = vget_high_f32(w);

	const float32x2x2_t zipL = vzip_f32(xx, yy);
	const float32x2x2_t zipH = vzip_f32(zz, ww);

	return vcombine_f32(zipL.val[0], zipH.val[0]);
}

inline Vec4V V4MergeY(const Vec4V& x, const Vec4V& y, const Vec4V& z, const Vec4V& w)
{
	const float32x2_t xx = vget_low_f32(x);
	const float32x2_t yy = vget_low_f32(y);
	const float32x2_t zz = vget_low_f32(z);
	const float32x2_t ww = vget_low_f32(w);

	const float32x2x2_t zipL = vzip_f32(xx, yy);
	const float32x2x2_t zipH = vzip_f32(zz, ww);

	return vcombine_f32(zipL.val[1], zipH.val[1]);
}

inline Vec4V V4MergeX(const Vec4V& x, const Vec4V& y, const Vec4V& z, const Vec4V& w)
{
	const float32x2_t xx = vget_low_f32(x);
	const float32x2_t yy = vget_low_f32(y);
	const float32x2_t zz = vget_low_f32(z);
	const float32x2_t ww = vget_low_f32(w);

	const float32x2x2_t zipL = vzip_f32(xx, yy);
	const float32x2x2_t zipH = vzip_f32(zz, ww);

	return vcombine_f32(zipL.val[0], zipH.val[0]);
}

inline Vec4V V4UnpackXY(const Vec4V& a, const Vec4V& b)
{
	return vzipq_f32(a, b).val[0];
}

inline Vec4V V4UnpackZW(const Vec4V& a, const Vec4V& b)
{
	return vzipq_f32(a, b).val[1];
}

inline Vec4V V4UnitW()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t zo = vext_f32(zeros, ones, 1);
	return vcombine_f32(zeros, zo);
}

inline Vec4V V4UnitX()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t oz = vext_f32(ones, zeros, 1);
	return vcombine_f32(oz, zeros);
}

inline Vec4V V4UnitY()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t zo = vext_f32(zeros, ones, 1);
	return vcombine_f32(zo, zeros);
}

inline Vec4V V4UnitZ()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t oz = vext_f32(ones, zeros, 1);
	return vcombine_f32(zeros, oz);
}

inline Vec4V V4PermYXWZ(const Vec4V a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2_t yx = vext_f32(xy, xy, 1);
	const float32x2_t wz = vext_f32(zw, zw, 1);
	return vcombine_f32(yx, wz);
}

inline Vec4V V4PermXZXZ(const Vec4V a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[0], xzyw.val[0]);
}

inline Vec4V V4PermYWYW(const Vec4V a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[1], xzyw.val[1]);
}

inline Vec4V V4PermYZXW(const Vec4V a)
{
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t yz = vext_u32(xy, zw, 1);
	const uint32x2_t xw = vrev64_u32(vext_u32(zw, xy, 1));
	return vreinterpretq_f32_u32(vcombine_u32(yz, xw));
}

inline Vec4V V4PermZWXY(const Vec4V a)
{
	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);
	return vcombine_f32(high, low);
}

template <unsigned char E0, unsigned char E1, unsigned char E2, unsigned char E3>
inline Vec4V V4Perm(const Vec4V V)
{
	static const uint32_t ControlElement[4] =
	{
#if 1
		0x03020100, // XM_SWIZZLE_X
		0x07060504, // XM_SWIZZLE_Y
		0x0B0A0908, // XM_SWIZZLE_Z
		0x0F0E0D0C, // XM_SWIZZLE_W
#else
		0x00010203, // XM_SWIZZLE_X
		0x04050607, // XM_SWIZZLE_Y
		0x08090A0B, // XM_SWIZZLE_Z
		0x0C0D0E0F, // XM_SWIZZLE_W
#endif
	};

	uint8x8x2_t tbl;
	tbl.val[0] = vreinterpret_u8_f32(vget_low_f32(V));
	tbl.val[1] = vreinterpret_u8_f32(vget_high_f32(V));

	uint8x8_t idx =
	    vcreate_u8(static_cast<uint64_t>(ControlElement[E0]) | (static_cast<uint64_t>(ControlElement[E1]) << 32));
	const uint8x8_t rL = vtbl2_u8(tbl, idx);
	idx = vcreate_u8(static_cast<uint64_t>(ControlElement[E2]) | (static_cast<uint64_t>(ControlElement[E3]) << 32));
	const uint8x8_t rH = vtbl2_u8(tbl, idx);
	return vreinterpretq_f32_u8(vcombine_u8(rL, rH));
}

inline Vec4V V4Zero()
{
	return vreinterpretq_f32_u32(vmovq_n_u32(0));
	//	return vmovq_n_f32(0.0f);
}

inline Vec4V V4One()
{
	return vmovq_n_f32(1.0f);
}

inline Vec4V V4Eps()
{
	//	return vmovq_n_f32(PX_EPS_REAL);
	return V4Load(1e-7);
}

inline Vec4V V4Neg(const Vec4V f)
{
	return vnegq_f32(f);
}

inline Vec4V V4Add(const Vec4V a, const Vec4V b)
{
	return vaddq_f32(a, b);
}

inline Vec4V V4Sub(const Vec4V a, const Vec4V b)
{
	return vsubq_f32(a, b);
}

inline Vec4V V4Scale(const Vec4V a, const FloatV b)
{
	return vmulq_lane_f32(a, b, 0);
}

inline Vec4V V4Mul(const Vec4V a, const Vec4V b)
{
	return vmulq_f32(a, b);
}

inline Vec4V V4ScaleInv(const Vec4V a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIP(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline Vec4V V4Div(const Vec4V a, const Vec4V b)
{
	const float32x4_t invB = VRECIPQ(b);
	return vmulq_f32(a, invB);
}

inline Vec4V V4ScaleInvFast(const Vec4V a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIPE(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline Vec4V V4DivFast(const Vec4V a, const Vec4V b)
{
	const float32x4_t invB = VRECIPEQ(b);
	return vmulq_f32(a, invB);
}

inline Vec4V V4Recip(const Vec4V a)
{
	return VRECIPQ(a);
}

inline Vec4V V4RecipFast(const Vec4V a)
{
	return VRECIPEQ(a);
}

inline Vec4V V4Rsqrt(const Vec4V a)
{
	return VRECIPSQRTQ(a);
}

inline Vec4V V4RsqrtFast(const Vec4V a)
{
	return VRECIPSQRTEQ(a);
}

inline Vec4V V4Sqrt(const Vec4V a)
{
	return V4Sel(V4IsEq(a, V4Zero()), a, V4Mul(a, VRECIPSQRTQ(a)));
}

inline Vec4V V4ScaleAdd(const Vec4V a, const FloatV b, const Vec4V c)
{
	ASSERT_ISVALIDFLOATV(b);
	return vmlaq_lane_f32(c, a, b, 0);
}

inline Vec4V V4NegScaleSub(const Vec4V a, const FloatV b, const Vec4V c)
{
	ASSERT_ISVALIDFLOATV(b);
	return vmlsq_lane_f32(c, a, b, 0);
}

inline Vec4V V4MulAdd(const Vec4V a, const Vec4V b, const Vec4V c)
{
	return vmlaq_f32(c, a, b);
}

inline Vec4V V4NegMulSub(const Vec4V a, const Vec4V b, const Vec4V c)
{
	return vmlsq_f32(c, a, b);
}

inline FloatV V4GetW(const Vec4V f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 1);
}

inline FloatV V4GetX(const Vec4V f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 0);
}

inline FloatV V4GetY(const Vec4V f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 1);
}

inline FloatV V4GetZ(const Vec4V f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 0);
}

inline Vec4V V4SetW(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTTF(), v, vcombine_f32(f, f));
}

inline Vec4V V4SetX(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BFTTT(), v, vcombine_f32(f, f));
}

inline Vec4V V4SetY(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTFTT(), v, vcombine_f32(f, f));
}

inline Vec4V V4SetZ(const Vec4V v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTFT(), v, vcombine_f32(f, f));
}

inline Vec4V V4ClearW(const Vec4V v)
{
	return V4Sel(BTTTF(), v, V4Zero());
}


inline Vec4V V4Abs(const Vec4V a)
{
	return vabsq_f32(a);
}

inline FloatV V4SumElements(const Vec4V a)
{
	const Vec4V xy = V4UnpackXY(a, a); // x,x,y,y
	const Vec4V zw = V4UnpackZW(a, a); // z,z,w,w
	const Vec4V xz_yw = V4Add(xy, zw); // x+z,x+z,y+w,y+w
	const FloatV xz = V4GetX(xz_yw);   // x+z
	const FloatV yw = V4GetZ(xz_yw);   // y+w
	return FAdd(xz, yw);               // sum
}

template <int index>
inline Vec4V V4SplatElement(Vec4V a)
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

inline FloatV V4Dot(const Vec4V a, const Vec4V b)
{
	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {z+w, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z+w, x+y+z+w}
	return sumWZYX;
}

inline FloatV V4Dot3(const Vec4V aa, const Vec4V bb)
{
	// PT: the V3Dot code relies on the fact that W=0 so we can't reuse it as-is, we need to clear W first.
	// TODO: find a better implementation that does not need to clear W.
	const Vec4V a = V4ClearW(aa);
	const Vec4V b = V4ClearW(bb);

	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return sum0ZYX;
}

inline Vec4V V4Cross(const Vec4V a, const Vec4V b)
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

inline FloatV V4Length(const Vec4V a)
{
	const float32x4_t tmp = vmulq_f32(a, a);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return FSqrt(sumWZYX);
}

inline FloatV V4LengthSq(const Vec4V a)
{
	return V4Dot(a, a);
}

inline Vec4V V4Normalize(const Vec4V a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return V4ScaleInv(a, V4Length(a));
}

inline Vec4V V4NormalizeFast(const Vec4V a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return V4Scale(a, FRsqrtFast(V4Dot(a, a)));
}

inline Vec4V V4NormalizeSafe(const Vec4V a, const Vec4V unsafeReturnValue)
{
	const FloatV zero = FZero();
	const FloatV length = V4Length(a);
	const uint32x4_t isGreaterThanZero = FIsGrtr(length, zero);
	return V4Sel(isGreaterThanZero, V4ScaleInv(a, length), unsafeReturnValue);
}

inline BoolV V4IsEqU32(const VecU32V a, const VecU32V b)
{
	return vceqq_u32(a, b);
}

inline Vec4V V4Sel(const BoolV c, const Vec4V a, const Vec4V b)
{
	return vbslq_f32(c, a, b);
}

inline BoolV V4IsGrtr(const Vec4V a, const Vec4V b)
{
	return vcgtq_f32(a, b);
}

inline BoolV V4IsGrtrOrEq(const Vec4V a, const Vec4V b)
{
	return vcgeq_f32(a, b);
}

inline BoolV V4IsEq(const Vec4V a, const Vec4V b)
{
	return vceqq_f32(a, b);
}

inline Vec4V V4Max(const Vec4V a, const Vec4V b)
{
	return vmaxq_f32(a, b);
}

inline Vec4V V4Min(const Vec4V a, const Vec4V b)
{
	return vminq_f32(a, b);
}

inline FloatV V4ExtractMax(const Vec4V a)
{
	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);

	const float32x2_t max0 = vpmax_f32(high, low);
	const float32x2_t max1 = vpmax_f32(max0, max0);

	return max1;
}

inline FloatV V4ExtractMin(const Vec4V a)
{
	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);

	const float32x2_t min0 = vpmin_f32(high, low);
	const float32x2_t min1 = vpmin_f32(min0, min0);

	return min1;
}

inline Vec4V V4Clamp(const Vec4V a, const Vec4V minV, const Vec4V maxV)
{
	return V4Max(V4Min(a, maxV), minV);
}

inline unsigned int V4AllGrtr(const Vec4V a, const Vec4V b)
{
	return internalUnitNeonSimd::BAllTrue4_R(V4IsGrtr(a, b));
}

inline unsigned int V4AllGrtrOrEq(const Vec4V a, const Vec4V b)
{
	return internalUnitNeonSimd::BAllTrue4_R(V4IsGrtrOrEq(a, b));
}

inline unsigned int V4AllGrtrOrEq3(const Vec4V a, const Vec4V b)
{
	return internalUnitNeonSimd::BAllTrue3_R(V4IsGrtrOrEq(a, b));
}

inline unsigned int V4AllEq(const Vec4V a, const Vec4V b)
{
	return internalUnitNeonSimd::BAllTrue4_R(V4IsEq(a, b));
}

inline unsigned int V4AnyGrtr3(const Vec4V a, const Vec4V b)
{
	return internalUnitNeonSimd::BAnyTrue3_R(V4IsGrtr(a, b));
}

inline Vec4V V4Round(const Vec4V a)
{
	// truncate(a + (0.5f - sign(a)))
	const Vec4V half = V4Load(0.5f);
	const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
	const Vec4V aPlusHalf = V4Add(a, half);
	const Vec4V aRound = V4Sub(aPlusHalf, sign);
	return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}

inline Vec4V V4Sin(const Vec4V a)
{
	const Vec4V recipTwoPi = V4LoadA(g_PXReciprocalTwoPi.f);
	const Vec4V twoPi = V4LoadA(g_PXTwoPi.f);
	const Vec4V tmp = V4Mul(a, recipTwoPi);
	const Vec4V b = V4Round(tmp);
	const Vec4V V1 = V4NegMulSub(twoPi, b, a);

	// sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
	//           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
	const Vec4V V2 = V4Mul(V1, V1);
	const Vec4V V3 = V4Mul(V2, V1);
	const Vec4V V5 = V4Mul(V3, V2);
	const Vec4V V7 = V4Mul(V5, V2);
	const Vec4V V9 = V4Mul(V7, V2);
	const Vec4V V11 = V4Mul(V9, V2);
	const Vec4V V13 = V4Mul(V11, V2);
	const Vec4V V15 = V4Mul(V13, V2);
	const Vec4V V17 = V4Mul(V15, V2);
	const Vec4V V19 = V4Mul(V17, V2);
	const Vec4V V21 = V4Mul(V19, V2);
	const Vec4V V23 = V4Mul(V21, V2);

	const Vec4V sinCoefficients0 = V4LoadA(g_PXSinCoefficients0.f);
	const Vec4V sinCoefficients1 = V4LoadA(g_PXSinCoefficients1.f);
	const Vec4V sinCoefficients2 = V4LoadA(g_PXSinCoefficients2.f);

	const FloatV S1 = V4GetY(sinCoefficients0);
	const FloatV S2 = V4GetZ(sinCoefficients0);
	const FloatV S3 = V4GetW(sinCoefficients0);
	const FloatV S4 = V4GetX(sinCoefficients1);
	const FloatV S5 = V4GetY(sinCoefficients1);
	const FloatV S6 = V4GetZ(sinCoefficients1);
	const FloatV S7 = V4GetW(sinCoefficients1);
	const FloatV S8 = V4GetX(sinCoefficients2);
	const FloatV S9 = V4GetY(sinCoefficients2);
	const FloatV S10 = V4GetZ(sinCoefficients2);
	const FloatV S11 = V4GetW(sinCoefficients2);

	Vec4V Result;
	Result = V4ScaleAdd(V3, S1, V1);
	Result = V4ScaleAdd(V5, S2, Result);
	Result = V4ScaleAdd(V7, S3, Result);
	Result = V4ScaleAdd(V9, S4, Result);
	Result = V4ScaleAdd(V11, S5, Result);
	Result = V4ScaleAdd(V13, S6, Result);
	Result = V4ScaleAdd(V15, S7, Result);
	Result = V4ScaleAdd(V17, S8, Result);
	Result = V4ScaleAdd(V19, S9, Result);
	Result = V4ScaleAdd(V21, S10, Result);
	Result = V4ScaleAdd(V23, S11, Result);

	return Result;
}

inline Vec4V V4Cos(const Vec4V a)
{
	const Vec4V recipTwoPi = V4LoadA(g_PXReciprocalTwoPi.f);
	const Vec4V twoPi = V4LoadA(g_PXTwoPi.f);
	const Vec4V tmp = V4Mul(a, recipTwoPi);
	const Vec4V b = V4Round(tmp);
	const Vec4V V1 = V4NegMulSub(twoPi, b, a);

	// cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
	//           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
	const Vec4V V2 = V4Mul(V1, V1);
	const Vec4V V4 = V4Mul(V2, V2);
	const Vec4V V6 = V4Mul(V4, V2);
	const Vec4V V8 = V4Mul(V4, V4);
	const Vec4V V10 = V4Mul(V6, V4);
	const Vec4V V12 = V4Mul(V6, V6);
	const Vec4V V14 = V4Mul(V8, V6);
	const Vec4V V16 = V4Mul(V8, V8);
	const Vec4V V18 = V4Mul(V10, V8);
	const Vec4V V20 = V4Mul(V10, V10);
	const Vec4V V22 = V4Mul(V12, V10);

	const Vec4V cosCoefficients0 = V4LoadA(g_PXCosCoefficients0.f);
	const Vec4V cosCoefficients1 = V4LoadA(g_PXCosCoefficients1.f);
	const Vec4V cosCoefficients2 = V4LoadA(g_PXCosCoefficients2.f);

	const FloatV C1 = V4GetY(cosCoefficients0);
	const FloatV C2 = V4GetZ(cosCoefficients0);
	const FloatV C3 = V4GetW(cosCoefficients0);
	const FloatV C4 = V4GetX(cosCoefficients1);
	const FloatV C5 = V4GetY(cosCoefficients1);
	const FloatV C6 = V4GetZ(cosCoefficients1);
	const FloatV C7 = V4GetW(cosCoefficients1);
	const FloatV C8 = V4GetX(cosCoefficients2);
	const FloatV C9 = V4GetY(cosCoefficients2);
	const FloatV C10 = V4GetZ(cosCoefficients2);
	const FloatV C11 = V4GetW(cosCoefficients2);

	Vec4V Result;
	Result = V4ScaleAdd(V2, C1, V4One());
	Result = V4ScaleAdd(V4, C2, Result);
	Result = V4ScaleAdd(V6, C3, Result);
	Result = V4ScaleAdd(V8, C4, Result);
	Result = V4ScaleAdd(V10, C5, Result);
	Result = V4ScaleAdd(V12, C6, Result);
	Result = V4ScaleAdd(V14, C7, Result);
	Result = V4ScaleAdd(V16, C8, Result);
	Result = V4ScaleAdd(V18, C9, Result);
	Result = V4ScaleAdd(V20, C10, Result);
	Result = V4ScaleAdd(V22, C11, Result);

	return Result;
}

inline void V4Transpose(Vec4V& col0, Vec4V& col1, Vec4V& col2, Vec4V& col3)
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



//////////////////////////////////
// Vec3V
//////////////////////////////////

inline Vec3V V3Load(const float f)
{
	alignas(16) float data[4] = { f, f, f, 0.0f };
	return V4LoadA(data);
}

inline Vec3V Vec3V_From_Vec4V(Vec4V v)
{
	return vsetq_lane_f32(0.0f, v, 3);
}

inline Vec3V Vec3V_From_FloatV(FloatV f)
{
	return Vec3V_From_Vec4V(Vec4V_From_FloatV(f));
}

inline Vec3V V3Splat(const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);

	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t uHigh = vreinterpret_u32_f32(f);
	const float32x2_t dHigh = vreinterpret_f32_u32(vand_u32(uHigh, mask));

	return vcombine_f32(f, dHigh);
}

inline Vec3V V3Merge(const FloatV &x, const FloatV &y, const FloatV &z)
{
	ASSERT_ISVALIDFLOATV(x);
	ASSERT_ISVALIDFLOATV(y);
	ASSERT_ISVALIDFLOATV(z);

	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t dHigh = vand_u32(vreinterpret_u32_f32(z), mask);
	const uint32x2_t dLow = vext_u32(vreinterpret_u32_f32(x), vreinterpret_u32_f32(y), 1);
	return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}

inline Vec3V V3UnitX()
{
	const float32x4_t x = { 1.0f, 0.0f, 0.0f, 0.0f };
	return x;
}

inline Vec3V V3UnitY()
{
	const float32x4_t y = { 0, 1.0f, 0, 0 };
	return y;
}

inline Vec3V V3UnitZ()
{
	const float32x4_t z = { 0, 0, 1.0f, 0 };
	return z;
}

inline FloatV V3GetX(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 0);
}

inline FloatV V3GetY(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 1);
}

inline FloatV V3GetZ(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 0);
}

inline Vec3V V3SetX(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BFTTT(), v, vcombine_f32(f, f));
}

inline Vec3V V3SetY(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTFTT(), v, vcombine_f32(f, f));
}

inline Vec3V V3SetZ(const Vec3V v, const FloatV f)
{
	ASSERT_ISVALIDVEC3V(v);
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTFT(), v, vcombine_f32(f, f));
}

inline Vec3V V3ColX(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);

	const float32x2_t aLow = vget_low_f32(a);
	const float32x2_t bLow = vget_low_f32(b);
	const float32x2_t cLow = vget_low_f32(c);
	const float32x2_t zero = vdup_n_f32(0.0f);

	const float32x2x2_t zipL = vzip_f32(aLow, bLow);
	const float32x2x2_t zipH = vzip_f32(cLow, zero);

	return vcombine_f32(zipL.val[0], zipH.val[0]);
}

inline Vec3V V3ColY(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);

	const float32x2_t aLow = vget_low_f32(a);
	const float32x2_t bLow = vget_low_f32(b);
	const float32x2_t cLow = vget_low_f32(c);
	const float32x2_t zero = vdup_n_f32(0.0f);

	const float32x2x2_t zipL = vzip_f32(aLow, bLow);
	const float32x2x2_t zipH = vzip_f32(cLow, zero);

	return vcombine_f32(zipL.val[1], zipH.val[1]);
}

inline Vec3V V3ColZ(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);

	const float32x2_t aHi = vget_high_f32(a);
	const float32x2_t bHi = vget_high_f32(b);
	const float32x2_t cHi = vget_high_f32(c);

	const float32x2x2_t zipL = vzip_f32(aHi, bHi);

	return vcombine_f32(zipL.val[0], cHi);
}

inline Vec3V V3Zero()
{
	return vdupq_n_f32(0.0f);
}

inline Vec3V V3Eps()
{
	return V3Load(1e-7);
}

inline Vec3V V3One()
{
	return V3Load(1.0f);
}

inline Vec3V V3Neg(const Vec3V f)
{
	ASSERT_ISVALIDVEC3V(f);
	const float32x4_t tmp = vnegq_f32(f);
	return vsetq_lane_f32(0.0f, tmp, 3);
}

inline Vec3V V3Add(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vaddq_f32(a, b);
}

inline Vec3V V3Add(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	return vaddq_f32(a, Vec3V_From_FloatV(b));
}

inline Vec3V V3Sub(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vsubq_f32(a, b);
}

inline Vec3V V3Sub(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	return vsubq_f32(a, Vec3V_From_FloatV(b));
}

inline Vec3V V3Scale(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	const float32x4_t tmp = vmulq_lane_f32(a, b, 0);
	return vsetq_lane_f32(0.0f, tmp, 3);
}

inline Vec3V V3Mul(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vmulq_f32(a, b);
}

inline Vec3V V3ScaleInv(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIP(b);
	const float32x4_t tmp = vmulq_lane_f32(a, invB, 0);
	return vsetq_lane_f32(0.0f, tmp, 3);
}

inline Vec3V V3Div(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	float32x4_t invB = VRECIPQ(b);
	invB = vsetq_lane_f32(0.0f, invB, 3);
	return vmulq_f32(a, invB);
}

inline Vec3V V3ScaleInvFast(const Vec3V a, const FloatV b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIPE(b);
	const float32x4_t tmp = vmulq_lane_f32(a, invB, 0);
	return vsetq_lane_f32(0.0f, tmp, 3);
}

inline Vec3V V3DivFast(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	float32x4_t invB = VRECIPEQ(b);
	invB = vsetq_lane_f32(0.0f, invB, 3);
	return vmulq_f32(a, invB);
}

inline Vec3V V3Recip(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const float32x4_t recipA = VRECIPQ(a);
	return vsetq_lane_f32(0.0f, recipA, 3);
}

inline Vec3V V3RecipFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const float32x4_t recipA = VRECIPEQ(a);
	return vsetq_lane_f32(0.0f, recipA, 3);
}

inline Vec3V V3Rsqrt(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const float32x4_t rSqrA = VRECIPSQRTQ(a);
	return vsetq_lane_f32(0.0f, rSqrA, 3);
}

inline Vec3V V3RsqrtFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const float32x4_t rSqrA = VRECIPSQRTEQ(a);
	return vsetq_lane_f32(0.0f, rSqrA, 3);
}

inline Vec3V V3ScaleAdd(const Vec3V a, const FloatV b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDVEC3V(c);

	float32x4_t tmp = vmlaq_lane_f32(c, a, b, 0);
	// using vsetq_lane_f32 resulted in failures,
	// probably related to a compiler bug on
	// ndk r9d-win32, gcc 4.8, cardhu/shield

	// code with issue
	// return vsetq_lane_f32(0.0f, tmp, 3);

	// workaround
	float32x2_t w_z = vget_high_f32(tmp);
	float32x2_t y_x = vget_low_f32(tmp);
	w_z = vset_lane_f32(0.0f, w_z, 1);
	return vcombine_f32(y_x, w_z);
}

inline Vec3V V3NegScaleSub(const Vec3V a, const FloatV b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDFLOATV(b);
	ASSERT_ISVALIDVEC3V(c);

	float32x4_t tmp = vmlsq_lane_f32(c, a, b, 0);
	// using vsetq_lane_f32 resulted in failures,
	// probably related to a compiler bug on
	// ndk r9d-win32, gcc 4.8, cardhu/shield

	// code with issue
	// return vsetq_lane_f32(0.0f, tmp, 3);

	// workaround
	float32x2_t w_z = vget_high_f32(tmp);
	float32x2_t y_x = vget_low_f32(tmp);
	w_z = vset_lane_f32(0.0f, w_z, 1);
	return vcombine_f32(y_x, w_z);
}

inline Vec3V V3MulAdd(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	return vmlaq_f32(c, a, b);
}

inline Vec3V V3NegMulSub(const Vec3V a, const Vec3V b, const Vec3V c)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	ASSERT_ISVALIDVEC3V(c);
	return vmlsq_f32(c, a, b);
}

inline Vec3V V3Abs(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return vabsq_f32(a);
}

inline FloatV V3Dot(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	
	//	const uint32x2_t mask = {0xffffFFFF, 0x0};
	const float32x4_t tmp = vmulq_f32(a, b);

	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);
	//	const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}

	return sum0ZYX;
}

inline Vec3V V3Cross(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);

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

inline Vec3V V3PrepareCross(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return a;
}

inline FloatV V3Length(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	//	const uint32x2_t mask = {0xffffFFFF, 0x0};

	const float32x4_t tmp = vmulq_f32(a, a);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);
	//	const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}

	return FSqrt(sum0ZYX);
}

inline FloatV V3LengthSq(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	return V3Dot(a, a);
}

inline Vec3V V3Normalize(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return V3ScaleInv(a, V3Length(a));
}

inline Vec3V V3NormalizeFast(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return V3Scale(a, VRECIPSQRTE(V3Dot(a, a)));
}

inline Vec3V V3NormalizeSafe(const Vec3V a, const Vec3V unsafeReturnValue)
{
	ASSERT_ISVALIDVEC3V(a);
	const FloatV zero = vdup_n_f32(0.0f);
	const FloatV length = V3Length(a);
	const uint32x4_t isGreaterThanZero = FIsGrtr(length, zero);
	return V3Sel(isGreaterThanZero, V3ScaleInv(a, length), unsafeReturnValue);
}

inline Vec3V V3Sel(const BoolV c, const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V( vbslq_f32(c, a, b));
	return vbslq_f32(c, a, b);
}

inline BoolV V3IsGrtr(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vcgtq_f32(a, b);
}

inline BoolV V3IsGrtrOrEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vcgeq_f32(a, b);
}

inline BoolV V3IsEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vceqq_f32(a, b);
}

inline Vec3V V3Max(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vmaxq_f32(a, b);
}

inline Vec3V V3Min(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return vminq_f32(a, b);
}

inline FloatV V3ExtractMax(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);

	const float32x2_t zz = vdup_lane_f32(high, 0);
	const float32x2_t max0 = vpmax_f32(zz, low);
	const float32x2_t max1 = vpmax_f32(max0, max0);

	return max1;
}

inline FloatV V3ExtractMin(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);

	const float32x2_t zz = vdup_lane_f32(high, 0);
	const float32x2_t min0 = vpmin_f32(zz, low);
	const float32x2_t min1 = vpmin_f32(min0, min0);

	return min1;
}

// return (a >= 0.0f) ? 1.0f : -1.0f;
inline Vec3V V3Sign(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const Vec3V zero = V3Zero();
	const Vec3V one = V3One();
	const Vec3V none = V3Neg(one);
	return V3Sel(V3IsGrtrOrEq(a, zero), one, none);
}

inline Vec3V V3Clamp(const Vec3V a, const Vec3V minV, const Vec3V maxV)
{
	ASSERT_ISVALIDVEC3V(minV);
	ASSERT_ISVALIDVEC3V(maxV);
	return V3Max(V3Min(a, maxV), minV);
}

inline unsigned int V3AllGrtr(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalUnitNeonSimd::BAllTrue3_R(V4IsGrtr(a, b));
}

inline unsigned int V3AllGrtrOrEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalUnitNeonSimd::BAllTrue3_R(V4IsGrtrOrEq(a, b));
}

inline unsigned int V3AllEq(const Vec3V a, const Vec3V b)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(b);
	return internalUnitNeonSimd::BAllTrue3_R(V4IsEq(a, b));
}

inline Vec3V V3Round(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	// truncate(a + (0.5f - sign(a)))
	const Vec3V half = V3Load(0.5f);
	const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
	const Vec3V aPlusHalf = V3Add(a, half);
	const Vec3V aRound = V3Sub(aPlusHalf, sign);
	return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}

inline Vec3V V3Sin(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	// Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
	const Vec4V recipTwoPi = V4LoadA(g_PXReciprocalTwoPi.f);
	const Vec4V twoPi = V4LoadA(g_PXTwoPi.f);
	const Vec3V tmp = V4Mul(a, recipTwoPi);
	const Vec3V b = V3Round(tmp);
	const Vec3V V1 = V4NegMulSub(twoPi, b, a);

	// sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
	//           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
	const Vec3V V2 = V3Mul(V1, V1);
	const Vec3V V3 = V3Mul(V2, V1);
	const Vec3V V5 = V3Mul(V3, V2);
	const Vec3V V7 = V3Mul(V5, V2);
	const Vec3V V9 = V3Mul(V7, V2);
	const Vec3V V11 = V3Mul(V9, V2);
	const Vec3V V13 = V3Mul(V11, V2);
	const Vec3V V15 = V3Mul(V13, V2);
	const Vec3V V17 = V3Mul(V15, V2);
	const Vec3V V19 = V3Mul(V17, V2);
	const Vec3V V21 = V3Mul(V19, V2);
	const Vec3V V23 = V3Mul(V21, V2);

	const Vec4V sinCoefficients0 = V4LoadA(g_PXSinCoefficients0.f);
	const Vec4V sinCoefficients1 = V4LoadA(g_PXSinCoefficients1.f);
	const Vec4V sinCoefficients2 = V4LoadA(g_PXSinCoefficients2.f);

	const FloatV S1 = V4GetY(sinCoefficients0);
	const FloatV S2 = V4GetZ(sinCoefficients0);
	const FloatV S3 = V4GetW(sinCoefficients0);
	const FloatV S4 = V4GetX(sinCoefficients1);
	const FloatV S5 = V4GetY(sinCoefficients1);
	const FloatV S6 = V4GetZ(sinCoefficients1);
	const FloatV S7 = V4GetW(sinCoefficients1);
	const FloatV S8 = V4GetX(sinCoefficients2);
	const FloatV S9 = V4GetY(sinCoefficients2);
	const FloatV S10 = V4GetZ(sinCoefficients2);
	const FloatV S11 = V4GetW(sinCoefficients2);

	Vec3V Result;
	Result = V4ScaleAdd(V3, S1, V1);
	Result = V4ScaleAdd(V5, S2, Result);
	Result = V4ScaleAdd(V7, S3, Result);
	Result = V4ScaleAdd(V9, S4, Result);
	Result = V4ScaleAdd(V11, S5, Result);
	Result = V4ScaleAdd(V13, S6, Result);
	Result = V4ScaleAdd(V15, S7, Result);
	Result = V4ScaleAdd(V17, S8, Result);
	Result = V4ScaleAdd(V19, S9, Result);
	Result = V4ScaleAdd(V21, S10, Result);
	Result = V4ScaleAdd(V23, S11, Result);

	return Result;
}

inline Vec3V V3Cos(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	
	// Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
	const Vec4V recipTwoPi = V4LoadA(g_PXReciprocalTwoPi.f);
	const Vec4V twoPi = V4LoadA(g_PXTwoPi.f);
	const Vec3V tmp = V4Mul(a, recipTwoPi);
	const Vec3V b = V3Round(tmp);
	const Vec3V V1 = V4NegMulSub(twoPi, b, a);

	// cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
	//           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
	const Vec3V V2 = V3Mul(V1, V1);
	const Vec3V V4 = V3Mul(V2, V2);
	const Vec3V V6 = V3Mul(V4, V2);
	const Vec3V V8 = V3Mul(V4, V4);
	const Vec3V V10 = V3Mul(V6, V4);
	const Vec3V V12 = V3Mul(V6, V6);
	const Vec3V V14 = V3Mul(V8, V6);
	const Vec3V V16 = V3Mul(V8, V8);
	const Vec3V V18 = V3Mul(V10, V8);
	const Vec3V V20 = V3Mul(V10, V10);
	const Vec3V V22 = V3Mul(V12, V10);

	const Vec4V cosCoefficients0 = V4LoadA(g_PXCosCoefficients0.f);
	const Vec4V cosCoefficients1 = V4LoadA(g_PXCosCoefficients1.f);
	const Vec4V cosCoefficients2 = V4LoadA(g_PXCosCoefficients2.f);

	const FloatV C1 = V4GetY(cosCoefficients0);
	const FloatV C2 = V4GetZ(cosCoefficients0);
	const FloatV C3 = V4GetW(cosCoefficients0);
	const FloatV C4 = V4GetX(cosCoefficients1);
	const FloatV C5 = V4GetY(cosCoefficients1);
	const FloatV C6 = V4GetZ(cosCoefficients1);
	const FloatV C7 = V4GetW(cosCoefficients1);
	const FloatV C8 = V4GetX(cosCoefficients2);
	const FloatV C9 = V4GetY(cosCoefficients2);
	const FloatV C10 = V4GetZ(cosCoefficients2);
	const FloatV C11 = V4GetW(cosCoefficients2);

	Vec3V Result;
	Result = V4ScaleAdd(V2, C1, V4One());
	Result = V4ScaleAdd(V4, C2, Result);
	Result = V4ScaleAdd(V6, C3, Result);
	Result = V4ScaleAdd(V8, C4, Result);
	Result = V4ScaleAdd(V10, C5, Result);
	Result = V4ScaleAdd(V12, C6, Result);
	Result = V4ScaleAdd(V14, C7, Result);
	Result = V4ScaleAdd(V16, C8, Result);
	Result = V4ScaleAdd(V18, C9, Result);
	Result = V4ScaleAdd(V20, C10, Result);
	Result = V4ScaleAdd(V22, C11, Result);

	return V4ClearW(Result);
}

inline Vec3V V3PermYZZ(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2_t yz = vext_f32(xy, zw, 1);
	return vcombine_f32(yz, zw);
}

inline Vec3V V3PermXYX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t xw = vand_u32(xy, mask);
	return vreinterpretq_f32_u32(vcombine_u32(xy, xw));
}

inline Vec3V V3PermYZX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t yz = vext_u32(xy, zw, 1);
	const uint32x2_t xw = vand_u32(xy, mask);
	return vreinterpretq_f32_u32(vcombine_u32(yz, xw));
}

inline Vec3V V3PermZXY(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);
	
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t wz = vrev64_u32(zw);

	const uint32x2_t zx = vext_u32(wz, xy, 1);
	const uint32x2_t yw = vext_u32(xy, wz, 1);

	return vreinterpretq_f32_u32(vcombine_u32(zx, yw));
}

inline Vec3V V3PermZZY(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));

	const uint32x2_t wz = vrev64_u32(zw);
	const uint32x2_t yw = vext_u32(xy, wz, 1);
	const uint32x2_t zz = vdup_lane_u32(wz, 1);

	return vreinterpretq_f32_u32(vcombine_u32(zz, yw));
}

inline Vec3V V3PermYXX(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t yx = vrev64_u32(xy);
	const uint32x2_t xw = vand_u32(xy, mask);
	return vreinterpretq_f32_u32(vcombine_u32(yx, xw));
}

inline Vec3V V3Perm_Zero_1Z_0Y(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);
	
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(v0));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(v1));
	const uint32x2_t wz = vrev64_u32(zw);
	const uint32x2_t yw = vext_u32(xy, wz, 1);

	return vreinterpretq_f32_u32(vcombine_u32(wz, yw));
}

inline Vec3V V3Perm_0Z_Zero_1X(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);

	const uint32x2_t mask = { 0xffffFFFF, 0x0 };

	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(v0));
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(v1));
	const uint32x2_t xw = vand_u32(xy, mask);

	return vreinterpretq_f32_u32(vcombine_u32(zw, xw));
}

inline Vec3V V3Perm_1Y_0X_Zero(const Vec3V v0, const Vec3V v1)
{
	ASSERT_ISVALIDVEC3V(v0);
	ASSERT_ISVALIDVEC3V(v1);
	
	const uint32x2_t axy = vget_low_u32(vreinterpretq_u32_f32(v0));
	const uint32x2_t bxy = vget_low_u32(vreinterpretq_u32_f32(v1));
	const uint32x2_t byax = vext_u32(bxy, axy, 1);
	const uint32x2_t ww = vdup_n_u32(0);

	return vreinterpretq_f32_u32(vcombine_u32(byax, ww));
}

inline FloatV V3SumElems(const Vec3V a)
{
	ASSERT_ISVALIDVEC3V(a);

	// const uint32x2_t mask = {0xffffFFFF, 0x0};

	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);
	// const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}

	return sum0ZYX;
}

inline unsigned int V3OutOfBounds(const Vec3V a, const Vec3V min, const Vec3V max)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(min);
	ASSERT_ISVALIDVEC3V(max);

	const BoolV c = BOr(V3IsGrtr(a, max), V3IsGrtr(min, a));
	return internalUnitNeonSimd::BAnyTrue3_R(c);
}

inline unsigned int V3InBounds(const Vec3V a, const Vec3V min, const Vec3V max)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(min);
	ASSERT_ISVALIDVEC3V(max);

	const BoolV c = BAnd(V3IsGrtrOrEq(a, min), V3IsGrtrOrEq(max, a));
	return internalUnitNeonSimd::BAllTrue4_R(c);
}

inline unsigned int V3OutOfBounds(const Vec3V a, const Vec3V bounds)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(bounds);

	const BoolV greater = V3IsGrtr(V3Abs(a), bounds);
	return internalUnitNeonSimd::BAnyTrue3_R(greater);
}

inline unsigned int V3InBounds(const Vec3V a, const Vec3V bounds)
{
	ASSERT_ISVALIDVEC3V(a);
	ASSERT_ISVALIDVEC3V(bounds);

	const BoolV greaterOrEq = V3IsGrtrOrEq(bounds, V3Abs(a));
	return internalUnitNeonSimd::BAllTrue4_R(greaterOrEq);
}

inline void V3Transpose(Vec3V& col0, Vec3V& col1, Vec3V& col2)
{
	ASSERT_ISVALIDVEC3V(col0);
	ASSERT_ISVALIDVEC3V(col1);
	ASSERT_ISVALIDVEC3V(col2);

	Vec3V col3 = V3Zero();
	const float32x4x2_t v0v1 = vzipq_f32(col0, col2);
	const float32x4x2_t v2v3 = vzipq_f32(col1, col3);
	const float32x4x2_t zip0 = vzipq_f32(v0v1.val[0], v2v3.val[0]);
	const float32x4x2_t zip1 = vzipq_f32(v0v1.val[1], v2v3.val[1]);
	col0 = zip0.val[0];
	col1 = zip0.val[1];
	col2 = zip1.val[0];
	// col3 = zip1.val[1];
}

//////////////////////////////////
// VecU32V / VecI32V
//////////////////////////////////

inline VecU32V U4LoadXYZW(unsigned int x, unsigned int y, unsigned int z, unsigned int w)
{
	const uint32x4_t ret = { x, y, z, w };
	return ret;
}

inline VecU32V U4Load(const unsigned int i)
{
	return vdupq_n_u32(i);
}

inline VecU32V U4LoadU(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline VecU32V U4LoadA(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline VecU32V V4U32Sel(const BoolV c, const VecU32V a, const VecU32V b)
{
	return vbslq_u32(c, a, b);
}

inline VecU32V V4U32or(VecU32V a, VecU32V b)
{
	return vorrq_u32(a, b);
}

inline VecU32V V4U32xor(VecU32V a, VecU32V b)
{
	return veorq_u32(a, b);
}

inline VecU32V V4U32and(VecU32V a, VecU32V b)
{
	return vandq_u32(a, b);
}

inline VecU32V V4U32Andc(VecU32V a, VecU32V b)
{
	// return vbicq_u32(a, b); // creates gcc compiler bug in RTreeQueries.cpp
	return vandq_u32(a, vmvnq_u32(b));
}

inline VecI32V I4Load(const int i)
{
	return vdupq_n_s32(i);
}

inline VecI32V I4LoadU(const int* i)
{
	return vld1q_s32(i);
}

inline VecI32V I4LoadA(const int* i)
{
	return vld1q_s32(i);
}

inline void U4StoreA(const VecU32V uv, unsigned int* u)
{
	ASSERT_ISALIGNED16(u);
	vst1q_u32(reinterpret_cast<uint32_t*>(u), uv);
}

inline VecI32V VecI32V_Add(const VecI32V& a, const VecI32V& b)
{
	return vaddq_s32(a, b);
}

inline VecI32V VecI32V_Sub(const VecI32V& a, const VecI32V& b)
{
	return vsubq_s32(a, b);
}

inline BoolV VecI32V_IsGrtr(const VecI32V& a, const VecI32V& b)
{
	return vcgtq_s32(a, b);
}

inline BoolV VecI32V_IsEq(const VecI32V& a, const VecI32V& b)
{
	return vceqq_s32(a, b);
}

inline VecI32V V4I32Sel(const BoolV c, const VecI32V a, const VecI32V b)
{
	return vbslq_s32(c, a, b);
}

inline VecI32V VecI32V_Zero()
{
	return vdupq_n_s32(0);
}

inline VecI32V VecI32V_One()
{
	return vdupq_n_s32(1);
}

inline VecI32V VecI32V_Two()
{
	return vdupq_n_s32(2);
}

inline VecI32V VecI32V_MinusOne()
{
	return vdupq_n_s32(-1);
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

inline VecI32V VecI32V_PrepareShift(const VecI32V& shift)
{
	return shift;
}

inline VecI32V VecI32V_LeftShift(const VecI32V& a, const VecI32V& count)
{
	return vshlq_s32(a, count);
}

inline VecI32V VecI32V_RightShift(const VecI32V& a, const VecI32V& count)
{
	return vshlq_s32(a, VecI32V_Sub(I4Load(0), count));
}

inline VecI32V VecI32V_And(const VecI32V& a, const VecI32V& b)
{
	return vandq_s32(a, b);
}

inline VecI32V VecI32V_Or(const VecI32V& a, const VecI32V& b)
{
	return vorrq_s32(a, b);
}

inline VecI32V VecI32V_GetX(const VecI32V& f)
{
	const int32x2_t fLow = vget_low_s32(f);
	return vdupq_lane_s32(fLow, 0);
}

inline VecI32V VecI32V_GetY(const VecI32V& f)
{
	const int32x2_t fLow = vget_low_s32(f);
	return vdupq_lane_s32(fLow, 1);
}

inline VecI32V VecI32V_GetZ(const VecI32V& f)
{
	const int32x2_t fHigh = vget_high_s32(f);
	return vdupq_lane_s32(fHigh, 0);
}

inline VecI32V VecI32V_GetW(const VecI32V& f)
{
	const int32x2_t fHigh = vget_high_s32(f);
	return vdupq_lane_s32(fHigh, 1);
}

inline VecI32V VecI32V_Sel(const BoolV c, const VecI32V& a, const VecI32V& b)
{
	return vbslq_s32(c, a, b);
}

inline void int_From_VecI32V(const VecI32V& a, int* i)
{
	*i = vgetq_lane_s32(a, 0);
}

inline VecI32V VecI32V_Merge(const VecI32V& a, const VecI32V& b, const VecI32V& c, const VecI32V& d)
{
	const int32x2_t aLow = vget_low_s32(a);
	const int32x2_t bLow = vget_low_s32(b);
	const int32x2_t cLow = vget_low_s32(c);
	const int32x2_t dLow = vget_low_s32(d);

	const int32x2_t low = vext_s32(aLow, bLow, 1);
	const int32x2_t high = vext_s32(cLow, dLow, 1);

	return vcombine_s32(low, high);
}

inline VecI32V VecI32V_From_BoolV(const BoolV& a)
{
	return vreinterpretq_s32_u32(a);
}

inline VecU32V VecU32V_From_BoolV(const BoolV& a)
{
	return a;
}

inline void V4U32StoreAligned(VecU32V val, VecU32V* address)
{
	vst1q_u32(reinterpret_cast<uint32_t*>(address), val);
}

inline Vec4V V4LoadAligned(Vec4V* addr)
{
	return vld1q_f32(reinterpret_cast<float32_t*>(addr));
}

inline Vec4V V4LoadUnaligned(Vec4V* addr)
{
	return vld1q_f32(reinterpret_cast<float32_t*>(addr));
}

inline Vec4V V4Andc(const Vec4V a, const VecU32V b)
{
	return vreinterpretq_f32_u32(V4U32Andc(vreinterpretq_u32_f32(a), b));
}

inline VecU32V V4IsGrtrV32u(const Vec4V a, const Vec4V b)
{
	return V4IsGrtr(a, b);
}

inline VecU16V V4U16LoadAligned(VecU16V* addr)
{
	return vld1q_u16(reinterpret_cast<uint16_t*>(addr));
}

inline VecU16V V4U16LoadUnaligned(VecU16V* addr)
{
	return vld1q_u16(reinterpret_cast<uint16_t*>(addr));
}

inline VecU16V V4U16CompareGt(VecU16V a, VecU16V b)
{
	return vcgtq_u16(a, b);
}

inline VecU16V V4I16CompareGt(VecI16V a, VecI16V b)
{
	return vcgtq_s16(a, b);
}

inline Vec4V Vec4V_From_VecU32V(VecU32V a)
{
	return vcvtq_f32_u32(a);
}

inline Vec4V Vec4V_From_VecI32V(VecI32V a)
{
	return vcvtq_f32_s32(a);
}

inline VecI32V VecI32V_From_Vec4V(Vec4V a)
{
	return vcvtq_s32_f32(a);
}

inline Vec4V Vec4V_ReinterpretFrom_VecU32V(VecU32V a)
{
	return vreinterpretq_f32_u32(a);
}

inline Vec4V Vec4V_ReinterpretFrom_VecI32V(VecI32V a)
{
	return vreinterpretq_f32_s32(a);
}

inline VecU32V VecU32V_ReinterpretFrom_Vec4V(Vec4V a)
{
	return vreinterpretq_u32_f32(a);
}

inline VecI32V VecI32V_ReinterpretFrom_Vec4V(Vec4V a)
{
	return vreinterpretq_s32_f32(a);
}


namespace internalUnitNeonSimd
{
inline unsigned int BAllTrue4_R(const BoolV a)
{
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	const uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	return (unsigned int)(vget_lane_u32(finalReduce, 0) == 0xffffFFFF);
}

inline unsigned int BAllTrue3_R(const BoolV a)
{
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	const uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	return (unsigned int)((vget_lane_u32(finalReduce, 0) & 0xffFFff) == 0xffFFff);
}

inline unsigned int BAnyTrue4_R(const BoolV a)
{
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	const uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	return (unsigned int)(vget_lane_u32(finalReduce, 0) != 0x0);
}

inline unsigned int BAnyTrue3_R(const BoolV a)
{
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	const uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	return (unsigned int)((vget_lane_u32(finalReduce, 0) & 0xffFFff) != 0);
}
}

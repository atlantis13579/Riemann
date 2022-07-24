
#pragma once

BVec4 BVec4_Or(const BVec4 a, const BVec4 b);
BVec4 BTTTT();
BVec4 BFFFF();
FloatV Vec4_GetX(const Vec4 f);
FloatV Vec4_GetY(const Vec4 f);
FloatV Vec4_GetZ(const Vec4 f);
FloatV Vec4_GetW(const Vec4 f);
Vec4 Vec4_LoadA(const float* const f);
Vec4 V4Sel(const BVec4 c, const Vec4 a, const Vec4 b);
BVec4 Vec4_Equal(const Vec4 a, const Vec4 b);

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

inline BVec4 FIsEq(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vdupq_lane_u32(vceq_f32(a, b), 0);
}

inline FloatV FSel(const BVec4 c, const FloatV a, const FloatV b)
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

inline BVec4 FIsGrtr(const FloatV a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(a);
	ASSERT_ISVALIDFLOATV(b);
	return vdupq_lane_u32(vcgt_f32(a, b), 0);
}

inline BVec4 FIsGrtrOrEq(const FloatV a, const FloatV b)
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

	const Vec4 sinCoefficients0 = Vec4_LoadA(g_PXSinCoefficients0.f);
	const Vec4 sinCoefficients1 = Vec4_LoadA(g_PXSinCoefficients1.f);
	const Vec4 sinCoefficients2 = Vec4_LoadA(g_PXSinCoefficients2.f);

	const FloatV S1 = Vec4_GetY(sinCoefficients0);
	const FloatV S2 = Vec4_GetZ(sinCoefficients0);
	const FloatV S3 = Vec4_GetW(sinCoefficients0);
	const FloatV S4 = Vec4_GetX(sinCoefficients1);
	const FloatV S5 = Vec4_GetY(sinCoefficients1);
	const FloatV S6 = Vec4_GetZ(sinCoefficients1);
	const FloatV S7 = Vec4_GetW(sinCoefficients1);
	const FloatV S8 = Vec4_GetX(sinCoefficients2);
	const FloatV S9 = Vec4_GetY(sinCoefficients2);
	const FloatV S10 = Vec4_GetZ(sinCoefficients2);
	const FloatV S11 = Vec4_GetW(sinCoefficients2);

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

	const Vec4 cosCoefficients0 = Vec4_LoadA(g_PXCosCoefficients0.f);
	const Vec4 cosCoefficients1 = Vec4_LoadA(g_PXCosCoefficients1.f);
	const Vec4 cosCoefficients2 = Vec4_LoadA(g_PXCosCoefficients2.f);

	const FloatV C1 = Vec4_GetY(cosCoefficients0);
	const FloatV C2 = Vec4_GetZ(cosCoefficients0);
	const FloatV C3 = Vec4_GetW(cosCoefficients0);
	const FloatV C4 = Vec4_GetX(cosCoefficients1);
	const FloatV C5 = Vec4_GetY(cosCoefficients1);
	const FloatV C6 = Vec4_GetZ(cosCoefficients1);
	const FloatV C7 = Vec4_GetW(cosCoefficients1);
	const FloatV C8 = Vec4_GetX(cosCoefficients2);
	const FloatV C9 = Vec4_GetY(cosCoefficients2);
	const FloatV C10 = Vec4_GetZ(cosCoefficients2);
	const FloatV C11 = Vec4_GetW(cosCoefficients2);

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

//////////////////////////////////
// BoolV
//////////////////////////////////

inline BVec4 BFFFF()
{
	return vmovq_n_u32(0);
}

inline BVec4 BFFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zeros, zo);
}

inline BVec4 BFFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zeros, oz);
}

inline BVec4 BFFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(zeros, ones);
}

inline BVec4 BFTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zeros);
}

inline BVec4 BFTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, zo);
}

inline BVec4 BFTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(zo, oz);
}

inline BVec4 BFTTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(zo, ones);
}

inline BVec4 BTFFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	// const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zeros);
}

inline BVec4 BTFFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, zo);
}

inline BVec4 BTFTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, oz);
}

inline BVec4 BTFTT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(oz, ones);
}

inline BVec4 BTTFF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	return vcombine_u32(ones, zeros);
}

inline BVec4 BTTFT()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t zo = vext_u32(zeros, ones, 1);
	return vcombine_u32(ones, zo);
}

inline BVec4 BTTTF()
{
	const uint32x2_t zeros = vmov_n_u32(0);
	const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
	const uint32x2_t oz = vext_u32(ones, zeros, 1);
	return vcombine_u32(ones, oz);
}

inline BVec4 BTTTT()
{
	return vmovq_n_u32(0xffffFFFF);
}

inline BVec4 BVec4_And(const BVec4 a, const BVec4 b)
{
	return vandq_u32(a, b);
}

inline BVec4 BVec4_Not(const BVec4 a)
{
	return vmvnq_u32(a);
}

inline BVec4 BVec4_AndNot(const BVec4 a, const BVec4 b)
{
	// return vbicq_u32(a, b);
	return vandq_u32(a, vmvnq_u32(b));
}

inline BVec4 BVec4_Or(const BVec4 a, const BVec4 b)
{
	return vorrq_u32(a, b);
}

inline BVec4 BVec4_All(const BVec4 a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vceq_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

inline BVec4 BVec4_Any(const BVec4 a)
{
	const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
	const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
	const uint16x4_t dLow = vmovn_u32(a);
	uint16x8_t combined = vcombine_u16(dLow, dHigh);
	const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
	const uint32x2_t result = vtst_u32(finalReduce, allTrue);
	return vdupq_lane_u32(result, 0);
}

//////////////////////////////////
// Vec4V
//////////////////////////////////

inline Vec4 Vec4_Load(const float f)
{
	return vdupq_n_f32(reinterpret_cast<const float32_t&>(f));
}

inline Vec4 Vec4_LoadA(const float* f)
{
	ASSERT_ISALIGNED16(f);
	return vld1q_f32(reinterpret_cast<const float32_t*>(f));
}

inline Vec4 Vec4_Load(Vec4* addr)
{
	return vld1q_f32(reinterpret_cast<float32_t*>(addr));
}

inline void Vec4_StoreA(Vec4 a, float* f)
{
	ASSERT_ISALIGNED16(f);
	vst1q_f32(reinterpret_cast<float32_t*>(f), a);
}

inline Vec4 Vec4V_From_FloatV(FloatV f)
{
	return vcombine_f32(f, f);
}

inline Vec4 Vec4_Load_Vector3d(const Vector3d& f)
{
	alignas(16) float data[4] = { f.x, f.y, f.z, 0.0f };
	return Vec4_LoadA(data);
}

inline Vec4 V4UnpackXY(const Vec4& a, const Vec4& b)
{
	return vzipq_f32(a, b).val[0];
}

inline Vec4 V4UnpackZW(const Vec4& a, const Vec4& b)
{
	return vzipq_f32(a, b).val[1];
}

inline Vec4 V4UnitW()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t zo = vext_f32(zeros, ones, 1);
	return vcombine_f32(zeros, zo);
}

inline Vec4 V4UnitX()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t oz = vext_f32(ones, zeros, 1);
	return vcombine_f32(oz, zeros);
}

inline Vec4 V4UnitY()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t zo = vext_f32(zeros, ones, 1);
	return vcombine_f32(zo, zeros);
}

inline Vec4 V4UnitZ()
{
	const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
	const float32x2_t ones = vmov_n_f32(1.0f);
	const float32x2_t oz = vext_f32(ones, zeros, 1);
	return vcombine_f32(zeros, oz);
}

inline Vec4 Vec4_PermYXWZ(const Vec4 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2_t yx = vext_f32(xy, xy, 1);
	const float32x2_t wz = vext_f32(zw, zw, 1);
	return vcombine_f32(yx, wz);
}

inline Vec4 Vec4_PermXZXZ(const Vec4 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[0], xzyw.val[0]);
}

inline Vec4 Vec4_PermYWYW(const Vec4 a)
{
	const float32x2_t xy = vget_low_f32(a);
	const float32x2_t zw = vget_high_f32(a);
	const float32x2x2_t xzyw = vzip_f32(xy, zw);
	return vcombine_f32(xzyw.val[1], xzyw.val[1]);
}

inline Vec4 Vec4_PermYZXW(const Vec4 a)
{
	const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
	const uint32x2_t yz = vext_u32(xy, zw, 1);
	const uint32x2_t xw = vrev64_u32(vext_u32(zw, xy, 1));
	return vreinterpretq_f32_u32(vcombine_u32(yz, xw));
}

inline Vec4 Vec4_PermZWXY(const Vec4 a)
{
	const float32x2_t low = vget_low_f32(a);
	const float32x2_t high = vget_high_f32(a);
	return vcombine_f32(high, low);
}

template <unsigned char E0, unsigned char E1, unsigned char E2, unsigned char E3>
inline Vec4 V4Perm(const Vec4 V)
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

inline Vec4 Vec4_Zero()
{
	return vreinterpretq_f32_u32(vmovq_n_u32(0));
}

inline Vec4 V4One()
{
	return vmovq_n_f32(1.0f);
}

inline Vec4 Vec4_Neg(const Vec4 f)
{
	return vnegq_f32(f);
}

inline Vec4 Vec4_Add(const Vec4 a, const Vec4 b)
{
	return vaddq_f32(a, b);
}

inline Vec4 Vec4_Sub(const Vec4 a, const Vec4 b)
{
	return vsubq_f32(a, b);
}

inline Vec4 Vec4_Scale(const Vec4 a, const FloatV b)
{
	return vmulq_lane_f32(a, b, 0);
}

inline Vec4 Vec4_Mul(const Vec4 a, const Vec4 b)
{
	return vmulq_f32(a, b);
}

inline Vec4 Vec4_ScaleInv(const Vec4 a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIP(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline Vec4 Vec4_Div(const Vec4 a, const Vec4 b)
{
	const float32x4_t invB = VRECIPQ(b);
	return vmulq_f32(a, invB);
}

inline Vec4 Vec4_ScaleInvFast(const Vec4 a, const FloatV b)
{
	ASSERT_ISVALIDFLOATV(b);
	const float32x2_t invB = VRECIPE(b);
	return vmulq_lane_f32(a, invB, 0);
}

inline Vec4 Vec4_DivFast(const Vec4 a, const Vec4 b)
{
	const float32x4_t invB = VRECIPEQ(b);
	return vmulq_f32(a, invB);
}

inline Vec4 Vec4_Recip(const Vec4 a)
{
	return VRECIPQ(a);
}

inline Vec4 Vec4_RecipFast(const Vec4 a)
{
	return VRECIPEQ(a);
}

inline Vec4 Vec4_RSqrt(const Vec4 a)
{
	return VRECIPSQRTQ(a);
}

inline Vec4 Vec4_RSqrtFast(const Vec4 a)
{
	return VRECIPSQRTEQ(a);
}

inline Vec4 Vec4_Sqrt(const Vec4 a)
{
	return V4Sel(Vec4_Equal(a, Vec4_Zero()), a, Vec4_Mul(a, VRECIPSQRTQ(a)));
}

inline Vec4 Vec4_ScaleAdd(const Vec4 a, const FloatV b, const Vec4 c)
{
	ASSERT_ISVALIDFLOATV(b);
	return vmlaq_lane_f32(c, a, b, 0);
}

inline Vec4 Vec4_MAdd(const Vec4 a, const Vec4 b, const Vec4 c)
{
	return vmlaq_f32(c, a, b);
}

inline Vec4 Vec4_NegMulSub(const Vec4 a, const Vec4 b, const Vec4 c)
{
	return vmlsq_f32(c, a, b);
}

inline FloatV Vec4_GetW(const Vec4 f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 1);
}

inline FloatV Vec4_GetX(const Vec4 f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 0);
}

inline FloatV Vec4_GetY(const Vec4 f)
{
	const float32x2_t fLow = vget_low_f32(f);
	return vdup_lane_f32(fLow, 1);
}

inline FloatV Vec4_GetZ(const Vec4 f)
{
	const float32x2_t fhigh = vget_high_f32(f);
	return vdup_lane_f32(fhigh, 0);
}

inline Vec4 V4SetW(const Vec4 v, const FloatV f)
{
	ASSERT_ISVALIDFLOATV(f);
	return V4Sel(BTTTF(), v, vcombine_f32(f, f));
}

inline Vec4 Vec4_ClearW(const Vec4 v)
{
	return V4Sel(BTTTF(), v, Vec4_Zero());
}


inline Vec4 Vec4_Abs(const Vec4 a)
{
	return vabsq_f32(a);
}

inline FloatV Vec4_SumElements(const Vec4 a)
{
	const Vec4 xy = V4UnpackXY(a, a); // x,x,y,y
	const Vec4 zw = V4UnpackZW(a, a); // z,z,w,w
	const Vec4 xz_yw = Vec4_Add(xy, zw); // x+z,x+z,y+w,y+w
	const FloatV xz = Vec4_GetX(xz_yw);   // x+z
	const FloatV yw = Vec4_GetZ(xz_yw);   // y+w
	return FAdd(xz, yw);               // sum
}

template <int index>
inline Vec4 Vec4_SplatElement(Vec4 a)
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

inline FloatV Vec4_Dot(const Vec4 a, const Vec4 b)
{
	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {z+w, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z+w, x+y+z+w}
	return sumWZYX;
}

inline FloatV Vec4_Dot3(const Vec4 aa, const Vec4 bb)
{
	// PT: the V3Dot code relies on the fact that W=0 so we can't reuse it as-is, we need to clear W first.
	// TODO: find a better implementation that does not need to clear W.
	const Vec4 a = Vec4_ClearW(aa);
	const Vec4 b = Vec4_ClearW(bb);

	const float32x4_t tmp = vmulq_f32(a, b);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return sum0ZYX;
}

inline Vec4 Vec4_Cross(const Vec4 a, const Vec4 b)
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

inline FloatV Vec4_Length(const Vec4 a)
{
	const float32x4_t tmp = vmulq_f32(a, a);
	const float32x2_t low = vget_low_f32(tmp);
	const float32x2_t high = vget_high_f32(tmp);

	const float32x2_t sumTmp = vpadd_f32(low, high);       // = {0+z, x+y}
	const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}
	return FSqrt(sumWZYX);
}

inline FloatV Vec4_SquareLength(const Vec4 a)
{
	return Vec4_Dot(a, a);
}

inline Vec4 Vec4_Normalize(const Vec4 a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return Vec4_ScaleInv(a, Vec4_Length(a));
}

inline Vec4 Vec4_NormalizeFast(const Vec4 a)
{
	//PX_ASSERT(!FAllEq(V4LengthSq(a), FZero()));
	return Vec4_Scale(a, FRsqrtFast(Vec4_Dot(a, a)));
}

inline BVec4 V4IsEqU32(const UVec4 a, const UVec4 b)
{
	return vceqq_u32(a, b);
}

inline Vec4 V4Sel(const BVec4 c, const Vec4 a, const Vec4 b)
{
	return vbslq_f32(c, a, b);
}

inline BVec4 Vec4_Greater(const Vec4 a, const Vec4 b)
{
	return vcgtq_f32(a, b);
}

inline BVec4 Vec4_GreaterOrEqual(const Vec4 a, const Vec4 b)
{
	return vcgeq_f32(a, b);
}

inline BVec4 Vec4_Equal(const Vec4 a, const Vec4 b)
{
	return vceqq_f32(a, b);
}

inline Vec4 Vec4_Max(const Vec4 a, const Vec4 b)
{
	return vmaxq_f32(a, b);
}

inline Vec4 Vec4_Min(const Vec4 a, const Vec4 b)
{
	return vminq_f32(a, b);
}

inline Vec4 Vec4_Round(const Vec4 a)
{
	// truncate(a + (0.5f - sign(a)))
	const Vec4 half = Vec4_Load(0.5f);
	const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
	const Vec4 aPlusHalf = Vec4_Add(a, half);
	const Vec4 aRound = Vec4_Sub(aPlusHalf, sign);
	return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}

inline Vec4 V4Sin(const Vec4 a)
{
	const Vec4 recipTwoPi = Vec4_LoadA(g_PXReciprocalTwoPi.f);
	const Vec4 twoPi = Vec4_LoadA(g_PXTwoPi.f);
	const Vec4 tmp = Vec4_Mul(a, recipTwoPi);
	const Vec4 b = Vec4_Round(tmp);
	const Vec4 V1 = Vec4_NegMulSub(twoPi, b, a);

	// sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
	//           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
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

	const Vec4 sinCoefficients0 = Vec4_LoadA(g_PXSinCoefficients0.f);
	const Vec4 sinCoefficients1 = Vec4_LoadA(g_PXSinCoefficients1.f);
	const Vec4 sinCoefficients2 = Vec4_LoadA(g_PXSinCoefficients2.f);

	const FloatV S1 = Vec4_GetY(sinCoefficients0);
	const FloatV S2 = Vec4_GetZ(sinCoefficients0);
	const FloatV S3 = Vec4_GetW(sinCoefficients0);
	const FloatV S4 = Vec4_GetX(sinCoefficients1);
	const FloatV S5 = Vec4_GetY(sinCoefficients1);
	const FloatV S6 = Vec4_GetZ(sinCoefficients1);
	const FloatV S7 = Vec4_GetW(sinCoefficients1);
	const FloatV S8 = Vec4_GetX(sinCoefficients2);
	const FloatV S9 = Vec4_GetY(sinCoefficients2);
	const FloatV S10 = Vec4_GetZ(sinCoefficients2);
	const FloatV S11 = Vec4_GetW(sinCoefficients2);

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
	const Vec4 recipTwoPi = Vec4_LoadA(g_PXReciprocalTwoPi.f);
	const Vec4 twoPi = Vec4_LoadA(g_PXTwoPi.f);
	const Vec4 tmp = Vec4_Mul(a, recipTwoPi);
	const Vec4 b = Vec4_Round(tmp);
	const Vec4 V1 = Vec4_NegMulSub(twoPi, b, a);

	// cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
	//           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
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

	const Vec4 cosCoefficients0 = Vec4_LoadA(g_PXCosCoefficients0.f);
	const Vec4 cosCoefficients1 = Vec4_LoadA(g_PXCosCoefficients1.f);
	const Vec4 cosCoefficients2 = Vec4_LoadA(g_PXCosCoefficients2.f);

	const FloatV C1 = Vec4_GetY(cosCoefficients0);
	const FloatV C2 = Vec4_GetZ(cosCoefficients0);
	const FloatV C3 = Vec4_GetW(cosCoefficients0);
	const FloatV C4 = Vec4_GetX(cosCoefficients1);
	const FloatV C5 = Vec4_GetY(cosCoefficients1);
	const FloatV C6 = Vec4_GetZ(cosCoefficients1);
	const FloatV C7 = Vec4_GetW(cosCoefficients1);
	const FloatV C8 = Vec4_GetX(cosCoefficients2);
	const FloatV C9 = Vec4_GetY(cosCoefficients2);
	const FloatV C10 = Vec4_GetZ(cosCoefficients2);
	const FloatV C11 = Vec4_GetW(cosCoefficients2);

	Vec4 Result;
	Result = Vec4_ScaleAdd(V2, C1, V4One());
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

inline void Vec4_Transpose(Vec4& col0, Vec4& col1, Vec4& col2, Vec4& col3)
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
// VecU32V / VecI32V
//////////////////////////////////

inline UVec4 UVec4_LoadXYZW(unsigned int x, unsigned int y, unsigned int z, unsigned int w)
{
	const uint32x4_t ret = { x, y, z, w };
	return ret;
}

inline UVec4 UVec4_Load(const unsigned int i)
{
	return vdupq_n_u32(i);
}

inline UVec4 UVec4_LoadU(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline UVec4 UVec4_LoadA(const unsigned int* i)
{
	return vld1q_u32(i);
}

inline UVec4 UVec4_OR(UVec4 a, UVec4 b)
{
	return vorrq_u32(a, b);
}

inline UVec4 UVec4_XOR(UVec4 a, UVec4 b)
{
	return veorq_u32(a, b);
}

inline UVec4 UVec4_AND(UVec4 a, UVec4 b)
{
	return vandq_u32(a, b);
}

inline UVec4 UVec4_ANDNOT(UVec4 a, UVec4 b)
{
	// return vbicq_u32(a, b); // creates gcc compiler bug in RTreeQueries.cpp
	return vandq_u32(a, vmvnq_u32(b));
}

inline IVec4 IVec4_Load(const int i)
{
	return vdupq_n_s32(i);
}

inline void UVec4_StoreA(const UVec4 uv, unsigned int* u)
{
	ASSERT_ISALIGNED16(u);
	vst1q_u32(reinterpret_cast<uint32_t*>(u), uv);
}

inline UVec4 UVec4_Load_BVec4(const BVec4& a)
{
	return a;
}

inline void UVec4_StoreAA(UVec4 val, UVec4* address)
{
	vst1q_u32(reinterpret_cast<uint32_t*>(address), val);
}

inline Vec4 Vec4V_From_VecU32V(UVec4 a)
{
	return vcvtq_f32_u32(a);
}

inline Vec4 Vec4V_From_VecI32V(IVec4 a)
{
	return vcvtq_f32_s32(a);
}

inline IVec4 VecI32V_From_Vec4V(Vec4 a)
{
	return vcvtq_s32_f32(a);
}

inline Vec4 Vec4V_ReinterpretFrom_VecU32V(UVec4 a)
{
	return vreinterpretq_f32_u32(a);
}

inline Vec4 Vec4V_ReinterpretFrom_VecI32V(IVec4 a)
{
	return vreinterpretq_f32_s32(a);
}

inline UVec4 VecU32V_ReinterpretFrom_Vec4V(Vec4 a)
{
	return vreinterpretq_u32_f32(a);
}

inline IVec4 VecI32V_ReinterpretFrom_Vec4V(Vec4 a)
{
	return vreinterpretq_s32_f32(a);
}


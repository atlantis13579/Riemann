
#pragma once

#define VRECIPEQ recipq_newton<1>
#define VRECIPE recip_newton<1>
#define VRECIPSQRTEQ rsqrtq_newton<1>
#define VRECIPSQRTE rsqrt_newton<1>
#define VRECIPQ recipq_newton<4>
#define VRECIP recip_newton<4>
#define VRECIPSQRTQ rsqrtq_newton<4>
#define VRECIPSQRT rsqrt_newton<4>

//////////////////////////////////
// Scaler
//////////////////////////////////

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

//////////////////////////////////
// BVec4
//////////////////////////////////

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

//////////////////////////////////
// Vec4
//////////////////////////////////

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

//////////////////////////////////
// UVec4
//////////////////////////////////

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




#pragma once

namespace internalWindowsSimd
{
__m128 m128_I2F(__m128i n);
__m128i m128_F2I(__m128 n);
alignas(16) const uint32_t gMaskXYZ[4] = { 0xffffffff, 0xffffffff, 0xffffffff, 0 };
}

//////////////////////////////////
// FLOATV
//////////////////////////////////
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

inline Scaler Scaler_Sel(const BVec4 c, const Scaler a, const Scaler b)
{
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline BVec4 Scaler_Greater(const Scaler a, const Scaler b)
{
	return _mm_cmpgt_ps(a, b);
}

inline BVec4 Scaler_GreaterEqual(const Scaler a, const Scaler b)
{
	return _mm_cmpge_ps(a, b);
}

inline BVec4 Scaler_Equal(const Scaler a, const Scaler b)
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

//////////////////////////////////
// BoolV
//////////////////////////////////

inline BVec4 BFFFF()
{
	return _mm_setzero_ps();
}

inline BVec4 BFFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0,0xFFFFFFFF};
	const __m128 ffft=_mm_load_ps((float*)&f);
	return ffft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, 0));
}

inline BVec4 BFFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0};
	const __m128 fftf=_mm_load_ps((float*)&f);
	return fftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, 0));
}

inline BVec4 BFFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fftt=_mm_load_ps((float*)&f);
	return fftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, 0));
}

inline BVec4 BFTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0};
	const __m128 ftff=_mm_load_ps((float*)&f);
	return ftff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, 0));
}

inline BVec4 BFTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ftft=_mm_load_ps((float*)&f);
	return ftft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, 0));
}

inline BVec4 BFTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 fttf=_mm_load_ps((float*)&f);
	return fttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, 0));
}

inline BVec4 BFTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 fttt=_mm_load_ps((float*)&f);
	return fttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, 0));
}

inline BVec4 BTFFF()
{
	// const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0};
	// const __m128 tfff=_mm_load_ps((float*)&f);
	// return tfff;
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, 0, -1));
}

inline BVec4 BTFFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0,0xFFFFFFFF};
	const __m128 tfft=_mm_load_ps((float*)&f);
	return tfft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, 0, -1));
}

inline BVec4 BTFTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0};
	const __m128 tftf=_mm_load_ps((float*)&f);
	return tftf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, 0, -1));
}

inline BVec4 BTFTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tftt=_mm_load_ps((float*)&f);
	return tftt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, 0, -1));
}

inline BVec4 BTTFF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0};
	const __m128 ttff=_mm_load_ps((float*)&f);
	return ttff;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, 0, -1, -1));
}

inline BVec4 BTTFT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0,0xFFFFFFFF};
	const __m128 ttft=_mm_load_ps((float*)&f);
	return ttft;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, 0, -1, -1));
}

inline BVec4 BTTTF()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0};
	const __m128 tttf=_mm_load_ps((float*)&f);
	return tttf;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(0, -1, -1, -1));
}

inline BVec4 BTTTT()
{
	/*const PX_ALIGN(16, uint32_t f[4])={0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	const __m128 tttt=_mm_load_ps((float*)&f);
	return tttt;*/
	return internalWindowsSimd::m128_I2F(_mm_set_epi32(-1, -1, -1, -1));
}

inline BVec4 BVec4_And(const BVec4 a, const BVec4 b)
{
	return _mm_and_ps(a, b);
}

inline BVec4 BVec4_Not(const BVec4 a)
{
	const BVec4 bAllTrue(BTTTT());
	return _mm_xor_ps(a, bAllTrue);
}

inline BVec4 BVec4_AndNot(const BVec4 a, const BVec4 b)
{
	return _mm_andnot_ps(b, a);
}

inline BVec4 BVec4_Or(const BVec4 a, const BVec4 b)
{
	return _mm_or_ps(a, b);
}

inline BVec4 BVec4_All(const BVec4 a)
{
	const BVec4 bTmp =
	    _mm_and_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_and_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                  _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline BVec4 BVec4_Any(const BVec4 a)
{
	const BVec4 bTmp =
	    _mm_or_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 1, 0, 1)), _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 2, 3)));
	return _mm_or_ps(_mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(0, 0, 0, 0)),
	                 _mm_shuffle_ps(bTmp, bTmp, _MM_SHUFFLE(1, 1, 1, 1)));
}

/////////////////////////////////////
// Vec4V
//////////////////////////////////

inline Vec4 Vec4_Load(const float f)
{
	return _mm_load1_ps(&f);
}

inline Vec4 Vec4_LoadA(const float* f)
{
	ASSERT_ISALIGNED16(f);
	return _mm_load_ps(f);
}

inline Vec4 Vec4_LoadU(const float* f)
{
	return _mm_loadu_ps(f);
}

inline void Vec4_StoreA(const Vec4 a, float* f)
{
	ASSERT_ISALIGNED16(f);
	_mm_store_ps(f, a);
}

inline void Vec4_StoreU(const Vec4 a, float* f)
{
	_mm_storeu_ps(f, a);
}

inline Scaler Vec4_GetX(const Vec4 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(0, 0, 0, 0));
}

inline Scaler Vec4_GetY(const Vec4 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(1, 1, 1, 1));
}

inline Scaler Vec4_GetZ(const Vec4 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(2, 2, 2, 2));
}

inline Scaler Vec4_GetW(const Vec4 f)
{
	return _mm_shuffle_ps(f, f, _MM_SHUFFLE(3, 3, 3, 3));
}

inline Vec4 Vec4_ClearW(const Vec4 v)
{
	return _mm_and_ps(v, (IVec4&)internalWindowsSimd::gMaskXYZ);
}

inline Vec4 Vec4V_From_FloatV(Scaler f)
{
	return f;
}

inline Vec4 Vec4_PermYXWZ(const Vec4 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 3, 0, 1));
}

inline Vec4 Vec4_PermXZXZ(const Vec4 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 0, 2, 0));
}

inline Vec4 Vec4_PermYWYW(const Vec4 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 3, 1));
}

inline Vec4 Vec4_PermYZXW(const Vec4 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
}

inline Vec4 Vec4_PermZWXY(const Vec4 a)
{
	return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
}

inline Vec4 Vec4_LoadXYZW(const float& x, const float& y, const float& z, const float& w)
{
	return _mm_set_ps(w, z, y, x);
}

inline Vec4 Vec4_Zero()
{
	return _mm_setzero_ps();
}

inline Vec4 Vec4_One()
{
	return Vec4_Load(1.0f);
}

inline Vec4 Vec4_Neg(const Vec4 f)
{
	return _mm_sub_ps(_mm_setzero_ps(), f);
}

inline Vec4 Vec4_Add(const Vec4 a, const Vec4 b)
{
	return _mm_add_ps(a, b);
}

inline Vec4 Vec4_Sub(const Vec4 a, const Vec4 b)
{
	return _mm_sub_ps(a, b);
}

inline Vec4 Vec4_Scale(const Vec4 a, const Scaler b)
{
	return _mm_mul_ps(a, b);
}

inline Vec4 Vec4_Mul(const Vec4 a, const Vec4 b)
{
	return _mm_mul_ps(a, b);
}

inline Vec4 Vec4_ScaleInv(const Vec4 a, const Scaler b)
{
	return _mm_div_ps(a, b);
}

inline Vec4 Vec4_Div(const Vec4 a, const Vec4 b)
{
	return _mm_div_ps(a, b);
}

inline Vec4 Vec4_ScaleInvFast(const Vec4 a, const Scaler b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Vec4 Vec4_DivFast(const Vec4 a, const Vec4 b)
{
	return _mm_mul_ps(a, _mm_rcp_ps(b));
}

inline Vec4 Vec4_Recip(const Vec4 a)
{
	return _mm_div_ps(Vec4_One(), a);
}

inline Vec4 Vec4_RecipFast(const Vec4 a)
{
	return _mm_rcp_ps(a);
}

inline Vec4 Vec4_RSqrt(const Vec4 a)
{
	return _mm_div_ps(Vec4_One(), _mm_sqrt_ps(a));
}

inline Vec4 Vec4_RSqrtFast(const Vec4 a)
{
	return _mm_rsqrt_ps(a);
}

inline Vec4 Vec4_Sqrt(const Vec4 a)
{
	return _mm_sqrt_ps(a);
}

inline Vec4 Vec4_ScaleAdd(const Vec4 a, const Scaler b, const Vec4 c)
{
	return Vec4_Add(Vec4_Scale(a, b), c);
}

inline Vec4 Vec4_MAdd(const Vec4 a, const Vec4 b, const Vec4 c)
{
	return Vec4_Add(Vec4_Mul(a, b), c);
}

inline Vec4 Vec4_NegMSub(const Vec4 a, const Vec4 b, const Vec4 c)
{
	return Vec4_Sub(c, Vec4_Mul(a, b));
}

inline Vec4 Vec4_Max(const Vec4 a, const Vec4 b)
{
	return _mm_max_ps(a, b);
}

inline Vec4 Vec4_Min(const Vec4 a, const Vec4 b)
{
	return _mm_min_ps(a, b);
}

inline Vec4 Vec4_Abs(const Vec4 a)
{
	return Vec4_Max(a, Vec4_Neg(a));
}

inline Scaler Vec4_Dot(const Vec4 a, const Vec4 b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // x,y,z,w
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 1, 0, 3)); // w,x,y,z
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 0, 3, 2)); // z,w,x,y
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 3, 2, 1)); // y,z,w,x
	return _mm_add_ps(_mm_add_ps(shuf2, shuf3), _mm_add_ps(dot1, shuf1));
}

inline Scaler Vec4_Dot3(const Vec4 a, const Vec4 b)
{
	const __m128 dot1 = _mm_mul_ps(a, b);                                     // aw*bw | az*bz | ay*by | ax*bx
	const __m128 shuf1 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(0, 0, 0, 0)); // ax*bx | ax*bx | ax*bx | ax*bx
	const __m128 shuf2 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(1, 1, 1, 1)); // ay*by | ay*by | ay*by | ay*by
	const __m128 shuf3 = _mm_shuffle_ps(dot1, dot1, _MM_SHUFFLE(2, 2, 2, 2)); // az*bz | az*bz | az*bz | az*bz
	return _mm_add_ps(_mm_add_ps(shuf1, shuf2), shuf3);                       // ax*bx + ay*by + az*bz in each component
}

inline Vec4 Vec4_Cross(const Vec4 a, const Vec4 b)
{
	const __m128 r1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	const __m128 r2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)); // y,z,x,w
	const __m128 l2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); // z,x,y,w
	return _mm_sub_ps(_mm_mul_ps(l1, l2), _mm_mul_ps(r1, r2));
}

inline Scaler Vec4_Length(const Vec4 a)
{
	return _mm_sqrt_ps(Vec4_Dot(a, a));
}

inline Scaler Vec4_SquareLength(const Vec4 a)
{
	return Vec4_Dot(a, a);
}

inline Vec4 Vec4_Normalize(const Vec4 a)
{
	return Vec4_ScaleInv(a, _mm_sqrt_ps(Vec4_Dot(a, a)));
}

inline Vec4 Vec4_NormalizeFast(const Vec4 a)
{
	return Vec4_ScaleInvFast(a, _mm_sqrt_ps(Vec4_Dot(a, a)));
}

inline Vec4 V4Sel(const BVec4 c, const Vec4 a, const Vec4 b)
{
	return _mm_or_ps(_mm_andnot_ps(c, b), _mm_and_ps(c, a));
}

inline BVec4 Vec4_Greater(const Vec4 a, const Vec4 b)
{
	return _mm_cmpgt_ps(a, b);
}

inline BVec4 Vec4_GreaterEqual(const Vec4 a, const Vec4 b)
{
	return _mm_cmpge_ps(a, b);
}

inline BVec4 Vec4_Equal(const Vec4 a, const Vec4 b)
{
	return _mm_cmpeq_ps(a, b);
}

inline Vec4 Vec4_Round(const Vec4 a)
{
	// return _mm_round_ps(a, 0x0);
	const Vec4 half = Vec4_Load(0.5f);
	const __m128 signBit = _mm_cvtepi32_ps(_mm_srli_epi32(_mm_cvtps_epi32(a), 31));
	const Vec4 aRound = Vec4_Sub(Vec4_Add(a, half), signBit);
	const __m128i tmp = _mm_cvttps_epi32(aRound);
	return _mm_cvtepi32_ps(tmp);
}

inline void Vec4_Transpose(Vec4& col0, Vec4& col1, Vec4& col2, Vec4& col3)
{
	Vec4 tmp0 = _mm_unpacklo_ps(col0, col1);
	Vec4 tmp2 = _mm_unpacklo_ps(col2, col3);
	Vec4 tmp1 = _mm_unpackhi_ps(col0, col1);
	Vec4 tmp3 = _mm_unpackhi_ps(col2, col3);
	col0 = _mm_movelh_ps(tmp0, tmp2);
	col1 = _mm_movehl_ps(tmp2, tmp0);
	col2 = _mm_movelh_ps(tmp1, tmp3);
	col3 = _mm_movehl_ps(tmp3, tmp1);
}

inline void UVec4_StoreAA(UVec4 val, UVec4* address)
{
	*address = val;
}

template <int index>
inline Vec4 Vec4_SplatElement(Vec4 a)
{
	return internalWindowsSimd::m128_I2F(
		_mm_shuffle_epi32(internalWindowsSimd::m128_F2I(a), _MM_SHUFFLE(index, index, index, index)));
}

inline Vec4 Vec4_Load_Vector3d(const Vector3d& f)
{
	return _mm_set_ps(0.0f, f.z, f.y, f.x);
}

//////////////////////////////////
// IVec4 / UVec4
//////////////////////////////////
inline IVec4 UVec4_Load(const uint32_t i)
{
	return _mm_load1_ps((float*)&i);
}

inline UVec4 UVec4_Load_BVec4(const BVec4 a)
{
	return a;
}

inline UVec4 UVec4_LoadU(const uint32_t* i)
{
	return _mm_loadu_ps((float*)i);
}

inline UVec4 UVec4_LoadA(const uint32_t* i)
{
	ASSERT_ISALIGNED16(i);
	return _mm_load_ps((float*)i);
}

inline void UVec4_StoreA(const UVec4 uv, uint32_t* u)
{
	ASSERT_ISALIGNED16(u);
	_mm_store_ps((float*)u, uv);
}

inline Vec4 Vec4V_ReinterpretFrom_VecU32V(UVec4 a)
{
	return Vec4(a);
}

inline UVec4 VecU32V_ReinterpretFrom_Vec4V(Vec4 a)
{
	return UVec4(a);
}

typedef union
{
	__m128 v;
	uint32_t m128_u32[4];
} _VecU32V;

inline UVec4 UVec4_LoadXYZW(uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
	_VecU32V t;
	t.m128_u32[0] = x;
	t.m128_u32[1] = y;
	t.m128_u32[2] = z;
	t.m128_u32[3] = w;
	return t.v;
}

inline UVec4 UVec4_OR(UVec4 a, UVec4 b)
{
	return internalWindowsSimd::m128_I2F(_mm_or_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline UVec4 UVec4_XOR(UVec4 a, UVec4 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_xor_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline UVec4 UVec4_AND(UVec4 a, UVec4 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_and_si128(internalWindowsSimd::m128_F2I(a), internalWindowsSimd::m128_F2I(b)));
}

inline UVec4 UVec4_ANDNOT(UVec4 a, UVec4 b)
{
	return internalWindowsSimd::m128_I2F(
		_mm_andnot_si128(internalWindowsSimd::m128_F2I(b), internalWindowsSimd::m128_F2I(a)));
}

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
} //internalWindowsSimd

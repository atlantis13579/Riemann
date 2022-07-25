
#pragma once

namespace internalWindowsSimd
{
__m128 m128_I2F(__m128i n);
__m128i m128_F2I(__m128 n);
alignas(16) const uint32_t gMaskXYZ[4] = { 0xffffffff, 0xffffffff, 0xffffffff, 0 };
}

//////////////////////////////////
// Scaler
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

//////////////////////////////////
// BVec4
//////////////////////////////////

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

/////////////////////////////////////
// Vec4
//////////////////////////////////

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

//////////////////////////////////
// UVec4
//////////////////////////////////
inline I128 U128_Load(const uint32_t i)
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

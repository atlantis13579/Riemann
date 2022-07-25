
#pragma once

#ifdef SIMD_SIMULATOR

/////////////////////////////////////
// Vec4
//////////////////////////////////

namespace SIMDSimulator
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

//////////////////////////////////
// BVec4
//////////////////////////////////

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

//////////////////////////////////
// UVec4
//////////////////////////////////
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
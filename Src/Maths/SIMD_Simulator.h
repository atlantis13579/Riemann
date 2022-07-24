
#pragma once

#ifdef SIMD_SIMULATOR

/////////////////////////////////////
// Vec4
//////////////////////////////////

namespace SIMDSimulator
{

inline Vec4 Vec4_Load_Vector3d(const Vector3d& f)
{
	return Vec4(f.x, f.y, f.z, 0.0f);
}

template <int index>
inline Vec4 Vec4_SplatElement(Vec4 a)
{
	return Vec4(a[index], a[index], a[index], a[index]);
}

inline Vec4 Vec4_Load(const float f)
{
	return Vec4(f, f, f, f);
}

inline Vec4 Vec4_LoadA(const float* f)
{
	return Vec4(f[0], f[1], f[2], f[3]);
}

inline Vec4 Vec4_LoadU(const float* f)
{
	return Vec4_LoadA(f);
}

inline void Vec4_StoreA(Vec4 a, float* f)
{
	f[0] = a.x;
	f[1] = a.y;
	f[2] = a.z;
	f[3] = a.w;
}

inline void Vec4_StoreU(Vec4 a, float* f)
{
	Vec4_StoreA(a, f);
}

inline BVec4 Vec4_Greater(const Vec4 a, const Vec4 b)
{
	return TVector4<bool>(a.x > b.x, a.y > b.y, a.z > b.z, a.w > b.w);
}

inline BVec4 Vec4_GreaterOrEqual(const Vec4 a, const Vec4 b)
{
	return TVector4<bool>(a.x >= b.x, a.y >= b.y, a.z >= b.z, a.w >= b.w);
}

inline BVec4 Vec4_Equal(const Vec4 a, const Vec4 b)
{
	return TVector4<bool>(a.x == b.x, a.y == b.y, a.z == b.z, a.w == b.w);
}

//////////////////////////////////
// BVec4
//////////////////////////////////

inline BVec4 BVec4_And(const BVec4 a, const BVec4 b)
{
	return TVector4<bool>(a.x && b.x, a.y && b.y, a.z && b.z, a.w && b.w);
}

inline BVec4 BVec4_Not(const BVec4 a)
{
	return TVector4<bool>(!a.x, !a.y, !a.z, !a.w);
}

inline BVec4 BVec4_AndNot(const BVec4 a, const BVec4 b)
{
	return TVector4<bool>(a.x && !b.x, a.y && !b.y, a.z && !b.z, a.w && !b.w);
}

inline BVec4 BVec4_Or(const BVec4 a, const BVec4 b)
{
	return TVector4<bool>(a.x || b.x, a.y || b.y, a.z || b.z, a.w || b.w);
}

//////////////////////////////////
// UVec4
//////////////////////////////////
inline UVec4 UVec4_Load_BVec4(const BVec4 a)
{
	return TVector4<unsigned int>(a.x, a.y, a.z, a.w);
}

inline void UVec4_StoreA(const UVec4 uv, uint32_t* u)
{
	f[0] = a.x;
	f[1] = a.y;
	f[2] = a.z;
	f[3] = a.w;
}
}
#endif
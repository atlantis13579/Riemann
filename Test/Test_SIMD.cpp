#include "Test.h"
#include "../Src/Maths/SIMD.h"

#include <limits>

namespace
{
	constexpr float SIMD_EPSILON = 1.0e-5f;
	constexpr float SIMD_FAST_EPSILON = 3.0e-3f;

	void ExpectF128Near(const F128 value, const float x, const float y, const float z, const float w, const float epsilon = SIMD_EPSILON)
	{
		alignas(16) float actual[4];
		F128_StoreA(value, actual);
		EXPECT_NEAR(actual[0], x, epsilon);
		EXPECT_NEAR(actual[1], y, epsilon);
		EXPECT_NEAR(actual[2], z, epsilon);
		EXPECT_NEAR(actual[3], w, epsilon);
	}

	void ExpectScalerNear(const Scaler value, const float expected, const float epsilon = SIMD_EPSILON)
	{
		float actual = 0.0f;
		Scaler_Store(value, &actual);
		EXPECT_NEAR(actual, expected, epsilon);
	}

	void ExpectU128(const U128 value, const uint32_t x, const uint32_t y, const uint32_t z, const uint32_t w)
	{
		alignas(16) uint32_t actual[4];
		U128_StoreA(value, actual);
		EXPECT(actual[0] == x);
		EXPECT(actual[1] == y);
		EXPECT(actual[2] == z);
		EXPECT(actual[3] == w);
	}

	void ExpectMask(const B128 value, const bool x, const bool y, const bool z, const bool w)
	{
		ExpectU128(
			U128_Load_B128(value),
			x ? UINT32_MAX : 0u,
			y ? UINT32_MAX : 0u,
			z ? UINT32_MAX : 0u,
			w ? UINT32_MAX : 0u);
	}

	void TestLoadStoreAndPermute()
	{
		alignas(16) float aligned[4] = { 1.0f, -2.0f, 3.5f, 4.0f };
		float unalignedStorage[5] = { 0.0f, 5.0f, 6.0f, 7.0f, 8.0f };
		float unalignedOutput[5] = {};

		const F128 alignedValue = F128_LoadA(aligned);
		ExpectF128Near(alignedValue, 1.0f, -2.0f, 3.5f, 4.0f);
		const F128 unalignedValue = F128_LoadU(unalignedStorage + 1);
		ExpectF128Near(unalignedValue, 5.0f, 6.0f, 7.0f, 8.0f);
		F128_StoreU(unalignedValue, unalignedOutput + 1);
		EXPECT(unalignedOutput[1] == 5.0f && unalignedOutput[4] == 8.0f);

		alignas(16) float stored[4] = {};
		F128_StoreA(alignedValue, stored);
		EXPECT(stored[0] == 1.0f && stored[1] == -2.0f && stored[2] == 3.5f && stored[3] == 4.0f);

		alignas(16) float scalarSource[4] = { -3.25f, 99.0f, 99.0f, 99.0f };
		ExpectScalerNear(Scaler_LoadA(scalarSource), -3.25f);
		ExpectScalerNear(Scaler_Load(2.5f), 2.5f);

		const F128 directValue = F128_LoadXYZW(9.0f, 8.0f, 7.0f, 6.0f);
		ExpectF128Near(F128_Load(&directValue), 9.0f, 8.0f, 7.0f, 6.0f);
		ExpectF128Near(F128_Load_Vector3d(Vector3(2.0f, 4.0f, 6.0f)), 2.0f, 4.0f, 6.0f, 0.0f);

		EXPECT(F128_ReadX(directValue) == 9.0f);
		EXPECT(F128_ReadY(directValue) == 8.0f);
		EXPECT(F128_ReadZ(directValue) == 7.0f);
		EXPECT(F128_ReadW(directValue) == 6.0f);
		const Vector3 xyz = F128_ReadXYZ(directValue);
		const Vector4 xyzw = F128_ReadXYZW(directValue);
		EXPECT(xyz.x == 9.0f && xyz.y == 8.0f && xyz.z == 7.0f);
		EXPECT(xyzw.x == 9.0f && xyzw.y == 8.0f && xyzw.z == 7.0f && xyzw.w == 6.0f);

		const F128 sequence = F128_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
		ExpectF128Near(F128_SplatElement<0>(sequence), 1.0f, 1.0f, 1.0f, 1.0f);
		ExpectF128Near(F128_SplatElement<1>(sequence), 2.0f, 2.0f, 2.0f, 2.0f);
		ExpectF128Near(F128_SplatElement<2>(sequence), 3.0f, 3.0f, 3.0f, 3.0f);
		ExpectF128Near(F128_SplatElement<3>(sequence), 4.0f, 4.0f, 4.0f, 4.0f);
		ExpectF128Near(F128_PermYXWZ(sequence), 2.0f, 1.0f, 4.0f, 3.0f);
		ExpectF128Near(F128_PermXZXZ(sequence), 1.0f, 3.0f, 1.0f, 3.0f);
		ExpectF128Near(F128_PermYWYW(sequence), 2.0f, 4.0f, 2.0f, 4.0f);
		ExpectF128Near(F128_PermYZXW(sequence), 2.0f, 3.0f, 1.0f, 4.0f);
		ExpectF128Near(F128_PermZWXY(sequence), 3.0f, 4.0f, 1.0f, 2.0f);
		ExpectF128Near(F128_SetW(sequence, Scaler_Load(9.0f)), 1.0f, 2.0f, 3.0f, 9.0f);
		ExpectF128Near(F128_ClearW(sequence), 1.0f, 2.0f, 3.0f, 0.0f);
		ExpectScalerNear(F128_GetX(sequence), 1.0f);
		ExpectScalerNear(F128_GetY(sequence), 2.0f);
		ExpectScalerNear(F128_GetZ(sequence), 3.0f);
		ExpectScalerNear(F128_GetW(sequence), 4.0f);

		F128 col0 = F128_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
		F128 col1 = F128_LoadXYZW(5.0f, 6.0f, 7.0f, 8.0f);
		F128 col2 = F128_LoadXYZW(9.0f, 10.0f, 11.0f, 12.0f);
		F128 col3 = F128_LoadXYZW(13.0f, 14.0f, 15.0f, 16.0f);
		F128_Transpose(col0, col1, col2, col3);
		ExpectF128Near(col0, 1.0f, 5.0f, 9.0f, 13.0f);
		ExpectF128Near(col1, 2.0f, 6.0f, 10.0f, 14.0f);
		ExpectF128Near(col2, 3.0f, 7.0f, 11.0f, 15.0f);
		ExpectF128Near(col3, 4.0f, 8.0f, 12.0f, 16.0f);
	}

	void TestArithmeticAndRound()
	{
		const F128 a = F128_LoadXYZW(2.0f, -4.0f, 8.0f, -16.0f);
		const F128 b = F128_LoadXYZW(4.0f, 2.0f, -2.0f, -4.0f);
		ExpectF128Near(F128_Zero(), 0.0f, 0.0f, 0.0f, 0.0f);
		ExpectF128Near(F128_One(), 1.0f, 1.0f, 1.0f, 1.0f);
		ExpectF128Near(F128_Load(3.0f), 3.0f, 3.0f, 3.0f, 3.0f);
		ExpectF128Near(F128_Neg(a), -2.0f, 4.0f, -8.0f, 16.0f);
		ExpectF128Near(F128_Add(a, b), 6.0f, -2.0f, 6.0f, -20.0f);
		ExpectF128Near(F128_Sub(a, b), -2.0f, -6.0f, 10.0f, -12.0f);
		ExpectF128Near(F128_Mul(a, b), 8.0f, -8.0f, -16.0f, 64.0f);
		ExpectF128Near(F128_Div(a, b), 0.5f, -2.0f, -4.0f, 4.0f);
		ExpectF128Near(F128_DivFast(a, b), 0.5f, -2.0f, -4.0f, 4.0f, SIMD_FAST_EPSILON);
		ExpectF128Near(F128_Scale(a, Scaler_Load(0.5f)), 1.0f, -2.0f, 4.0f, -8.0f);
		ExpectF128Near(F128_ScaleInv(a, Scaler_Load(2.0f)), 1.0f, -2.0f, 4.0f, -8.0f);
		ExpectF128Near(F128_ScaleInvFast(a, Scaler_Load(2.0f)), 1.0f, -2.0f, 4.0f, -8.0f, SIMD_FAST_EPSILON);
		ExpectF128Near(F128_ScaleAdd(a, Scaler_Load(0.5f), F128_One()), 2.0f, -1.0f, 5.0f, -7.0f);
		ExpectF128Near(F128_MAdd(a, b, F128_One()), 9.0f, -7.0f, -15.0f, 65.0f);
		ExpectF128Near(F128_NegMSub(a, b, F128_One()), -7.0f, 9.0f, 17.0f, -63.0f);
		ExpectF128Near(F128_Abs(a), 2.0f, 4.0f, 8.0f, 16.0f);
		ExpectF128Near(F128_Min(a, b), 2.0f, -4.0f, -2.0f, -16.0f);
		ExpectF128Near(F128_Max(a, b), 4.0f, 2.0f, 8.0f, -4.0f);

		const F128 positive = F128_LoadXYZW(1.0f, 4.0f, 16.0f, 64.0f);
		ExpectF128Near(F128_Sqrt(positive), 1.0f, 2.0f, 4.0f, 8.0f);
		ExpectF128Near(F128_RSqrt(positive), 1.0f, 0.5f, 0.25f, 0.125f, SIMD_FAST_EPSILON);
		ExpectF128Near(F128_RSqrtFast(positive), 1.0f, 0.5f, 0.25f, 0.125f, SIMD_FAST_EPSILON);
		ExpectF128Near(F128_Recip(positive), 1.0f, 0.25f, 0.0625f, 0.015625f, SIMD_FAST_EPSILON);
		ExpectF128Near(F128_RecipFast(positive), 1.0f, 0.25f, 0.0625f, 0.015625f, SIMD_FAST_EPSILON);

		ExpectScalerNear(Scaler_Add(Scaler_Load(2.0f), Scaler_Load(3.0f)), 5.0f);
		ExpectScalerNear(Scaler_Sub(Scaler_Load(2.0f), Scaler_Load(3.0f)), -1.0f);
		ExpectScalerNear(Scaler_Mul(Scaler_Load(2.0f), Scaler_Load(3.0f)), 6.0f);
		ExpectScalerNear(Scaler_Div(Scaler_Load(6.0f), Scaler_Load(3.0f)), 2.0f);
		ExpectScalerNear(Scaler_DivFast(Scaler_Load(6.0f), Scaler_Load(3.0f)), 2.0f, SIMD_FAST_EPSILON);
		ExpectScalerNear(Scaler_Recip(Scaler_Load(4.0f)), 0.25f, SIMD_FAST_EPSILON);
		ExpectScalerNear(Scaler_RecipFast(Scaler_Load(4.0f)), 0.25f, SIMD_FAST_EPSILON);
		ExpectScalerNear(Scaler_Sqrt(Scaler_Load(16.0f)), 4.0f);
		ExpectScalerNear(Scaler_RSqrt(Scaler_Load(16.0f)), 0.25f, SIMD_FAST_EPSILON);
		ExpectScalerNear(Scaler_RSqrtFast(Scaler_Load(16.0f)), 0.25f, SIMD_FAST_EPSILON);
		ExpectScalerNear(Scaler_MAdd(Scaler_Load(2.0f), Scaler_Load(3.0f), Scaler_Load(4.0f)), 10.0f);
		ExpectScalerNear(Scaler_NegMSub(Scaler_Load(2.0f), Scaler_Load(3.0f), Scaler_Load(4.0f)), -2.0f);
		ExpectScalerNear(Scaler_Zero(), 0.0f);
		ExpectScalerNear(Scaler_One(), 1.0f);
		ExpectScalerNear(Scaler_Neg(Scaler_Load(3.0f)), -3.0f);
		ExpectScalerNear(Scaler_Abs(Scaler_Load(-3.0f)), 3.0f);
		ExpectScalerNear(Scaler_Min(Scaler_Load(2.0f), Scaler_Load(3.0f)), 2.0f);
		ExpectScalerNear(Scaler_Max(Scaler_Load(2.0f), Scaler_Load(3.0f)), 3.0f);
		ExpectScalerNear(Scaler_Clamp(Scaler_Load(5.0f), Scaler_Load(-1.0f), Scaler_Load(3.0f)), 3.0f);

		ExpectF128Near(F128_Round(F128_LoadXYZW(-2.5f, -1.5f, -0.5f, 0.5f)), -3.0f, -2.0f, -1.0f, 1.0f);
		ExpectF128Near(F128_Round(F128_LoadXYZW(1.49f, 1.5f, 2.5f, -1.49f)), 1.0f, 2.0f, 3.0f, -1.0f);
		ExpectScalerNear(Scaler_Round(Scaler_Load(-0.5f)), -1.0f);
		ExpectScalerNear(Scaler_Round(Scaler_Load(2.5f)), 3.0f);

		const float infinity = std::numeric_limits<float>::infinity();
		const float quietNaN = std::numeric_limits<float>::quiet_NaN();
		const F128 zeroes = F128_LoadXYZW(0.0f, -0.0f, 1.0f, -1.0f);
		ExpectU128(U128_ReinterpretFrom_F128(F128_Neg(zeroes)),
			0x80000000u, 0x00000000u, 0xbf800000u, 0x3f800000u);
		ExpectU128(U128_ReinterpretFrom_F128(F128_Abs(zeroes)),
			0x00000000u, 0x00000000u, 0x3f800000u, 0x3f800000u);

		alignas(16) float specialRound[4];
		F128_StoreA(F128_Round(F128_LoadXYZW(-0.0f, infinity, -infinity, quietNaN)), specialRound);
		EXPECT(std::signbit(specialRound[0]));
		EXPECT(std::isinf(specialRound[1]) && specialRound[1] > 0.0f);
		EXPECT(std::isinf(specialRound[2]) && specialRound[2] < 0.0f);
		EXPECT(std::isnan(specialRound[3]));
		float roundedNegativeZero = 0.0f;
		Scaler_Store(Scaler_Round(Scaler_Load(-0.0f)), &roundedNegativeZero);
		EXPECT(std::signbit(roundedNegativeZero));

		alignas(16) float specialSqrt[4];
		F128_StoreA(F128_Sqrt(F128_LoadXYZW(0.0f, -0.0f, infinity, 4.0f)), specialSqrt);
		EXPECT(specialSqrt[0] == 0.0f && !std::signbit(specialSqrt[0]));
		EXPECT(specialSqrt[1] == 0.0f && std::signbit(specialSqrt[1]));
		EXPECT(std::isinf(specialSqrt[2]) && specialSqrt[2] > 0.0f);
		EXPECT_NEAR(specialSqrt[3], 2.0f, SIMD_EPSILON);
	}

	void TestVectorMath()
	{
		const F128 a = F128_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
		const F128 b = F128_LoadXYZW(-2.0f, 0.5f, 4.0f, 10.0f);
		ExpectScalerNear(F128_Dot(a, b), 51.0f);
		ExpectScalerNear(F128_Dot3(a, b), 11.0f);
		ExpectF128Near(
			F128_Cross(F128_LoadXYZW(1.0f, 0.0f, 0.0f, 5.0f), F128_LoadXYZW(0.0f, 1.0f, 0.0f, 7.0f)),
			0.0f, 0.0f, 1.0f, 0.0f);

		const F128 lengthVector = F128_LoadXYZW(1.0f, 2.0f, 2.0f, 0.0f);
		ExpectScalerNear(F128_SquareLength(lengthVector), 9.0f);
		ExpectScalerNear(F128_Length(lengthVector), 3.0f);
		ExpectF128Near(F128_Normalize(lengthVector), 1.0f / 3.0f, 2.0f / 3.0f, 2.0f / 3.0f, 0.0f);
		ExpectF128Near(F128_NormalizeFast(lengthVector), 1.0f / 3.0f, 2.0f / 3.0f, 2.0f / 3.0f, 0.0f, SIMD_FAST_EPSILON);
	}

	void TestMasksAndSelect()
	{
		ExpectMask(BFFFF(), false, false, false, false);
		ExpectMask(BFFFT(), false, false, false, true);
		ExpectMask(BFFTF(), false, false, true, false);
		ExpectMask(BFFTT(), false, false, true, true);
		ExpectMask(BFTFF(), false, true, false, false);
		ExpectMask(BFTFT(), false, true, false, true);
		ExpectMask(BFTTF(), false, true, true, false);
		ExpectMask(BFTTT(), false, true, true, true);
		ExpectMask(BTFFF(), true, false, false, false);
		ExpectMask(BTFFT(), true, false, false, true);
		ExpectMask(BTFTF(), true, false, true, false);
		ExpectMask(BTFTT(), true, false, true, true);
		ExpectMask(BTTFF(), true, true, false, false);
		ExpectMask(BTTFT(), true, true, false, true);
		ExpectMask(BTTTF(), true, true, true, false);
		ExpectMask(BTTTT(), true, true, true, true);

		const F128 a = F128_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
		const F128 b = F128_LoadXYZW(4.0f, 3.0f, 2.0f, 1.0f);
		ExpectMask(F128_Greater(a, b), false, false, true, true);
		const F128 c = F128_LoadXYZW(1.0f, 1.0f, 4.0f, 4.0f);
		ExpectMask(F128_Greater(a, c), false, true, false, false);
		ExpectMask(F128_GreaterEqual(a, c), true, true, false, true);
		ExpectMask(F128_Equal(a, a), true, true, true, true);
		ExpectMask(Scaler_Greater(Scaler_Load(2.0f), Scaler_Load(1.0f)), true, true, true, true);
		ExpectMask(Scaler_GreaterEqual(Scaler_Load(1.0f), Scaler_Load(1.0f)), true, true, true, true);
		ExpectMask(Scaler_Equal(Scaler_Load(1.0f), Scaler_Load(2.0f)), false, false, false, false);

		ExpectF128Near(F128_Sel(BTFTF(), a, b), 1.0f, 3.0f, 3.0f, 1.0f);
		ExpectScalerNear(Scaler_Sel(BTFFF(), Scaler_Load(7.0f), Scaler_Load(9.0f)), 7.0f);
		ExpectScalerNear(Scaler_Sel(BFFFF(), Scaler_Load(7.0f), Scaler_Load(9.0f)), 9.0f);
		ExpectMask(B128_And(BTTFF(), BTFTF()), true, false, false, false);
		ExpectMask(B128_Or(BTTFF(), BTFTF()), true, true, true, false);
		ExpectMask(B128_Not(BTFTF()), false, true, false, true);
		ExpectMask(B128_AndNot(BTTFF(), BTFTF()), false, true, false, false);
		ExpectMask(B128_All(BTTTT()), true, true, true, true);
		ExpectMask(B128_All(BTTTF()), false, false, false, false);
		ExpectMask(B128_Any(BFFFF()), false, false, false, false);
		ExpectMask(B128_Any(BFFTF()), true, true, true, true);
	}

	void TestU128ConversionsAndBits()
	{
		alignas(16) uint32_t aligned[4] = { 1u, 2u, 0x80000000u, UINT32_MAX };
		uint32_t unalignedStorage[5] = { 0u, 5u, 6u, 7u, 8u };
		ExpectU128(U128_Load(7u), 7u, 7u, 7u, 7u);
		ExpectU128(U128_LoadA(aligned), 1u, 2u, 0x80000000u, UINT32_MAX);
		ExpectU128(U128_LoadU(unalignedStorage + 1), 5u, 6u, 7u, 8u);
		ExpectU128(U128_Zero(), 0u, 0u, 0u, 0u);

		const U128 lhs = U128_LoadXYZW(0x0f0f0f0fu, 0xaaaaaaaau, 0x12345678u, UINT32_MAX);
		const U128 rhs = U128_LoadXYZW(0x33333333u, 0x55555555u, 0xffff0000u, 0x00ff00ffu);
		U128 storedVector = U128_Zero();
		U128_StoreAA(lhs, &storedVector);
		ExpectU128(storedVector, 0x0f0f0f0fu, 0xaaaaaaaau, 0x12345678u, UINT32_MAX);
		ExpectU128(U128_AND(lhs, rhs), 0x0f0f0f0fu & 0x33333333u, 0xaaaaaaaau & 0x55555555u, 0x12345678u & 0xffff0000u, UINT32_MAX & 0x00ff00ffu);
		ExpectU128(U128_OR(lhs, rhs), 0x0f0f0f0fu | 0x33333333u, 0xaaaaaaaau | 0x55555555u, 0x12345678u | 0xffff0000u, UINT32_MAX | 0x00ff00ffu);
		ExpectU128(U128_XOR(lhs, rhs), 0x0f0f0f0fu ^ 0x33333333u, 0xaaaaaaaau ^ 0x55555555u, 0x12345678u ^ 0xffff0000u, UINT32_MAX ^ 0x00ff00ffu);
		ExpectU128(U128_ANDNOT(lhs, rhs), 0x0f0f0f0fu & ~0x33333333u, 0xaaaaaaaau & ~0x55555555u, 0x12345678u & ~0xffff0000u, UINT32_MAX & ~0x00ff00ffu);

		const U128 integers = U128_LoadXYZW(0u, 1u, 16777216u, UINT32_MAX);
		ExpectF128Near(
			F128_From_U128(integers),
			static_cast<float>(0u),
			static_cast<float>(1u),
			static_cast<float>(16777216u),
			static_cast<float>(UINT32_MAX),
			1.0f);

		const F128 floats = F128_LoadXYZW(0.0f, 1.9f, 123456.75f, 3000000000.0f);
		ExpectU128(
			U128_From_F128(floats),
			static_cast<uint32_t>(0.0f),
			static_cast<uint32_t>(1.9f),
			static_cast<uint32_t>(123456.75f),
			static_cast<uint32_t>(3000000000.0f));

		const U128 floatBits = U128_LoadXYZW(0x3f800000u, 0xbf800000u, 0x40490fdbu, 0x80000000u);
		const F128 reinterpretedFloats = F128_ReinterpretFrom_U128(floatBits);
		ExpectF128Near(reinterpretedFloats, 1.0f, -1.0f, 3.1415927f, -0.0f);
		ExpectU128(U128_ReinterpretFrom_F128(reinterpretedFloats), 0x3f800000u, 0xbf800000u, 0x40490fdbu, 0x80000000u);
		ExpectU128(U128_ReinterpretFrom_F128(F128_LoadXYZW(1.0f, 2.0f, 4.0f, 8.0f)), 0x3f800000u, 0x40000000u, 0x40800000u, 0x41000000u);
	}

	void TestSinCos()
	{
		const float pi = 3.14159265358979323846f;
		const F128 angles = F128_LoadXYZW(0.0f, pi / 6.0f, -pi / 2.0f, 7.0f * pi / 4.0f);
		alignas(16) float input[4];
		F128_StoreA(angles, input);
		const F128 sinValue = F128_Sin(angles);
		const F128 cosValue = F128_Cos(angles);
		alignas(16) float actualSin[4];
		alignas(16) float actualCos[4];
		F128_StoreA(sinValue, actualSin);
		F128_StoreA(cosValue, actualCos);
		for (int i = 0; i < 4; ++i)
		{
			EXPECT_NEAR(actualSin[i], std::sin(input[i]), 2.0e-5f);
			EXPECT_NEAR(actualCos[i], std::cos(input[i]), 2.0e-5f);
		}
	}

}

void TestSIMD()
{
	printf("Running TestSIMD\n");
	TestLoadStoreAndPermute();
	TestArithmeticAndRound();
	TestVectorMath();
	TestMasksAndSelect();
	TestU128ConversionsAndBits();
	TestSinCos();
}

#if defined(SIMD_STANDALONE_TEST)
int main()
{
	TestResetFailures();
	TestSIMD();
	return TestFailureCount() == 0 ? 0 : 1;
}
#endif

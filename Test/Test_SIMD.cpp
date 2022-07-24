
#include "Test.h"
#include "../Src/Maths/SIMD.h"

void TestSIMD()
{
	printf("Running TestSIMD\n");
	Vec4 v1 = Vec4_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
	Vec4 v2 = Vec4_Load(2.0f);
	Vec4 v3 = v1 * v2;
	Vector3d vv = Vec4_ReadXYZ(v3);
	v2 = Vec4_Load_Vector3d(vv);
	float* p = (float*)&v2;
	EXPECT(p[0] == 2.0f && p[1] == 4.0f && p[2] == 6.0f)

	Vec4 v10 = Vec4_SplatElement<0>(v1);
	Vec4 v11 = Vec4_SplatElement<1>(v1);
	Vec4 v12 = Vec4_SplatElement<2>(v1);

	float s[] = { 1, 2, 3, 4 };
	const Vec4 v4 = Vec4_LoadU(s);
	Scaler y = Vec4_GetY(v4);

	return;
}

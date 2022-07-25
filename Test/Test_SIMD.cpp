
#include "Test.h"
#include "../Src/Maths/SIMD.h"

void TestSIMD()
{
	printf("Running TestSIMD\n");
	F128 v1 = F128_LoadXYZW(1.0f, 2.0f, 3.0f, 4.0f);
	F128 v2 = F128_Load(2.0f);
	F128 v3 = v1 * v2;
	Vector3 vv = F128_ReadXYZ(v3);
	v2 = F128_Load_Vector3d(vv);
	float* p = (float*)&v2;
	EXPECT(p[0] == 2.0f && p[1] == 4.0f && p[2] == 6.0f)

	F128 v10 = F128_SplatElement<0>(v1);
	F128 v11 = F128_SplatElement<1>(v1);
	F128 v12 = F128_SplatElement<2>(v1);

	float s[] = { 1, 2, 3, 4 };
	const F128 v4 = F128_LoadU(s);
	Scaler y = F128_GetY(v4);

	return;
}

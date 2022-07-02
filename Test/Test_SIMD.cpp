
#include "Test.h"
#include "../Src/Maths/SIMD.h"

void TestSIMD()
{
	printf("Running TestSIMD\n");
	Vec4V v1 = V4Load(1.2f);
	Vec4V v2 = V4Load(1.2f);
	Vec4V v3 = V4Mul(v1, v2);
	Vector3d vv = V4ReadXYZ(v3);
	return;
}

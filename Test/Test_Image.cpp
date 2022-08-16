
#include <vector>

#include "Test.h"

#include "../Src/Modules/Tools/ContinuousBitmap.h"

void TestBitmap()
{
	printf("Running TestBitmap\n");
	int a[] = { 1, 1, 0, 0, 1, 0, 1, 0,
				1, 1, 1, 1, 1, 1, 1, 1,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 1, 0, 1, 1 };
	ContinuousBitmap<uint16_t> bitmap;
	bitmap.Build<int>(a, 8, 4, -4, -4, 4, 4);
	bitmap.SerializeToFile("data/cbit.map");
	bitmap.SerializeFromFile("data/cbit.map");
	EXPECT(bitmap.QueryBitmapSpace(3, 0) == false);
	EXPECT(bitmap.QueryBitmapSpace(4, 0) == true);
	EXPECT(bitmap.QueryBitmapSpace(3, 1) == true);
	EXPECT(bitmap.QueryBitmapSpace(13, 1) == false);
	EXPECT(bitmap.QueryBitmapSpace(7, 3) == true);
}

void TestImage()
{
	TestBitmap();
}

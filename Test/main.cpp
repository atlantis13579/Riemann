
#include "stdio.h"

extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestPhysics3d();
extern void TestPhysics2d();
extern void TestPython();
extern void TestSIMD();
extern void TestMatrix();
extern void TestVehicle();

void TestAll()
{
	TestMaths();
	TestCollision();
	TestGeometry();
	TestPhysics3d();
	TestPhysics2d();
	TestSIMD();
	TestMatrix();
	TestVehicle();
}

int main()
{
	printf("Running ...\n");
	TestGeometry();
	printf("All Test Done\n");
	return 0;
}

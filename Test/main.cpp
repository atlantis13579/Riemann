
#include "stdio.h"

extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestPhysics();
extern void TestPython();
extern void TestSIMD();
extern void TestMatrix();
extern void TestVehicle();

void TestAll()
{
	TestMaths();
	TestCollision();
	TestGeometry();
	TestPhysics();
	TestSIMD();
	TestMatrix();
	TestVehicle();
}

int main()
{
	printf("Running ...\n");
	TestCollision();
	printf("All Test Done\n");
	return 0;
}


#include "stdio.h"

extern void TestCores();
extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestPhysics3d();
extern void TestPython();
extern void TestSIMD();
extern void TestMatrix();
extern void TestVehicle();
extern void TestDestruction();
extern void TestOptimization();

void TestAll()
{
	TestCores();
	TestMaths();
	TestCollision();
	TestGeometry();
	TestDestruction();
	TestPhysics3d();
	TestSIMD();
	TestMatrix();
	TestVehicle();
	TestOptimization();
}

int main()
{
	printf("Running ...\n");
	TestAll();
	printf("All Test Done\n");
	return 0;
}

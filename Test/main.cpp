
#include "stdio.h"

extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestPhysics();
extern void TestMainEntry();
extern void TestPython();
extern void TestSIMD();
extern void TestMatrix();
extern void TestVehicle();

void TestAll()
{
	TestMainEntry();
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
	TestMaths();
	// TestAll();
	printf("All Test Done\n");
	return 0;
}

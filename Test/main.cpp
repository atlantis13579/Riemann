
#include "stdio.h"

extern void TestCores();
extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestPhysics3d();
extern void TestPhysics2d();
extern void TestPython();
extern void TestSIMD();
extern void TestMatrix();
extern void TestVehicle();
extern void TestOptimization();

void TestAll()
{
	// TestCores();
	TestMaths();
	// TestCollision();
	// TestGeometry();
	// TestPhysics3d();
	// TestPhysics2d();
	// TestSIMD();
	// TestMatrix();
	// TestVehicle();
	// TestOptimization();
}

#include "../Src/Maths/Matrix3.h"

int main()
{
	printf("Running ...\n");
	TestAll();
	printf("All Test Done\n");
	return 0;
}


#include "Test.h"

#include "../Src/Physics2d/Box2d.h"
#include "../Src/Physics2d/Circle2d.h"

void TestBox()
{
	Box2d b1(Point2(0, 0), Point2(1, 1), 0.0f);
	Circle2d c1(Point2(0, 0), 1.0f);
}

void TestPhysics2d()
{
	TestBox();
	return;
}


#include "Test.h"

#include "../Src/Modules/Physics2d/Rect.h"
#include "../Src/Modules/Physics2d/Circle.h"

void TestRect()
{
	Rect b1(Point2(0, 0), Point2(1, 1), 0.0f);
	Circle c1(Point2(0, 0), 1.0f);
}

void TestPhysics2d()
{
	TestRect();
	return;
}

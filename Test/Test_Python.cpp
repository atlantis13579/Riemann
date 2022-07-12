
#include <vector>

#include "Test.h"
#include "../Src/ImageSpace/ContinuousBitmap.h"
#include "../Src/Tools/PhysxBinaryParser.h"
#include "../Src/Tools/PythonModule.h"

void TestDepthImage()
{
	void* p = LoadPhysxScene("data/Japan.xml.bin");
	std::vector<float> image;
	int w = 100;
	int h = 100;
	image.resize(w * h);
	//RenderDepthImage(p, &image[0], w, h, 1.570796327f, 0.1f, 50.0f,
 	//				-521.23f, 56.87f, 399.15f, 0.45f, 0.0f, 0.45f, 0.0f, 1.0f, 0.0f);
	RenderDepthImage(p, &image[0], w, h, 1.570796327f, 0.1f, 200.0f,
					-0.0f, 156.87f, 0.0f, -0.45f, -0.45f, -0.45f, 0.0f, 1.0f, 0.0f, true);
	return;
}

void TestPython()
{
    TestDepthImage();
}

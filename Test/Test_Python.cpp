
#include <vector>

#include "Test.h"
#include "../Src/ImageSpace/ContinuousBitmap.h"
#include "../Src/Tools/PhysxBinaryParser.h"
#include "../Src/Tools/PythonModule.h"

void TestDepthImage()
{
	void* p = LoadPhysxScene("data/Japan.xml.bin");
	std::vector<float> image;
	image.resize(1024 * 768);
	// RenderDepthImage(p, &image[0], 200, 200, 1.570796327f, 0.1f, 50.0f,
 	//				-521.23f, 56.87f, 399.15f, 0.45f, 0.0f, 0.45f, 0.0f, 1.0f, 0.0f);
	RenderDepthImage(p, &image[0], 100, 100, 1.570796327f, 0.1f, 200.0f,
					-0.0f, 156.87f, 0.0f, -0.45f, -0.45f, 0.6f, 0.0f, 1.0f, 0.0f);
	return;
}

void TestPython()
{
    TestDepthImage();
}

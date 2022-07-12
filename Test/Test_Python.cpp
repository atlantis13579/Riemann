
#include <vector>
#include <chrono>

#include "Test.h"
#include "../Src/ImageSpace/ContinuousBitmap.h"
#include "../Src/Tools/PhysxBinaryParser.h"
#include "../Src/Tools/PythonModule.h"

void TestDepthImage()
{
	printf("Running TestDepthImage\n");
	void* p = LoadPhysxScene("data/Japan.xml.bin");
	std::vector<float> image;
	int w = 400;
	int h = 400;
	image.resize(w * h);

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	//RenderDepthImage(p, &image[0], w, h, 1.0f, 0.1f, 50.0f,
 	// 				-521.23f, 56.87f, 399.15f, 0.45f, 0.0f, 0.45f, 0.0f, 1.0f, 0.0f, true);
	RenderDepthImage(p, &image[0], w, h, 1.0f, 0.1f, 200.0f,
					1768.8f, 19.5f + 1.8f, 1551.5f, sinf(-0.27f), 0.0f, cosf(-0.27f), 0.0f, 1.0f, 0.0f, true);
	//RenderDepthImage(p, &image[0], w, h, 1.0f, 0.1f, 200.0f,
	// 				-0.0f, 156.87f, 0.0f, -0.45f, -0.45f, -0.45f, 0.0f, 1.0f, 0.0f, true);

	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	printf("cost = %d\n", diff);
	return;
}

void TestPython()
{
    TestDepthImage();
}


#include "stdio.h"
#include <string>
#include <vector>

#include "Test.h"

extern void TestCores();
extern void TestMaths();
extern void TestCollision();
extern void TestGeometry();
extern void TestSIMD();
extern void TestMatrix();
extern void TestDestruction();
extern void TestOptimization();

static bool ShouldRun(const char* name, const std::vector<std::string>& filters)
{
	if (filters.empty())
	{
		return true;
	}
	for (const std::string& filter : filters)
	{
		if (filter == name || filter == "All")
		{
			return true;
		}
	}
	return false;
}

static void RunTestGroup(const char* name, void (*func)(), const std::vector<std::string>& filters)
{
	if (!ShouldRun(name, filters))
	{
		return;
	}

	printf("Running %s\n", name);
	func();
}

void TestAll(const std::vector<std::string>& filters)
{
	RunTestGroup("Core", TestCores, filters);
	RunTestGroup("Maths", TestMaths, filters);
	RunTestGroup("Collision", TestCollision, filters);
	RunTestGroup("Geometry", TestGeometry, filters);
	RunTestGroup("Destruction", TestDestruction, filters);
	RunTestGroup("SIMD", TestSIMD, filters);
	RunTestGroup("Matrix", TestMatrix, filters);
	RunTestGroup("Optimization", TestOptimization, filters);
}

int main(int argc, char** argv)
{
	setvbuf(stdout, nullptr, _IONBF, 0);
	TestResetFailures();

	std::vector<std::string> filters;
	for (int i = 1; i < argc; ++i)
	{
		filters.push_back(argv[i]);
	}

	printf("Running tests ...\n");
	TestAll(filters);

	const int failures = TestFailureCount();
	if (failures == 0)
	{
		printf("All Test Done\n");
		return 0;
	}

	printf("Tests FAILED: %d\n", failures);
	return 1;
}


#include "stdio.h"
#include <string>
#include <vector>

#include "Test.h"

extern void TestCores();
extern void TestMaths();
extern void TestCollision();
extern void TestFiveByFiveBoxStackGridStability();
extern void TestCollisionPrimitive();
extern void TestGeometry();
extern void TestSIMD();
extern void TestMatrix();
extern void TestDestruction();
extern void TestOptimization();
#if defined(RIEMANN_ENABLE_CUDA)
extern void TestCUDA();
#endif
extern void TestCompoundNonConvexRigidBody();
extern void TestContinuousCollisionDetection();
extern void TestPositionConstraints();
extern void TestRopeSystem();
extern void TestClothSystem();
extern void TestRagdollSystem();
extern void TestMutableTerrain();
extern void TestUE5StaticMeshParser();

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

static bool HasExplicitFilter(const char* name, const std::vector<std::string>& filters)
{
	for (const std::string& filter : filters)
	{
		if (filter == name)
		{
			return true;
		}
	}
	return false;
}

void TestAll(const std::vector<std::string>& filters)
{
	RunTestGroup("Core", TestCores, filters);
	RunTestGroup("Maths", TestMaths, filters);
	RunTestGroup("Collision", TestCollision, filters);
	if (HasExplicitFilter("LCPStack", filters))
	{
		RunTestGroup("LCPStack", TestFiveByFiveBoxStackGridStability, filters);
	}
	RunTestGroup("CompoundRigidBody", TestCompoundNonConvexRigidBody, filters);
	RunTestGroup("CCD", TestContinuousCollisionDetection, filters);
	RunTestGroup("Constraints", TestPositionConstraints, filters);
	RunTestGroup("Rope", TestRopeSystem, filters);
	RunTestGroup("Cloth", TestClothSystem, filters);
	RunTestGroup("Ragdoll", TestRagdollSystem, filters);
	RunTestGroup("Terrain", TestMutableTerrain, filters);
	RunTestGroup("CollisionPrimitive", TestCollisionPrimitive, filters);
	RunTestGroup("Geometry", TestGeometry, filters);
	RunTestGroup("UEStaticMesh", TestUE5StaticMeshParser, filters);
	RunTestGroup("Destruction", TestDestruction, filters);
	RunTestGroup("SIMD", TestSIMD, filters);
	RunTestGroup("Matrix", TestMatrix, filters);
	RunTestGroup("Optimization", TestOptimization, filters);
#if defined(RIEMANN_ENABLE_CUDA)
	RunTestGroup("CUDA", TestCUDA, filters);
#endif
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

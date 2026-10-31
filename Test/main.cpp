
#include "stdio.h"
#include <string>
#include <vector>

#include "Test.h"

extern void TestCores();
extern void TestFluidOptimization();
extern void TestSmoke();
extern void TestSmokeBackends();
extern void TestSmokeMovingObstacles();
extern void TestSmokeRigidCoupling();
extern void TestSolverPanel();
extern void TestSolverSwitch();
extern void TestPBDBackendScenes();
extern void TestPBDBackends();
extern void TestVehicleScene();
extern void TestVehicleDeformation();
extern void TestRopeTowingDemo();
extern void TestXPBDScene();
extern void TestXPBDRigidSolver();
extern void TestPBDScene();
extern void TestPBDRigidSolver();
extern void TestVoxelScene();
extern void TestVoxelMesh();
extern void TestTerrainScene();
extern void TestTerrainEditing();
extern void TestPackedGEMM();
extern void TestStressIntegration();
extern void TestStressSolver();
extern void TestDeformableSystem();
extern void TestSceneLoading();
extern void TestRopeRegression();
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
extern void TestConstraintSolverRegression();
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
	RunTestGroup("FluidOptimization", TestFluidOptimization, filters);
	RunTestGroup("Smoke", TestSmoke, filters);
	RunTestGroup("SmokeBackends", TestSmokeBackends, filters);
	RunTestGroup("SmokeMovingObstacles", TestSmokeMovingObstacles, filters);
	RunTestGroup("SmokeRigidCoupling", TestSmokeRigidCoupling, filters);
	RunTestGroup("SolverPanel", TestSolverPanel, filters);
	RunTestGroup("SolverSwitch", TestSolverSwitch, filters);
	RunTestGroup("PBDBackendScenes", TestPBDBackendScenes, filters);
	RunTestGroup("PBDBackends", TestPBDBackends, filters);
	RunTestGroup("VehicleScene", TestVehicleScene, filters);
	RunTestGroup("VehicleDeformation", TestVehicleDeformation, filters);
	RunTestGroup("RopeTowing", TestRopeTowingDemo, filters);
	RunTestGroup("XPBDScene", TestXPBDScene, filters);
	RunTestGroup("XPBD", TestXPBDRigidSolver, filters);
	RunTestGroup("PBDScene", TestPBDScene, filters);
	RunTestGroup("PBD", TestPBDRigidSolver, filters);
	RunTestGroup("VoxelScene", TestVoxelScene, filters);
	RunTestGroup("VoxelMesh", TestVoxelMesh, filters);
	RunTestGroup("TerrainScene", TestTerrainScene, filters);
	RunTestGroup("TerrainEditing", TestTerrainEditing, filters);
	RunTestGroup("GEMM", TestPackedGEMM, filters);
	RunTestGroup("StressIntegration", TestStressIntegration, filters);
	RunTestGroup("Stress", TestStressSolver, filters);
	RunTestGroup("Deformation", TestDeformableSystem, filters);
	RunTestGroup("Scenes", TestSceneLoading, filters);
	RunTestGroup("RopeRegression", TestRopeRegression, filters);
	RunTestGroup("Maths", TestMaths, filters);
	RunTestGroup("Collision", TestCollision, filters);
	if (HasExplicitFilter("LCPStack", filters))
	{
		RunTestGroup("LCPStack", TestFiveByFiveBoxStackGridStability, filters);
	}
	RunTestGroup("CompoundRigidBody", TestCompoundNonConvexRigidBody, filters);
	RunTestGroup("CCD", TestContinuousCollisionDetection, filters);
	RunTestGroup("Constraints", TestPositionConstraints, filters);
	RunTestGroup("ConstraintRegression", TestConstraintSolverRegression, filters);
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

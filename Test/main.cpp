
#include "stdio.h"
#include <string>
#include <vector>

#include "Test.h"

extern void TestCores();
extern void TestContactPersistence();
extern void TestMeshContacts();
extern void TestCollisionFiltering();
extern void TestContactOffsets();
extern void TestRuntimeDistanceField();
extern void TestDistanceFieldCollision();
extern void TestDistanceFieldMesh();
extern void TestDistanceFieldMeshSimulation();
extern void TestDistanceFieldSweeps();
extern void TestDistanceFieldCCD();
extern void TestConstraintGraph();
extern void TestFrictionPatches();
extern void TestJointDrives();
extern void TestJointReactions();
extern void TestCCDContinuation();
extern void TestTemporalSubsteps();
extern void TestSupportStabilization();
extern void TestIslandSleeping();
extern void TestFeatherstone();
extern void TestReducedRagdoll();
extern void TestReducedRagdollQueries();
extern void TestRagdollMechanism();
extern void TestPhysicalAnimation();
extern void TestClothModel();
extern void TestSurfaceCollision();
extern void TestSurfaceContact();
extern void TestDeformableAttachment();
extern void TestDeformableCollision();
extern void TestDeformableRigidContact();
extern void TestTetrahedralMaterial();
extern void TestTetrahedralAsset();
extern void TestProjectiveDynamics();
extern void TestDestructionAsset();
extern void TestDestructionEventStream();
extern void TestDestructionCache();
extern void TestDestructionPlayback();
extern void TestPhysicsField();
extern void TestStrands();
extern void TestStrandGroom();
extern void TestStrandCollision();
extern void TestRigidAttachment();
extern void TestClothCollision();
extern void TestClothContinuous();
extern void TestClothInteraction();
extern void TestClothRigidCoupling();
extern void TestUnifiedBroadPhase();
extern void TestWorldAerodynamics();
extern void TestAerodynamicsPressureAccuracy();
extern void TestVehicleDynamics();
extern void TestSparseJointSpaceDynamics();
extern void TestDistanceFieldMargins();
extern void TestFluidOptimization();
extern void TestAerodynamicsIncompressible();
extern void TestAerodynamicsInterface();
extern void TestAerodynamicsSimulation();
extern void TestAerodynamicsRigidCoupling();
extern void TestAerodynamicsBackends();
extern void TestAerodynamicsMovingObstacles();
extern void TestAerodynamicsBoundaryCoupling();
extern void TestPBDBackends();
extern void TestBackendAvailability();
extern void TestVehicleDeformation();
extern void TestXPBDRigidSolver();
extern void TestPBDScene();
extern void TestPBDRigidSolver();
extern void TestVoxelMesh();
extern void TestTerrainEditing();
extern void TestPackedGEMM();
extern void TestStressIntegration();
extern void TestStressSolver();
extern void TestDeformableSystem();
extern void TestRopeRegression();
extern void TestMaths();
extern void TestCollision();
extern void TestFiveByFiveBoxStackGridStability();
extern void TestCollisionPrimitive();
extern void TestGeometry();
extern void TestGeometryBoolean();
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
	RunTestGroup("ConstraintGraph", TestConstraintGraph, filters);
	RunTestGroup("FrictionPatches", TestFrictionPatches, filters);
	RunTestGroup("JointDrives", TestJointDrives, filters);
	RunTestGroup("JointReactions", TestJointReactions, filters);
	RunTestGroup("CCDContinuation", TestCCDContinuation, filters);
	RunTestGroup("TemporalSubsteps", TestTemporalSubsteps, filters);
	RunTestGroup("SupportStabilization", TestSupportStabilization, filters);
	RunTestGroup("IslandSleeping", TestIslandSleeping, filters);
	RunTestGroup("Featherstone", TestFeatherstone, filters);
	RunTestGroup("ReducedRagdoll", TestReducedRagdoll, filters);
	RunTestGroup("ReducedRagdollQueries", TestReducedRagdollQueries, filters);
	RunTestGroup("RagdollMechanism", TestRagdollMechanism, filters);
	RunTestGroup("PhysicalAnimation", TestPhysicalAnimation, filters);
	RunTestGroup("ClothModel", TestClothModel, filters);
	RunTestGroup("SurfaceCollision", TestSurfaceCollision, filters);
	RunTestGroup("ClothCollision", TestClothCollision, filters);
	RunTestGroup("ClothContinuous", TestClothContinuous, filters);
	RunTestGroup("ClothInteraction", TestClothInteraction, filters);
	RunTestGroup("ClothRigidCoupling", TestClothRigidCoupling, filters);
	RunTestGroup("UnifiedBroadPhase", TestUnifiedBroadPhase, filters);
	RunTestGroup("WorldAerodynamics", TestWorldAerodynamics, filters);
	if (!filters.empty()) RunTestGroup("AerodynamicsPressureAccuracy", TestAerodynamicsPressureAccuracy, filters);
	RunTestGroup("SurfaceContact", TestSurfaceContact, filters);
	RunTestGroup("DeformableAttachment", TestDeformableAttachment, filters);
	RunTestGroup("DeformableCollision", TestDeformableCollision, filters);
	RunTestGroup("DeformableRigidContact", TestDeformableRigidContact, filters);
	RunTestGroup("TetrahedralMaterial", TestTetrahedralMaterial, filters);
	RunTestGroup("TetrahedralAsset", TestTetrahedralAsset, filters);
	RunTestGroup("ProjectiveDynamics", TestProjectiveDynamics, filters);
	RunTestGroup("DestructionAsset", TestDestructionAsset, filters);
	RunTestGroup("DestructionEvents", TestDestructionEventStream, filters);
	RunTestGroup("DestructionCache", TestDestructionCache, filters);
	RunTestGroup("DestructionPlayback", TestDestructionPlayback, filters);
	RunTestGroup("PhysicsField", TestPhysicsField, filters);
	RunTestGroup("Strands", TestStrands, filters);
	RunTestGroup("StrandGroom", TestStrandGroom, filters);
	RunTestGroup("StrandCollision", TestStrandCollision, filters);
	RunTestGroup("RigidAttachment", TestRigidAttachment, filters);
	RunTestGroup("VehicleDynamics", TestVehicleDynamics, filters);
	RunTestGroup("SparseJointSpaceDynamics", TestSparseJointSpaceDynamics, filters);
	RunTestGroup("ContactPersistence", TestContactPersistence, filters);
	RunTestGroup("MeshContacts", TestMeshContacts, filters);
	RunTestGroup("CollisionFiltering", TestCollisionFiltering, filters);
	RunTestGroup("ContactOffsets", TestContactOffsets, filters);
	RunTestGroup("RuntimeDistanceField", TestRuntimeDistanceField, filters);
	RunTestGroup("DistanceFieldCollision", TestDistanceFieldCollision, filters);
	RunTestGroup("DistanceFieldMesh", TestDistanceFieldMesh, filters);
	RunTestGroup("DistanceFieldMeshSimulation", TestDistanceFieldMeshSimulation, filters);
	RunTestGroup("DistanceFieldSweeps", TestDistanceFieldSweeps, filters);
	RunTestGroup("DistanceFieldCCD", TestDistanceFieldCCD, filters);
	RunTestGroup("DistanceFieldMargins", TestDistanceFieldMargins, filters);
	RunTestGroup("FluidOptimization", TestFluidOptimization, filters);
	RunTestGroup("AerodynamicsIncompressible", TestAerodynamicsIncompressible, filters);
	RunTestGroup("AerodynamicsInterface", TestAerodynamicsInterface, filters);
	RunTestGroup("AerodynamicsBarotropic", TestAerodynamicsSimulation, filters);
	RunTestGroup("AerodynamicsSurfaceCoupling", TestAerodynamicsRigidCoupling, filters);
	RunTestGroup("AerodynamicsBackends", TestAerodynamicsBackends, filters);
	RunTestGroup("AerodynamicsMovingObstacles", TestAerodynamicsMovingObstacles, filters);
	RunTestGroup("AerodynamicsBoundaryCoupling", TestAerodynamicsBoundaryCoupling, filters);
	RunTestGroup("PBDBackends", TestPBDBackends, filters);
    RunTestGroup("BackendAvailability", TestBackendAvailability, filters);
	RunTestGroup("VehicleDeformation", TestVehicleDeformation, filters);
	RunTestGroup("XPBD", TestXPBDRigidSolver, filters);
	RunTestGroup("PBDScene", TestPBDScene, filters);
	RunTestGroup("PBD", TestPBDRigidSolver, filters);
	RunTestGroup("VoxelMesh", TestVoxelMesh, filters);
	RunTestGroup("TerrainEditing", TestTerrainEditing, filters);
	RunTestGroup("GEMM", TestPackedGEMM, filters);
	RunTestGroup("StressIntegration", TestStressIntegration, filters);
	RunTestGroup("Stress", TestStressSolver, filters);
	RunTestGroup("Deformation", TestDeformableSystem, filters);
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
	if (HasExplicitFilter("GeometryBoolean", filters))
	{
		RunTestGroup("GeometryBoolean", TestGeometryBoolean, filters);
	}
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

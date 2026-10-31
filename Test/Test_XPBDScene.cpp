#include "Test.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Deformation/XPBDSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

void TestXPBDScene()
{
	printf("Running TestXPBDScene\n");
	SceneWorld scene;
	std::string error;
	const bool loaded = scene.LoadFromFile(TestContentsPath() + "Scenes/17_xpbd_rigid.json", &error);
	EXPECT(loaded);
	if (!loaded) { printf("%s\n", error.c_str()); return; }
	EXPECT(scene.GetSimulation()->GetResolutionSolver() == ResolutionPhaseSolver::XPBD);
	EXPECT(scene.GetSimulation()->GetSolverBackend() == SolverBackend::CPU);
	const XPBDSolver* solver = dynamic_cast<const XPBDSolver*>(scene.GetSimulation()->GetPositionSolver());
	EXPECT(solver != nullptr);
	if (solver)
	{
		EXPECT(solver->GetSettings().PositionIterations == 16);
		EXPECT_NEAR(solver->GetXPBDSettings().ContactCompliance, 0.003f, 1.0e-7f);
	}
	EXPECT(scene.GetSimulation()->GetConstraintCount() == 2);
	RigidBody* sphere = nullptr;
	for (const SceneObjectInstance& object : scene.GetObjects())
		if (object.Id == "contact_sphere") sphere = object.GeometryPtr->GetParent<RigidBody>();
	EXPECT(sphere != nullptr);
	if (!sphere) return;
	float maxSoftDistance = 0.0f, maxHardError = 0.0f;
	for (int frame = 0; frame < 360; ++frame)
	{
		scene.Step(1.0f / 120.0f);
		const std::vector<ConstraintControlState> states = scene.GetConstraintControls();
		if (states.size() != 2) { EXPECT(false); return; }
		maxSoftDistance = std::max(maxSoftDistance, states[0].CurrentValue);
		maxHardError = std::max(maxHardError, std::abs(states[1].CurrentValue - 1.5f));
		EXPECT(std::isfinite(sphere->X.y));
	}
	printf("  XPBD scene: soft distance max=%.4f hard error=%.6f contact y=%.4f\n",
		maxSoftDistance, maxHardError, sphere->X.y);
	EXPECT(maxSoftDistance > 1.65f && maxSoftDistance < 2.3f);
	EXPECT(maxHardError < 0.01f);
	EXPECT(sphere->X.y > 0.45f && sphere->X.y < 0.55f);
	const Vector3 before = sphere->X;
	scene.Step(0.0f);
	scene.Step(-1.0f);
	EXPECT_SAME(sphere->X, before);
	EXPECT(scene.LoadFromFile(TestContentsPath() + "Scenes/17_xpbd_rigid.json", &error));
	EXPECT(scene.GetSimulation()->GetResolutionSolver() == ResolutionPhaseSolver::XPBD);
}

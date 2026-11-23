#include "Test.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Solver/XPBDSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include <fstream>
#include <iterator>

using namespace Riemann;

void TestXPBDScene()
{
	printf("Running TestXPBDScene\n");
	SceneWorld scene;
	std::string error;
	std::ifstream file(TestContentsPath() + "Scenes/06_constraints.json", std::ios::binary);
	EXPECT(!!file);
	if (!file) return;
	std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	const size_t physics = text.find("\"physics\": {");
	EXPECT(physics != std::string::npos);
	if (physics == std::string::npos) return;
	text.insert(physics + 12, "\"resolutionSolver\": \"xpbd\", ");
	const bool loaded = scene.LoadFromText(text, "06_constraints_xpbd", &error);
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
	EXPECT(scene.GetSimulation()->GetConstraintCount() == 10);
	RigidBody* sphere = nullptr;
	for (const SceneObjectInstance& object : scene.GetObjects())
		if (object.Id == "bounce") sphere = object.GeometryPtr->GetParent<RigidBody>();
	EXPECT(sphere != nullptr);
	if (!sphere) return;
	float maxSoftDistance = 0.0f, maxHardError = 0.0f;
	for (int frame = 0; frame < 360; ++frame)
	{
		scene.Step(1.0f / 120.0f);
		const std::vector<ConstraintControlState> states = scene.GetConstraintControls();
		if (states.size() != 10) { EXPECT(false); return; }
		maxSoftDistance = std::max(maxSoftDistance, states[8].CurrentValue);
		maxHardError = std::max(maxHardError, std::abs(states[9].CurrentValue - 1.5f));
		EXPECT(std::isfinite(sphere->X.y));
	}
	EXPECT(maxSoftDistance > 1.65f);
	EXPECT(maxHardError < 0.01f);
	EXPECT(sphere->X.y > -1.6f && sphere->X.y < 0.5f);
	const Vector3 before = sphere->X;
	scene.Step(0.0f);
	scene.Step(-1.0f);
	EXPECT_SAME(sphere->X, before);
	EXPECT(scene.LoadFromText(text, "06_constraints_xpbd", &error));
	EXPECT(scene.GetSimulation()->GetResolutionSolver() == ResolutionPhaseSolver::XPBD);
}

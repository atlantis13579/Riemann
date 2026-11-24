#pragma once

#include <cstdint>
#include <string>

namespace Riemann
{
	// UI order is SI, LCP, PBD, XPBD; backend order is CPU, SIMD, ComputeShader, CUDA.
	// These indices are deliberately independent of the engine enum values.
	struct SolverPanelState
	{
		int ResolutionIndex = 0;
		int BackendIndex = 0;
		int ActiveBackendIndex = 0;
		int ActiveRequestedBackendIndex = 0;
		bool AllowFallback = true;
		bool HasClothDemo = false;
		int ClothAlgorithm = 0;
		float ClothIterations = 12, ClothSubsteps = 1, ClothDamping = 0.995f;
		float ClothStretchUCompliance = 0.0001f, ClothStretchVCompliance = 0.0002f;
		float ClothShearCompliance = 0.01f, ClothBendingCompliance = 25;
		bool ClothSelfCollision = false, ClothRigidCoupling = false;
		bool Available[4][4] = {
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
		};
		bool Enabled = false;
		bool SolverSwitchable = false;
		bool SolverChanged = false;
		bool Pending = false;
		std::string Status;
		uint32_t SceneGeneration = 0;
	};

	// Draw inside an existing imgui scroll area. Changes are applied on the simulation thread.
	bool DrawSolverPanel(SolverPanelState& State);
	bool DrawClothPanel(SolverPanelState& State);
}

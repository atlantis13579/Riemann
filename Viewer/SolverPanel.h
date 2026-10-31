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
		int ActiveResolutionIndex = 0;
		int ActiveBackendIndex = 0;
		int ActiveRequestedBackendIndex = 0;
		bool AllowFallback = true;
		bool Available[4][4] = {
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
		};
		bool Enabled = false;
		bool Pending = false;
		std::string Status;
		uint32_t SceneGeneration = 0;
	};

	// Draw inside an existing imgui scroll area. Only draft selections are changed;
	// true requests application by the simulation thread before its next physics step.
	bool DrawSolverPanel(SolverPanelState& State);
}

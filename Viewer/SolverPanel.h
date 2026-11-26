#pragma once

#include <cstdint>
#include <string>

namespace Riemann
{
	class SceneWorld;

	// UI order is SI, LCP, PBD, XPBD; backend order is CPU, SIMD, ComputeShader, CUDA, Metal.
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
		bool Available[4][5] = {
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

		// Physics-field and destruction-cache demo in/out state. The flags on the
		// left are owned by the UI; the readouts are filled by
		// RefreshSolverPanelTelemetry so the panel never queries the scene itself.
		bool PhysicsFieldsDemo = false;
		float DestructionSeekFraction = 0.0f;
		bool DestructionRecording = false;
		bool DestructionPlaying = false;
		double DestructionCacheTime = 0;
		double DestructionCacheDuration = 0;
		uint64_t DestructionEvents = 0;
		uint64_t FieldBrokenBonds = 0;

		// Solver actually running; differs from the requested indices when the
		// backend fell back to CPU.
		int ActiveResolutionIndex = 0;

		// Strand preview readouts. Zero when the scene has no strand demo.
		bool HasStrands = false;
		uint64_t StrandGuides = 0;
		uint64_t StrandActiveGuides = 0;
		uint64_t StrandRenderCurves = 0;
		uint64_t StrandConstraintSolves = 0;
		double StrandEnergy = 0;
		uint64_t StrandContactRows = 0;
		uint64_t StrandContactCandidates = 0;
		uint64_t StrandContactRefinements = 0;
		double StrandContactPenetration = 0;
	};

	// Copies the demo/telemetry readouts from the scene into the panel state.
	void RefreshSolverPanelTelemetry(SolverPanelState& State, const SceneWorld& Scene);

	// Draw inside an existing imgui scroll area. Changes are applied on the simulation thread.
	bool DrawSolverPanel(SolverPanelState& State);
	bool DrawClothPanel(SolverPanelState& State);
}

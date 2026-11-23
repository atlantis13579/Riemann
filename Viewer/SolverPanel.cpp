#include "SolverPanel.h"
#include "SceneWorld.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Deformation/StrandSystem.h"

#include <algorithm>
#include <cstdio>

#include "../Renderer/imgui.h"

namespace Riemann
{
	namespace
	{
		int ValidIndex(int Index) { return Index >= 0 && Index < 4 ? Index : 0; }

		void DrawStatus(const std::string& Status)
		{
			// The Viewer uses a narrow side panel. Wrap errors as well as normal status.
			const size_t LineWidth = 27;
			size_t Start = 0;
			while (Start < Status.size())
			{
				while (Start < Status.size() && (Status[Start] == ' ' || Status[Start] == '\n' || Status[Start] == '\r')) ++Start;
				if (Start == Status.size()) break;
				size_t End = std::min(Start + LineWidth, Status.size());
				const size_t Newline = Status.find_first_of("\r\n", Start);
				if (Newline < End) End = Newline;
				else if (End < Status.size() && Status[End] != ' ')
				{
					const size_t Space = Status.rfind(' ', End);
					if (Space != std::string::npos && Space > Start) End = Space;
				}
				imguiValue(Status.substr(Start, End - Start).c_str());
				Start = End;
			}
		}
	}

	void RefreshSolverPanelTelemetry(SolverPanelState& State, const SceneWorld& Scene)
	{
		// These values change while a scene runs. Editable options are initialized
		// separately on reload, so telemetry must not overwrite an unapplied edit.
		const auto* World = Scene.GetSimulation();
		State.FieldSamples = World ? World->GetPhysicsFieldStats().Samples : 0;
		State.FieldBodyApplications = World ? World->GetPhysicsFieldStats().BodyApplications : 0;
		State.FieldBrokenBonds = World ? World->GetPhysicsFieldStats().BrokenBonds : 0;
		State.DestructionRecording = Scene.IsDestructionCacheRecording();
		State.DestructionPlaying = Scene.IsDestructionCachePlaying();
		State.DestructionCacheTime = Scene.GetDestructionCacheTime();
		State.DestructionCacheDuration = Scene.GetDestructionCacheDuration();
		State.DestructionEvents = Scene.GetDestructionEventCount();
		State.DestructionDroppedEvents = Scene.GetDestructionDroppedEventCount();
		const auto* Strands=Scene.GetDemoStrands(); State.HasStrands=Strands!=nullptr;
		State.StrandGuides=Strands ? Strands->GetGuideCount() : 0;
		State.StrandActiveGuides=Strands ? Strands->GetStats().SimulatedGuides : 0;
		State.StrandRenderCurves=Strands && Strands->GetGroom() ? Strands->GetGroom()->GetStrandCount() : 0;
		State.StrandContactRows=Strands ? Strands->GetCollisionStats().Contacts : 0;
		State.StrandContactCandidates=Strands ? Strands->GetCollisionStats().Candidates : 0;
		State.StrandContactRefinements=Strands ? Strands->GetCollisionStats().Refinements : 0;
		State.StrandContactPenetration=Strands ? Strands->GetCollisionStats().MaximumPenetration : 0;
		State.StrandConstraintSolves=Strands ? Strands->GetStats().ConstraintSolves : 0;
		State.StrandEnergy=Strands ? Strands->GetStats().ElasticEnergy : 0;
		State.StrandStretchShear=Strands ? Strands->GetStats().MaximumStretchShear : 0;
		State.StrandBendTwist=Strands ? Strands->GetStats().MaximumBendTwist : 0;
	}

	bool DrawSolverPanel(SolverPanelState& State)
	{
		const char* ResolutionLabels[] = { "Sequential Impulse", "LCP", "PBD", "XPBD" };
		const char* BackendNames[] = { "CPU", "SIMD", "ComputeShader", "CUDA" };
		State.ResolutionIndex = ValidIndex(State.ResolutionIndex);
		State.BackendIndex = ValidIndex(State.BackendIndex);
		const bool Editable = State.Enabled && !State.Pending;
		const bool SolverEditable = Editable && State.SolverSwitchable;
		const int PreviousResolution = State.ResolutionIndex;
		const int PreviousBackend = State.BackendIndex;
		State.SolverChanged = false;

		imguiSeparatorLine();
		imguiLabel("Rigid Solver");
		imguiLabel("resolutionSolver");
		imguiCombo("", &State.ResolutionIndex, ResolutionLabels, 4, SolverEditable);
		const bool SequentialImpulse = State.ResolutionIndex == 0;
		if (SequentialImpulse) State.BackendIndex = 0;

		std::string BackendLabels[4];
		const char* BackendItems[4];
		for (int Index = 0; Index < 4; ++Index)
		{
			BackendLabels[Index] = BackendNames[Index];
			if (!State.Available[State.ResolutionIndex][Index]) BackendLabels[Index] += " (N/A)";
			BackendItems[Index] = BackendLabels[Index].c_str();
		}
		imguiLabel("Backend");
		imguiCombo("", &State.BackendIndex, BackendItems, 4, SolverEditable && !SequentialImpulse);
		bool Available = State.Available[State.ResolutionIndex][State.BackendIndex];
		State.SolverChanged = SolverEditable && (State.ResolutionIndex != PreviousResolution || State.BackendIndex != PreviousBackend);
		if (State.SolverChanged && !Available && !State.AllowFallback)
		{
			State.ResolutionIndex = PreviousResolution;
			State.BackendIndex = PreviousBackend;
			State.SolverChanged = false;
			State.Status = "Backend unavailable without CPU fallback.";
			Available = State.Available[State.ResolutionIndex][State.BackendIndex];
		}
		if (SequentialImpulse) imguiValue("SI uses CPU only");
		else if (!Available)
		{
			imguiValue("Backend unavailable");
			imguiValue(State.AllowFallback ? "Will use CPU fallback" : "Choose another backend");
		}

		if (State.ActiveRequestedBackendIndex != State.ActiveBackendIndex)
		{
			char Message[64];
			std::snprintf(Message, sizeof(Message), "Fallback: %s -> %s",
				BackendNames[ValidIndex(State.ActiveRequestedBackendIndex)], BackendNames[ValidIndex(State.ActiveBackendIndex)]);
			DrawStatus(Message);
		}
		DrawStatus(State.Status);
		bool ApplyCloth = false;
		bool ClothAlgorithmChanged = false;
		bool ClothSelfCollisionChanged = false;
		if (State.HasClothDemo)
		{
			imguiSeparatorLine();
			imguiLabel("Cloth");
			const char* Algorithms[] = { "Distance Links", "Triangle Strain" };
			imguiLabel("Algorithm");
			ClothAlgorithmChanged = imguiCombo("", &State.ClothAlgorithm, Algorithms, 2, Editable);
			if (imguiCheck("Two-way rigid", State.ClothRigidCoupling, Editable)) State.ClothRigidCoupling = !State.ClothRigidCoupling;
			if (imguiCheck("Self collision", State.ClothSelfCollision, Editable))
			{
				State.ClothSelfCollision = !State.ClothSelfCollision;
				ClothSelfCollisionChanged = true;
			}
			imguiSlider("Iterations", &State.ClothIterations, 1, 32, 1, Editable);
			imguiSlider("Substeps", &State.ClothSubsteps, 1, 8, 1, Editable);
			imguiSlider("Damping", &State.ClothDamping, 0.9f, 1.0f, 0.005f, Editable);
			imguiSlider("Warp Compliance", &State.ClothStretchUCompliance, 0, 0.01f, 0.00001f, Editable);
			imguiSlider("Weft Compliance", &State.ClothStretchVCompliance, 0, 0.01f, 0.00001f, Editable);
			imguiSlider("Shear Compliance", &State.ClothShearCompliance, 0, 0.1f, 0.0001f, Editable);
			imguiSlider("Bending Compliance", &State.ClothBendingCompliance, 0, 50, 0.01f, Editable);
			ApplyCloth = imguiButton("Apply Cloth", Editable);
		}
		return State.SolverChanged || ClothAlgorithmChanged || ClothSelfCollisionChanged || ApplyCloth;
	}
}

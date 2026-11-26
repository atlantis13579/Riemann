#include "SolverPanel.h"

#include <algorithm>
#include <cstdio>

#include "../Renderer/imgui.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "SceneWorld.h"

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

	bool DrawSolverPanel(SolverPanelState& State)
	{
		const char* ResolutionLabels[] = { "Sequential Impulse", "LCP", "PBD", "XPBD" };
		const char* BackendNames[] = { "CPU", "SIMD", "DirectCompute", "CUDA", "Metal" };
		State.ResolutionIndex = ValidIndex(State.ResolutionIndex);
		State.BackendIndex = State.BackendIndex >= 0 && State.BackendIndex < 5 ? State.BackendIndex : 0;
		const bool Editable = State.Enabled && !State.Pending;
		const bool SolverEditable = Editable && State.SolverSwitchable;
		const int PreviousResolution = State.ResolutionIndex;
		const int PreviousBackend = State.BackendIndex;
		State.SolverChanged = false;

		// This imgui uses the text as the widget id, so the solver combo keeps the
		// stable 'resolutionSolver' heading and the combo box follows it.
		imguiLabel("resolutionSolver");
		imguiCombo("", &State.ResolutionIndex, ResolutionLabels, 4, SolverEditable);
		const bool SequentialImpulse = State.ResolutionIndex == 0;
		if (SequentialImpulse || (State.ResolutionIndex != PreviousResolution
            && !State.Available[State.ResolutionIndex][State.BackendIndex])) State.BackendIndex = 0;

		std::string BackendLabels[5];
		const char* BackendItems[5];
		for (int Index = 0; Index < 5; ++Index)
		{
			BackendLabels[Index] = BackendNames[Index];
			if (!State.Available[State.ResolutionIndex][Index]) BackendLabels[Index] += " (N/A)";
			BackendItems[Index] = BackendLabels[Index].c_str();
		}
		imguiLabel("Backend");
		imguiCombo("", &State.BackendIndex, BackendItems, 5, SolverEditable && !SequentialImpulse, State.Available[State.ResolutionIndex]);
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

		if (State.ActiveResolutionIndex != State.ResolutionIndex)
		{
			char Message[64];
			std::snprintf(Message, sizeof(Message), "Active: %s",
				ResolutionLabels[ValidIndex(State.ActiveResolutionIndex)]);
			DrawStatus(Message);
		}
		if (State.ActiveRequestedBackendIndex != State.ActiveBackendIndex)
		{
			char Message[64];
			std::snprintf(Message, sizeof(Message), "Fallback: %s -> %s",
				BackendNames[std::max(0, std::min(4, State.ActiveRequestedBackendIndex))], BackendNames[std::max(0, std::min(4, State.ActiveBackendIndex))]);
			DrawStatus(Message);
		}
		DrawStatus(State.Status);
		return State.SolverChanged;
	}

	bool DrawClothPanel(SolverPanelState& State)
	{
		const bool Editable = State.Enabled && !State.Pending;
		bool ApplyCloth = false;
		bool ClothAlgorithmChanged = false;
		bool ClothSelfCollisionChanged = false;
		if (State.HasClothDemo)
		{
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
		return ClothAlgorithmChanged || ClothSelfCollisionChanged || ApplyCloth;
	}

	void RefreshSolverPanelTelemetry(SolverPanelState& State, const SceneWorld& Scene)
	{
		State.DestructionRecording = Scene.IsDestructionCacheRecording();
		State.DestructionPlaying = Scene.IsDestructionCachePlaying();
		State.DestructionCacheTime = Scene.GetDestructionCacheTime();
		State.DestructionCacheDuration = Scene.GetDestructionCacheDuration();
		State.DestructionEvents = Scene.GetDestructionEventCount();
		const PhysicsWorld* World = Scene.GetSimulation();
		State.FieldBrokenBonds = World != nullptr ? World->GetPhysicsFieldStats().BrokenBonds : 0;

		const StrandSystem* Strands = Scene.GetDemoStrands();
		State.HasStrands = Strands != nullptr;
		State.StrandGuides = Strands != nullptr ? Strands->GetGuideCount() : 0;
		State.StrandConstraintSolves = Strands != nullptr ? Strands->GetStats().ConstraintSolves : 0;
		State.StrandEnergy = Strands != nullptr ? Strands->GetStats().ElasticEnergy : 0;
		State.StrandContactRows = Strands != nullptr ? Strands->GetCollisionStats().Contacts : 0;
		State.StrandContactCandidates = Strands != nullptr ? Strands->GetCollisionStats().Candidates : 0;
		State.StrandContactRefinements = Strands != nullptr ? Strands->GetCollisionStats().Refinements : 0;
		State.StrandContactPenetration = Strands != nullptr ? Strands->GetCollisionStats().MaximumPenetration : 0;

		// Render curves and the guides feeding them come from the groom, which
		// scales with the active LOD.
		const StrandGroom* Groom = Strands != nullptr ? Strands->GetGroom() : nullptr;
		State.StrandRenderCurves = Groom != nullptr ? Groom->GetStrandCount() : 0;
		State.StrandActiveGuides = 0;
		if (Groom != nullptr)
		{
			const size_t Lod = Scene.GetStrandDemoLod();
			if (Lod < Groom->GetLodCount())
				State.StrandActiveGuides = Groom->GetLod(Lod).Guides.size();
		}
	}
}

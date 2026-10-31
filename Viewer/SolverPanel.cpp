#include "SolverPanel.h"

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

	bool DrawSolverPanel(SolverPanelState& State)
	{
		const char* ResolutionLabels[] = { "Sequential Impulse", "LCP", "PBD", "XPBD" };
		const char* BackendNames[] = { "CPU", "SIMD", "ComputeShader", "CUDA" };
		State.ResolutionIndex = ValidIndex(State.ResolutionIndex);
		State.BackendIndex = ValidIndex(State.BackendIndex);
		const bool Editable = State.Enabled && !State.Pending;

		imguiSeparatorLine();
		imguiLabel("Rigid Solver");
		imguiLabel("resolutionSolver");
		imguiCombo("", &State.ResolutionIndex, ResolutionLabels, 4, Editable);
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
		imguiCombo("", &State.BackendIndex, BackendItems, 4, Editable && !SequentialImpulse);
		if (imguiCheck("Allow CPU fallback", State.AllowFallback, Editable && !SequentialImpulse))
			State.AllowFallback = !State.AllowFallback;

		const bool Available = State.Available[State.ResolutionIndex][State.BackendIndex];
		const bool CanApply = Editable && (Available || (!SequentialImpulse && State.AllowFallback));
		const bool Apply = imguiButton(State.Pending ? "Applying..." : "Apply Solver", CanApply);
		if (SequentialImpulse) imguiValue("SI uses CPU only");
		else if (!Available)
		{
			imguiValue("Backend unavailable");
			imguiValue(State.AllowFallback ? "Will use CPU fallback" : "Enable fallback or change");
		}

		imguiLabel("Actual solver / backend");
		imguiValue(ResolutionLabels[ValidIndex(State.ActiveResolutionIndex)]);
		imguiValue(BackendNames[ValidIndex(State.ActiveBackendIndex)]);
		if (State.ActiveRequestedBackendIndex != State.ActiveBackendIndex)
		{
			char Message[64];
			std::snprintf(Message, sizeof(Message), "Fallback: %s -> %s",
				BackendNames[ValidIndex(State.ActiveRequestedBackendIndex)], BackendNames[ValidIndex(State.ActiveBackendIndex)]);
			DrawStatus(Message);
		}
		DrawStatus(State.Status);
		return Apply;
	}
}

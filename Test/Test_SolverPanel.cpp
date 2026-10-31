#include "Test.h"

#include <cstring>

#include "../Renderer/imgui.h"
#include "../Viewer/SolverPanel.h"

namespace
{
	using Riemann::SolverPanelState;

	bool Frame(SolverPanelState& State, int X = -1, int Y = -1, unsigned char Buttons = 0)
	{
		int Scroll = 0;
		imguiBeginFrame(X, Y, Buttons, 0);
		imguiBeginScrollArea("", 10, 10, 220, 1000, &Scroll);
		const bool Apply = Riemann::DrawSolverPanel(State);
		imguiEndScrollArea();
		imguiEndFrame();
		return Apply;
	}

	int FindText(const char* Text, int Alignment = -1)
	{
		const imguiGfxCmd* Queue = imguiGetRenderQueue();
		for (int Index = 0; Index < imguiGetRenderQueueSize(); ++Index)
		{
			const imguiGfxCmd& Command = Queue[Index];
			if (Command.type == IMGUI_GFXCMD_TEXT && Command.text.text
				&& (Alignment < 0 || Command.text.align == Alignment) && std::strcmp(Command.text.text, Text) == 0)
				return Index;
		}
		return -1;
	}

	bool ClickAt(SolverPanelState& State, int Y)
	{
		// Existing imgui first acquires hover, then captures press and reports release.
		Frame(State, 100, Y);
		Frame(State, 100, Y, IMGUI_MBUT_LEFT);
		return Frame(State, 100, Y);
	}

	bool ClickText(SolverPanelState& State, const char* Text)
	{
		Frame(State);
		const int Index = FindText(Text, IMGUI_ALIGN_LEFT);
		EXPECT(Index >= 0);
		if (Index < 0) return false;
		return ClickAt(State, imguiGetRenderQueue()[Index].text.y + 4);
	}

	void SelectItem(SolverPanelState& State, const char* Heading, const char* Item)
	{
		Frame(State);
		const int Index = FindText(Heading, IMGUI_ALIGN_LEFT);
		EXPECT(Index >= 0);
		if (Index < 0) return;
		// A heading consumes 20 pixels; the combo is the next row below it.
		EXPECT(!ClickAt(State, imguiGetRenderQueue()[Index].text.y - 20 + 4));
		const int ItemIndex = FindText(Item, IMGUI_ALIGN_LEFT);
		EXPECT(ItemIndex >= 0);
		if (ItemIndex >= 0) EXPECT(!ClickAt(State, imguiGetRenderQueue()[ItemIndex].text.y + 4));
	}
}

void TestSolverPanel()
{
	printf("Running TestSolverPanel\n");
	SolverPanelState State;
	State.Enabled = true;
	State.SceneGeneration = 19;
	State.AllowFallback = false;
	State.Available[2][1] = true;
	Frame(State);
	EXPECT(FindText("resolutionSolver") >= 0);
	EXPECT(FindText("SI uses CPU only") >= 0);
	EXPECT(ClickText(State, "Apply Solver"));

	SelectItem(State, "resolutionSolver", "PBD");
	EXPECT(State.ResolutionIndex == 2);
	SelectItem(State, "Backend", "SIMD");
	EXPECT(State.BackendIndex == 1);
	EXPECT(State.ActiveResolutionIndex == 0 && State.ActiveBackendIndex == 0);
	EXPECT(State.SceneGeneration == 19);
	EXPECT(ClickText(State, "Apply Solver"));

	SelectItem(State, "Backend", "CUDA (N/A)");
	EXPECT(State.BackendIndex == 3);
	EXPECT(!ClickText(State, "Apply Solver"));
	EXPECT(FindText("Backend unavailable") >= 0);
	EXPECT(FindText("Enable fallback or change") >= 0);
	EXPECT(!ClickText(State, "Allow CPU fallback"));
	EXPECT(State.AllowFallback);
	EXPECT(ClickText(State, "Apply Solver"));
	EXPECT(FindText("Will use CPU fallback") >= 0);

	// Active fallback remains visible while a different draft is selected.
	State.ActiveResolutionIndex = 2;
	State.ActiveRequestedBackendIndex = 3;
	State.ActiveBackendIndex = 0;
	SelectItem(State, "Backend", "SIMD");
	Frame(State);
	EXPECT(FindText("Fallback: CUDA -> CPU") >= 0);
	EXPECT(State.ActiveRequestedBackendIndex == 3);

	// SI never submits an unsupported non-CPU request and its backend is read-only.
	SelectItem(State, "resolutionSolver", "Sequential Impulse");
	EXPECT(State.ResolutionIndex == 0 && State.BackendIndex == 0);
	EXPECT(!ClickText(State, "Allow CPU fallback"));
	EXPECT(State.AllowFallback);
	Frame(State);
	const int BackendHeading = FindText("Backend", IMGUI_ALIGN_LEFT);
	EXPECT(BackendHeading >= 0);
	if (BackendHeading >= 0) EXPECT(!ClickAt(State, imguiGetRenderQueue()[BackendHeading].text.y - 16));
	EXPECT(FindText("CUDA (N/A)", IMGUI_ALIGN_LEFT) < 0);
	EXPECT(State.BackendIndex == 0);

	State.Pending = true;
	EXPECT(!ClickText(State, "Applying..."));
	EXPECT(State.ResolutionIndex == 0 && State.BackendIndex == 0);
	State.Pending = false;
	State.Enabled = false;
	EXPECT(!ClickText(State, "Apply Solver"));

	State.Status = "Requested backend initialization failed; previous solver is still active.";
	Frame(State);
	EXPECT(FindText("Requested backend") >= 0);
	EXPECT(FindText("initialization failed;") >= 0);
	EXPECT(FindText("previous solver is still") >= 0);
	EXPECT(FindText("active.") >= 0);
	EXPECT(State.Status == "Requested backend initialization failed; previous solver is still active.");
}

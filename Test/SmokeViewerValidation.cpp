#if defined(RIEMANN_SMOKE_VIEWER_VALIDATION)
#define NOMINMAX
#include <windows.h>
#include <cstdio>
#include <fstream>
#include <memory>
#include "../Renderer/RiemannRenderer.h"
#include "../Viewer/RenderThread.h"
#include "../Viewer/Viewer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Fluid/SmokeRigidCoupling.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"
#include <chrono>

namespace Riemann
{
	struct SmokeViewerValidationAccess
	{
		static RigidBody* Body(WorldViewer& Viewer)
		{
			for (const auto& Object : Viewer.m_World->GetObjects())
				if (Object.Id == "smoke_body_0") return Object.GeometryPtr->GetParent<RigidBody>();
			return nullptr;
		}
		static bool Ready(WorldViewer& Viewer)
		{
			return Viewer.m_SmokeSimulation && Viewer.m_SmokeCoupling
				&& Viewer.m_SmokeCoupling->IsValid() && !Viewer.m_SmokePaused;
		}
		static bool Empty(WorldViewer& Viewer)
		{
			if (!Ready(Viewer) || Viewer.m_World->GetSmokeRigidBodyCount() != 0
				|| Viewer.m_ImguiSmokePanel.RigidBodyCount != 0) return false;
			for (unsigned char Solid : Viewer.m_SmokeSimulation->GetSolidCells()) if (Solid) return false;
			return true;
		}
		static void AddBody(WorldViewer& Viewer, int Count = 1)
		{
			Viewer.m_ImguiPendingSmokeAddBodies = Count;
			Viewer.m_ImguiPendingSmokeAddGeneration = Viewer.m_SceneGeneration;
		}
		static int Count(WorldViewer& Viewer) { return Viewer.m_World->GetSmokeRigidBodyCount(); }
		static double Time(WorldViewer& Viewer) { return Viewer.m_SmokeSimulation->GetTime(); }
		static bool CheckEmptyBaseline(WorldViewer& Viewer)
		{
			if (!Empty(Viewer)) return false;
			SmokeSimulation Reference;
			if (!Reference.Initialize(Viewer.m_SmokeSimulation->GetSettings())) return false;
			for (int Frame = 0; Frame < 30; ++Frame)
			{
				Viewer.UpdateSimulator(1.0f / 60);
				Reference.Step(1.0f / 60);
			}
			const bool Same = Empty(Viewer) && Viewer.m_SmokeSimulation->GetDensity() == Reference.GetDensity()
				&& Viewer.m_SmokeSimulation->GetTemperature() == Reference.GetTemperature()
				&& Viewer.m_SmokeSimulation->GetVelocity() == Reference.GetVelocity();
			std::printf("Zero-body plume versus standalone smoke: %s\n", Same ? "identical" : "FAILED");
			return Same;
		}
		static bool CheckPausedAddAndLimit(WorldViewer& Viewer)
		{
			if (!Empty(Viewer)) return false;
			AddBody(Viewer);
			--Viewer.m_ImguiPendingSmokeAddGeneration;
			Viewer.UpdateSimulator(1.0f / 60);
			if (!Empty(Viewer)) return false;
			Viewer.m_ImguiPendingSmokePause = true;
			Viewer.UpdateSimulator(1.0f / 60);
			const double Before = Time(Viewer);
			AddBody(Viewer);
			Viewer.UpdateSimulator(1.0f / 60);
			if (Count(Viewer) != 1 || !Viewer.m_SmokePaused || Time(Viewer) != Before) return false;
			AddBody(Viewer, 8);
			Viewer.UpdateSimulator(1.0f / 60);
			if (Count(Viewer) != SceneWorld::MaxSmokeRigidBodies || Time(Viewer) != Before
				|| Viewer.m_ImguiSmokePanel.RigidBodyCount != SceneWorld::MaxSmokeRigidBodies
				|| Viewer.m_World->AddSmokeRigidBody()) return false;
			for (const auto& Object : Viewer.m_World->GetObjects())
				if (Object.Id.find("smoke_body_") == 0 && !Viewer.m_World->IsMouseControlGeometry(Object.GeometryPtr)) return false;
			return true;
		}
		static bool CheckPauseAndRestart(WorldViewer& Viewer)
		{
			RigidBody* Rigid = Body(Viewer);
			if (!Rigid || !Ready(Viewer)) return false;
			const Vector3 Position = Rigid->X;
			const double Time = Viewer.m_SmokeSimulation->GetTime();
			Viewer.m_ImguiPendingSmokePause = true;
			for (int I = 0; I < 3; ++I) Viewer.UpdateSimulator(1.0f / 60);
			if (Rigid->X != Position || Viewer.m_SmokeSimulation->GetTime() != Time) return false;
			for (int Backend = 0; Backend < 3; ++Backend)
			{
#if !defined(RIEMANN_ENABLE_CUDA)
				if (Backend == 1) continue;
#endif
				Viewer.m_ImguiPendingSmokePanel = Viewer.m_ImguiSmokePanel;
				Viewer.m_ImguiPendingSmokePanel.Backend = Backend;
				Viewer.m_ImguiPendingSmokePanel.Quality = 0;
				Viewer.m_ImguiPendingSmokeRestart = true;
				Viewer.UpdateSimulator(1.0f / 60);
				if (!Ready(Viewer) || (int)Viewer.m_SmokeSimulation->GetBackend() != Backend
					|| Viewer.m_SmokeSimulation->GetTime() > 0.02 || Count(Viewer) != 3) return false;
			}
			Geometry* GeometryObject = Rigid->Geometries().front();
			const auto& Settings = Viewer.m_SmokeSimulation->GetSettings();
			if (!Viewer.m_World->SelectMouseControlGeometry(GeometryObject)
				|| !Viewer.m_World->SetMouseControlPosition(Vector3(0.5f, 2.5f, 0), 0.1f, Settings.BoundsMin, Settings.BoundsMax)) return false;
			Viewer.UpdateSimulator(1.0f / 60);
			SmokeObstacle Obstacle;
			if (!SmokeRigidCoupling::BuildObstacle(GeometryObject, Obstacle)) return false;
			int Cell[3];
			for (int A = 0; A < 3; ++A) Cell[A] = (int)((Obstacle.Center[A] - Settings.BoundsMin[A])
				* Settings.Resolution[A] / (Settings.BoundsMax[A] - Settings.BoundsMin[A]));
			const int Index = Cell[0] + Settings.Resolution.x * (Cell[1] + Settings.Resolution.y * Cell[2]);
			if (!Ready(Viewer) || !Viewer.m_SmokeSimulation->GetSolidCells()[Index]
				|| Viewer.m_SmokeSimulation->GetDensity()[Index] != 0) return false;
			return true;
		}
	};
}

// Runs the real viewer and renderer in a hidden window, without desktop input.
int main(int Argc, char** Argv)
{
	using namespace Riemann;
	setvbuf(stdout, nullptr, _IONBF, 0);
	RECT Rect = { 0, 0, 1280, 900 };
	AdjustWindowRect(&Rect, WS_OVERLAPPEDWINDOW, FALSE);
	HWND Window = CreateWindowExA(0, "STATIC", "Smoke visual validation", WS_OVERLAPPEDWINDOW,
		0, 0, Rect.right - Rect.left, Rect.bottom - Rect.top, nullptr, nullptr, GetModuleHandle(nullptr), nullptr);
	if (!Window) return 1;
	int Result = 0;
	{
		std::unique_ptr<Renderer> Graphics(Renderer::CreateDX11Renderer(Window, "Renderer/"));
		if (!Graphics) { DestroyWindow(Window); return 1; }
		RenderThread Render(Graphics.get());
		WorldViewer Viewer(&Render, "Contents/Scenes/12_3d_smoke.json");
		if (!SmokeViewerValidationAccess::CheckEmptyBaseline(Viewer)) return 1;
		if (!Viewer.LoadScene("Contents/Scenes/12_3d_smoke.json")) return 1;
		RigidBody* Body = nullptr;
		Vector3 Start = Vector3::Zero();
		const auto Started = std::chrono::steady_clock::now();
		const bool Animation = Argc > 1 && std::string(Argv[1]) == "animation";
		for (int Frame = 0; Frame <= 720; ++Frame)
		{
			const bool Add = Frame >= 240 && Frame <= 600 && Frame % 120 == 0;
			const int BeforeCount = SmokeViewerValidationAccess::Count(Viewer);
			const double BeforeTime = SmokeViewerValidationAccess::Time(Viewer);
			if (Add) SmokeViewerValidationAccess::AddBody(Viewer);
			Viewer.UpdateSimulator(1.0f / 60);
			if (Add)
			{
				const int Expected = BeforeCount < SceneWorld::MaxSmokeRigidBodies ? BeforeCount + 1 : BeforeCount;
				if (SmokeViewerValidationAccess::Count(Viewer) != Expected || SmokeViewerValidationAccess::Time(Viewer) <= BeforeTime)
					{ Result = 1; break; }
				std::printf("Add button: %d -> %d bodies; smoke time retained\n", BeforeCount, Expected);
				if (!Body) { Body = SmokeViewerValidationAccess::Body(Viewer); if (!Body) return 1; Start = Body->X; }
			}
			if (Frame % 120 == 0 || (Animation && Frame >= 240 && Frame % 6 == 0))
			{
				std::vector<uint32_t> Pixels; uint32_t Width, Height;
				if (!Graphics->CaptureFrame(&Pixels, &Width, &Height)) { Result = 1; break; }
				char Path[128]; std::snprintf(Path, sizeof(Path), "TestOutput/smoke_viewer_%04d.ppm", Frame);
				std::ofstream File(Path, std::ios::binary);
				File << "P6\n" << Width << " " << Height << "\n255\n";
				for (uint32_t Pixel : Pixels) File.write((const char*)&Pixel, 3);
				if (!File) { Result = 1; break; }
				if (Frame % 120 == 0) std::printf("Captured actual viewer: %.1fs\n", Frame / 60.0f);
			}
		}
		std::printf("Coupled viewer: body displacement %.4f m; mean update/capture %.2f ms\n", Body ? (Body->X - Start).Length() : 0,
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count() / 721);
		if (!Body || (Body->X - Start).Length() < 0.05f || !SmokeViewerValidationAccess::CheckPauseAndRestart(Viewer)) Result = 1;
		// Switching away must remove the volume; loading again must reset it.
		if (!Viewer.LoadScene("Contents/Scenes/01_demo.json")) Result = 1;
		Graphics->Render();
		if (!Viewer.LoadScene("Contents/Scenes/12_3d_smoke.json")) Result = 1;
		Viewer.UpdateSimulator(1.0f / 60); Graphics->Render();
		if (!SmokeViewerValidationAccess::CheckPausedAddAndLimit(Viewer)) Result = 1;
		if (!Viewer.LoadScene("Contents/Scenes/12_3d_smoke.json") || !SmokeViewerValidationAccess::Empty(Viewer)) Result = 1;
	}
	DestroyWindow(Window);
	std::printf("Smoke viewer integration: %s\n", Result == 0 ? "passed" : "FAILED");
	return Result;
}
#endif

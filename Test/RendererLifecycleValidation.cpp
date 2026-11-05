#if defined(RIEMANN_RENDERER_LIFECYCLE_VALIDATION)
#define NOMINMAX
#include <windows.h>
#include <cstdio>
#include <memory>
#include "../Renderer/RiemannRenderer.h"

// Hidden-window integration check for mesh outlines, resizing, volume
// removal, reset, reupload, and invalid updates. The DX11 volume path is
// selected through SupportsDensityVolume; other platforms retain isosurfaces.

int main()
{
	using namespace Riemann;
	HWND Window = CreateWindowExA(0, "STATIC", "Riemann renderer validation",
		WS_OVERLAPPEDWINDOW, 0, 0, 640, 480, nullptr, nullptr, GetModuleHandle(nullptr), nullptr);
	if (!Window) return 1;
	bool Passed = true;
	{
		std::unique_ptr<Renderer> Graphics(Renderer::CreateDX11Renderer(Window, "Renderer/"));
		if (!Graphics) { DestroyWindow(Window); return 1; }
		Passed = Graphics->SupportsDensityVolume();
		CameraDesc Camera;
		Camera.Eye = Vector3(0, 0, -4); Camera.At = Vector3::Zero();
		Graphics->SetCamera(Camera);
		RenderMeshDesc Mesh;
		Mesh.Id = "occluder";
		Mesh.Vertices = { Vertex1(Vector3(-0.5f, -0.5f, 0), Vector3(0, 0, -1)),
			Vertex1(Vector3(0, 0.5f, 0), Vector3(0, 0, -1)),
			Vertex1(Vector3(0.5f, -0.5f, 0), Vector3(0, 0, -1)) };
		Mesh.Indices = { 0, 1, 2 };
		Passed = Graphics->AddMesh(Mesh) && Passed;
		Graphics->SetMeshOutline(Mesh.Id.c_str(), true, Vector4(1, 1, 0, 1), 0.03f);
		DensityVolumeDesc Volume;
		Volume.Resolution = Vector3i(8, 8, 8); Volume.Density.assign(512, 2 * Volume.AmbientDensity);
		Passed = Graphics->UpdateDensityVolume(Volume) && Passed;
		Graphics->Render();
		SetWindowPos(Window, nullptr, 0, 0, 960, 540, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
		Graphics->Render();
		Graphics->ClearDensityVolume(); Graphics->Render();
		Passed = Graphics->Reset() && Passed;
		Volume.Resolution = Vector3i(16, 8, 8); Volume.Density.assign(1024, 0.5f * Volume.AmbientDensity);
		Passed = Graphics->UpdateDensityVolume(Volume) && Passed;
		Graphics->Render();
		Volume.Density.clear();
		Passed = !Graphics->UpdateDensityVolume(Volume) && Passed;
		Graphics->Render();
	}
	DestroyWindow(Window);
	std::printf("Hidden-window renderer lifecycle: %s\n", Passed ? "passed" : "FAILED");
	return Passed ? 0 : 1;
}

#endif // RIEMANN_RENDERER_LIFECYCLE_VALIDATION

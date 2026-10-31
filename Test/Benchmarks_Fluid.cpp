// Standalone benchmark entry points for the Fluid category.
// Each CMake target defines exactly one RIEMANN_BENCHMARK_* selector.
// The normal Test target compiles this file without a selector.

#if defined(RIEMANN_BENCHMARK_FLUID)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"

// Reproduce with:
// cmake -S Benchmarks/Fluid -B Build/CMakeBuild/FluidBenchmark -A x64
// cmake --build Build/CMakeBuild/FluidBenchmark --config Release
// Build/CMakeBuild/FluidBenchmark/Release/FluidBenchmark.exe
// Reference run: Win64, i9-10900F, MSVC 19.44 Release, CPU backend, demo sizes,
// five warmup frames and 30 measured frames at 1/60 s. Rigid coupling and GPU
// drawing are excluded. Baseline -> optimized median step time, in milliseconds:
// water grid 38.738 -> 9.820; air grid 59.881 -> 9.666;
// water SPH (11,661 particles) 176.231 -> 6.595;
// air SPH (12,800 particles) 633.176 -> 20.079.
// SPH density-field generation: water 20.656 -> 0.834; air 23.740 -> 1.449.
// Water maximum divergence: 0.331112 -> 0.007579. Relative grid mass error
// remained 2.15e-5 (water) and 1.08e-4 (air); particle mass was unchanged.
// Air pending simulation time stayed zero. Resolution, timestep, acoustic CFL,
// and particle count were held fixed. Pressure trajectories differ because the
// linear system is solved more accurately. Wall times vary with system load.

using namespace Riemann;
using Clock = std::chrono::steady_clock;

template<class Simulation>
static bool Measure(const char* Name, Simulation& Solver)
{
    const float InitialMass = Solver.GetTotalMass();
    std::vector<double> Times;
    for (int Frame = 0; Frame < 35; ++Frame)
    {
        const auto Start = Clock::now();
        Solver.Step(1.0f / 60.0f);
        const double Milliseconds = std::chrono::duration<double, std::milli>(Clock::now() - Start).count();
        if (Frame >= 5) Times.push_back(Milliseconds);
    }
    std::sort(Times.begin(), Times.end());
    std::vector<float> Density;
    const auto Start = Clock::now();
    for (int Repeat = 0; Repeat < 10; ++Repeat) Solver.BuildDensityField(&Density);
    const double FieldTime = std::chrono::duration<double, std::milli>(Clock::now() - Start).count() / 10;
    const float MassError = std::fabs(Solver.GetTotalMass() / InitialMass - 1.0f);
    bool Valid = Solver.IsInitialized() && MassError < 0.001f;
    for (float Value : Density) Valid = Valid && std::isfinite(Value) && Value >= 0.0f;
    std::printf("%s: step_median_ms=%.3f p95_ms=%.3f field_ms=%.3f mass_relative_error=%.8f valid=%d\n",
        Name, Times[Times.size() / 2], Times[Times.size() * 95 / 100], FieldTime, MassError, Valid ? 1 : 0);
    return Valid;
}

int main()
{
    bool Valid = true;
    for (int Method = 0; Method < 2; ++Method)
    {
        FluidSimulationSettings Water;
        Water.Method = (FluidSimulationMethod)Method;
        Water.BoundsMin = Vector3(-5, 0, -3); Water.BoundsMax = Vector3(5, 6, 3);
        Water.Resolution = Vector3i(32, 24, 20); Water.ParticleSpacing = 0.25f;
        Water.SmoothingRadius = 0.55f; Water.PressureIterations = 48;
        Water.DynamicViscosity = 0.02f; Water.MaxTimeStep = 1.0f / 120.0f;
        FluidSimulation Fluid(Water);
        Fluid.AddFluidBox(Vector3(-4.9f, 0.1f, -2.9f), Vector3(4.9f, 3.35f, 2.9f), Vector3(0.12f, 0, 0));
        Valid = Measure(Method == 0 ? "water_grid" : "water_sph", Fluid) && Valid;
        std::printf("  particles=%zu divergence=%.6f\n", Fluid.GetParticles().size(), Fluid.GetMaxDivergence());

        AerodynamicsSimulationSettings Air;
        Air.Method = (FluidSimulationMethod)Method;
        Air.BoundsMin = Vector3(-4, -2.5f, -2.5f); Air.BoundsMax = Vector3(4, 2.5f, 2.5f);
        Air.Resolution = Vector3i(32, 20, 20); Air.ParticleSpacing = 0.25f;
        Air.SmoothingRadius = 0.5f; Air.SpeedOfSound = 8.0f;
        Air.DynamicViscosity = 0.005f; Air.MaxTimeStep = 1.0f / 180.0f; Air.MaxSubsteps = 32;
        AerodynamicsSimulation Gas(Air);
        Gas.SetDensityBox(Vector3(-1.75f, -2, -2), Vector3(-0.75f, 2, 2), 1.53125f);
        Gas.SetDensityBox(Vector3(0.75f, -2, -2), Vector3(1.75f, 2, 2), 1.53125f);
        Gas.SetDensityBox(Vector3(-0.5f, -2, -2), Vector3(0.5f, 2, 2), 0.6125f);
        Valid = Measure(Method == 0 ? "air_grid" : "air_sph", Gas) && Valid;
        std::printf("  particles=%zu pending_seconds=%.9f\n", Gas.GetParticles().size(), Gas.GetPendingTime());
        Valid = Valid && Gas.GetPendingTime() < 1.0e-6;
    }
    return Valid ? 0 : 1;
}

#endif // RIEMANN_BENCHMARK_FLUID

#if defined(RIEMANN_BENCHMARK_VOLUME_VALIDATION)
#if defined(RIEMANN_SMOKE_PREVIEW)
#include "../Renderer/SmokeVolume.h"
#include "../Viewer/SceneWorld.h"
#include <chrono>
#endif
#include <cstdio>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <d3d11sdklayers.h>
#include "../Renderer/DX11VolumeRenderer.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"

// Reproduce with:
// cmake --build Build/CMakeBuild/FluidBenchmark --config Release --target VolumeRenderValidation
// Build/CMakeBuild/FluidBenchmark/Release/VolumeRenderValidation.exe 1920 1080
// The offscreen checks cover invisible ambient air, analytic transmittance,
// step-size independence, depth clipping, inside-camera rays, invalid uploads,
// clearing, resize, and D3D11 debug-layer warnings. The density snapshots are
// written to TestOutput/air_volume_*.ppm. The occlusion image uses an analytic
// sphere and is not a capture of the interactive viewer.
// Reference GPU run: RTX 4060, evolved demo, 1920 x 1080, 0.541 ms per volume
// pass averaged over 20 draws using GPU timestamps. Cost depends on screen
// coverage, density, and sample interval.

using namespace Riemann;
using Microsoft::WRL::ComPtr;

static void Check(bool Passed, const char* Message)
{
	if (!Passed) { std::fprintf(stderr, "FAIL: %s\n", Message); std::exit(1); }
}

int main(int Argc, char** Argv)
{
	setvbuf(stdout, nullptr, _IONBF, 0);
	const unsigned Width = Argc > 1 ? (unsigned)std::strtoul(Argv[1], nullptr, 10) : 641;
	const unsigned Height = Argc > 2 ? (unsigned)std::strtoul(Argv[2], nullptr, 10) : 481;
	Check(Width >= 64 && Width <= 4096 && Height >= 64 && Height <= 4096, "valid render dimensions");
	ComPtr<ID3D11Device> Device;
	ComPtr<ID3D11DeviceContext> Context;
	D3D_FEATURE_LEVEL Level;
	HRESULT Status = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
		D3D11_CREATE_DEVICE_DEBUG, nullptr, 0, D3D11_SDK_VERSION, &Device, &Level, &Context);
	if (FAILED(Status)) Status = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
		0, nullptr, 0, D3D11_SDK_VERSION, &Device, &Level, &Context);
	Check(SUCCEEDED(Status), "create D3D11 device");
	ComPtr<IDXGIDevice> DXGI;
	ComPtr<IDXGIAdapter> Adapter;
	Device.As(&DXGI); DXGI->GetAdapter(&Adapter);
	DXGI_ADAPTER_DESC AdapterInfo = {};
	Adapter->GetDesc(&AdapterInfo);
	std::printf("GPU: %ls\n", AdapterInfo.Description);
	ComPtr<ID3D11InfoQueue> Debug;
	Device.As(&Debug);
	DX11VolumeRenderer Renderer;
	Check(Renderer.Initialize(Device.Get(), L"Renderer/dx11_shader.hlsl"), "compile volume shaders and create pipeline");

	D3D11_TEXTURE2D_DESC Texture = {};
	Texture.Width = Width; Texture.Height = Height; Texture.MipLevels = Texture.ArraySize = 1;
	Texture.Format = DXGI_FORMAT_R32G32B32A32_FLOAT; Texture.SampleDesc.Count = 1;
	Texture.Usage = D3D11_USAGE_DEFAULT; Texture.BindFlags = D3D11_BIND_RENDER_TARGET;
	ComPtr<ID3D11Texture2D> Target, Staging, DepthTexture;
	ComPtr<ID3D11RenderTargetView> TargetView;
	ComPtr<ID3D11ShaderResourceView> DepthView;
	ComPtr<ID3D11DepthStencilView> DepthStencilView;
	Check(SUCCEEDED(Device->CreateTexture2D(&Texture, nullptr, &Target)), "create render target");
	Check(SUCCEEDED(Device->CreateRenderTargetView(Target.Get(), nullptr, &TargetView)), "create target view");
	Texture.Usage = D3D11_USAGE_STAGING; Texture.BindFlags = 0; Texture.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	Check(SUCCEEDED(Device->CreateTexture2D(&Texture, nullptr, &Staging)), "create readback");
	Texture.Usage = D3D11_USAGE_DEFAULT; Texture.CPUAccessFlags = 0;
	Texture.Format = DXGI_FORMAT_R24G8_TYPELESS;
	Texture.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_DEPTH_STENCIL;
	Check(SUCCEEDED(Device->CreateTexture2D(&Texture, nullptr, &DepthTexture)), "create depth input");
	D3D11_SHADER_RESOURCE_VIEW_DESC SRV = {};
	SRV.Format = DXGI_FORMAT_R24_UNORM_X8_TYPELESS; SRV.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	SRV.Texture2D.MipLevels = 1;
	Check(SUCCEEDED(Device->CreateShaderResourceView(DepthTexture.Get(), &SRV, &DepthView)), "create depth view");
	D3D11_DEPTH_STENCIL_VIEW_DESC DSV = {};
	DSV.Format = DXGI_FORMAT_D24_UNORM_S8_UINT; DSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
	Check(SUCCEEDED(Device->CreateDepthStencilView(DepthTexture.Get(), &DSV, &DepthStencilView)), "create depth stencil view");
	D3D11_VIEWPORT Viewport = { 0, 0, (float)Width, (float)Height, 0, 1 };
	Context->RSSetViewports(1, &Viewport);
	CameraDesc Camera; Camera.Eye = Vector3(0, 0, -4); Camera.At = Vector3::Zero();
	Camera.FovY = 0.9f; Camera.NearPlane = 0.1f; Camera.FarPlane = 40;
	DirectionalLightDesc Light;
	std::vector<float> Depth(Width * Height, 1.0f);
	std::vector<Vector4> Pixels(Width * Height);
	std::vector<Vector4> Background(Width * Height, Vector4(0, 0, 0, 0));
	std::vector<uint32_t> PackedDepth(Width * Height);
	const size_t Center = Width / 2 + Width * (Height / 2);
	bool LinearOutput = false;
	auto DepthFromDistance = [&](float Z) {
		return Camera.FarPlane / (Camera.FarPlane - Camera.NearPlane)
			- Camera.FarPlane * Camera.NearPlane / ((Camera.FarPlane - Camera.NearPlane) * Z);
	};
	auto Draw = [&](const char* ImagePath) {
		for (size_t I = 0; I < Depth.size(); ++I) PackedDepth[I] = (uint32_t)(Depth[I] * 16777215.0f);
		Context->UpdateSubresource(DepthTexture.Get(), 0, nullptr, PackedDepth.data(), Width * sizeof(uint32_t), 0);
		Context->UpdateSubresource(Target.Get(), 0, nullptr, Background.data(), Width * sizeof(Vector4), 0);
		ID3D11RenderTargetView* RTV = TargetView.Get();
		Context->OMSetRenderTargets(1, &RTV, DepthStencilView.Get());
		Renderer.Render(Context.Get(), TargetView.Get(), DepthView.Get(), Camera, Light, Width, Height);
		Context->CopyResource(Staging.Get(), Target.Get());
		D3D11_MAPPED_SUBRESOURCE Map = {};
		Check(SUCCEEDED(Context->Map(Staging.Get(), 0, D3D11_MAP_READ, 0, &Map)), "read render result");
		for (unsigned Y = 0; Y < Height; ++Y)
		{
			const Vector4* Row = (const Vector4*)((const char*)Map.pData + Y * Map.RowPitch);
			std::copy(Row, Row + Width, Pixels.begin() + Y * Width);
		}
		Context->Unmap(Staging.Get(), 0);
		for (const Vector4& Pixel : Pixels)
			for (int C = 0; C < 4; ++C) Check(std::isfinite(Pixel[C]), "finite pixel output");
		if (ImagePath)
		{
			std::ofstream File(ImagePath, std::ios::binary);
			File << "P6\n" << Width << " " << Height << "\n255\n";
			for (const Vector4& Pixel : Pixels)
			{
				const float Background[3] = { 0.035f, 0.045f, 0.065f };
				unsigned char RGB[3];
				for (int C = 0; C < 3; ++C)
					RGB[C] = (unsigned char)(255 * std::pow(std::min(1.0f, std::max(0.0f,
						Pixel[C] + (1 - Pixel.w) * Background[C])), LinearOutput ? 1.0f : 1.0f / 2.2f));
				File.write((const char*)RGB, 3);
			}
			Check(!!File, "write volume preview");
		}
	};
	DensityVolumeDesc Volume;
	Volume.Resolution = Vector3i(16, 16, 16);
	Volume.Density.assign(4096, Volume.AmbientDensity);
	Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload ambient volume");
	Draw(nullptr);
	for (const Vector4& Pixel : Pixels) Check(Pixel.w == 0, "ambient air is invisible");
	Volume.Density.assign(4096, 2 * Volume.AmbientDensity);
	Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload uniform volume");
	Draw(nullptr);
	const float FullAlpha = Pixels[Center].w;
	Check(std::fabs(FullAlpha - (1 - std::exp(-4.0f))) < 1.0e-4f, "Beer-Lambert analytic transmittance");
	Volume.StepScale = 0.25f;
	Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload fine-step volume");
	Draw(nullptr);
	Check(std::fabs(Pixels[Center].w - FullAlpha) < 1.0e-4f, "step-size independent opacity");
	std::fill(Depth.begin(), Depth.end(), DepthFromDistance(2));
	Draw(nullptr); Check(Pixels[Center].w == 0, "foreground occludes volume");
	std::fill(Depth.begin(), Depth.end(), DepthFromDistance(4));
	Draw(nullptr);
	Check(std::fabs(Pixels[Center].w - (1 - std::exp(-2.0f))) < 1.0e-4f, "partial depth clipping");
	std::fill(Depth.begin(), Depth.end(), 1.0f);
	Camera.Eye = Vector3::Zero(); Camera.At = Vector3::UnitZ();
	Draw(nullptr);
	Check(std::fabs(Pixels[Center].w - (1 - std::exp(-1.8f))) < 1.0e-4f, "camera inside volume");
	Renderer.Clear(); Draw(nullptr); Check(Pixels[Center].w == 0, "clear removes previous volume");
	Volume.Density.pop_back(); Check(!Renderer.Update(Device.Get(), Context.Get(), Volume), "reject invalid dimensions");

#if defined(RIEMANN_SMOKE_PREVIEW)
	if (Argc > 3 && std::string(Argv[3]) == "smoke")
	{
		SceneWorld World;
		std::string Error;
		Check(World.LoadFromFile("Contents/Scenes/12_3d_smoke.json", &Error), "load smoke scene");
		const SceneFieldSimulationDesc& Field = World.GetFieldSimulation();
		SmokeSimulation Smoke;
		SmokeSimulationSettings SmokeSettings = Field.Smoke;
		const std::string Backend = Argc > 4 ? Argv[4] : "compute";
		SmokeSettings.Backend = Backend == "cpu" ? SmokeBackend::CPU : Backend == "cuda" ? SmokeBackend::CUDA : SmokeBackend::ComputeShader;
		SmokeSettings.AllowBackendFallback = false;
		if (Argc > 5 && std::string(Argv[5]) == "interactive") SmokeSettings.Resolution = Vector3i(48,80,48);
		if (Argc > 5 && std::string(Argv[5]) == "detailed") SmokeSettings.Resolution = Vector3i(112,184,112);
		Check(Smoke.Initialize(SmokeSettings), "initialize requested smoke backend without fallback");
		std::printf("Smoke backend: %s; grid: %dx%dx%d\n", Backend.c_str(), SmokeSettings.Resolution.x, SmokeSettings.Resolution.y, SmokeSettings.Resolution.z);
		const int Frames = Argc > 6 ? std::max(120, std::min(1800, std::atoi(Argv[6]))) : 600;
		Camera = World.GetCamera(); Light = World.GetLight();
		LinearOutput = true;
		double Milliseconds = 0;
		for (int Frame = 1; Frame <= Frames; ++Frame)
		{
			const auto Start = std::chrono::steady_clock::now();
			Smoke.Step(1.0f / 60);
			Milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
			if (Frame % 120 == 0)
			{
				BuildSmokeDensityVolume(Smoke, &Volume, Field.SmokeExtinction, Field.SmokeColor);
				Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload simulated smoke");
				const std::string Path = "TestOutput/smoke_" + Backend + "_" + std::to_string(Frame / 60) + "s.ppm";
				Draw(Path.c_str());
				std::printf("Smoke %.1fs: %.2f ms/step, divergence %.5f, pending %.6fs\n", Smoke.GetTime(),
					Milliseconds / Frame, Smoke.GetDivergenceRMS(), Smoke.GetPendingTime());
			}
		}
		Camera.Eye = Vector3(-8, 5.5f, 9);
		Draw("TestOutput/smoke_side.ppm");
		// Analytic depth of the scene's deflector verifies smoke on both sides.
		Camera = World.GetCamera();
		const Vector3 Forward = (Camera.At - Camera.Eye).Unit(), Right = Camera.Up.Cross(Forward).Unit();
		const Vector3 Up = Forward.Cross(Right);
		const SmokeObstacle& Obstacle = Field.Smoke.Obstacles.front();
		for (unsigned Y = 0; Y < Height; ++Y)
		for (unsigned X = 0; X < Width; ++X)
		{
			const float U = (2.0f * (X + 0.5f) / Width - 1) * std::tan(Camera.FovY * 0.5f) * Width / Height;
			const float V = (1 - 2.0f * (Y + 0.5f) / Height) * std::tan(Camera.FovY * 0.5f);
			const Vector3 Ray = (Forward + Right * U + Up * V).Unit(), Offset = Camera.Eye - Obstacle.Center;
			const float B = Offset.Dot(Ray), D = B * B - Offset.SquareLength() + Obstacle.Radius * Obstacle.Radius;
			if (D < 0) continue;
			const float T = -B - std::sqrt(D);
			if (T <= 0) continue;
			const Vector3 Normal = (Camera.Eye + Ray * T - Obstacle.Center).Unit();
			const float Shade = 0.22f + 0.78f * std::max(0.0f, -Normal.Dot(Light.Direction.Unit()));
			Background[X + Y * Width] = Vector4(0.25f * Shade, 0.32f * Shade, 0.41f * Shade, 1);
			Depth[X + Y * Width] = DepthFromDistance(T * Ray.Dot(Forward));
		}
		Draw("TestOutput/smoke_occlusion.ppm");
		// Smoke has its own density interpretation, independent of ambient air.
		Volume.BoundsMin = Vector3(-1); Volume.BoundsMax = Vector3(1);
		Volume.Resolution = Vector3i(8, 8, 8); Volume.Density.assign(512, 0.0f);
		Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload empty smoke");
		std::fill(Background.begin(), Background.end(), Vector4(0, 0, 0, 0));
		std::fill(Depth.begin(), Depth.end(), 1.0f);
		Camera.Eye = Vector3(0, 0, -4); Camera.At = Vector3::Zero();
		Draw(nullptr);
		for (const Vector4& Pixel : Pixels) Check(Pixel.w == 0, "empty smoke is transparent");
		Volume.Extinction = 1; Volume.Density.assign(512, 0.5f);
		Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload uniform smoke");
		Draw(nullptr);
		Check(std::fabs(Pixels[Center].w - (1 - std::exp(-1.0f))) < 0.002f, "smoke analytic opacity");
		BuildSmokeDensityVolume(Smoke, &Volume, Field.SmokeExtinction, Field.SmokeColor);
		Renderer.Update(Device.Get(), Context.Get(), Volume); Camera = World.GetCamera();
	}
	else
#endif
	{
	// Render actual demo density, with a synthetic opaque center for depth QA.
	AerodynamicsSimulationSettings Settings;
	Settings.BoundsMin = Vector3(-4, -2.5f, -2.5f); Settings.BoundsMax = Vector3(4, 2.5f, 2.5f);
	Settings.Resolution = Vector3i(32, 20, 20); Settings.SpeedOfSound = 8;
	Settings.DynamicViscosity = 0.005f; Settings.MaxTimeStep = 1.0f / 180;
	AerodynamicsSimulation Gas(Settings);
	Gas.SetDensityBox(Vector3(-1.75f, -2, -2), Vector3(-0.75f, 2, 2), 1.53125f);
	Gas.SetDensityBox(Vector3(0.75f, -2, -2), Vector3(1.75f, 2, 2), 1.53125f);
	Gas.SetDensityBox(Vector3(-0.5f, -2, -2), Vector3(0.5f, 2, 2), 0.6125f);
	Volume.BoundsMin = Settings.BoundsMin; Volume.BoundsMax = Settings.BoundsMax;
	Volume.Resolution = Settings.Resolution; Volume.StepScale = 0.75f;
	Camera.Eye = Vector3(7.5f, 3.5f, -9.0f); Camera.At = Vector3::Zero(); Camera.FovY = 0.9f;
	Gas.BuildDensityField(&Volume.Density);
	Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "resize volume to demo");
	Draw("TestOutput/air_volume_initial.ppm");
	for (int Frame = 0; Frame < 45; ++Frame) Gas.Step(1.0f / 60.0f);
	Gas.BuildDensityField(&Volume.Density);
	Check(Renderer.Update(Device.Get(), Context.Get(), Volume), "upload evolved gas");
	Draw("TestOutput/air_volume_evolved.ppm");
	const Vector3 Forward = (Camera.At - Camera.Eye).Unit();
	const Vector3 Right = Camera.Up.Cross(Forward).Unit();
	const Vector3 Up = Forward.Cross(Right);
	for (unsigned Y = 0; Y < Height; ++Y)
	for (unsigned X = 0; X < Width; ++X)
	{
		const float U = (2.0f * (X + 0.5f) / Width - 1) * std::tan(Camera.FovY * 0.5f) * Width / Height;
		const float V = (1 - 2.0f * (Y + 0.5f) / Height) * std::tan(Camera.FovY * 0.5f);
		const Vector3 Ray = (Forward + Right * U + Up * V).Unit();
		const Vector3 SphereCenter(0.44f, 0, 0.12f);
		const Vector3 Offset = Camera.Eye - SphereCenter;
		const float B = Offset.Dot(Ray), C = Offset.SquareLength() - 0.43f * 0.43f;
		const float D = B * B - C;
		if (D < 0) continue;
		const float T = -B - std::sqrt(D);
		if (T <= 0) continue;
		const Vector3 Normal = (Camera.Eye + Ray * T - SphereCenter).Unit();
		const float Shading = 0.25f + 0.75f * std::max(0.0f, -Normal.Dot(Light.Direction.Unit()));
		Background[X + Y * Width] = Vector4(0.36f * Shading, 0.94f * Shading, 0.52f * Shading, 1);
		Depth[X + Y * Width] = DepthFromDistance(T * Ray.Dot(Forward));
	}
	Draw("TestOutput/air_volume_occlusion.ppm");
	}

	D3D11_QUERY_DESC QueryDesc = { D3D11_QUERY_TIMESTAMP_DISJOINT, 0 };
	ComPtr<ID3D11Query> Disjoint, Begin, End;
	Device->CreateQuery(&QueryDesc, &Disjoint); QueryDesc.Query = D3D11_QUERY_TIMESTAMP;
	Device->CreateQuery(&QueryDesc, &Begin); Device->CreateQuery(&QueryDesc, &End);
	Check(Disjoint && Begin && End, "create GPU timing queries");
	Context->Begin(Disjoint.Get()); Context->End(Begin.Get());
	for (int Frame = 0; Frame < 20; ++Frame)
		Renderer.Render(Context.Get(), TargetView.Get(), DepthView.Get(), Camera, Light, Width, Height);
	Context->End(End.Get()); Context->End(Disjoint.Get()); Context->Flush();
	D3D11_QUERY_DATA_TIMESTAMP_DISJOINT Timing = {};
	while (Context->GetData(Disjoint.Get(), &Timing, sizeof(Timing), 0) == S_FALSE) Sleep(1);
	UINT64 StartTime = 0, EndTime = 0;
	Check(Context->GetData(Begin.Get(), &StartTime, sizeof(StartTime), 0) == S_OK, "read GPU start");
	Check(Context->GetData(End.Get(), &EndTime, sizeof(EndTime), 0) == S_OK, "read GPU end");
	Check(!Timing.Disjoint, "stable GPU clock");
	std::printf("Volume GPU time at %ux%u: %.3f ms\n", Width, Height,
		1000.0 * (EndTime - StartTime) / Timing.Frequency / 20);
	if (Debug)
	{
		for (UINT64 I = 0; I < Debug->GetNumStoredMessages(); ++I)
		{
			SIZE_T Bytes = 0; Debug->GetMessage(I, nullptr, &Bytes);
			std::vector<char> Storage(Bytes); auto* Message = (D3D11_MESSAGE*)Storage.data();
			Debug->GetMessage(I, Message, &Bytes);
			if (Message->Severity <= D3D11_MESSAGE_SEVERITY_WARNING)
			{
				std::fprintf(stderr, "D3D11: %s\n", Message->pDescription);
				Check(false, "D3D11 debug layer warning/error");
			}
		}
	}
	std::printf("Volume rendering validation passed (debug layer: %s).\n", Debug ? "enabled" : "unavailable");
	return 0;
}

#endif // RIEMANN_BENCHMARK_VOLUME_VALIDATION

#if defined(RIEMANN_BENCHMARK_RENDERER_LIFECYCLE)
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

#endif // RIEMANN_BENCHMARK_RENDERER_LIFECYCLE

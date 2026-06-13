#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <windows.h>
#include <shellapi.h>

#include "RenderThread.h"
#include "Viewer.h"
#include "resource.h"
#include "../Renderer/Renderer.h"

HINSTANCE g_hInst = nullptr;
HWND g_hWnd = nullptr;
Riemann::Renderer* g_Renderer = nullptr;
Riemann::RenderThread* g_RenderThread = nullptr;
Riemann::WorldViewer* g_Viewer = nullptr;

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

namespace
{
	bool FileExists(const std::string& fileName)
	{
		std::ifstream file(fileName.c_str(), std::ios::in | std::ios::binary);
		return !!file;
	}

	std::string WideToUtf8(const wchar_t* text)
	{
		if (text == nullptr || text[0] == L'\0')
		{
			return std::string();
		}

		const int needed = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
		if (needed <= 1)
		{
			return std::string();
		}

		std::string result;
		result.resize(needed - 1);
		WideCharToMultiByte(CP_UTF8, 0, text, -1, &result[0], needed, nullptr, nullptr);
		return result;
	}

	std::string GetCommandLineScene()
	{
		int argc = 0;
		LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
		if (argv == nullptr)
		{
			return std::string();
		}

		std::string scene;
		if (argc > 1)
		{
			scene = WideToUtf8(argv[1]);
		}
		LocalFree(argv);
		return scene;
	}

	std::string ResolveShaderPath()
	{
		const char* candidates[] =
		{
			"Renderer/",
			"../Renderer/",
			"../../Renderer/",
			"../../../Renderer/",
			"../../../../Renderer/",
		};

		for (const char* candidate : candidates)
		{
			if (FileExists(std::string(candidate) + "simple.fxh"))
			{
				return candidate;
			}
		}

		return "../Renderer/";
	}
}

HRESULT InitWindow(HINSTANCE hInstance, int nCmdShow)
{
	WNDCLASSEXW wcex = {};
	wcex.cbSize = sizeof(WNDCLASSEXW);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIconW(hInstance, MAKEINTRESOURCEW(IDI_APP));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszClassName = L"RiemannViewerWindowClass";
	wcex.hIconSm = LoadIconW(wcex.hInstance, MAKEINTRESOURCEW(IDI_APP));
	if (!RegisterClassExW(&wcex))
	{
		return E_FAIL;
	}

	g_hInst = hInstance;
	RECT rc = { 0, 0, 1280, 800 };
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
	g_hWnd = CreateWindowW(
		L"RiemannViewerWindowClass",
		L"Riemann Viewer",
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		rc.right - rc.left,
		rc.bottom - rc.top,
		nullptr,
		nullptr,
		hInstance,
		nullptr);
	if (!g_hWnd)
	{
		return E_FAIL;
	}

	ShowWindow(g_hWnd, nCmdShow);
	return S_OK;
}

int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	if (FAILED(InitWindow(hInstance, nCmdShow)))
	{
		return 0;
	}

	const std::string shaderPath = ResolveShaderPath();
	g_Renderer = Riemann::Renderer::CreateDX11Renderer(g_hWnd, shaderPath.c_str());
	if (g_Renderer == nullptr)
	{
		return 0;
	}

	g_RenderThread = new Riemann::RenderThread(g_Renderer);
	g_RenderThread->Start();

	const std::string scene = GetCommandLineScene();
	g_Viewer = new Riemann::WorldViewer(g_RenderThread, scene.empty() ? "demo.json" : scene);

	auto last = std::chrono::steady_clock::now();

	MSG msg = { 0 };
	while (WM_QUIT != msg.message)
	{
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			const auto curr = std::chrono::steady_clock::now();
			const std::chrono::duration<double> elapsedSeconds = curr - last;
			if (elapsedSeconds < std::chrono::milliseconds(16))
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			else
			{
				if (g_Viewer)
				{
					g_Viewer->UpdateSimulator(0.016f);
				}
				last = curr;
			}
		}
	}

	return static_cast<int>(msg.wParam);
}

#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#endif
#ifndef GET_Y_LPARAM
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))
#endif

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		EndPaint(hWnd, &ps);
		break;

	case WM_DESTROY:
		delete g_Viewer;
		g_Viewer = nullptr;

		delete g_RenderThread;
		g_RenderThread = nullptr;

		delete g_Renderer;
		g_Renderer = nullptr;

		PostQuitMessage(0);
		break;

	case WM_CHAR:
		if (g_Viewer)
		{
			g_Viewer->KeyboardMsg(static_cast<char>(wParam));
		}
		break;

	case WM_MOUSEMOVE:
		if (g_Viewer)
		{
			g_Viewer->MouseMsg(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (MK_LBUTTON & wParam) != 0);
		}
		break;

	case WM_MOUSEWHEEL:
		if (g_Viewer)
		{
			g_Viewer->MouseWheel(GET_WHEEL_DELTA_WPARAM(wParam), (wParam & MK_CONTROL) != 0);
		}
		break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

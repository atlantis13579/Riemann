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
#include "../RiemannRenderer/RiemannRenderer.h"

HINSTANCE g_hInst = nullptr;
HWND g_hWnd = nullptr;
Riemann::Renderer* g_Renderer = nullptr;
Riemann::RenderThread* g_RenderThread = nullptr;
Riemann::WorldViewer* g_Viewer = nullptr;

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

namespace
{
	const unsigned char kImguiMouseLeft = 0x01;
	const unsigned char kImguiMouseRight = 0x02;

	int g_MouseX = 0;
	int g_MouseY = 0;
	unsigned char g_ImguiMouseButtons = 0;
	bool g_ImguiMouseCapture = false;

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
			"RiemannRenderer/",
			"../RiemannRenderer/",
			"../../RiemannRenderer/",
			"../../../RiemannRenderer/",
			"../../../../RiemannRenderer/",
		};

		for (const char* candidate : candidates)
		{
			if (FileExists(std::string(candidate) + "dx11_shader.hlsl"))
			{
				return candidate;
			}
		}

		return "../RiemannRenderer/";
	}

	bool IsMouseOverImgui(int x, int y)
	{
		if (g_Viewer == nullptr || g_hWnd == nullptr)
		{
			return false;
		}

		RECT rc = {};
		GetClientRect(g_hWnd, &rc);
		return g_Viewer->IsImguiPanelHovered(x, y, rc.right - rc.left, rc.bottom - rc.top);
	}

	void SubmitImguiInput(int wheel)
	{
		if (g_Renderer == nullptr)
		{
			return;
		}

		Riemann::ImguiInputState input;
		input.MouseX = g_MouseX;
		input.MouseY = g_MouseY;
		input.MouseButtons = g_ImguiMouseButtons;
		input.MouseWheel = wheel;
		g_Renderer->UpdateImguiInput(input);
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
	const DWORD windowStyle = WS_OVERLAPPEDWINDOW;
	RECT rc = { 0, 0, 1280, 800 };
	AdjustWindowRect(&rc, windowStyle, FALSE);
	g_hWnd = CreateWindowW(
		L"RiemannViewerWindowClass",
		L"Riemann Viewer",
		windowStyle,
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
	SubmitImguiInput(0);

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
				const float fixedDt = 0.016f;
				if (g_Viewer)
				{
					g_Viewer->UpdateCameraMovement(fixedDt);
					g_Viewer->UpdateSimulator(fixedDt);
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
		if (g_RenderThread)
		{
			g_RenderThread->Stop();
		}

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

	case WM_KEYDOWN:
		if (g_Viewer)
		{
			g_Viewer->SetMovementKey(static_cast<char>(wParam), true);
		}
		break;

	case WM_KEYUP:
		if (g_Viewer)
		{
			g_Viewer->SetMovementKey(static_cast<char>(wParam), false);
		}
		break;

	case WM_KILLFOCUS:
		if (g_Viewer)
		{
			g_Viewer->SetMovementKey('w', false);
			g_Viewer->SetMovementKey('a', false);
			g_Viewer->SetMovementKey('s', false);
			g_Viewer->SetMovementKey('d', false);
		}
		break;

	case WM_MOUSEMOVE:
		g_MouseX = GET_X_LPARAM(lParam);
		g_MouseY = GET_Y_LPARAM(lParam);
		g_ImguiMouseButtons = 0;
		if ((MK_LBUTTON & wParam) != 0)
		{
			g_ImguiMouseButtons |= kImguiMouseLeft;
		}
		if ((MK_RBUTTON & wParam) != 0)
		{
			g_ImguiMouseButtons |= kImguiMouseRight;
		}
		SubmitImguiInput(0);

		if (g_Viewer)
		{
			const bool imguiActive = g_ImguiMouseCapture || IsMouseOverImgui(g_MouseX, g_MouseY);
			g_Viewer->MouseMsg(g_MouseX, g_MouseY, !imguiActive && (MK_RBUTTON & wParam) != 0);
		}
		break;

	case WM_LBUTTONDOWN:
		g_MouseX = GET_X_LPARAM(lParam);
		g_MouseY = GET_Y_LPARAM(lParam);
		g_ImguiMouseButtons |= kImguiMouseLeft;
		g_ImguiMouseCapture = IsMouseOverImgui(g_MouseX, g_MouseY);
		SetCapture(hWnd);
		SubmitImguiInput(0);
		if (g_Viewer)
		{
			g_Viewer->MouseMsg(g_MouseX, g_MouseY, false);
			if (!g_ImguiMouseCapture)
			{
				RECT rc = {};
				GetClientRect(hWnd, &rc);
				g_Viewer->SceneRayMsg(g_MouseX, g_MouseY, rc.right - rc.left, rc.bottom - rc.top);
			}
		}
		break;

	case WM_LBUTTONUP:
		g_MouseX = GET_X_LPARAM(lParam);
		g_MouseY = GET_Y_LPARAM(lParam);
		g_ImguiMouseButtons &= ~kImguiMouseLeft;
		SubmitImguiInput(0);
		if (g_Viewer)
		{
			g_Viewer->MouseMsg(g_MouseX, g_MouseY, false);
		}
		g_ImguiMouseCapture = false;
		if (g_ImguiMouseButtons == 0)
		{
			ReleaseCapture();
		}
		break;

	case WM_RBUTTONDOWN:
		g_MouseX = GET_X_LPARAM(lParam);
		g_MouseY = GET_Y_LPARAM(lParam);
		g_ImguiMouseButtons |= kImguiMouseRight;
		g_ImguiMouseCapture = IsMouseOverImgui(g_MouseX, g_MouseY);
		SetCapture(hWnd);
		SubmitImguiInput(0);
		if (g_Viewer)
		{
			g_Viewer->MouseMsg(g_MouseX, g_MouseY, !g_ImguiMouseCapture);
		}
		break;

	case WM_RBUTTONUP:
		g_MouseX = GET_X_LPARAM(lParam);
		g_MouseY = GET_Y_LPARAM(lParam);
		g_ImguiMouseButtons &= ~kImguiMouseRight;
		SubmitImguiInput(0);
		if (g_Viewer)
		{
			g_Viewer->MouseMsg(g_MouseX, g_MouseY, false);
		}
		g_ImguiMouseCapture = false;
		if (g_ImguiMouseButtons == 0)
		{
			ReleaseCapture();
		}
		break;

	case WM_MOUSEWHEEL:
	{
		POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
		ScreenToClient(hWnd, &pt);
		g_MouseX = pt.x;
		g_MouseY = pt.y;
		const int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		SubmitImguiInput(zDelta > 0 ? -1 : 1);

		if (g_Viewer)
		{
			if (!IsMouseOverImgui(g_MouseX, g_MouseY))
			{
				g_Viewer->MouseWheel(zDelta, (wParam & MK_CONTROL) != 0);
			}
		}
		break;
	}

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

//--------------------------------------------------------------------------------------
//
// http://msdn.microsoft.com/en-us/library/windows/apps/ff729721.aspx
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License (MIT).
//--------------------------------------------------------------------------------------
#include <chrono>
#include <string>
#include <thread>
#include <windows.h>
#include "resource.h"

#include "Viewer.h"
#include "../Src/RigidBodyDynamics/RigidBodySimulation.h"
#include "../Renderer/Renderer.h"

//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------

HINSTANCE               g_hInst = nullptr;
HWND                    g_hWnd = nullptr;
Renderer*               g_Renderer = nullptr;

LRESULT CALLBACK    WndProc( HWND, UINT, WPARAM, LPARAM );

WorldViewer* g_Viewer;

//--------------------------------------------------------------------------------------
// Register class and create window
//--------------------------------------------------------------------------------------
HRESULT InitWindow(HINSTANCE hInstance, int nCmdShow)
{
    // Register class
    WNDCLASSEX wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, (LPCTSTR)IDI_APP);
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = L"DX11WindowClass";
    wcex.hIconSm = LoadIcon(wcex.hInstance, (LPCTSTR)IDI_APP);
    if (!RegisterClassEx(&wcex))
        return E_FAIL;

    // Create window
    g_hInst = hInstance;
    RECT rc = { 0, 0, 1024, 768 };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    g_hWnd = CreateWindow(L"DX11WindowClass", L"DX11 Renderer",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hInstance,
        nullptr);
    if (!g_hWnd)
        return E_FAIL;

    ShowWindow(g_hWnd, nCmdShow);

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Entry point to the program. Initializes everything and goes into a message processing 
// loop. Idle time is used to render the scene.
//--------------------------------------------------------------------------------------
int WINAPI wWinMain( _In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow )
{
    UNREFERENCED_PARAMETER( hPrevInstance );
    UNREFERENCED_PARAMETER( lpCmdLine );

    if( FAILED( InitWindow( hInstance, nCmdShow ) ) )
        return 0;

    g_Renderer = Renderer::CreateDX11Renderer(g_hWnd, "../Renderer/");
    if (g_Renderer == nullptr)
    {
        return 0;
    }

    g_Viewer = new WorldViewer(g_Renderer);

    auto last = std::chrono::steady_clock::now();

    // Main message loop
    MSG msg = {0};
    while( WM_QUIT != msg.message )
    {
        if( PeekMessage( &msg, nullptr, 0, 0, PM_REMOVE ) )
        {
            TranslateMessage( &msg );
            DispatchMessage( &msg );
        }
        else
        {
            auto curr = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = curr - last;
            if (elapsed_seconds < std::chrono::milliseconds(16))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                g_Viewer->UpdateSimulator(0.016f);
				g_Renderer->Render();
                last = curr;
            }
        }
    }


    return ( int )msg.wParam;
}

#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp)                        ((int)(short)LOWORD(lp))
#endif
#ifndef GET_Y_LPARAM
#define GET_Y_LPARAM(lp)                        ((int)(short)HIWORD(lp))
#endif

//--------------------------------------------------------------------------------------
// Called every time the application receives a message
//--------------------------------------------------------------------------------------
LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam )
{
    PAINTSTRUCT ps;
    HDC hdc;

    switch( message )
    {
    case WM_PAINT:
        hdc = BeginPaint( hWnd, &ps );
        EndPaint( hWnd, &ps );
        break;

    case WM_DESTROY:
        delete g_Renderer;
        delete g_Viewer;
        g_Viewer = nullptr;
        PostQuitMessage( 0 );
        break;

    case WM_CHAR:
    {
        g_Viewer->KeyboardMsg((char)wParam);
        break;
    }

    case WM_LBUTTONDOWN:
        break;

    case WM_LBUTTONUP:
        break;

    case WM_MOUSEMOVE:
    {
        g_Viewer->MouseMsg(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), MK_LBUTTON & wParam);
    }
    break;

    case WM_MOUSEWHEEL:
    {
        auto zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        g_Viewer->MouseWheel(zDelta, wParam & MK_CONTROL);
    }
        break;
        // Note that this tutorial does not handle resizing (WM_SIZE) requests,
        // so we created the window without the resize border.

    default:
        return DefWindowProc( hWnd, message, wParam, lParam );
    }

    return 0;
}

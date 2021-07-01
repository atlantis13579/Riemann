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
#include <windows.h>
#include "resource.h"

#include "../Renderer/Renderer.h"

//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------

HINSTANCE               g_hInst = nullptr;
HWND                    g_hWnd = nullptr;
Renderer*               g_Renderer = nullptr;
Vector3d   g_CamParam = Vector3d(1.0f, 1.0f, 5.0f);

LRESULT CALLBACK    WndProc( HWND, UINT, WPARAM, LPARAM );

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

Vector3d GetCameraPosition()
{
    return Vector3d(sinf(g_CamParam.x) * cosf(g_CamParam.y), sinf(g_CamParam.y), cosf(g_CamParam.x) * cosf(g_CamParam.y)) * g_CamParam.z;
}

void UpdateCamera()
{
    Vector3d Eye = GetCameraPosition();
    Vector3d Center = Vector3d(0.0f, 0.0f, 0.0f);
    g_Renderer->SetCameraLookAt(Eye, Center);
}

void InitScene()
{
    Vertex1 Grounds_vertices[] =
    {
        { Vector3d(-100.0f, -5.0f, -100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
        { Vector3d(100.0f, -5.0f, -100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
        { Vector3d(100.0f, -5.0f, 100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
        { Vector3d(-100.0f, -5.0f, 100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
    };
    unsigned int Grounds_indices[] =
    {
        2,1,0,
        2,3,0,
    };
    g_Renderer->AddMesh("Ground", Grounds_vertices, sizeof(Grounds_vertices) / sizeof(Grounds_vertices[0]), Grounds_indices, sizeof(Grounds_indices) / sizeof(Grounds_indices[0]));

    Vertex1 vertices[] =
    {
        { Vector3d(-1.0f, 1.0f, -1.0f), Vector4d(0.0f, 0.0f, 1.0f, 1.0f) },
        { Vector3d(1.0f, 1.0f, -1.0f), Vector4d(0.0f, 1.0f, 0.0f, 1.0f) },
        { Vector3d(1.0f, 1.0f, 1.0f), Vector4d(0.0f, 1.0f, 1.0f, 1.0f) },
        { Vector3d(-1.0f, 1.0f, 1.0f), Vector4d(1.0f, 0.0f, 0.0f, 1.0f) },
        { Vector3d(-1.0f, -1.0f, -1.0f), Vector4d(1.0f, 0.0f, 1.0f, 1.0f) },
        { Vector3d(1.0f, -1.0f, -1.0f), Vector4d(1.0f, 1.0f, 0.0f, 1.0f) },
        { Vector3d(1.0f, -1.0f, 1.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) },
        { Vector3d(-1.0f, -1.0f, 1.0f), Vector4d(0.0f, 0.0f, 0.0f, 1.0f) },
    };

    // Create index buffer
    unsigned int indices[] =
    {
        3,1,0,
        2,1,3,

        0,5,4,
        1,5,0,

        3,4,7,
        0,4,3,

        1,6,5,
        2,6,1,

        2,7,6,
        3,7,2,

        6,4,5,
        7,4,6,
    };

    g_Renderer->AddMesh("Cube", vertices, sizeof(vertices) / sizeof(vertices[0]), indices, sizeof(indices) / sizeof(indices[0]));


    // dungeon, fighting
    g_Renderer->LoadObj("D:\\src\\client\\tools\\RecastEditor\\RecastDemo\\Release\\Meshes\\fighting.obj");
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

    InitScene();
    UpdateCamera();

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
            g_Renderer->Render();
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
        PostQuitMessage( 0 );
        break;

    case WM_LBUTTONDOWN:
        break;

    case WM_LBUTTONUP:
        break;

    case WM_MOUSEMOVE:
    {
        static WORD xPosPrev = 0, yPosPrev = 0;
        bool LButtonDown = MK_LBUTTON & wParam;
        if (LButtonDown)
        {
            if (xPosPrev * yPosPrev != 0)
            {
                WORD xPos = GET_X_LPARAM(lParam);
                WORD yPos = GET_Y_LPARAM(lParam);
                g_CamParam.x += (xPos - xPosPrev) * 0.01f;
                g_CamParam.y += (yPos - yPosPrev) * 0.01f;
                if (g_CamParam.y > 1.5)
                    g_CamParam.y = 1.5;
                if (g_CamParam.y < -1.5)
                    g_CamParam.y = -1.5;
                UpdateCamera();
            }

            xPosPrev = GET_X_LPARAM(lParam);
            yPosPrev = GET_Y_LPARAM(lParam);
        }
        else
        {
            xPosPrev = yPosPrev = 0;
        }
    }
    break;

    case WM_MOUSEWHEEL:
    {
        auto zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        float Scale = wParam & MK_CONTROL ? 10.0f : 1.0f;
        Scale *= 1.01f;
        if (zDelta > 0)
            Scale = 1.0f / Scale;
        g_CamParam.z *= Scale;
        UpdateCamera();
    }
        break;
        // Note that this tutorial does not handle resizing (WM_SIZE) requests,
        // so we created the window without the resize border.

    default:
        return DefWindowProc( hWnd, message, wParam, lParam );
    }

    return 0;
}

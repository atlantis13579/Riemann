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

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/RigidBodyDynamics/PhysicsWorldRB.h"
#include "../Renderer/Renderer.h"
#include "../Src/Tools/PhysxBinaryParser.h"

PhysicsWorldRB* g_World = nullptr;

//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------

HINSTANCE               g_hInst = nullptr;
HWND                    g_hWnd = nullptr;
Renderer*               g_Renderer = nullptr;
Vector3d    g_CamCenter = Vector3d::Zero();
Vector3d    g_CamParam = Vector3d(1.0f, 1.0f, 10.0f);

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
    return g_CamCenter + Vector3d(sinf(g_CamParam.x) * cosf(g_CamParam.y), sinf(g_CamParam.y), cosf(g_CamParam.x) * cosf(g_CamParam.y)) * g_CamParam.z;
}

void UpdateCamera()
{
    Vector3d Eye = GetCameraPosition();
    Vector3d Center = g_CamCenter;
    g_Renderer->SetCameraLookAt(Eye, Center);
}

#pragma optimize("", off)
void InitScene()
{
    PhysicsWorldRBParam param;
    param.Gravity = Vector3d(0, -9.8f, 0);
    RigidBodyParam rp;
    g_World = new PhysicsWorldRB(param);

    if (1)
    {
        Vertex1 Grounds_vertices[] =
        {
            { Vector3d(-100.0f,-5.0f, -100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
            { Vector3d(100.0f, -5.0f, -100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
            { Vector3d(100.0f, -5.0f, 100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
            { Vector3d(-100.0f, -5.0f, 100.0f), Vector4d(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f },
        };
        unsigned short Grounds_indices[] =
        {
            2,1,0,
            2,3,0,
        };

        rp.mass = 1.0f;
        rp.Static = true;
        Geometry* plane = GeometryFactory::CreatePlane(Vector3d(0, Grounds_vertices[0].Pos.y, 0), Vector3d::UnitY(), -0);
        g_World->CreateRigidBody(plane, rp);

        g_Renderer->AddMesh("Ground", plane->GetTransform(), Grounds_vertices, sizeof(Grounds_vertices) / sizeof(Grounds_vertices[0]), Grounds_indices, sizeof(Grounds_indices) / sizeof(Grounds_indices[0]), 2);
    }

    // Vector3d water_pos = Vector3d(-710.0f, 20.1f, 1184.0f);
    Vector3d water_pos = Vector3d(-710.0f, 20.1f, 1184.0f);
    Vector3d house_pos = Vector3d(-2222.0f, -81.0f, -773.0f);
    Vector3d bridge_pos = Vector3d(737.0f, -29.0f, -1495.0f);
    if (0)
    {
        Mesh mesh;
        mesh.LoadFlat("E:/Temp/iceland.flat");

        Transform* t = new Transform;
        t->SetScale(Vector3d(0.01f, 0.01f, 0.01f));
        g_CamCenter = t->LocalToWorld(bridge_pos);

        // t->SetTranslation(-Vector3d(-1.0f, 0, 0));

        g_Renderer->AddMesh(&mesh, t);
    }

    if (0)
    {
        VoxelField field;

        field.SerializeFrom("E:/Temp/fighting_new.voxel");

        Vector3d c = Vector3d(2556.8f, -104.0f, 1104.7f);

        float water_y = -106.65f;
		unsigned short water_low = field.WorldSpaceToVoxelSpaceY(water_y);
		unsigned short water_high = field.WorldSpaceToVoxelSpaceY(water_y + 1.5f);

        int idx = field.WorldSpaceToVoxelIndex(c);
		int cz = idx / field.GetSizeX();
		int cx = idx - field.GetSizeX() * cz;
		Voxel* v = field.GetVoxel(idx);

        field.MakeComplementarySet();

		Vector3d pos_main = field.GetBoundingVolume().GetCenter();
        pos_main.y = field.GetBoundingVolume().Max.y - 5.0f;

		const Vector3d Thr = Vector3d(0, 0.5f, 0);
		std::vector<Vector3d> water_list;

		water_list.emplace_back(-2790.0f, -106.65f, -1835.0f);

        vx_uint64 vol = field.Separate(pos_main, 1, 0.1f);
		for (size_t i = 0; i < water_list.size(); ++i)
		{
			vol = field.Separate(water_list[i] + Thr, (int)i + 2, 0.5f);
		}

		int Count = field.Filter(
			[&](Voxel* v) -> bool {
				return v->data == 0;
			});

        field.MakeComplementarySet();

		std::vector<int> data;
		field.IntersectYPlane(18.0f, data, 2.0f);

        Mesh* draw_mesh = field.CreateDebugMesh(cx - 100, cx + 100, cz - 100, cz + 100);

        float water_level = 19.9f;
        float water_level2 = water_level + 1.0f;

        int idx_water_level = field.WorldSpaceToVoxelSpaceY(water_level);
        int idx_water_level2 = field.WorldSpaceToVoxelSpaceY(water_level);

        Vector3d pmin = Vector3d(c.x - 200.0f, water_level, c.z - 200.0f);
        Vector3d pmax = Vector3d(c.x + 200.0f, water_level, c.z + 200.0f);
        // draw_mesh->AddAABB(Vector3d(pmin.x, water_level - 0.01f, pmin.z), Vector3d(pmax.x, water_level + 0.01f, pmax.z));
        // draw_mesh->AddAABB(Vector3d(pmin.x, water_level2 - 0.01f, pmin.z), Vector3d(pmax.x, water_level2 + 0.01f, pmax.z));

        Transform* t = new Transform;
        t->SetScale(Vector3d(0.02f, 0.02f, 0.02f));
        g_CamCenter = t->LocalToWorld(c);

        g_Renderer->AddMesh(draw_mesh, t);
    }

    if (1)
    {
        rp.mass = 1.0f;
        rp.Static = false;
        Geometry* aabb = GeometryFactory::CreateOBB(Vector3d(0.0f, 10.0f, 0.0f), Vector3d(-0.5f, -0.5f, -0.5f), Vector3d(0.5f, 0.5f, 0.5f));
        g_World->CreateRigidBody(aabb, rp);

        Mesh cube;
        cube.AddAABB(Vector3d(-0.5f, -0.5f, -0.5f), Vector3d(0.5f, 0.5f, 0.5f));
        g_Renderer->AddMesh(&cube, aabb->GetTransform());
    }

}
#pragma optimize("", on)



#pragma optimize("", off)
void InitPhysxScene()
{
    PhysicsWorldRBParam param;
    param.Gravity = Vector3d(0, -9.8f, 0);
    RigidBodyParam rp;
    g_World = new PhysicsWorldRB(param);

    Collections collection;
    PhysxBinaryParser::ParseCollectionFromBinary("e:/temp/physx/sss.bin", &collection);

    for (auto it : collection.mObjects)
    {
        Geometry* Geom = (Geometry*)it.first;
        if (Geom->GetShapeType() == TRIANGLE_MESH)
        {
            g_Renderer->AddMesh((Mesh*)Geom->GetShapeGeometry(), Geom->GetTransform());
        }
    }

}
#pragma optimize("", on)

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

    InitPhysxScene();
    UpdateCamera();

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
				g_World->Simulate(0.016f);
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
        delete g_World;
        PostQuitMessage( 0 );
        break;

    case WM_CHAR:
    {
        char c = (char)wParam;
        if (c == 'a')
            g_CamCenter.x -= 0.1f;
        else if (c == 'd')
            g_CamCenter.x += 0.1f;
        else if (c == 'w')
            g_CamCenter.z -= 0.1f;
        else if (c == 's')
            g_CamCenter.z += 0.1f;
        UpdateCamera();
        break;
    }

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
        Scale *= 1.1f;
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

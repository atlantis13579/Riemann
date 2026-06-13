cmake_minimum_required(VERSION 3.15)
project(Riemann)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(GLOB_RECURSE RIEMANN_SOURCES 
    "${CMAKE_SOURCE_DIR}/../Src/*.cpp"
    "${CMAKE_SOURCE_DIR}/../Src/*.cc"
    "${CMAKE_SOURCE_DIR}/../Src/*.c"
)

file(GLOB_RECURSE RIEMANN_HEADERS
    "${CMAKE_SOURCE_DIR}/../Src/*.h"
    "${CMAKE_SOURCE_DIR}/../Src/*.hpp"
)

file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_SOURCE_DIR}/../Test/*.cpp"
    "${CMAKE_SOURCE_DIR}/../Test/*.cc"
    "${CMAKE_SOURCE_DIR}/../Test/*.c"
)

add_library(Riemann STATIC ${RIEMANN_SOURCES} ${RIEMANN_HEADERS})

target_include_directories(Riemann PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../Src>
    $<INSTALL_INTERFACE:include>
)

set(RIEMANN_RENDERER_SOURCES
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/RiemannRenderer.cpp"
)

set(RIEMANN_RENDERER_HEADERS
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/RiemannRenderer.h"
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/DX11Renderer.h"
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/MetalRenderer.h"
)

set(IMGUI_SOURCES
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/imgui.cpp"
)

set(IMGUI_HEADERS
    "${CMAKE_SOURCE_DIR}/../RiemannRenderer/imgui.h"
)

if(WIN32)
    list(APPEND IMGUI_SOURCES "${CMAKE_SOURCE_DIR}/../RiemannRenderer/ImguiRenderDX11.cpp")
    list(APPEND IMGUI_HEADERS "${CMAKE_SOURCE_DIR}/../RiemannRenderer/ImguiRenderDX11.h")
endif()

add_library(Imgui STATIC ${IMGUI_SOURCES} ${IMGUI_HEADERS})
target_include_directories(Imgui PUBLIC
    ${CMAKE_SOURCE_DIR}/../RiemannRenderer
)
if(MSVC)
    target_compile_definitions(Imgui PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
target_compile_definitions(Imgui PUBLIC IMGUI_STATIC)

if(WIN32)
    target_link_libraries(Imgui PRIVATE d3d11 d3dcompiler gdi32)
endif()

if(WIN32)
    list(APPEND RIEMANN_RENDERER_SOURCES "${CMAKE_SOURCE_DIR}/../RiemannRenderer/DX11Renderer.cpp")
elseif(APPLE)
    enable_language(OBJCXX)
    list(APPEND RIEMANN_RENDERER_SOURCES "${CMAKE_SOURCE_DIR}/../RiemannRenderer/MetalRenderer.cpp")
    set_source_files_properties("${CMAKE_SOURCE_DIR}/../RiemannRenderer/MetalRenderer.cpp" PROPERTIES LANGUAGE OBJCXX)
endif()

add_library(RiemannRenderer STATIC ${RIEMANN_RENDERER_SOURCES} ${RIEMANN_RENDERER_HEADERS})
target_compile_definitions(RiemannRenderer PUBLIC RIEMANN_RENDERER_STATIC)
target_include_directories(RiemannRenderer PUBLIC
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../Src
)
target_link_libraries(RiemannRenderer PUBLIC Imgui)

if(WIN32)
    target_link_libraries(RiemannRenderer PRIVATE d3d11 d3dcompiler dxguid winmm comctl32 gdi32)
elseif(APPLE)
    find_library(METAL_FRAMEWORK Metal REQUIRED)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(COCOA_FRAMEWORK Cocoa REQUIRED)
    find_library(COREGRAPHICS_FRAMEWORK CoreGraphics REQUIRED)
    target_link_libraries(RiemannRenderer PRIVATE ${METAL_FRAMEWORK} ${QUARTZCORE_FRAMEWORK} ${COCOA_FRAMEWORK} ${COREGRAPHICS_FRAMEWORK})
endif()

set(RIEMANN_VIEWER_CORE_SOURCES
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/MeshRenderer.cpp"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/RenderThread.cpp"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/SceneWorld.cpp"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/Viewer.cpp"
)

set(RIEMANN_VIEWER_CORE_HEADERS
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/MeshRenderer.h"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/RenderThread.h"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/SceneWorld.h"
    "${CMAKE_SOURCE_DIR}/../RiemannViewer/Viewer.h"
)

add_library(RiemannViewerCore STATIC ${RIEMANN_VIEWER_CORE_SOURCES} ${RIEMANN_VIEWER_CORE_HEADERS})
target_include_directories(RiemannViewerCore PUBLIC
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../Src
)
target_link_libraries(RiemannViewerCore PUBLIC Riemann RiemannRenderer)

add_executable(Test ${TEST_SOURCES})

target_link_libraries(Test PRIVATE Riemann)

target_include_directories(Test PRIVATE 
    ${CMAKE_SOURCE_DIR}/../Src
    ${CMAKE_SOURCE_DIR}/../Test
)

if(WIN32)
    # Windows
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Test)

    add_executable(RiemannViewer WIN32
        "${CMAKE_SOURCE_DIR}/../RiemannViewer/WinApplication.cpp"
        "${CMAKE_SOURCE_DIR}/../RiemannViewer/RiemannViewer.rc"
    )
    target_include_directories(RiemannViewer PRIVATE
        ${CMAKE_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/../Src
    )
    target_link_libraries(RiemannViewer PRIVATE RiemannViewerCore RiemannRenderer Riemann shell32)
    
    # Visual Studio
    set_property(TARGET Test PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    
elseif(APPLE)
    # macOS
    set_target_properties(Riemann PROPERTIES
        XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17"
        XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++"
    )
    
elseif(UNIX)
    # Linux
    target_compile_options(Riemann PRIVATE -fPIC)
endif()

source_group(TREE "${CMAKE_SOURCE_DIR}/../Src" PREFIX "Source Files" FILES ${RIEMANN_SOURCES} ${RIEMANN_HEADERS})
source_group(TREE "${CMAKE_SOURCE_DIR}/../Test" PREFIX "Test Files" FILES ${TEST_SOURCES})
source_group(TREE "${CMAKE_SOURCE_DIR}/../RiemannRenderer" PREFIX "RiemannRenderer" FILES ${IMGUI_SOURCES} ${IMGUI_HEADERS})

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Library sources: ${RIEMANN_SOURCES}")
message(STATUS "Test sources: ${TEST_SOURCES}")

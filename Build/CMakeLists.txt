cmake_minimum_required(VERSION 3.15)
project(Riemann)

if(NOT MSVC)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE STRING "Supported configurations" FORCE)
endif()

get_filename_component(RIEMANN_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

if(WIN32)
    set(RIEMANN_PLATFORM_NAME "Win64")
elseif(APPLE)
    set(RIEMANN_PLATFORM_NAME "Mac")
elseif(UNIX)
    set(RIEMANN_PLATFORM_NAME "Linux")
else()
    set(RIEMANN_PLATFORM_NAME "${CMAKE_SYSTEM_NAME}")
endif()

set(RIEMANN_OUTPUT_DIRECTORY "${RIEMANN_ROOT_DIR}/Binaries/${RIEMANN_PLATFORM_NAME}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${RIEMANN_OUTPUT_DIRECTORY}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${RIEMANN_OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${RIEMANN_OUTPUT_DIRECTORY}")

foreach(OUTPUT_CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${OUTPUT_CONFIG}" OUTPUT_CONFIG_UPPER)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} "${RIEMANN_OUTPUT_DIRECTORY}/${OUTPUT_CONFIG}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} "${RIEMANN_OUTPUT_DIRECTORY}/${OUTPUT_CONFIG}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} "${RIEMANN_OUTPUT_DIRECTORY}/${OUTPUT_CONFIG}")
endforeach()

if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(GLOB_RECURSE RIEMANN_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../Src/*.cpp"
    "${CMAKE_SOURCE_DIR}/../Src/*.cc"
    "${CMAKE_SOURCE_DIR}/../Src/*.c"
)

file(GLOB_RECURSE RIEMANN_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../Src/*.h"
    "${CMAKE_SOURCE_DIR}/../Src/*.hpp"
)

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../Test/*.cpp"
    "${CMAKE_SOURCE_DIR}/../Test/*.cc"
    "${CMAKE_SOURCE_DIR}/../Test/*.c"
)

add_library(Riemann STATIC ${RIEMANN_SOURCES} ${RIEMANN_HEADERS})
if(MSVC)
    target_compile_definitions(Riemann PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_include_directories(Riemann PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../Src>
    $<INSTALL_INTERFACE:include>
)

set(RIEMANN_RENDERER_SOURCES
    "${CMAKE_SOURCE_DIR}/../Renderer/RiemannRenderer.cpp"
    "${CMAKE_SOURCE_DIR}/../Renderer/imgui.cpp"
)

set(RIEMANN_RENDERER_HEADERS
    "${CMAKE_SOURCE_DIR}/../Renderer/RiemannRenderer.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/DX11Renderer.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/MetalRenderer.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/imgui.h"
)

set(RIEMANN_RENDERER_SHADERS
    "${CMAKE_SOURCE_DIR}/../Renderer/dx11_shader.hlsl"
    "${CMAKE_SOURCE_DIR}/../Renderer/metal_shader.metal"
)
set_source_files_properties(${RIEMANN_RENDERER_SHADERS} PROPERTIES HEADER_FILE_ONLY TRUE)

if(WIN32)
    list(APPEND RIEMANN_RENDERER_SOURCES
        "${CMAKE_SOURCE_DIR}/../Renderer/DX11Renderer.cpp"
        "${CMAKE_SOURCE_DIR}/../Renderer/ImguiRenderDX11.cpp"
    )
    list(APPEND RIEMANN_RENDERER_HEADERS "${CMAKE_SOURCE_DIR}/../Renderer/ImguiRenderDX11.h")
elseif(APPLE)
    enable_language(OBJCXX)
    list(APPEND RIEMANN_RENDERER_SOURCES "${CMAKE_SOURCE_DIR}/../Renderer/MetalRenderer.cpp")
    set_source_files_properties("${CMAKE_SOURCE_DIR}/../Renderer/MetalRenderer.cpp" PROPERTIES LANGUAGE OBJCXX)
endif()

if(WIN32)
    set(RIEMANN_RENDERER_LIBRARY_TYPE SHARED)
else()
    set(RIEMANN_RENDERER_LIBRARY_TYPE STATIC)
endif()

add_library(Renderer ${RIEMANN_RENDERER_LIBRARY_TYPE} ${RIEMANN_RENDERER_SOURCES} ${RIEMANN_RENDERER_HEADERS} ${RIEMANN_RENDERER_SHADERS})
if(WIN32)
    target_compile_definitions(Renderer PRIVATE RIEMANN_RENDERER_EXPORT IMGUI_EXPORT _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_definitions(Renderer PUBLIC RIEMANN_RENDERER_STATIC IMGUI_STATIC)
endif()
target_include_directories(Renderer PUBLIC
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../Src
)

if(WIN32)
    target_link_libraries(Renderer PRIVATE d3d11 d3dcompiler dxguid winmm comctl32 gdi32)
elseif(APPLE)
    find_library(METAL_FRAMEWORK Metal REQUIRED)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(COCOA_FRAMEWORK Cocoa REQUIRED)
    find_library(COREGRAPHICS_FRAMEWORK CoreGraphics REQUIRED)
    target_link_libraries(Renderer PRIVATE ${METAL_FRAMEWORK} ${QUARTZCORE_FRAMEWORK} ${COCOA_FRAMEWORK} ${COREGRAPHICS_FRAMEWORK})
endif()

set(RIEMANN_VIEWER_CORE_SOURCES
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshRenderer.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/RenderThread.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/SceneWorld.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/Viewer.cpp"
)

set(RIEMANN_VIEWER_CORE_HEADERS
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshRenderer.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/RenderThread.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/SceneWorld.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/Viewer.h"
)

add_executable(Test ${TEST_SOURCES})
if(MSVC)
    target_compile_definitions(Test PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(Test PRIVATE $<$<CONFIG:Release>:/openmp>)
endif()

target_link_libraries(Test PRIVATE Riemann)

target_include_directories(Test PRIVATE 
    ${CMAKE_SOURCE_DIR}/../Src
    ${CMAKE_SOURCE_DIR}/../Test
)

if(WIN32)
    # Windows
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Test)

    add_executable(Viewer WIN32
        ${RIEMANN_VIEWER_CORE_SOURCES}
        ${RIEMANN_VIEWER_CORE_HEADERS}
        "${CMAKE_SOURCE_DIR}/../Viewer/WinApplication.cpp"
    )
    target_compile_definitions(Viewer PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_include_directories(Viewer PRIVATE
        ${CMAKE_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/../Src
    )
    target_link_libraries(Viewer PRIVATE
        d3d11
        d3dcompiler
        dxguid
        winmm
        comctl32
        shell32
        Renderer
        Riemann
    )
    
    # Visual Studio
    set_property(TARGET Test PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:Test>")
    
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
source_group(TREE "${CMAKE_SOURCE_DIR}/../Renderer" PREFIX "Renderer" FILES ${RIEMANN_RENDERER_SOURCES} ${RIEMANN_RENDERER_HEADERS} ${RIEMANN_RENDERER_SHADERS})
source_group(TREE "${CMAKE_SOURCE_DIR}/../Viewer" PREFIX "Viewer" FILES ${RIEMANN_VIEWER_CORE_SOURCES} ${RIEMANN_VIEWER_CORE_HEADERS})

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Library sources: ${RIEMANN_SOURCES}")
message(STATUS "Test sources: ${TEST_SOURCES}")

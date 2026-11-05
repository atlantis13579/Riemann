cmake_minimum_required(VERSION 3.20)
project(Riemann)

option(RIEMANN_ENABLE_CUDA "Build the optional CUDA solver backends" ON)
option(RIEMANN_ENABLE_COMPUTE_SHADER "Build the DirectCompute solver backends" ON)

set(RIEMANN_CUDA_AVAILABLE OFF)
set(RIEMANN_CUDA_TOOLKIT_ONLY OFF)
if(RIEMANN_ENABLE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(RIEMANN_CUDA_AVAILABLE ON)
        if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18 AND NOT CMAKE_CUDA_ARCHITECTURES)
            # CUDA 13 no longer emits device code for older GPUs by default.
            # Include native binaries for the supported Turing-through-Ada
            # generations so execution does not depend on driver-side PTX JIT.
            if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 13.0)
                set(CMAKE_CUDA_ARCHITECTURES "75;80;86;89")
            endif()
        endif()
        message(STATUS "CUDA solver backends enabled")
    else()
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND AND CUDAToolkit_NVCC_EXECUTABLE)
            set(RIEMANN_CUDA_AVAILABLE ON)
            set(RIEMANN_CUDA_TOOLKIT_ONLY ON)
            message(STATUS "CUDA standalone toolkit backends enabled")
        else()
            message(STATUS "CUDA compiler not found; building without CUDA solver backends")
        endif()
    endif()
endif()

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

set(RIEMANN_OUTPUT_DIRECTORY "${RIEMANN_ROOT_DIR}/Binaries/${RIEMANN_PLATFORM_NAME}"
    CACHE PATH "Build output directory")

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
    add_compile_options("$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:/W3>")
    add_compile_options("$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:/MP>")
else()
    add_compile_options(
        "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-Wall>"
        "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-Wextra>"
        "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-Wpedantic>"
    )
endif()

file(GLOB_RECURSE RIEMANN_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../Src/*.cpp"
    "${CMAKE_SOURCE_DIR}/../Src/*.cc"
    "${CMAKE_SOURCE_DIR}/../Src/*.c"
)

if(RIEMANN_CUDA_AVAILABLE AND NOT RIEMANN_CUDA_TOOLKIT_ONLY)
    list(APPEND RIEMANN_SOURCES
        "${CMAKE_SOURCE_DIR}/../Src/RigidBodyDynamics/LCPSolverCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Fluid/FluidBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Fluid/SmokeBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Cloth/PBDBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Continuum/SnowBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Continuum/SandBackendCUDA.cu"
    )
    # Keep contact projection arithmetic consistent with the scalar/SIMD paths.
    set_source_files_properties("${CMAKE_SOURCE_DIR}/../Src/Cloth/PBDBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Continuum/SnowBackendCUDA.cu"
        "${CMAKE_SOURCE_DIR}/../Src/Continuum/SandBackendCUDA.cu"
        PROPERTIES COMPILE_OPTIONS "--fmad=false")
endif()

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
if(WIN32 AND RIEMANN_ENABLE_COMPUTE_SHADER)
    target_compile_definitions(Riemann PUBLIC RIEMANN_ENABLE_COMPUTE_SHADER=1)
    target_compile_definitions(Riemann PRIVATE RIEMANN_SNOW_COMPUTE=1)
    target_link_libraries(Riemann PRIVATE d3d11 d3dcompiler dxguid)
elseif(APPLE AND RIEMANN_ENABLE_COMPUTE_SHADER)
    enable_language(OBJCXX)
    target_sources(Riemann PRIVATE "${RIEMANN_ROOT_DIR}/Src/Continuum/MetalComputeDevice.mm")
    set_source_files_properties("${RIEMANN_ROOT_DIR}/Src/Continuum/MetalComputeDevice.mm"
        PROPERTIES COMPILE_FLAGS "-fobjc-arc")
    set_property(TARGET Riemann PROPERTY OBJCXX_STANDARD 14)
    target_compile_definitions(Riemann PRIVATE RIEMANN_SNOW_COMPUTE=1)
    find_library(SNOW_METAL_FRAMEWORK Metal REQUIRED)
    find_library(SNOW_FOUNDATION_FRAMEWORK Foundation REQUIRED)
    target_link_libraries(Riemann PRIVATE ${SNOW_METAL_FRAMEWORK} ${SNOW_FOUNDATION_FRAMEWORK})
endif()
if(RIEMANN_CUDA_AVAILABLE)
    # CUDA is part of the public solver API when the backend is compiled in.
    # Propagate the feature macro so callers and tests cannot reference CUDA
    # declarations in a non-CUDA build.
    target_compile_definitions(Riemann PUBLIC RIEMANN_ENABLE_CUDA=1)
    target_compile_definitions(Riemann PRIVATE RIEMANN_SNOW_CUDA=1)
    set_target_properties(Riemann PROPERTIES
        CUDA_STANDARD 14
        CUDA_STANDARD_REQUIRED ON
    )
    if(RIEMANN_CUDA_TOOLKIT_ONLY)
        set(RIEMANN_CUDA_ARCHITECTURES "75;86" CACHE STRING "CUDA real and virtual architectures")
        set(cuda_architecture_flags)
        foreach(architecture IN LISTS RIEMANN_CUDA_ARCHITECTURES)
            list(APPEND cuda_architecture_flags "--generate-code=arch=compute_${architecture},code=[sm_${architecture},compute_${architecture}]")
        endforeach()
        set(cuda_host_flags)
        if(MSVC)
            list(APPEND cuda_host_flags "-Xcompiler=$<IF:$<CONFIG:Debug>,/MDd,/MD>" "-Xcompiler=/EHsc")
        else()
            list(APPEND cuda_host_flags "-Xcompiler=-fPIC")
        endif()
        foreach(cuda_path RigidBodyDynamics/LCPSolverCUDA Fluid/FluidBackendCUDA Fluid/SmokeBackendCUDA Cloth/PBDBackendCUDA Continuum/SnowBackendCUDA Continuum/SandBackendCUDA)
            get_filename_component(cuda_name "${cuda_path}" NAME)
            set(cuda_object "${CMAKE_CURRENT_BINARY_DIR}/Cuda/$<CONFIG>/${cuda_name}${CMAKE_CXX_OUTPUT_EXTENSION}")
            add_custom_command(OUTPUT "${cuda_object}"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/Cuda/$<CONFIG>"
                COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}" --std=c++14 -O3 --fmad=false
                    ${cuda_architecture_flags} ${cuda_host_flags} -ccbin "${CMAKE_CXX_COMPILER}"
                    -DRIEMANN_ENABLE_CUDA=1 -DRIEMANN_SNOW_CUDA=1
                    "-I${RIEMANN_ROOT_DIR}/Src" -c "${RIEMANN_ROOT_DIR}/Src/${cuda_path}.cu" -o "${cuda_object}"
                DEPENDS "${RIEMANN_ROOT_DIR}/Src/${cuda_path}.cu" ${RIEMANN_HEADERS}
                VERBATIM)
            set_source_files_properties("${cuda_object}" PROPERTIES GENERATED TRUE EXTERNAL_OBJECT TRUE)
            target_sources(Riemann PRIVATE "${cuda_object}")
        endforeach()
        target_link_libraries(Riemann PRIVATE CUDA::cudart_static)
    endif()
endif()
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
    "${CMAKE_SOURCE_DIR}/../Renderer/ImguiInputQueue.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/RiemannRenderer.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/DensityVolume.h"
    "${CMAKE_SOURCE_DIR}/../Renderer/DX11VolumeRenderer.h"
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
    "${CMAKE_SOURCE_DIR}/../Viewer/SnowDemo.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/SnowSurfaceBuilder.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/SolverPanel.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshRenderer.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshCuttingPanel.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshSimplificationPanel.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/RenderThread.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/SceneWorld.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/Viewer.cpp"
)

set(RIEMANN_VIEWER_CORE_HEADERS
    "${CMAKE_SOURCE_DIR}/../Viewer/SnowSurfaceBuilder.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/SolverPanel.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshRenderer.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshCuttingPanel.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/MeshSimplificationPanel.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/RenderThread.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/SceneWorld.h"
    "${CMAKE_SOURCE_DIR}/../Viewer/Viewer.h"
)

add_executable(Test ${TEST_SOURCES})
target_sources(Test PRIVATE
    "${CMAKE_SOURCE_DIR}/../Viewer/SceneWorld.cpp"
    "${CMAKE_SOURCE_DIR}/../Viewer/SolverPanel.cpp"
    "${CMAKE_SOURCE_DIR}/../Renderer/imgui.cpp"
)
target_compile_definitions(Test PRIVATE IMGUI_STATIC)
if(MSVC)
    target_compile_definitions(Test PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(Test PRIVATE $<$<CONFIG:Release>:/openmp>)
endif()

target_link_libraries(Test PRIVATE Riemann)

enable_testing()
add_test(NAME FluidOptimization COMMAND Test FluidOptimization)
set_tests_properties(FluidOptimization PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
add_test(NAME Smoke COMMAND Test Smoke)
set_tests_properties(Smoke PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
add_test(NAME SmokeBackends COMMAND Test SmokeBackends)
set_tests_properties(SmokeBackends PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
add_test(NAME SmokeMovingObstacles COMMAND Test SmokeMovingObstacles)
set_tests_properties(SmokeMovingObstacles PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
add_test(NAME SmokeRigidCoupling COMMAND Test SmokeRigidCoupling)
set_tests_properties(SmokeRigidCoupling PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
foreach(TEST_GROUP SolverPanel SolverSwitch PBDBackendScenes PBDBackends VehicleScene VehicleDeformation RopeTowing XPBDScene XPBD PBDScene PBD VoxelScene VoxelMesh TerrainScene TerrainEditing GEMM StressIntegration Stress Deformation Scenes RopeRegression Constraints ConstraintRegression Rope Cloth Ragdoll Terrain Matrix Destruction CCD)
    add_test(NAME ${TEST_GROUP} COMMAND Test ${TEST_GROUP})
    set_tests_properties(${TEST_GROUP} PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")
endforeach()

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
    
    add_executable(RendererLifecycleValidation "${CMAKE_SOURCE_DIR}/../Test/RendererLifecycleValidation.cpp")
    target_compile_definitions(RendererLifecycleValidation PRIVATE RIEMANN_RENDERER_LIFECYCLE_VALIDATION)
    target_link_libraries(RendererLifecycleValidation PRIVATE Renderer)
    add_test(NAME RendererLifecycleValidation COMMAND RendererLifecycleValidation)
    set_tests_properties(RendererLifecycleValidation PROPERTIES WORKING_DIRECTORY "${RIEMANN_ROOT_DIR}")

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

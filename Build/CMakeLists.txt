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

add_executable(Test ${TEST_SOURCES})

target_link_libraries(Test PRIVATE Riemann)

target_include_directories(Test PRIVATE 
    ${CMAKE_SOURCE_DIR}/../Src
    ${CMAKE_SOURCE_DIR}/../Test
)

if(WIN32)
    # Windows
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Test)
    
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

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Library sources: ${RIEMANN_SOURCES}")
message(STATUS "Test sources: ${TEST_SOURCES}")
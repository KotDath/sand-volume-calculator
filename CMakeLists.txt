cmake_minimum_required(VERSION 3.21)
project(sand-volume-calculator)

set(CMAKE_CXX_STANDARD 20)

set(Open3D_ROOT ./open3d)

option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

find_package(Open3D REQUIRED)

add_executable(sand-volume-calculator main.cpp)

target_sources(sand-volume-calculator PRIVATE main.cpp)
target_link_libraries(sand-volume-calculator PRIVATE Open3D::Open3D)

if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET sand-volume-calculator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()
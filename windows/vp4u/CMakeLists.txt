# Lidar2Kinect for DanceAround: the replacement VisionPoseWrapper.dll (vp4u API 210). 64-bit: the
# game (Unity 2020.3, x64) loads it from game\dancearound_data\plugins\x86_64\.
#   cmake -S windows/vp4u -B build/vp4u -A x64
#   cmake --build build/vp4u --config Release
#   ctest --test-dir build/vp4u -C Release --output-on-failure
# Output: build/vp4u/bin/<config>/  visionposewrapper.dll, vp4u_probe.exe, test_vp4u.exe, the shared tools.
cmake_minimum_required(VERSION 3.15)
project(lidar2kinect_vp4u C CXX)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Build the vp4u wrapper as 64-bit: cmake -A x64")
endif()

# static C runtime: nothing to install next to the DLL
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()
add_subdirectory(../../shared shared)

set(L2KW_DEFS WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)

# ---- the Kinect-free part of windows/core: config, log, receiver ----
add_library(l2kw_base STATIC
    ../core/l2kw_util.cpp
    ../core/l2kw_receiver.cpp
    ../core/l2kw_jpeg.cpp
    ../core/l2kw_dnssd.cpp)
target_include_directories(l2kw_base PUBLIC ../core)
target_link_libraries(l2kw_base PUBLIC l2k ws2_32 windowscodecs ole32 oleaut32)
target_compile_definitions(l2kw_base PUBLIC ${L2KW_DEFS})
target_compile_options(l2kw_base PRIVATE /W4 /wd4100)

# ---- skeleton mapping (no Windows dependency) ----
add_library(vp4u_map STATIC vp4u_map.cpp)
target_include_directories(vp4u_map PUBLIC .)
target_link_libraries(vp4u_map PUBLIC l2k)
target_compile_options(vp4u_map PRIVATE /W4)

# ---- visionposewrapper.dll ----
add_library(visionposewrapper SHARED vp4u_wrapper.cpp visionposewrapper.def)
set_target_properties(visionposewrapper PROPERTIES OUTPUT_NAME visionposewrapper)
target_link_libraries(visionposewrapper PRIVATE l2kw_base vp4u_map)
target_compile_options(visionposewrapper PRIVATE /W4 /wd4100)

# ---- probe: drives the DLL like VisionPoseForUnity.dll ----
add_executable(vp4u_probe probe/vp4u_probe.cpp)
target_include_directories(vp4u_probe PRIVATE .)
target_compile_definitions(vp4u_probe PRIVATE ${L2KW_DEFS})
add_dependencies(vp4u_probe visionposewrapper l2k_fake_sender)

# ---- tests ----
add_executable(test_vp4u tests/test_vp4u.cpp)
target_link_libraries(test_vp4u PRIVATE vp4u_map)
add_test(NAME vp4u_map COMMAND test_vp4u)

set_target_properties(l2k_dump l2k_fake_sender PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# the default: SKELETON + BODY at 60 frames a second, frames from the sender's BODY
add_test(NAME vp4u_probe
    COMMAND vp4u_probe --check -t 6 --sender $<TARGET_FILE:l2k_fake_sender>)
# a receiver that does not ask for BODY: every body made from SKELETON here
file(WRITE ${CMAKE_BINARY_DIR}/probe_skeleton.ini "[vp4u]\nbody=0\n")
add_test(NAME vp4u_probe_skeleton
    COMMAND vp4u_probe --check -t 4 --expect-source skeleton --sender $<TARGET_FILE:l2k_fake_sender>)
set_tests_properties(vp4u_probe PROPERTIES
    ENVIRONMENT "L2K_PORT=24812;L2K_INI=none.ini"
    RUN_SERIAL TRUE
    TIMEOUT 90)
set_tests_properties(vp4u_probe_skeleton PROPERTIES
    ENVIRONMENT "L2K_PORT=24813;L2K_INI=${CMAKE_BINARY_DIR}/probe_skeleton.ini"
    RUN_SERIAL TRUE
    TIMEOUT 90)

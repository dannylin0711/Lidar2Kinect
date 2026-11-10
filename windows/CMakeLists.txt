# Lidar2Kinect, Windows side. Everything is 32-bit: the game's DLL is x86.
#   cmake -S windows -B build/windows -A Win32
#   cmake --build build/windows --config Release
#   ctest --test-dir build/windows -C Release --output-on-failure
# Output: build/windows/bin/<config>/  Kinect10.dll, l2k_spicehook.dll, l2k_probe.exe, tools.
cmake_minimum_required(VERSION 3.15)
project(lidar2kinect_windows C CXX)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "Build the Windows side as 32-bit: cmake -A Win32")
endif()

# static C runtime: nothing to install next to the DLLs
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()
add_subdirectory(../shared shared)

# ---- core: receiver + virtual sensor ----
add_library(l2kw_core STATIC
    core/l2kw_util.cpp
    core/l2kw_receiver.cpp
    core/l2kw_sensor.cpp
    core/l2kw_convert.cpp
    core/l2kw_jpeg.cpp
    core/l2kw_dnssd.cpp)
target_include_directories(l2kw_core PUBLIC core)
target_link_libraries(l2kw_core PUBLIC l2k ws2_32 windowscodecs ole32 oleaut32)
target_compile_definitions(l2kw_core PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)
target_compile_options(l2kw_core PRIVATE /W4 /wd4100)

# ---- 2b: Kinect10.dll ----
add_library(kinect10 SHARED kinect10/dllmain.cpp kinect10/kinect10.def)
set_target_properties(kinect10 PROPERTIES OUTPUT_NAME Kinect10)
target_link_libraries(kinect10 PRIVATE l2kw_core)

# ---- 2a: spice2x hook ----
add_library(l2k_spicehook SHARED spicehook/spicehook.cpp)
target_link_libraries(l2k_spicehook PRIVATE l2kw_core)

# ---- probe: imports Kinect10.dll by ordinal, like gamekdm.dll ----
set(GAME_IMPLIB ${CMAKE_CURRENT_BINARY_DIR}/kinect10_game.lib)
add_custom_command(OUTPUT ${GAME_IMPLIB}
    COMMAND ${CMAKE_AR} /nologo /def:${CMAKE_CURRENT_SOURCE_DIR}/probe/kinect10_game.def /machine:x86 /out:${GAME_IMPLIB}
    DEPENDS probe/kinect10_game.def
    COMMENT "Import library for Kinect10.dll by ordinal (the game's five imports)")
add_custom_target(kinect10_game_implib DEPENDS ${GAME_IMPLIB})
add_executable(l2k_probe probe/probe.cpp)
add_dependencies(l2k_probe kinect10_game_implib kinect10)
target_link_libraries(l2k_probe PRIVATE ${GAME_IMPLIB} windowscodecs ole32)
target_compile_definitions(l2k_probe PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS)

# ---- tools ----
add_executable(l2k_smoothcmp tools/smoothcmp.cpp)
target_link_libraries(l2k_smoothcmp PRIVATE l2kw_core)

# the shared tools next to the DLL, so one folder holds a complete test setup
set_target_properties(l2k_dump l2k_fake_sender PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- tests ----
add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core PRIVATE l2kw_core)
add_test(NAME core COMMAND test_core)

set(TESTCARD ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tests/vectors/testcard_640x480.jpg)
add_test(NAME probe_stream
    COMMAND l2k_probe --check -t 8 --players 2 --sender $<TARGET_FILE:l2k_fake_sender> --jpeg ${TESTCARD})
add_test(NAME probe_reconnect
    COMMAND l2k_probe --check --reconnect -t 6 --players 1 --sender $<TARGET_FILE:l2k_fake_sender> --jpeg ${TESTCARD})
set_tests_properties(probe_stream probe_reconnect PROPERTIES
    ENVIRONMENT "L2K_PORT=24811;L2K_INI=none.ini"
    RUN_SERIAL TRUE
    TIMEOUT 90)

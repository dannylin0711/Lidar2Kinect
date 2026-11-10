# Lidar2Kinect shared library (portable C99). Used by the Windows side; the Apple side builds the
# same sources through ../Package.swift.
#   cmake -S shared -B build/shared -A Win32      (the game is 32-bit: the DLLs that link this must be too)
#   cmake --build build/shared --config Release
#   ctest --test-dir build/shared -C Release
cmake_minimum_required(VERSION 3.15)
project(l2k_shared C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(l2k STATIC
    src/l2k_protocol.c
    src/l2k_depth.c
    src/l2k_map.c
    src/l2k_synth.c)
target_include_directories(l2k PUBLIC include)
if(MSVC)
    target_compile_options(l2k PRIVATE /W4)
    target_compile_definitions(l2k PUBLIC _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(l2k PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(l2k PUBLIC m)
endif()

option(L2K_BUILD_TOOLS "Build l2k_dump and l2k_fake_sender" ON)
option(L2K_BUILD_TESTS "Build the tests" ON)

if(L2K_BUILD_TOOLS)
    foreach(tool l2k_dump l2k_fake_sender)
        add_executable(${tool} tools/${tool}.c)
        target_include_directories(${tool} PRIVATE tools)
        target_link_libraries(${tool} PRIVATE l2k)
        if(WIN32)
            target_link_libraries(${tool} PRIVATE ws2_32)
        endif()
    endforeach()
endif()

if(L2K_BUILD_TESTS)
    enable_testing()
    add_executable(test_l2k tests/test_l2k.c)
    target_link_libraries(test_l2k PRIVATE l2k)
    add_test(NAME l2k COMMAND test_l2k ${CMAKE_CURRENT_SOURCE_DIR}/tests/vectors)
endif()
